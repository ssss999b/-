/*
 * OmniFlight — F103 无人机遥控器（Remote Controller）
 *
 * 本目标的作用：
 *   1. ADC 读摇杆的 2 路电位器（油门 PA2、高度设定 PA3 = ADC1 CH2/CH3）；
 *   2. 把 ADC 原始值映射成 0~250 数值，高度再换算为 0.0~2.5 m；
 *   3. 通过 NRF24L01（2.4G）把控制帧（油门 + 目标高度）发送到飞控端；
 *   4. 轮询接收飞控回传的遥测并打印；
 *   5. OLED 显示机器状态 + 油门 + 目标高度；
 *   6. USART1（PA9/PA10）作为调试口（板子只引出串口一/二，USART3 未引出）。
 *
 * 其余配置（TIM/PWM/I2C/KEY/MPU6050）仅为调试占位，本目标不调用。
 *
 * 串口分配（F103C8T6）：
 *   USART1 TX=PA9 RX=PA10  — 调试（唯一可用调试口）
 *   USART3 TX=PB10 RX=PB11 — 未引出，保留注册但不再打印
 *
 * 引脚映射详见 Enroll/103_hw_config.h。
 */

#include "Enroll.h"
#include "sys.h"
#include "Delay.h"

#include "usart.h"
#include "adc.h"
#include "gpio.h"
#include "pwm.h"
#include "API_SPI.h"
#include "My_Usart/My_Usart.h"

#include "LED.h"
#include "KEY.h"
#include "NRF24L01.h"
#include "OLED.h"

/* 主循环阻塞节拍（毫秒）。 */
#define REMOTE_LOOP_DELAY_MS   10U
/* OLED 刷新周期（主循环次数），10 * 10ms = 100ms。 */
#define OLED_REFRESH_DIV       10U
/* 串口调试打印周期（主循环次数），50 * 10ms = 500ms。 */
#define UART_PRINT_DIV         50U

/* PA8 (TIM1_CH1) 1Hz 闪烁 PWM：
 * 72MHz / 7200(PSC+1) = 10kHz，ARR=9999 -> 1Hz，CCR=5000 -> 50% 占空比。
 * 即 0.5s 亮 / 0.5s 灭（一秒一闪）。 */
#define BLINK_PWM_ARR   9999U
#define BLINK_PWM_PSC   7199U
#define BLINK_PWM_CCR   5000U

/* ============================================================
 * 摇杆 ADC 通道定义（ADC1）
 * ============================================================ */
#define STICK_ADC          API_ADC1
#define STICK_CH_THROTTLE  API_ADC_CH2   /* PA2 摇杆1 Y 油门（实际接线） */
#define STICK_CH_ALT       API_ADC_CH3   /* PA3 摇杆2 Y 高度设定（实际接线） */

/* 摇杆映射范围：ADC 0~4095 -> 0~250（与飞控端油门映射一致） */
#define STICK_OUT_MIN   0U
#define STICK_OUT_MAX   250U
#define STICK_ADC_MAX   4095U

/* 高度设定映射：摇杆 0~250 -> 0.0 ~ 2.5 m（每格 1 cm） */
#define ALT_SET_MIN   0.0f
#define ALT_SET_MAX   2.5f

/* ============================================================
 * 2.4G 发送包布局（32 字节）
 *
 * 严格按《遥控通讯协议》：
 *   [0] Mode     ：1 = 请求回传（飞控立刻回发遥测）/ 0 = 纯控制包
 *   [1] Key      ：1 = 解锁（PB7 高电平）/ 0 = 锁定
 *   [2] Throttle ：油门 0~250
 *   [3]          ：保留 0
 *   [4..7] Set_Alt：目标高度（float，单位 m，小端）
 *   [8..31]      ：保留 0（帧尾清零）
 *
 * 注意：收发地址 NRF24L01_TxAddress / NRF24L01_RxAddress 默认都是
 *       {0x11,0x22,0x33,0x44,0x55}，遥控器与飞控端必须一致，
 *       如不一致需在此处或 NRF24L01.c 里同步修改。
 * ============================================================ */

/* 串口中断回调：排空 TX 队列（printf 异步发送核心），RX 暂不解析。 */
static void Remote_USART_Callback(API_USART_Id_t id)
{
	uint32_t data = 0U;
	uint8_t rxValid = 0U;

	usart_irq_dispatch_by_id(id, &data, &rxValid);
	(void)data;
	(void)rxValid;
}

/* ADC 原始值映射到摇杆输出范围。 */
static uint16_t Remote_MapStick(uint16_t raw)
{
	uint32_t v = (uint32_t)raw * STICK_OUT_MAX / STICK_ADC_MAX;
	if (v > STICK_OUT_MAX)
	{
		v = STICK_OUT_MAX;
	}
	return (uint16_t)v;
}

/*
 * 二档开关 → Key（公共端接 GND，PB6/PB7 为两个掷端，同一时刻只有一个接地）：
 *   位置一：PB6=0, PB7=1 → 解锁(1)
 *   位置二：PB6=1, PB7=0 → 锁定(0)
 *   异常（PB6/PB7 同高 = 断线悬空；同低 = 短路）→ 锁定(0)，失效安全
 *
 * 单靠 PB7 判断不够：断线时上拉会让 PB7 恒为高，被误判成解锁。
 * 因此必须两路交叉校验，并做 3 次（约 30ms）一致确认，过滤机械抖动。
 */
static uint8_t Remote_ReadKey(void)
{
	static uint8_t s_stable = 0U;   /* 已确认的稳定档位 */
	static uint8_t s_count  = 0U;   /* 与稳定值不一致的连续次数 */
	uint8_t pb6;
	uint8_t pb7;
	uint8_t raw;

	pb6 = (API_GPIO_Read(HW_SWITCH_A_PORT, HW_SWITCH_A_PIN) != 0U) ? 1U : 0U;
	pb7 = (API_GPIO_Read(HW_SWITCH_B_PORT, HW_SWITCH_B_PIN) != 0U) ? 1U : 0U;

	/* 只有「PB6 接地且 PB7 悬空高」才判为解锁；其余一律锁定。 */
	raw = ((pb6 == 0U) && (pb7 == 1U)) ? 1U : 0U;

	if (raw == s_stable)
	{
		s_count = 0U;
	}
	else if (++s_count >= 3U)
	{
		s_count  = 0U;
		s_stable = raw;
	}

	return s_stable;
}

/* 填充并清空发送包（控制帧：Mode + Key + 油门 + 目标高度）。 */
static void Remote_BuildPacket(uint8_t mode, uint8_t key, uint8_t throttle, float altSet)
{
	uint8_t i;

	NRF24L01_TxPacket[0] = mode;     /* Mode: 1=请求回传 / 0=纯控制 */
	NRF24L01_TxPacket[1] = key;      /* Key: 1=解锁(PB7高) / 0=锁定 */
	NRF24L01_TxPacket[2] = throttle; /* 油门 0~250 */
	NRF24L01_TxPacket[3] = 0U;       /* 保留 */

	/* 目标高度（m），占用 [4..7]，与飞控端 NRF24L01_Data() 的 Set_Alt 对齐 */
	*(float *)&NRF24L01_TxPacket[4] = altSet;

	for (i = 8U; i < NRF24L01_TX_PACKET_WIDTH; i++)
	{
		NRF24L01_TxPacket[i] = 0U;
	}
}

int main(void)
{
	uint16_t throttle, alt;
	float altSet;
	uint8_t key  = 0U;       /* 二档开关：1=解锁(PB7高) / 0=锁定 */
	uint8_t mode = 0U;       /* 回传请求：1=请求回传 / 0=纯控制 */
	uint32_t loop = 0U;
	uint8_t nrf_send = 0U;   /* NRF24L01_Send() 返回值：1=TX_DS 2=MAX_RT 3=异常 4=超时 */
	uint8_t nrf_retr = 0U;   /* 上电自检：SETUP_RETR 回读原始值（A1=OK FF/00=接线/供电问题） */
	uint8_t nrf_loop = 0U;   /* SPI 回环：PB13(MOSI) 短接 PA5(MISO) 时发 0xA5 应回读 0xA5 */

	/* 飞控回传遥测（布局见飞控端 NRF24L01_Data()，32 字节） */
	uint8_t  nrf_rx   = 0U;    /* NRF24L01_Receive() 返回值：1=收到飞控回传包 */
	uint16_t rx_cnt   = 0U;    /* 有效回传包计数（链路通=持续增长） */
	uint16_t rx_speed = 0U;    /* [1..2] speed_temp 基础油门 */
	float rx_pitch = 0.0f;     /* [4..7]   Pitch */
	float rx_roll  = 0.0f;     /* [8..11]  Roll */
	float rx_yaw   = 0.0f;     /* [12..15] IMU_Yaw */
	float rx_alt   = 0.0f;     /* [16..19] Alt_Fused */
	float rx_pp    = 0.0f;     /* [20..23] pid_rate_pitch.output */
	float rx_pr    = 0.0f;     /* [24..27] pid_rate_roll.output */
	float rx_pa    = 0.0f;     /* [28..31] pid_alt.output */

	/* 系统时钟 */
	SYS_Init();

	/* 注册层 */
	Enroll_LED_Register();
	Enroll_KEY_Register();
	Enroll_USART_Register();
	Enroll_SPI_Register();
	Enroll_NRF24L01_Register();
	Enroll_OLED_RegisterSpiCtrl();
	Enroll_ADC_Register();
	Enroll_PWM_Register(); 

	/* 注册串口中断回调（异步 TX 排空） */
	Enroll_USART_RegisterIrqHandler(Remote_USART_Callback);

	/* 初始化串口 */
	API_USART_Init(API_USART1, 115200U);
	API_USART_Init(API_USART3, 115200U);

	/* NRF24L01 上电复位约需 100ms，复位期间 SPI 不应答；先等它完成再初始化。 */
	Delay_ms(150U);

	/* 2.4G + OLED（内部各自调用 API_SPI_Init 初始化自身总线） */
	NRF24L01_Init();

	/* ── NRF24L01 本地自检（无需飞机）：写 SETUP_RETR=0xA1 再回读 ── */
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0xA1U);
	nrf_retr = NRF24L01_ReadReg(NRF24L01_SETUP_RETR);   /* 期望 0xA1；FF/00 = 接线/供电/模块问题 */
	NRF24L01_WriteReg(NRF24L01_SETUP_RETR, NRF24L01_SETUP_RETR_VALUE);      /* 恢复 Init 里的默认重传参数 */

	OLED_Init(OLED_IF_SPI);
    
	/* ADC */
	API_ADC_Init(STICK_ADC);

	/* 二档开关输入初始化：公共端接 GND，PB6/PB7 两个掷端都做上拉输入。
	 * 未被选中的一端浮空 → 上拉拉成高电平；被选中那端被开关拉到 GND → 读低。
	 * 故 PB7=1 表示开关在 PB6 侧（位置一），PB7=0 表示开关在 PB7 侧（位置二）。 */
	API_GPIO_InitInputPullUp(HW_SWITCH_A_PORT, HW_SWITCH_A_PIN);
	API_GPIO_InitInputPullUp(HW_SWITCH_B_PORT, HW_SWITCH_B_PIN);

	/* 板载 LED 心跳 */
	LED_Init(LED_LOW);

	/* PA8 (TIM1_CH1) 输出 1Hz PWM，0.5s 亮 / 0.5s 灭（一秒一闪） */
	API_PWM_Init(API_PWM_TIM1, BLINK_PWM_ARR, BLINK_PWM_PSC);
	API_PWM_Setcom(API_PWM_TIM1, API_PWM_CH1, BLINK_PWM_CCR);

	/* ── OLED 自检：显存全填 0xFF（全屏点亮），定位模组/接线 vs 数据路径问题 ── */
	OLED_Clear();
	OLED_Reverse();   /* 全 0 -> 全 0xFF，整屏白色 */
	OLED_Update();
	Delay_ms(200U);   /* 白屏保持 200ms，便于确认模组已点亮 */

	/* 擦除白屏，避免残留白色直到主循环第一次刷新 */
	OLED_Clear();
	OLED_Update();

	while (1)
	{
		/* 读 2 路摇杆并映射（油门 + 高度设定） */
		throttle = Remote_MapStick(API_ADC_GetValue(STICK_ADC, STICK_CH_THROTTLE));
		alt      = Remote_MapStick(API_ADC_GetValue(STICK_ADC, STICK_CH_ALT));
		altSet   = ALT_SET_MIN + (float)alt * (ALT_SET_MAX - ALT_SET_MIN) / (float)STICK_OUT_MAX;

		/* 读二档开关 → Key（PB7 高电平 = 解锁） */
		key = Remote_ReadKey();

		/* Mode 按协议作为回传请求标志：每 100ms 置 1，其余 100Hz 纯控制 */
		mode = ((loop % 10U) == 0U) ? 1U : 0U;

		/* 填充并发送控制帧 */
		Remote_BuildPacket(mode, key, (uint8_t)throttle, altSet);
		nrf_send = NRF24L01_Send();

		/* 接收飞控回传并解析（回传包 ID=0x01，布局见飞控端 NRF24L01_Data()）。
		 * 协议要求 Mode=1 发完后在几 ms 内等回传，故此处做短超时轮询。 */
		if (mode == 1U)
		{
			uint32_t wait = 0U;

			nrf_rx = 0U;
			do
			{
				nrf_rx = NRF24L01_Receive();
				if (nrf_rx == 1U)
				{
					break;
				}
				Delay_us(100U);
				wait++;
			} while (wait < 200U);  /* 200 * 100us ≈ 20ms：飞控是 100Hz 任务，
			                         * 收到 Mode=1 后最多延迟 10ms 才回，再花 ~4ms 发送，
			                         * 5ms 窗口永远收不到，导致飞控端遥测 MAX_RT 反复重初始化掉链 */
		}
		else
		{
			nrf_rx = NRF24L01_Receive();
		}

		if (nrf_rx == 1U)
		{
			if (NRF24L01_RxPacket[0] == 0x01U)
			{
				rx_speed = (uint16_t)(NRF24L01_RxPacket[1] | ((uint16_t)NRF24L01_RxPacket[2] << 8));
				rx_pitch = *(float *)&NRF24L01_RxPacket[4];
				rx_roll  = *(float *)&NRF24L01_RxPacket[8];
				rx_yaw   = *(float *)&NRF24L01_RxPacket[12];
				rx_alt   = *(float *)&NRF24L01_RxPacket[16];
				rx_pp    = *(float *)&NRF24L01_RxPacket[20];
				rx_pr    = *(float *)&NRF24L01_RxPacket[24];
				rx_pa    = *(float *)&NRF24L01_RxPacket[28];
				rx_cnt++;
			}
		}

		/* 周期重测 NRF（每 1s）：写 0xA1 回读，OLED 实时反映「上电时序 vs 接线」是否恢复 */
		if ((loop % (OLED_REFRESH_DIV * 10U)) == 0U)
		{
			NRF24L01_WriteReg(NRF24L01_SETUP_RETR, 0xA1U);
			nrf_retr = NRF24L01_ReadReg(NRF24L01_SETUP_RETR);
			NRF24L01_WriteReg(NRF24L01_SETUP_RETR, NRF24L01_SETUP_RETR_VALUE);

			/* SPI 回环自检：发 0xA5 读回（不经过 NRF 模块，验证软件写/读路径本身） */
			API_SPI_SelectBus(NRF24L01_SPI_BUS);
			API_SPI_Start();
			nrf_loop = API_SPI_SwapByte(0xA5U);
			API_SPI_Stop();
		}

		loop++;

		/* 周期刷新 OLED：本地状态 + 链路状态 + 飞控回传遥测 */
		if ((loop % OLED_REFRESH_DIV) == 0U)
		{
			/* 每次刷新先清显存，否则白屏自检留下的 0xFF 和上一帧旧数字会一直残留 */
			OLED_Clear();

			/* 行0：遥控器输入（Key + 油门） */
			OLED_ShowString(0, 0, "KEY:", OLED_6X8);
			OLED_ShowNum(24, 0, key,     1, OLED_6X8);
			OLED_ShowString(42, 0, "THR:", OLED_6X8);
			OLED_ShowNum(66, 0, throttle, 3, OLED_6X8);

			/* 行1：目标高度 */
			OLED_ShowString(0, 8, "ALT:", OLED_6X8);
			OLED_ShowNum(24, 8, alt,      3, OLED_6X8);
			OLED_ShowString(42, 8, "cm",  OLED_6X8);

			/* 行2：NRF 状态（回读 A1=OK FF/00=接线供电问题；发送 1=ACK 2=NOACK；回环 A5=OK） */
			OLED_Printf(0, 16, OLED_6X8, "NRF:%02X S%d LB:%02X",
			            (unsigned int)nrf_retr, nrf_send, (unsigned int)nrf_loop);

			/* 行3：回传心跳计数 + 飞控基础油门 */
			OLED_ShowString(0, 24, "RX:", OLED_6X8);
			OLED_ShowNum(18, 24, rx_cnt,   4, OLED_6X8);
			OLED_ShowString(54, 24, "SPD:", OLED_6X8);
			OLED_ShowNum(78, 24, rx_speed, 4, OLED_6X8);

			/* 行4：Pitch / Roll（DMP 姿态，联调时晃动飞控应变化） */
			OLED_ShowString(0, 32, "P:", OLED_6X8);
			OLED_ShowFloatNum(12, 32, rx_pitch, 3, 1, OLED_6X8);
			OLED_ShowString(54, 32, "R:", OLED_6X8);
			OLED_ShowFloatNum(66, 32, rx_roll, 3, 1, OLED_6X8);

			/* 行5：Yaw / 融合高度（若飞控端 IMU/气压任务未开，此两项恒 0） */
			OLED_ShowString(0, 40, "Y:", OLED_6X8);
			OLED_ShowFloatNum(12, 40, rx_yaw, 3, 1, OLED_6X8);
			OLED_ShowString(54, 40, "AL:", OLED_6X8);
			OLED_ShowFloatNum(72, 40, rx_alt, 1, 2, OLED_6X8);

			/* 行6：角速度环 PID 输出（pitch/roll） */
			OLED_ShowString(0, 48, "PP:", OLED_6X8);
			OLED_ShowFloatNum(18, 48, rx_pp, 2, 1, OLED_6X8);
			OLED_ShowString(54, 48, "PR:", OLED_6X8);
			OLED_ShowFloatNum(72, 48, rx_pr, 2, 1, OLED_6X8);

			/* 行7：高度环 PID 输出 */
			OLED_ShowString(0, 56, "PA:", OLED_6X8);
			OLED_ShowFloatNum(18, 56, rx_pa, 2, 1, OLED_6X8);

			OLED_Update();

			/* 10Hz 串口打印回传遥测（USART1 调试口，PC 端可看精确浮点值） */
			usart_printf(USART1, "RX:%u SPD:%u P:%.1f R:%.1f Y:%.1f A:%.2f PP:%.2f PR:%.2f PA:%.2f\r\n",
			             (unsigned int)rx_cnt, (unsigned int)rx_speed,
			             (double)rx_pitch, (double)rx_roll, (double)rx_yaw, (double)rx_alt,
			             (double)rx_pp, (double)rx_pr, (double)rx_pa);

			/* 心跳灯翻转（100ms 亮 / 100ms 灭） */
			LED_Control(LED1, ((loop / OLED_REFRESH_DIV) & 1U) ? LED_HIGH : LED_LOW);
		}

		Delay_ms(REMOTE_LOOP_DELAY_MS);
	}
}
