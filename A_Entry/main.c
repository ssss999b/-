/*
* OmniFlight - 四轴飞控
* 初始化相关外设并校准各个传感器：
* 陀螺仪、磁力计、气压计各五秒，共计飞控需静止10S左右-蓝灯亮

* 校准完毕蓝灯灭
* 全部外设初始化完-蜂鸣器鸣笛蓝灯灭

* 锁定油门-红灯亮
*
* 串口分配：
*   USART1 (PA9/PA10) — 电机1 (MOTOR_ID_1)
*   USART2 (PD5/PD6)  — 电机2 (MOTOR_ID_2)
*   USART3 (PD8/PD9)  — 调试打印 (printf 输出)
*/

/* Enroll 注册层，负责把板级资源注册到 BSP */
#include "Enroll.h"

/*系统sys层*/
#include "sys.h"
#include "Delay.h"

/*API层 MCU片内外设*/
#include "usart.h"
#include "tim.h"
#include "pwm.h"

/*
 * 重定义 printf 输出到 USART3，避免与电机 USART1/2 冲突。
 * 必须在 My_Usart.h 首次被 include 之前定义。
 */
#define PRINTF_USART USART3

/*app应用层*/
#include "My_Usart/My_Usart.h"
#include "API_I2C.h"
#include "API_SPI.h"
#include "PID/PID.h"
#include "Control/Control.h"
#include "Control_Task/Control_Task.h"
#include "AimControl.h"
/*BSP硬件抽象层*/
#include "LED.h"
#include "KEY.h"
#include "MPU6050.h"
#include "MPU6050_Int.h"
#include "Motor.h"
#include "QMC5883P.h"
#include "BMP280.h"
#include "Dshot.h" /* DShot协议 初始化 */
#include "NRF24L01.h"
#include "Buzzer.h"
#include "IMU.h"
#include "Altitude.h"
#include "Stepmotor.h" /* Emm42 闭环步进电机 */

int main(void)
{
/* 系统时钟配置初始化 */
	SYS_Init();
/* 注册层：注册相关资源，登记资源映射 */
	Enroll_LED_Register();					/* LED 资源注册 */
	Enroll_KEY_Register();					/* KEY 资源注册 */
	Enroll_USART_Register();				/* USART 资源注册 */
	Enroll_PWM_Register();					/* PWM 资源注册（Buzzer: TIM3 CH4） */
	Enroll_TIM_Register();					/* TIM 资源注册 */
	Enroll_I2C_Register();					/* I2C 资源注册 */
	Enroll_SPI_Register();					/* SPI 资源注册 */
	Enroll_NRF24L01_Register();				/* NRF24L01 CE 引脚注册 */

	/* 注册后绑定中断回调*/
	Enroll_USART_RegisterIrqHandler(Control_Task_USART_Callback); 		/* USART 中断回调：注册时自动使能异步 TX */
	API_TIM_RegisterIrqHandler(API_TIM1, Control_Task1_Callback);           	/* TIM1: PID */
	API_TIM_RegisterIrqHandler(API_TIM2, Control_Task2_Callback);               /* TIM2: printf/时间戳 */

/* 初始化层：初始化相关外设，启动硬件功能 */
	API_USART_Init(API_USART1, 115200U); // 初始化 USART1 (电机1)
	API_USART_Init(API_USART2, 115200U); // 初始化 USART2 (电机2)
	API_USART_Init(API_USART3, 115200U); // 初始化 USART3 (调试打印)
	API_USART_Init(API_USART4, 115200U); // 初始化 UART4 (PC10/PC11)
	Stepmotor_Init();                // 关电机 USART 的 RXNEIE + NVIC IRQ

	API_TIM_Init(API_TIM1, 1U); /* TIM1: PID 节拍，每 1ms */
	API_TIM_Init(API_TIM2, 1U); /* TIM2: printf/时间戳  每 1ms */
	API_PWM_Init(API_PWM_TIM3, (1000000U / 2700U) - 1, 84U - 1U);

/* 通信协议初始化 */
	API_I2C_Init();						/* 软件 I2C 初始化 */
	API_SPI_Init();						/* 软件 SPI 初始化 */
	// App_I2C_ScanOnce();				/* 开机执行一次 I2C 扫描 */
	// App_SPI_TestOnce();				/* 开机执行一次 SPI 测试 */

/*BSP硬件抽象层初始化*/
	LED_Init(LED_LOW);	/* LED 初始化-低电平 */
	KEY_Init();	/* KEY 初始化 */
    AimControl_Init();
	/* ═══════════════════════════════════════════════════════════
	 * 闭环步进电机 — 上电自检 + 运动测试
	 *
	 * STEPMOTOR1 → USART1 (PA9/PA10)
	 * STEPMOTOR2 → USART2 (PD5/PD6)
	 * 调试输出  → USART3 (PD8/PD9), 串口助手 115200bps
	 *
	 * 直接改下面函数里的角度和速度即可，后续可替换为变量。
	 *
	 * 首次校准：取消 Calibrate/SetOrigin/GoHome 注释，空载跑一次后注回。
	 * ═══════════════════════════════════════════════════════════ */
	// {
	// 	MotorErrCode e;

	// 	usart_printf(USART3, "[SM] Init\r\n");

	// 	/* ── STEPMOTOR1 (USART1) ── */
	// 	e = Stepmotor_SelfTest(STEPMOTOR1);
	// 	usart_printf(USART3, "[SM] Motor1 scan: %s\r\n",
	// 	             (e == MOTOR_OK) ? "FOUND" : "none");

	// 	if (e == MOTOR_OK)
	// 	{
	// 		e = Stepmotor_Enable(STEPMOTOR1);
	// 		usart_printf(USART3, "[SM] Motor1 En=%d St=0x%02X\r\n",
	// 		             (int)e, Stepmotor_ReadStatus(STEPMOTOR1));

	// 		/* 首次校准（空载！） */
	// 		// e = Stepmotor_Calibrate(STEPMOTOR1, 30000U);
	// 		// usart_printf(USART3, "[SM] Motor1 Cal=%d\r\n", (int)e);
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_SetOrigin(STEPMOTOR1, 1U);
	// 		//     usart_printf(USART3, "[SM] Motor1 Org=%d\r\n", (int)e);
	// 		// }
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_GoHome(STEPMOTOR1, 60000U);
	// 		//     usart_printf(USART3, "[SM] Motor1 Home=%d\r\n", (int)e);
	// 		// }

	// 		/* 运动：+90° @30RPM，直启，相对模式 */
	// 		e = Stepmotor_GoTo(STEPMOTOR1, 360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor1 +90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR1));
	// 		Delay_ms(500U);

	// 		e = Stepmotor_GoTo(STEPMOTOR1, -360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor1 -90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR1));
	// 	}

	// 	/* ── STEPMOTOR2 (USART2) ── */
	// 	e = Stepmotor_SelfTest(STEPMOTOR2);
	// 	usart_printf(USART3, "[SM] Motor2 scan: %s\r\n",
	// 	             (e == MOTOR_OK) ? "FOUND" : "none");

	// 	if (e == MOTOR_OK)
	// 	{
	// 		e = Stepmotor_Enable(STEPMOTOR2);
	// 		usart_printf(USART3, "[SM] Motor2 En=%d St=0x%02X\r\n",
	// 		             (int)e, Stepmotor_ReadStatus(STEPMOTOR2));

	// 		/* 首次校准（空载！） */
	// 		// e = Stepmotor_Calibrate(STEPMOTOR2, 30000U);
	// 		// usart_printf(USART3, "[SM] Motor2 Cal=%d\r\n", (int)e);
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_SetOrigin(STEPMOTOR2, 1U);
	// 		//     usart_printf(USART3, "[SM] Motor2 Org=%d\r\n", (int)e);
	// 		// }
	// 		// if (e == MOTOR_OK) {
	// 		//     e = Stepmotor_GoHome(STEPMOTOR2, 60000U);
	// 		//     usart_printf(USART3, "[SM] Motor2 Home=%d\r\n", (int)e);
	// 		// }

	// 		/* 运动：+90° @30RPM，直启，相对模式 */
	// 		e = Stepmotor_GoTo(STEPMOTOR2, 360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor2 +90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR2));
	// 		Delay_ms(500U);

	// 		e = Stepmotor_GoTo(STEPMOTOR2, -360.0f, 3U, 0U,
	// 		                   MOTOR_MODE_REL, 10000U);
	// 		usart_printf(USART3, "[SM] Motor2 -90 =%d ang=%.1f\r\n",
	// 		             (int)e, (double)Stepmotor_ReadAngle(STEPMOTOR2));
	// 	}
	// }

	while (1)
	{
		//LED_Control(LED2, LED_LOW);
         key_Get(); /* 按键扫描 */
		 usart_printf(USART4, "KEY=%d\r\n", Key);
        /*
         * K230 数据包格式：
         *   s target_x,target_y,ref_x,ref_y,ex,ey,status e
         * 示例：
         *   s358,259,284,117,74,142,1e
         *
         * 你的解析器收到完整一帧后：USART_DataTypeStruct.state == 2U
         */
        if (USART_DataTypeStruct.state == 2U)
        {
            uint8_t i;

            USART_Packet_Count = USART_DataTypeStruct.count;

            for (i = 0U; i < USART_Packet_Count; i++)
            {
                USART_Packet_Data[i] = USART_Deal(&USART_DataTypeStruct, (int8_t)i);
            }

            USART_DataTypeStruct.state = 0U;

            if (USART_Packet_Count == 7U)
            {
                int16_t target_x = USART_Packet_Data[0];
                int16_t target_y = USART_Packet_Data[1];
                int16_t ref_x    = USART_Packet_Data[2];
                int16_t ref_y    = USART_Packet_Data[3];
                int16_t ex       = USART_Packet_Data[4];
                int16_t ey       = USART_Packet_Data[5];
                int16_t status   = USART_Packet_Data[6];

                /*
                 * 只调用这一句即可。
                 * AimControl 内部会根据 ex/ey/status 做 PID 和步进电机控制。
                 */
                AimControl_Update(target_x, target_y, ref_x, ref_y, ex, ey, status);
            }
            else
            {
                /*
                 * 数据数量不对时，不控制电机。
                 * 如果需要调试，可以在 AimControl.c 里打开调试，或这里用 API_USART4 打印。
                 */
            }
        }
	}
}
