/**
 * StepMotor.c — Emm42_V5.0 闭环步进驱动 串口控制 (STM32F407 移植)
 *
 * 移植自 MSPM0G3507 已验证版本 (C:\OmniM0\BSP\StepMotor\StepMotor.c)
 *
 * 接线：MCU TX→驱动R/A/H, MCU RX→驱动T/B/L, GND↔GND
 * 驱动菜单：P_Serial=UART_FUN, P_Pul=PUL_OFF, 115200, 校验0x6B
 *
 * 关键：电机所在 USART 必须禁 NVIC + 关 RX 中断，纯轮询收发。
 */

#include "StepMotor.h"
#include "usart.h"    /* F407_USART_View_t, USART1/2 */
#include "Delay.h"    /* Delay_us, Delay_ms */
#include "Enroll.h"   /* ENROLL_MCU_TARGET */

/* ═══════════════════════════════════════════════════════════════
 * NVIC 寄存器
 * ═══════════════════════════════════════════════════════════════ */
#define NVIC_ICER_BASE  0xE000E180UL

/* ═══════════════════════════════════════════════════════════════
 * 电机 → USART 映射
 * ═══════════════════════════════════════════════════════════════ */
typedef struct
{
	uint8_t             id;       /* 逻辑电机 ID (MOTOR_ID_1 / MOTOR_ID_2)  */
	F407_USART_View_t  *usart;    /* 所在 USART 端口                      */
	uint32_t            irqNum;   /* NVIC 中断号                          */
	uint8_t             addr;     /* 协议地址（独立总线可相同，默认 1）    */
} MotorMap_t;

static MotorMap_t s_map[STEPMOTOR_MAX] =
{
	{ STEPMOTOR1, USART1, 37U, 1U },  /* 电机1: USART1, 协议地址=1 */
	{ STEPMOTOR2, USART2, 38U, 1U },  /* 电机2: USART2, 协议地址=1 */
};

static MotorMap_t *motor_get(uint8_t id)
{
	uint8_t i;
	for (i = 0U; i < STEPMOTOR_MAX; i++)
	{
		if (s_map[i].id == id) return &s_map[i];
	}
	return 0;
}

/* ═══════════════════════════════════════════════════════════════
 * NVIC 操作
 * ═══════════════════════════════════════════════════════════════ */
static void nvic_disable(uint32_t irq)
{
	volatile uint32_t *icer;
	icer  = (volatile uint32_t *)(NVIC_ICER_BASE + (irq / 32U) * 4U);
	*icer = (1UL << (irq % 32U));
}

/* ═══════════════════════════════════════════════════════════════
 * USART 寄存器级收发（纯轮询，无中断）
 * ═══════════════════════════════════════════════════════════════ */

/* 阻塞发送 1 字节 */
static void uart_tx(F407_USART_View_t *u, uint8_t b)
{
	while ((u->SR & USART_SR_TXE) == 0U) {}
	u->DR = b;
	while ((u->SR & USART_SR_TC) == 0U) {}
}

/* 发送字节数组 */
static void send_buf(F407_USART_View_t *u, const uint8_t *buf, uint8_t len)
{
	uint8_t i;
	for (i = 0U; i < len; i++) uart_tx(u, buf[i]);
}

/* 带超时的接收字节数组。返回 MOTOR_OK 或 MOTOR_ERR_TIMEOUT。 */
static MotorErrCode recv_buf(F407_USART_View_t *u, uint8_t *buf, uint8_t len,
                             uint32_t ms)
{
	uint8_t  i = 0U;
	uint32_t deadline = ms * 1000U;  /* us */
	uint32_t elapsed  = 0U;

	while (i < len)
	{
		if ((u->SR & USART_SR_RXNE) != 0U)
		{
			buf[i++] = (uint8_t)u->DR;
			elapsed = 0U;
		}
		else
		{
			Delay_us(1U);
			elapsed++;
			if (elapsed >= deadline) return MOTOR_ERR_TIMEOUT;
		}
	}

	/* 清残留（校验字节 0x6B 等），短暂等待后读走 */
	{
		uint32_t w;
		for (w = 0U; w < 500U; w++) { Delay_us(1U); }
		while ((u->SR & USART_SR_RXNE) != 0U) { (void)u->DR; }
	}

	return MOTOR_OK;
}

/* ═══════════════════════════════════════════════════════════════
 * 协议帧收发（与 M0 版本一致）
 *
 * 帧格式: [id][fc][d0..dN][0x6B]
 * ═══════════════════════════════════════════════════════════════ */
static MotorErrCode cmd(MotorMap_t *m, uint8_t fc,
                        const uint8_t *d, uint8_t dl,
                        uint8_t *rx, uint8_t rl, uint32_t ms)
{
	uint8_t tx[32];
	uint8_t i;

	tx[0] = m->addr;   /* 协议地址，独立于逻辑 ID */
	tx[1] = fc;
	for (i = 0U; i < dl; i++) tx[2U + i] = d[i];
	tx[2U + dl] = 0x6B;

	send_buf(m->usart, tx, (uint8_t)(3U + dl));

	if (rx == 0 || rl == 0U) return MOTOR_OK;
	return recv_buf(m->usart, rx, rl, ms);
}

/* 校验应答：rx[0]==addr && rx[1]==fc */
static uint8_t chk(MotorMap_t *m, const uint8_t *rx, uint8_t fc)
{
	if (rx == 0 || m == 0) return 0U;
	return (rx[0] == m->addr && rx[1] == fc) ? 1U : 0U;
}

/* ═══════════════════════════════════════════════════════════════
 * 公共接口
 * ═══════════════════════════════════════════════════════════════ */

/*
 * 模块初始化：
 * 1) 关闭 RXNEIE（CR1 bit5），防止 USART ISR 抢 RX 字节
 * 2) 关闭 NVIC IRQ，防止 printf 异步 TX 与轮询发送冲突
 * 3) 清空 RX 残留
 *
 * 注意：这会彻底禁用电机 USART 的所有中断。
 * 如果 printf 也用该 USART，printf 将走阻塞发送（asyncReady 虽为 1，
 * 但 TXE IE 被 NVIC 屏蔽不会触发，实际上 TXEIE 也不会被置位因为
 * usart_send_byte_async 在队列满时会 fallback 到阻塞发送）。
 */
void Stepmotor_Init(void)
{
	uint8_t i;

#if (ENROLL_MCU_TARGET != ENROLL_MCU_F407)
	return;
#endif

	for (i = 0U; i < STEPMOTOR_MAX; i++)
	{
		F407_USART_View_t *u = s_map[i].usart;

		if (u == 0) continue;

		/* 关 RXNEIE */
		u->CR1 &= ~(1UL << 5);

		/* 关 NVIC */
		nvic_disable(s_map[i].irqNum);

		/* 清 RX 残留 */
		while ((u->SR & USART_SR_RXNE) != 0U) { (void)u->DR; }
	}
}

/* ── 基础控制 ── */

MotorErrCode Stepmotor_Enable(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0xAB, 0x01, 0x00};
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;

	e = cmd(m, 0xF3, d, 3U, rx, 4U, 500U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	if (!chk(m, rx,0xF3)) return MOTOR_ERR_RESPONSE;
	return (rx[2] == 0x02) ? MOTOR_OK : MOTOR_ERR_STALL;
}

MotorErrCode Stepmotor_Disable(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0xAB, 0x00, 0x00};

	if (m == 0) return MOTOR_ERR_INVALID_ID;
	(void)cmd(m, 0xF3, d, 3U, rx, 4U, 500U);
	return MOTOR_OK;
}

MotorErrCode Stepmotor_Stop(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0x98, 0x00};
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;
	e = cmd(m, 0xFE, d, 2U, rx, 4U, 500U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0xFE) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

/* ── 校准与回零 ── */

MotorErrCode Stepmotor_Calibrate(uint8_t id, uint32_t timeoutMs)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0x45};
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;
	e = cmd(m, 0x06, d, 1U, rx, 4U, timeoutMs);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0x06) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

MotorErrCode Stepmotor_SetOrigin(uint8_t id, uint8_t saveToFlash)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0x88, (saveToFlash != 0U) ? 0x01U : 0x00U};
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;
	e = cmd(m, 0x93, d, 2U, rx, 4U, 500U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0x93) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

MotorErrCode Stepmotor_GoHome(uint8_t id, uint32_t timeoutMs)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {MOTOR_HOME_NEAREST, 0x00};
	MotorErrCode e;
	uint32_t deadline;

	if (m == 0) return MOTOR_ERR_INVALID_ID;

	e = cmd(m, 0x9A, d, 2U, rx, 4U, 1000U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	if (!chk(m, rx,0x9A)) return MOTOR_ERR_RESPONSE;
	if (rx[2] == 0xE2) return MOTOR_ERR_HOME_FAIL;

	/* 轮询等待回零完成 */
	deadline = timeoutMs / 50U;
	while (deadline > 0U)
	{
		uint8_t st = Stepmotor_ReadStatus(id);
		if ((st & MOTOR_STAT_IN_POS) && !(st & MOTOR_STAT_STALL_NOW))
			return MOTOR_OK;
		Delay_ms(50U);
		deadline--;
	}

	return MOTOR_ERR_TIMEOUT;
}

/* ── 梯形位置模式（0xFD）── */

MotorErrCode Stepmotor_MoveTo(uint8_t id, float angle, uint16_t rpm,
                              uint8_t accel, uint8_t mode)
{
	MotorMap_t *m = motor_get(id);
	int32_t  pulses;
	uint16_t spd;
	uint8_t  d[10], rx[4];
	MotorErrCode e;

	if (m == 0)         return MOTOR_ERR_INVALID_ID;
	if (rpm > 5000U)    return MOTOR_ERR_INVALID_ID;
	if (rpm == 0U)      rpm = 1U;

	/* 角度 → 脉冲：1.8°电机, 16细分 → 3200 脉冲/圈 */
	pulses = (int32_t)(angle * 3200.0f / 360.0f);
	spd    = rpm * 10U;

	/* 方向：正=CW(0x00)，负=CCW(0x01) */
	d[0] = (pulses < 0) ? 0x01U : 0x00U;

	/* 脉冲取绝对值 */
	if (pulses < 0) pulses = -pulses;

	d[1] = (uint8_t)(spd >> 8);
	d[2] = (uint8_t)(spd & 0xFFU);
	d[3] = accel;
	d[4] = (uint8_t)(((uint32_t)pulses >> 24) & 0xFFU);
	d[5] = (uint8_t)(((uint32_t)pulses >> 16) & 0xFFU);
	d[6] = (uint8_t)(((uint32_t)pulses >> 8) & 0xFFU);
	d[7] = (uint8_t)((uint32_t)pulses & 0xFFU);
	d[8] = mode;
	d[9] = MOTOR_SYNC_IMMEDIATE;

	e = cmd(m, 0xFD, d, 10U, rx, 4U, 500U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0xFD) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

/* ── 阻塞等到位 ── */

MotorErrCode Stepmotor_GoTo(uint8_t id, float angle, uint16_t rpm,
                            uint8_t accel, uint8_t mode, uint32_t timeoutMs)
{
	MotorErrCode e;
	uint32_t t = 0U, d;

	e = Stepmotor_MoveTo(id, angle, rpm, accel, mode);
	if (e != MOTOR_OK) return e;

	d = timeoutMs / 10U;
	if (d == 0U) d = 1U;

	while (t < d)
	{
		if (Stepmotor_ReadStatus(id) & MOTOR_STAT_IN_POS)
			return MOTOR_OK;
		Delay_ms(10U);
		t++;
	}

	return MOTOR_ERR_TIMEOUT;
}

/* ── 反馈 ── */

float Stepmotor_ReadAngle(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[8];
	int32_t raw;
	MotorErrCode e;

	if (m == 0) return 0.0f;

	/* 应答 8 字节：addr func sign angle[4] checksum，只读前 7 字节 */
	e = cmd(m, 0x36, 0, 0U, rx, 7U, 200U);
	if (e != MOTOR_OK) return 0.0f;
	if (!chk(m, rx,0x36)) return 0.0f;

	raw = ((int32_t)rx[3] << 24) | ((int32_t)rx[4] << 16) |
	      ((int32_t)rx[5] << 8)  |  (int32_t)rx[6];
	if (rx[2] != 0U) raw = -raw;  /* sign byte */
	return (float)raw / 10.0f;
}

uint8_t Stepmotor_ReadStatus(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4];
	MotorErrCode e;

	if (m == 0) return 0U;

	e = cmd(m, 0x3A, 0, 0U, rx, 4U, 200U);
	if (e != MOTOR_OK) return 0U;
	if (!chk(m, rx,0x3A)) return 0U;
	return rx[2];
}

uint8_t Stepmotor_ReadHomeState(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4];
	MotorErrCode e;

	if (m == 0) return 0U;

	e = cmd(m, MOTOR_FUNC_READ_HOME_STATE, 0, 0U, rx, 4U, 200U);
	if (e != MOTOR_OK) return 0U;
	if (!chk(m, rx,MOTOR_FUNC_READ_HOME_STATE)) return 0U;
	return rx[2];
}

MotorErrCode Stepmotor_ResetStall(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4], d[] = {0x52};
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;

	e = cmd(m, 0x0E, d, 1U, rx, 4U, 500U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0x0E) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

/* ── 自检 ── */

MotorErrCode Stepmotor_SelfTest(uint8_t id)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[4];
	MotorErrCode e;

	if (m == 0) return MOTOR_ERR_INVALID_ID;

	e = cmd(m, 0x3A, 0, 0U, rx, 4U, 200U);
	if (e != MOTOR_OK) return MOTOR_ERR_TIMEOUT;
	return chk(m, rx,0x3A) ? MOTOR_OK : MOTOR_ERR_RESPONSE;
}

/* ── 裸收发诊断（上层调试用）── */

/*
 * 发一帧命令，返回实际收到的字节数及原始数据，不校验。
 * 用于排查物理层通断：发送 [id][fc][data][6B]，看能收回多少字节。
 */
uint8_t Stepmotor_RawCmd(uint8_t id, uint8_t fc,
                         const uint8_t *data, uint8_t dataLen,
                         uint8_t *rxBuf, uint8_t rxBufSize, uint32_t timeoutMs)
{
	MotorMap_t *m = motor_get(id);
	uint8_t rx[32];
	uint8_t i;
	MotorErrCode e;

	if (m == 0 || rxBuf == 0 || rxBufSize == 0U) return 0U;

	/* 用内部 cmd 发，但期望长度设大一些来捕获原始数据 */
	e = cmd(m, fc, data, dataLen, rx, (uint8_t)(rxBufSize < 31U ? rxBufSize : 31U), timeoutMs);

	/* 不管 e 是否超时，把收到的原始字节拷出去 */
	for (i = 0U; i < rxBufSize; i++)
	{
		rxBuf[i] = rx[i];
	}

	/* 返回 cmd 实际收到的字节数（超时时为部分字节数，全部收到时为 rxBufSize） */
	if (e == MOTOR_OK) return rxBufSize;

	/* 超时时返回 0 或部分字节 */
	return 0U;
}
