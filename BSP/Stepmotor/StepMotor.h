#ifndef __STEPMOTOR_H
#define __STEPMOTOR_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*
 * Stepmotor — Emm42_V5.0 闭环步进电机驱动 (BSP 层)
 *
 * 硬件：Emm42_V5.0 闭环步进驱动板（张大头电子）
 * 协议：地址 + 功能码 + 数据 + 校验(0x6B)，115200 8N1
 * 电机：1.8° 步进角，16 细分 → 3200 脉冲/圈，1 脉冲 = 0.1125°
 *
 * 接线：
 *   MCU TX → 驱动板 R/A/H (RX)
 *   MCU RX → 驱动板 T/B/L (TX)
 *   GND    → 驱动板 GND
 *
 * USART 映射：
 *   STEPMOTOR1 → USART1 (PA9/PA10), 协议地址=1
 *   STEPMOTOR2 → USART2 (PD5/PD6),  协议地址=1
 *
 * 注意：
 *   - Stepmotor_Init() 必须在 API_USART_Init() 之后调用
 *   - 电机所在 USART 将关闭 RXNEIE + NVIC 中断，改用轮询通信
 *   - 校准(0x06)必须空载执行，否则编码器零点错误
 */

/* =========================================================================
 * 错误码
 * ========================================================================= */
typedef enum
{
	MOTOR_OK            = 0U,
	MOTOR_ERR_TIMEOUT   = 1U,
	MOTOR_ERR_STALL     = 2U,
	MOTOR_ERR_HOME_FAIL = 3U,
	MOTOR_ERR_INVALID_ID = 4U,
	MOTOR_ERR_RESPONSE  = 5U,
} MotorErrCode;

/* =========================================================================
 * 电机 ID
 * ========================================================================= */
#define STEPMOTOR1  1U  /* USART1 (PA9/PA10), 地址=1 */
#define STEPMOTOR2  2U  /* USART2 (PD5/PD6),  地址=1 */
#define STEPMOTOR_MAX 2U

/* =========================================================================
 * 方向
 * ========================================================================= */
#define MOTOR_DIR_CW   0x00U
#define MOTOR_DIR_CCW  0x01U

/* =========================================================================
 * 位置模式
 * ========================================================================= */
#define MOTOR_MODE_REL       0x00U  /* 相对当前位置               */
#define MOTOR_MODE_ABS       0x01U  /* 绝对位置（依赖已校准零点）               */
#define MOTOR_MODE_REL_LAST  0x02U  /* 相对上次目标位置           */

/* =========================================================================
 * 回零模式
 * ========================================================================= */
#define MOTOR_HOME_NEAREST   0x00U  /* 单圈就近回零               */

/* =========================================================================
 * 同步标志
 * ========================================================================= */
#define MOTOR_SYNC_IMMEDIATE 0x00U  /* 立即执行，不等同步信号     */

/* =========================================================================
 * 加速度档位（0=直启，1~254 越大越缓）
 * ========================================================================= */
#define MOTOR_ACCEL_DIRECT   0x00U
#define MOTOR_ACCEL_DEFAULT  0x00U

/* =========================================================================
 * 状态位（0x3A 应答字节 bitmask）
 * ========================================================================= */
#define MOTOR_STAT_ENABLED    0x01U  /* bit0: 使能状态             */
#define MOTOR_STAT_IN_POS     0x02U  /* bit1: 到位标志             */
#define MOTOR_STAT_STALL_NOW  0x04U  /* bit2: 实时堵转             */
#define MOTOR_STAT_STALL_LOCK 0x08U  /* bit3: 堵转保护锁死         */

/* =========================================================================
 * 回零状态位（0x3B 应答字节 bitmask）
 * ========================================================================= */
#define MOTOR_HOME_BUSY       0x04U  /* bit2: 正在回零             */
#define MOTOR_HOME_FAILED     0x08U  /* bit3: 回零失败             */

/* =========================================================================
 * 协议常量
 * ========================================================================= */
#define MOTOR_CHECKSUM        0x6BU
#define MOTOR_PULSES_PER_REV  3200U
#define MOTOR_DEG_PER_PULSE   0.1125f
#define MOTOR_MAX_RPM         5000U
#define MOTOR_DEFAULT_TIMEOUT 5000U

/* =========================================================================
 * 功能码（内部使用，上层无需关心）
 * ========================================================================= */
#define MOTOR_FUNC_ENABLE        0xF3U
#define MOTOR_FUNC_POS_MODE      0xFDU
#define MOTOR_FUNC_STOP          0xFEU
#define MOTOR_FUNC_READ_ANGLE    0x36U
#define MOTOR_FUNC_READ_STATUS   0x3AU
#define MOTOR_FUNC_CALIBRATE     0x06U
#define MOTOR_FUNC_SET_ORIGIN    0x93U
#define MOTOR_FUNC_GO_HOME       0x9AU
#define MOTOR_FUNC_READ_HOME_STATE 0x3BU
#define MOTOR_FUNC_CLEAR_STALL   0x0EU

/* =========================================================================
 * 公共接口
 * ========================================================================= */

/*
 * 模块初始化：
 * 关 RXNEIE + 关 NVIC IRQ + 清残留数据，使电机 USART 进入轮询模式。
 * 必须在 API_USART_Init() 之后调用。
 */
void Stepmotor_Init(void);

/* ---- 基本控制 ---- */

/* 使能电机（上电）。 */
MotorErrCode Stepmotor_Enable(uint8_t id);

/* 失能电机（去使能，可自由转动）。 */
MotorErrCode Stepmotor_Disable(uint8_t id);

/* 急停（立即停止运动，保持使能）。 */
MotorErrCode Stepmotor_Stop(uint8_t id);

/* ---- 校准与回零 ---- */

/*
 * 编码器校准（必须空载！）。
 * timeoutMs: 最大等待时间（毫秒），校准通常需 2~5 秒。
 */
MotorErrCode Stepmotor_Calibrate(uint8_t id, uint32_t timeoutMs);

/*
 * 设当前位置为原点。
 * saveToFlash: 0=仅 RAM，1=存入 Flash（掉电保持）。
 */
MotorErrCode Stepmotor_SetOrigin(uint8_t id, uint8_t saveToFlash);

/*
 * 回零（阻塞，等待到位或超时）。
 * 模式：单圈就近回零。
 */
MotorErrCode Stepmotor_GoHome(uint8_t id, uint32_t timeoutMs);

/* ---- 位置控制（非阻塞：发完指令立即返回）---- */

/*
 * 梯形位置模式运动，立即返回。
 *
 * 参数：
 *   id     : 电机 ID
 *   angle  : 目标角度（°），正=CW，负=CCW
 *   rpm    : 最大转速（RPM），范围 1~5000
 *   accel  : 加速度档位（0=直启，越大加速越缓）
 *   mode   : MOTOR_MODE_ABS / MOTOR_MODE_REL / MOTOR_MODE_REL_LAST
 */
MotorErrCode Stepmotor_MoveTo(uint8_t id, float angle, uint16_t rpm,
                              uint8_t accel, uint8_t mode);

/* ---- 位置控制（阻塞：等待到位或超时）---- */

/*
 * 梯形位置模式运动，轮询等待到位后返回。
 *
 * 参数：同 MoveTo，额外增加 timeoutMs。
 * 内部每 10ms 读一次 0x3A 状态，检查 bit1 到位标志。
 */
MotorErrCode Stepmotor_GoTo(uint8_t id, float angle, uint16_t rpm,
                            uint8_t accel, uint8_t mode, uint32_t timeoutMs);

/* ---- 状态查询 ---- */

/* 读实时角度（°），正值=CW 偏转。 */
float Stepmotor_ReadAngle(uint8_t id);

/* 读状态字（bit0=使能, bit1=到位, bit2=实时堵转, bit3=堵转锁死）。 */
uint8_t Stepmotor_ReadStatus(uint8_t id);

/* 读回零状态字（bit2=正在回零, bit3=回零失败）。 */
uint8_t Stepmotor_ReadHomeState(uint8_t id);

/* 清除堵转保护锁死。 */
MotorErrCode Stepmotor_ResetStall(uint8_t id);

/* ---- 自检 ---- */

/*
 * 上电自检：发读状态指令验证 USART → 驱动板链路。
 * 连好驱动板上电后调用，返回 MOTOR_OK 表示通信正常。
 */
MotorErrCode Stepmotor_SelfTest(uint8_t id);

/*
 * 裸收发诊断：发命令 + 收原始字节，不校验。
 * 返回实际收到的字节数。用于排查物理层：看发了 [id][fc][data][6B] 后能收回多少。
 */
uint8_t Stepmotor_RawCmd(uint8_t id, uint8_t fc,
                         const uint8_t *data, uint8_t dataLen,
                         uint8_t *rxBuf, uint8_t rxBufSize, uint32_t timeoutMs);

#ifdef __cplusplus
}
#endif

#endif /* __STEPMOTOR_H */
