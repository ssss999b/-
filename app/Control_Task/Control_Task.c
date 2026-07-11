#include "Control_Task.h"

#include "tim.h"
#include "usart.h"
#include "My_Usart/My_Usart.h"
#include "Control/Control.h"
#include "Dshot.h"
#include "Motor.h"
#include "MPU6050.h"
#include "IMU.h"
#include "KEY.h"

/* =========================================================================
 * 任务标志位
 *
 * 全部由定时器 ISR 置位（1），主循环消费后清零（0）。
 * 统一模式：ISR 管时间 → 置标志 → 主循环管执行
 * ========================================================================= */

/* 程序运行的毫秒时间戳。
 *
 * TIM2 每 1ms 进入一次 Control_Task2_Callback()，这里每次 +1。
 * 后面任务模式状态机需要用它做：
 *   1. 几秒没按键后自动 READY；
 *   2. TASK3 4 秒超时；
 *   3. STM32 -> K230 UI 状态包重复发送间隔。
 */
volatile uint32_t Timer_Ms_t = 0U;

/* 程序运行的时间戳（s），TIM2 每 1s 递增。
 * 保留原变量，兼容你工程里其它可能使用 Timer_Bsp_t 的地方。
 */
volatile uint32_t Timer_Bsp_t = 0U;

/* 串口数据包解析结果缓存（新包到达时自动刷新） */
int16_t USART_Packet_Data[USART_PACKET_DATA_LEN] = {0};
uint8_t USART_Packet_Count = 0;

/* TIM2 分发：慢速外设任务 */
volatile uint8_t print_task_flag = 0U;   /* 10Hz  串口打印 */

/* =========================================================================
 * Control_Task_GetMillis
 *
 * 获取当前毫秒时间戳。
 * STM32F407 是 32 位 MCU，对齐的 uint32_t 读写是原子的；
 * 这里直接读取 volatile 变量即可。
 * ========================================================================= */
uint32_t Control_Task_GetMillis(void)
{
	return (uint32_t)Timer_Ms_t;
}

/* =========================================================================
 * Control_Task1_Callback — TIM1 (TIM3) 1ms ISR
 *
 * 职责：飞控心脏节拍
 *   - PID 控制 + 电机输出：500Hz（每 2ms）
 *
 * 当前瞄准项目暂时不在这里添加内容，避免影响其它控制任务。
 * ========================================================================= */
void Control_Task1_Callback(API_TIM_Id_t id)
{
	if (id != API_TIM1)
	{
		return;
	}
}

/* =========================================================================
 * Control_Task2_Callback — TIM2 1ms ISR
 *
 * 职责：所有非飞控周期任务的统一调度中心
 *
 *   每 1ms  → Timer_Ms_t++      (毫秒时间戳)
 *   每 1ms  → Key_Tick()        (按键扫描入口，内部再做 20ms 采样)
 *   1000ms → Timer_Bsp_t++     (1Hz 秒级时间戳)
 *    100ms → print_task_flag   (10Hz 串口打印)
 *
 * 所有计数器和标志位集中在此，改频率只需改这若干个数字。
 * ========================================================================= */
void Control_Task2_Callback(API_TIM_Id_t id)
{
	static uint8_t printf_tick = 0U;
	static uint16_t sec_tick = 0U;

	if (id != API_TIM2)
	{
		return;
	}

	/* ---- 1ms 毫秒时间戳 ---- */
	Timer_Ms_t++;

	/* ---- 按键扫描：Key_Tick 内部会按 20ms 采样并消抖 ---- */
	Key_Tick();

	/* ---- printf: 10Hz (每 100ms) ---- */
	printf_tick++;
	if (printf_tick >= 100U)
	{
		printf_tick = 0U;
		print_task_flag = 1U;
	}

	/* ---- 秒级时间戳: 1Hz (每 1000ms) ---- */
	sec_tick++;
	if (sec_tick >= 1000U)
	{
		sec_tick = 0U;
		Timer_Bsp_t++;
	}
}

/* =========================================================================
 * Control_Task_USART_Callback — USART 中断回调
 *
 * 由 USART ISR 触发，调度 TX 队列排空（printf 异步发送的核心）。
 * 传 NULL 跳过 RX 接收处理，仅做 TX 排空。
 * 此回调必须注册（Enroll_USART_RegisterIrqHandler），否则 TXE 中断会无限循环。
 * ========================================================================= */
void Control_Task_USART_Callback(API_USART_Id_t id)
{
	uint32_t data;
	uint8_t rxValid;

	data = 0U;
	rxValid = 0U;

	usart_irq_dispatch_by_id(id, &data, &rxValid);

	if (rxValid != 0U)
	{
		if (id == API_USART3)
		{
			usart_Dispose_Data(USART3, &USART_DataTypeStruct, (uint8_t)data);
		}
	}
}
