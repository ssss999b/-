#ifndef __CONTROL_TASK_H
#define __CONTROL_TASK_H

#include <stdint.h>
#include "tim.h"
#include "usart.h"

/* 串口数据包解析结果缓存（全局可读，新包到达时自动刷新） */
#define USART_PACKET_DATA_LEN 10U
extern int16_t USART_Packet_Data[USART_PACKET_DATA_LEN];
extern uint8_t USART_Packet_Count;

/*
 * 任务标志位 — 由定时器 ISR 置位，主循环轮询消费。
 */
extern volatile uint8_t print_task_flag;    /* 10Hz 串口打印标志 */
extern volatile uint8_t task4_control_flag;  /* TASK4 固定周期控制标志 */

/*
 * 程序运行时间戳。
 *
 * Timer_Ms_t:
 *   毫秒级时间戳，TIM2 每 1ms 自动 +1。
 *   后面任务模式状态机、TASK3 超时、K230 UI 包重发都会用到。
 *
 * Timer_Bsp_t:
 *   秒级时间戳，保留原变量，兼容旧代码。
 */
extern volatile uint32_t Timer_Ms_t;
extern volatile uint32_t Timer_Bsp_t;

/* 获取当前毫秒时间戳。 */
uint32_t Control_Task_GetMillis(void);

void Control_Task1_Callback(API_TIM_Id_t id);  /* TIM1: PID 控制节拍 */
void Control_Task2_Callback(API_TIM_Id_t id);  /* TIM2: 慢任务调度，1ms 基准 */

/*
 * USART 中断回调。
 * 由 USART ISR 触发，内部调度 usart_irq_dispatch_by_id 处理 TX 队列排空和 RX 接收。
 * 必须在 main.c 中通过 Enroll_USART_RegisterIrqHandler 注册，否则 TX 队列不会被排空，
 * 会导致 TXE 中断无限循环卡死程序。
 */
void Control_Task_USART_Callback(API_USART_Id_t id);

#endif /* __CONTROL_TASK_H */
