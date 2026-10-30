/**
 * Dshot_f103_stub.c — DShot300 的 STM32F103C8T6 占位实现
 *
 * STM32F103C8T6 无法承载 DShot 四轴电调协议，原因：
 * 1) 无 DMA2：DShot 依赖 TIM1 的 DMA burst（DCR/DMAR + DMA2 Stream5），
 *    F103C8T6 仅具备 DMA1（7 通道），且本工程未实现 F1 的 DMA burst 驱动。
 * 2) 引脚硬冲突：DShot 需要 TIM1 CH1~CH4（PA8/PA9/PA10/PA11），与打靶
 *    闭环步进电机的 USART1（PA9/PA10）、NRF24L01 的 SPI1（PA8/PA11）冲突。
 *
 * 因此 F103 目标只保留打靶闭环步进电机（Stepmotor）功能，四轴 DShot
 * 油门链在此降级为空实现，保证 BSP/Motor 与 NRF24L01 遥测链可正常链接。
 */

#include "Dshot.h"

void DShot_Init(void)
{
	/* F103C8T6 无 DMA2 + TIM1 引脚冲突，四轴 DShot 电调不启用。 */
}

void DShot_Write(uint16_t m1, uint16_t m2, uint16_t m3, uint16_t m4)
{
	(void)m1;
	(void)m2;
	(void)m3;
	(void)m4;
}
