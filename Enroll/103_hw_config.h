#ifndef __103_HW_CONFIG_H
#define __103_HW_CONFIG_H

#include "f103_gpio.h" /* GPIO_Pin_x + GPIOA/B/C 原语，其余枚举值在 Enroll.c 展开时由 Enroll_Internal.h 提供 */

/*
 * 103_hw_config.h 板级硬件映射宏
 *
 * 本文件当前服务于「F103 无人机遥控器（Remote Controller）」目标：
 *   - ADC 读 2 个摇杆 4 路电位器（PA0~PA3）；
 *   - NRF24L01（2.4G）发送到飞控端 + 接收回传遥测；
 *   - OLED 显示油门值；
 *   - USART1 / USART3 作为调试口。
 *
 * 保留未用外设（TIM/PWM/I2C/KEY/MPU6050 INT）仅映射到空闲脚，
 * 目的是让 Enroll.c 的 X-Macro 展开继续编译通过，遥控器并不实际调用。
 */

/* LED 板级映射：注册 1 路板载心跳灯（PC13） */
#define HW_LED_MAP(X) \
	X(LED1, GPIOC, GPIO_Pin_13)

/* KEY 板级映射：注册 1 路按键（PB2，预留未用） */
#define HW_KEY_MAP(X) \
	X(KEY1, GPIOB, GPIO_Pin_2)

/* USART1 引脚定义：TX=PA9，RX=PA10 */
#define HW_USART1_TX_PORT GPIOA
#define HW_USART1_TX_PIN  GPIO_Pin_9
#define HW_USART1_RX_PORT GPIOA
#define HW_USART1_RX_PIN  GPIO_Pin_10

/* USART3 引脚定义：TX=PB10，RX=PB11 */
#define HW_USART3_TX_PORT GPIOB
#define HW_USART3_TX_PIN  GPIO_Pin_10
#define HW_USART3_RX_PORT GPIOB
#define HW_USART3_RX_PIN  GPIO_Pin_11

/* USART 板级映射：注册 2 路串口（USART1 + USART3） */
#define HW_USART_MAP(X) \
	X(API_USART1, API_USART_CORE_USART1, HW_USART1_TX_PORT, HW_USART1_TX_PIN, HW_USART1_RX_PORT, HW_USART1_RX_PIN) \
	X(API_USART3, API_USART_CORE_USART3, HW_USART3_TX_PORT, HW_USART3_TX_PIN, HW_USART3_RX_PORT, HW_USART3_RX_PIN)

/* TIM 板级映射：保留 TIM3 + TIM2（遥控器不用，仅为 Enroll 编译） */
#define HW_TIM_MAP(X) \
	X(API_TIM1, API_TIM_CORE_TIM3) \
	X(API_TIM2, API_TIM_CORE_TIM2)

/* PWM 板级映射：PA8 = TIM1_CH1，输出 1Hz 方波用于 LED 闪烁（一秒一闪） */
#define HW_PWM_MAP(X) \
	X(API_PWM_TIM1, API_PWM_CH1, API_PWM_CORE_TIM1, API_PWM_CORE_CH1, GPIOA, GPIO_Pin_8)

/* 软件 I2C1 引脚定义：SCL=PB8，SDA=PA12（遥控器不用，保留占位，避开三档开关 PB6/PB7） */
#define HW_I2C1_SCL_PORT GPIOB
#define HW_I2C1_SCL_PIN  GPIO_Pin_8
#define HW_I2C1_SDA_PORT GPIOA
#define HW_I2C1_SDA_PIN  GPIO_Pin_12

/* I2C 板级映射：注册 1 路软件 I2C */
#define HW_I2C_MAP(X) \
	X(API_I2C1, HW_I2C1_SCL_PORT, HW_I2C1_SCL_PIN, HW_I2C1_SDA_PORT, HW_I2C1_SDA_PIN, 0, 0) \

/* ── 软件 SPI1（OLED）：CLK=PB1, MOSI=PB0, CS=PB9, MISO=PA11（OLED 只写，占位） ── */
#define HW_SPI1_SCK_PORT  GPIOB
#define HW_SPI1_SCK_PIN   GPIO_Pin_1
#define HW_SPI1_MOSI_PORT GPIOB
#define HW_SPI1_MOSI_PIN  GPIO_Pin_0

#define HW_SPI1_MISO_PORT GPIOA
#define HW_SPI1_MISO_PIN  GPIO_Pin_11

#define HW_SPI1_CS_PORT   GPIOB
#define HW_SPI1_CS_PIN    GPIO_Pin_9

/* ── 软件 SPI2（NRF24L01）：SCK=PA4, MOSI=PB13, MISO=PA5, CSN=PB14, IRQ=PB12 ── */
#define HW_SPI2_SCK_PORT  GPIOA
#define HW_SPI2_SCK_PIN   GPIO_Pin_4

#define HW_SPI2_MOSI_PORT GPIOB
#define HW_SPI2_MOSI_PIN  GPIO_Pin_13

#define HW_SPI2_MISO_PORT GPIOA
#define HW_SPI2_MISO_PIN  GPIO_Pin_5

#define HW_SPI2_CS_PORT   GPIOB
#define HW_SPI2_CS_PIN    GPIO_Pin_14

/* SPI 板级映射：注册 2 路软件 SPI（OLED + NRF24L01） */
#define HW_SPI_MAP(X) \
	X(API_SPI1, HW_SPI1_CS_PORT, HW_SPI1_CS_PIN, \
	  HW_SPI1_SCK_PORT, HW_SPI1_SCK_PIN, \
	  HW_SPI1_MOSI_PORT, HW_SPI1_MOSI_PIN, \
	  HW_SPI1_MISO_PORT, HW_SPI1_MISO_PIN, 0, 0, 0, 0) \
	X(API_SPI2, HW_SPI2_CS_PORT, HW_SPI2_CS_PIN, \
	  HW_SPI2_SCK_PORT, HW_SPI2_SCK_PIN, \
	  HW_SPI2_MOSI_PORT, HW_SPI2_MOSI_PIN, \
	  HW_SPI2_MISO_PORT, HW_SPI2_MISO_PIN, 0, 0, 0, 0)

/* NRF24L01 控制引脚定义：CE=PB15 */
#define HW_NRF24L01_CE_PORT GPIOB
#define HW_NRF24L01_CE_PIN  GPIO_Pin_15

/* NRF24L01 控制引脚映射：注册 1 组 CE */
#define HW_NRF24L01_CTRL_MAP(X) \
	X(HW_NRF24L01_CE_PORT, HW_NRF24L01_CE_PIN)

/* OLED SPI 模式控制脚：DC=PA6, RES=PA7 */
#define HW_OLED_DC_PORT   GPIOA
#define HW_OLED_DC_PIN    GPIO_Pin_6
#define HW_OLED_RES_PORT  GPIOA
#define HW_OLED_RES_PIN   GPIO_Pin_7

/* ADC 板级映射：2 个摇杆 4 路电位器（PA0~PA3 = ADC1 CH0~CH3） */
#define HW_ADC_MAP(X) \
	X(API_ADC1, API_ADC_CH0, GPIOA, GPIO_Pin_0) \
	X(API_ADC1, API_ADC_CH1, GPIOA, GPIO_Pin_1) \
	X(API_ADC1, API_ADC_CH2, GPIOA, GPIO_Pin_2) \
	X(API_ADC1, API_ADC_CH3, GPIOA, GPIO_Pin_3)

/* 三档开关：公共端接 GND，两个掷端分别接 PB6 / PB7（上拉输入）。
 *   上档：PB6 被拉低、PB7 高   -> 状态 0
 *   中档：PB6/PB7 都高         -> 状态 1
 *   下档：PB7 被拉低、PB6 高   -> 状态 2
 */
#define HW_SWITCH_A_PORT  GPIOB
#define HW_SWITCH_A_PIN   GPIO_Pin_6
#define HW_SWITCH_B_PORT  GPIOB
#define HW_SWITCH_B_PIN   GPIO_Pin_7

/* MPU6050 INT 板级映射：仅维护引脚资源（PB5，遥控器未使用，占位） */
#define HW_MPU6050_INT_PORT             GPIOB
#define HW_MPU6050_INT_PIN              GPIO_Pin_5

/* 当前板子上注册了 1 个 LED */
#define HW_LED_COUNT  1U
/* 当前板子上注册了 1 个按键 */
#define HW_KEY_COUNT  1U
/* 当前板子上注册了 2 路 USART */
#define HW_USART_COUNT  2U
/* 当前板子上注册了 2 路 TIM */
#define HW_TIM_COUNT  2U
/* 当前板子上注册了 1 路 PWM 通道 */
#define HW_PWM_COUNT  1U
/* 当前板子上注册了 1 路软件 I2C */
#define HW_I2C_COUNT  1U
/* 当前板子上注册了 2 路软件 SPI */
#define HW_SPI_COUNT  2U
/* 当前板子上注册了 1 组 NRF24L01 控制引脚 */
#define HW_NRF24L01_CTRL_COUNT  1U
/* 当前板子上注册了 1 个 MPU6050 INT 引脚 */
#define HW_MPU6050_INT_COUNT 1U
/* 当前板子上注册了 4 路 ADC */
#define HW_ADC_COUNT  4U

#endif /* __103_HW_CONFIG_H */
