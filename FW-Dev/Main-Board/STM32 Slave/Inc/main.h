/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BUZZ GPIO_PIN_9
#define BUZZ_GPIO_Port GPIOA
#define OUT_DIR GPIO_PIN_8
#define OUT_DIR_GPIO_Port GPIOE
#define PC14_OSC32_IN_Pin GPIO_PIN_14
#define PC14_OSC32_IN_GPIO_Port GPIOC
#define PC15_OSC32_OUT_Pin GPIO_PIN_15
#define PC15_OSC32_OUT_GPIO_Port GPIOC
#define PH0_OSC_IN_Pin GPIO_PIN_0
#define PH0_OSC_IN_GPIO_Port GPIOH
#define PH1_OSC_OUT_Pin GPIO_PIN_1
#define PH1_OSC_OUT_GPIO_Port GPIOH
#define OTG_FS_PowerSwitchOn_Pin GPIO_PIN_0
#define OTG_FS_PowerSwitchOn_GPIO_Port GPIOC
#define PDM_OUT_Pin GPIO_PIN_3
#define PDM_OUT_GPIO_Port GPIOC
#define B1_Pin GPIO_PIN_0
#define B1_GPIO_Port GPIOA
#define I2S3_WS_Pin GPIO_PIN_4
#define I2S3_WS_GPIO_Port GPIOA
#define SPI1_SCK_Pin GPIO_PIN_5
#define SPI1_SCK_GPIO_Port GPIOA
#define SPI1_MISO_Pin GPIO_PIN_6
#define SPI1_MISO_GPIO_Port GPIOA
#define SPI1_MOSI_Pin GPIO_PIN_7
#define SPI1_MOSI_GPIO_Port GPIOA
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define CLK_IN_Pin GPIO_PIN_10
#define CLK_IN_GPIO_Port GPIOB
#define LD4_Pin GPIO_PIN_12
#define LD4_GPIO_Port GPIOD
#define LD3_Pin GPIO_PIN_13
#define LD3_GPIO_Port GPIOD
#define LD5_Pin GPIO_PIN_14
#define LD5_GPIO_Port GPIOD
#define LD6_Pin GPIO_PIN_15
#define LD6_GPIO_Port GPIOD
#define I2S3_MCK_Pin GPIO_PIN_7
#define I2S3_MCK_GPIO_Port GPIOC
#define VBUS_FS_Pin GPIO_PIN_9
#define VBUS_FS_GPIO_Port GPIOA
#define OTG_FS_ID_Pin GPIO_PIN_10
#define OTG_FS_ID_GPIO_Port GPIOA
#define OTG_FS_DM_Pin GPIO_PIN_11
#define OTG_FS_DM_GPIO_Port GPIOA
#define OTG_FS_DP_Pin GPIO_PIN_12
#define OTG_FS_DP_GPIO_Port GPIOA
#define SWDIO_Pin GPIO_PIN_13
#define SWDIO_GPIO_Port GPIOA
#define SWCLK_Pin GPIO_PIN_14
#define SWCLK_GPIO_Port GPIOA
#define I2S3_SCK_Pin GPIO_PIN_10
#define I2S3_SCK_GPIO_Port GPIOC
#define I2S3_SD_Pin GPIO_PIN_12
#define I2S3_SD_GPIO_Port GPIOC
#define Audio_RST_Pin GPIO_PIN_4
#define Audio_RST_GPIO_Port GPIOD
#define OTG_FS_OverCurrent_Pin GPIO_PIN_5
#define OTG_FS_OverCurrent_GPIO_Port GPIOD
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define Audio_SCL_Pin GPIO_PIN_6
#define Audio_SCL_GPIO_Port GPIOB
#define Audio_SDA_Pin GPIO_PIN_7
#define Audio_SDA_GPIO_Port GPIOB
#define MEMS_INT2_Pin GPIO_PIN_1
#define MEMS_INT2_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#define PID_TOP         GPIO_PIN_9  
#define DIR_TOP         GPIO_PIN_5  
#define DIR_BOT         GPIO_PIN_11  
#define PID_BOT         GPIO_PIN_12  

#define F1            0x01
#define F2            0x02
#define F3            0x03

#define  _HAL_RCC_TIM1_CLK_ENABLE()       (RCC->APB2ENR |=  (1 << 0) )

#define  _HAL_RCC_TIM3_CLK_ENABLE()       (RCC->APB1ENR |=  (1 << 1) )
#define  _HAL_RCC_TIM4_CLK_ENABLE()       (RCC->APB1ENR |=  (1 << 2) )
#define  _HAL_RCC_TIM5_CLK_ENABLE()       (RCC->APB1ENR |=  (1 << 3) )
#define  _HAL_RCC_INT_CLK_ENABLE()        (RCC->APB2ENR |=  (1 << 14) )
#define  _HAL_RCC_ADC1_CLK_ENABLE()       (RCC->APB2ENR |=  (1 << 8) )
#define  _HAL_RCC_ADC2_CLK_ENABLE()       (RCC->APB2ENR |=  (1 << 9) )
#define  _HAL_RCC_TIM9_CLK_ENABLE()       (RCC->APB2ENR |=  (1 << 16) )

/* Definition for USARTx Pins */

/* SOM BOARD UART1*/
#define GPIO_PIN_10_SEL_UART1                      10
#define GPIO_PIN_9_SEL_UART1                        9  
#define USART1_TX_PIN                    GPIO_PIN_9_SEL_UART1
#define USART1_TX_GPIO_PORT              GPIOA  
#define USART1_TX_AF                     GPIO_AF7_USART1
#define USART1_RX_PIN                    GPIO_PIN_10_SEL_UART1
#define USART1_RX_GPIO_PORT              GPIOA 
#define USART1_RX_AF                     GPIO_AF7_USART1

/* POWER BOARD UART4*/
#define GPIO_PIN_10_SEL_UART4                      10
#define GPIO_PIN_11_SEL_UART4                      11     
#define USART4_TX_PIN                    GPIO_PIN_10_SEL_UART4
#define USART4_TX_GPIO_PORT              GPIOC  
#define USART4_TX_AF                     GPIO_AF8_UART4
#define USART4_RX_PIN                    GPIO_PIN_11_SEL_UART4
#define USART4_RX_GPIO_PORT              GPIOC 
#define USART4_RX_AF                     GPIO_AF8_UART4

/* MASTER-SLAVE BOARD UART6*/
#define GPIO_PIN_6_SEL_UART6                        6
#define GPIO_PIN_7_SEL_UART6                        7   
#define USART6_TX_PIN                    GPIO_PIN_6_SEL_UART6
#define USART6_TX_GPIO_PORT              GPIOC  
#define USART6_TX_AF                     GPIO_AF8_USART6
#define USART6_RX_PIN                    GPIO_PIN_7_SEL_UART6
#define USART6_RX_GPIO_PORT              GPIOC 
#define USART6_RX_AF                     GPIO_AF8_USART6

/* Definition for USARTx's NVIC */
#define USART1_IRQn_EN                   USART1_IRQn
#define USART4_IRQn_EN                   UART4_IRQn
#define USART6_IRQn_EN                   USART6_IRQn
#define USARTx_IRQHandler                USART1_IRQHandler
#define USART4x_IRQHandler               USART4_IRQHandler
#define USART6x_IRQHandler               USART6_IRQHandler

/* MODES */
#define SLAVE_MASTER						( (uint32_t)0x32 )
#define MASTER_REQUEST					( (uint32_t)0x31 )
#define MCU_POWER_SEND					( (uint32_t)0xBBBBBB )

/* PARAMETER ID'S */
#define EXP_FLOW								( (uint32_t)0x51 )
#define PROX_FLOW								( (uint32_t)0x52 )
#define PROX_PRESSURE						( (uint32_t)0x53 )
#define EXP_TEMP								( (uint32_t)0x54 )
#define PROX_TEMP								( (uint32_t)0x55 )
#define BAROMETRIC_PRES					( (uint32_t)0x56 )
#define AMB_TEMP								( (uint32_t)0x57 )
#define POWER_SEND							( (uint32_t)0x60 )															//////ADDED ID
#define ALARM_FRAME							( (uint32_t)0xF5A9 )



#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
