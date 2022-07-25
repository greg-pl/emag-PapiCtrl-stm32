/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
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
#include "stm32f1xx_hal.h"

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
#define KEY1_Pin GPIO_PIN_0
#define KEY1_GPIO_Port GPIOC
#define LED_KEY1_Pin GPIO_PIN_1
#define LED_KEY1_GPIO_Port GPIOC
#define KEY2_Pin GPIO_PIN_2
#define KEY2_GPIO_Port GPIOC
#define LED_KEY2_Pin GPIO_PIN_3
#define LED_KEY2_GPIO_Port GPIOC
#define LED_OK_Pin GPIO_PIN_0
#define LED_OK_GPIO_Port GPIOA
#define ZAS_UART2_RTS_Pin GPIO_PIN_1
#define ZAS_UART2_RTS_GPIO_Port GPIOA
#define ZAS_UART2_TX_Pin GPIO_PIN_2
#define ZAS_UART2_TX_GPIO_Port GPIOA
#define ZAS_UART2_RX_Pin GPIO_PIN_3
#define ZAS_UART2_RX_GPIO_Port GPIOA
#define X_SPI1_CS_Pin GPIO_PIN_4
#define X_SPI1_CS_GPIO_Port GPIOA
#define X_SLEEP_REQ_Pin GPIO_PIN_5
#define X_SLEEP_REQ_GPIO_Port GPIOA
#define X_STATUS_Pin GPIO_PIN_4
#define X_STATUS_GPIO_Port GPIOC
#define X_SPI1_ATTN_Pin GPIO_PIN_5
#define X_SPI1_ATTN_GPIO_Port GPIOC
#define X_RSSI_Pin GPIO_PIN_0
#define X_RSSI_GPIO_Port GPIOB
#define X_RST_Pin GPIO_PIN_1
#define X_RST_GPIO_Port GPIOB
#define BOOT1_Pin GPIO_PIN_2
#define BOOT1_GPIO_Port GPIOB
#define X_UART3_TX_Pin GPIO_PIN_10
#define X_UART3_TX_GPIO_Port GPIOB
#define X_UART3_RX_Pin GPIO_PIN_11
#define X_UART3_RX_GPIO_Port GPIOB
#define Z1_SPI2_CS_Pin GPIO_PIN_12
#define Z1_SPI2_CS_GPIO_Port GPIOB
#define Z1_SPI2_CLK_Pin GPIO_PIN_13
#define Z1_SPI2_CLK_GPIO_Port GPIOB
#define Z1_SPI2_MISO_Pin GPIO_PIN_14
#define Z1_SPI2_MISO_GPIO_Port GPIOB
#define Z1_SPI2_MOSI_Pin GPIO_PIN_15
#define Z1_SPI2_MOSI_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_6
#define LED_R_GPIO_Port GPIOC
#define LED_G_Pin GPIO_PIN_7
#define LED_G_GPIO_Port GPIOC
#define Z1_RTS4_Pin GPIO_PIN_8
#define Z1_RTS4_GPIO_Port GPIOC
#define Z1_EXP_RST_Pin GPIO_PIN_9
#define Z1_EXP_RST_GPIO_Port GPIOC
#define BLE_UART1_TX_Pin GPIO_PIN_9
#define BLE_UART1_TX_GPIO_Port GPIOA
#define BLE_UART1_RX_Pin GPIO_PIN_10
#define BLE_UART1_RX_GPIO_Port GPIOA
#define BLE_UART1_CTS_Pin GPIO_PIN_11
#define BLE_UART1_CTS_GPIO_Port GPIOA
#define BLE_UART1_RTS_Pin GPIO_PIN_12
#define BLE_UART1_RTS_GPIO_Port GPIOA
#define BLE_RST_Pin GPIO_PIN_15
#define BLE_RST_GPIO_Port GPIOA
#define Z1_UART4_TX_Pin GPIO_PIN_10
#define Z1_UART4_TX_GPIO_Port GPIOC
#define Z1_UART4_RX_Pin GPIO_PIN_11
#define Z1_UART4_RX_GPIO_Port GPIOC
#define GPS_UART5_TX_Pin GPIO_PIN_12
#define GPS_UART5_TX_GPIO_Port GPIOC
#define GPS_UART5_RX_Pin GPIO_PIN_2
#define GPS_UART5_RX_GPIO_Port GPIOD
#define GPS_RST_Pin GPIO_PIN_3
#define GPS_RST_GPIO_Port GPIOB
#define GPS_PPS_Pin GPIO_PIN_4
#define GPS_PPS_GPIO_Port GPIOB
#define HW_VER_Pin GPIO_PIN_5
#define HW_VER_GPIO_Port GPIOB
#define I2C_SCL_Pin GPIO_PIN_6
#define I2C_SCL_GPIO_Port GPIOB
#define I2C_SDA_Pin GPIO_PIN_7
#define I2C_SDA_GPIO_Port GPIOB
#define EXP_INT1_Pin GPIO_PIN_8
#define EXP_INT1_GPIO_Port GPIOB
#define EXP_INT2_Pin GPIO_PIN_9
#define EXP_INT2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
