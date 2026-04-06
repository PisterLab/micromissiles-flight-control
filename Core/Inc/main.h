/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
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
#include "stm32g4xx_hal.h"

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
#define VIN_SENSE_Pin GPIO_PIN_0
#define VIN_SENSE_GPIO_Port GPIOF
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define FAULT1_U1_Pin GPIO_PIN_0
#define FAULT1_U1_GPIO_Port GPIOA
#define FAULT2_U1_Pin GPIO_PIN_1
#define FAULT2_U1_GPIO_Port GPIOA
#define MODE_1X_Pin GPIO_PIN_2
#define MODE_1X_GPIO_Port GPIOA
#define STEP2_U1_Pin GPIO_PIN_3
#define STEP2_U1_GPIO_Port GPIOA
#define DIR2_U2_Pin GPIO_PIN_4
#define DIR2_U2_GPIO_Port GPIOA
#define DIR1_U2_Pin GPIO_PIN_5
#define DIR1_U2_GPIO_Port GPIOA
#define STEP2_U2_Pin GPIO_PIN_6
#define STEP2_U2_GPIO_Port GPIOA
#define DIR2_U1_Pin GPIO_PIN_7
#define DIR2_U1_GPIO_Port GPIOA
#define DIR1_U1_Pin GPIO_PIN_0
#define DIR1_U1_GPIO_Port GPIOB
#define STEP1_U1_Pin GPIO_PIN_1
#define STEP1_U1_GPIO_Port GPIOB
#define STEP1_U2_Pin GPIO_PIN_2
#define STEP1_U2_GPIO_Port GPIOB
#define MAGNET_DETECT_Pin GPIO_PIN_11
#define MAGNET_DETECT_GPIO_Port GPIOB
#define MODE_0X_Pin GPIO_PIN_12
#define MODE_0X_GPIO_Port GPIOB
#define FAULT2_U2_Pin GPIO_PIN_13
#define FAULT2_U2_GPIO_Port GPIOB
#define FAULT1_U2_Pin GPIO_PIN_14
#define FAULT1_U2_GPIO_Port GPIOB
#define ENABLE_DRIVERS_Pin GPIO_PIN_15
#define ENABLE_DRIVERS_GPIO_Port GPIOB
#define USART1_TX_Pin GPIO_PIN_9
#define USART1_TX_GPIO_Port GPIOA
#define USART1_RX_Pin GPIO_PIN_10
#define USART1_RX_GPIO_Port GPIOA
#define RED_LED_Pin GPIO_PIN_11
#define RED_LED_GPIO_Port GPIOA
#define BRIGHT_LED_Pin GPIO_PIN_12
#define BRIGHT_LED_GPIO_Port GPIOA
#define SPI1_CS_Pin GPIO_PIN_15
#define SPI1_CS_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
