/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define LED0_RED_Pin GPIO_PIN_9
#define LED0_RED_GPIO_Port GPIOF
#define LED1_GREEN_Pin GPIO_PIN_10
#define LED1_GREEN_GPIO_Port GPIOF
#define VF_Pin GPIO_PIN_1
#define VF_GPIO_Port GPIOA
#define VRL_Pin GPIO_PIN_2
#define VRL_GPIO_Port GPIOA
#define VRR_Pin GPIO_PIN_3
#define VRR_GPIO_Port GPIOA
#define HL_Pin GPIO_PIN_0
#define HL_GPIO_Port GPIOB
#define HR_Pin GPIO_PIN_1
#define HR_GPIO_Port GPIOB
#define LoRa_TX_Pin GPIO_PIN_8
#define LoRa_TX_GPIO_Port GPIOD
#define LoRa_RX_Pin GPIO_PIN_9
#define LoRa_RX_GPIO_Port GPIOD
#define IMU_TX_Pin GPIO_PIN_6
#define IMU_TX_GPIO_Port GPIOC
#define IMU_RX_Pin GPIO_PIN_7
#define IMU_RX_GPIO_Port GPIOC
#define PDS_SCL_Pin GPIO_PIN_6
#define PDS_SCL_GPIO_Port GPIOB
#define PDS_SDA_Pin GPIO_PIN_7
#define PDS_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
