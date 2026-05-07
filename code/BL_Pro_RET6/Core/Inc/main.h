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
#define Motor_EN_Pin GPIO_PIN_13
#define Motor_EN_GPIO_Port GPIOC
#define NRST_Pin GPIO_PIN_10
#define NRST_GPIO_Port GPIOG
#define pwmL_A_Pin GPIO_PIN_0
#define pwmL_A_GPIO_Port GPIOC
#define pwmL_B_Pin GPIO_PIN_2
#define pwmL_B_GPIO_Port GPIOC
#define pwmL_C_Pin GPIO_PIN_3
#define pwmL_C_GPIO_Port GPIOC
#define CurrentL_B_Pin GPIO_PIN_1
#define CurrentL_B_GPIO_Port GPIOA
#define CurrentL_A_Pin GPIO_PIN_2
#define CurrentL_A_GPIO_Port GPIOA
#define EcdR_CS_Pin GPIO_PIN_4
#define EcdR_CS_GPIO_Port GPIOA
#define EcdR_SCK_Pin GPIO_PIN_5
#define EcdR_SCK_GPIO_Port GPIOA
#define EcdR_MISO_Pin GPIO_PIN_6
#define EcdR_MISO_GPIO_Port GPIOA
#define EcdR_MOSI_Pin GPIO_PIN_7
#define EcdR_MOSI_GPIO_Port GPIOA
#define CurrentR_A_Pin GPIO_PIN_5
#define CurrentR_A_GPIO_Port GPIOC
#define batV_Pin GPIO_PIN_1
#define batV_GPIO_Port GPIOB
#define CurrentR_B_Pin GPIO_PIN_2
#define CurrentR_B_GPIO_Port GPIOB
#define IMU_SCK_Pin GPIO_PIN_13
#define IMU_SCK_GPIO_Port GPIOB
#define IMU_MISO_Pin GPIO_PIN_14
#define IMU_MISO_GPIO_Port GPIOB
#define IMU_MOSI_Pin GPIO_PIN_15
#define IMU_MOSI_GPIO_Port GPIOB
#define IMU_CS_Pin GPIO_PIN_6
#define IMU_CS_GPIO_Port GPIOC
#define IMU_INT_Pin GPIO_PIN_7
#define IMU_INT_GPIO_Port GPIOC
#define IMU_INT_EXTI_IRQn EXTI9_5_IRQn
#define LED_Pin GPIO_PIN_8
#define LED_GPIO_Port GPIOA
#define EcdL_SCK_Pin GPIO_PIN_10
#define EcdL_SCK_GPIO_Port GPIOC
#define EcdL_MISO_Pin GPIO_PIN_11
#define EcdL_MISO_GPIO_Port GPIOC
#define EcdL_MOSI_Pin GPIO_PIN_12
#define EcdL_MOSI_GPIO_Port GPIOC
#define EcdL_CS_Pin GPIO_PIN_2
#define EcdL_CS_GPIO_Port GPIOD
#define pwmR_A_Pin GPIO_PIN_7
#define pwmR_A_GPIO_Port GPIOB
#define pwmR_B_Pin GPIO_PIN_8
#define pwmR_B_GPIO_Port GPIOB
#define pwmR_C_Pin GPIO_PIN_9
#define pwmR_C_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
