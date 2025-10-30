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
#include "stm32f1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <data_templates.h>
#include  "motor/motor_functions.h"
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Ncoder_A_Pin GPIO_PIN_6
#define Ncoder_A_GPIO_Port GPIOA
#define Ncoder_B_Pin GPIO_PIN_7
#define Ncoder_B_GPIO_Port GPIOA
#define AXIS_TX_Pin GPIO_PIN_10
#define AXIS_TX_GPIO_Port GPIOB
#define AXIS_RX_Pin GPIO_PIN_11
#define AXIS_RX_GPIO_Port GPIOB
#define ENDSTOP_Pin GPIO_PIN_9
#define ENDSTOP_GPIO_Port GPIOA
#define ENDSTOP_EXTI_IRQn EXTI9_5_IRQn
#define DIR_Pin GPIO_PIN_10
#define DIR_GPIO_Port GPIOA
#define PULL_MANUAL_Pin GPIO_PIN_11
#define PULL_MANUAL_GPIO_Port GPIOA
#define ENABLE_Pin GPIO_PIN_12
#define ENABLE_GPIO_Port GPIOA
#define PULL_PWM_Pin GPIO_PIN_7
#define PULL_PWM_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
