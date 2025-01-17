/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define SEN_CS_Pin GPIO_PIN_4
#define SEN_CS_GPIO_Port GPIOA
#define LED_G_Pin GPIO_PIN_0
#define LED_G_GPIO_Port GPIOB
#define LED_R_Pin GPIO_PIN_1
#define LED_R_GPIO_Port GPIOB
#define DBG_OUT2_Pin GPIO_PIN_2
#define DBG_OUT2_GPIO_Port GPIOB
#define DBG_OUT1_Pin GPIO_PIN_11
#define DBG_OUT1_GPIO_Port GPIOB
#define CD_CS_Pin GPIO_PIN_12
#define CD_CS_GPIO_Port GPIOB
#define CD_INT_Pin GPIO_PIN_6
#define CD_INT_GPIO_Port GPIOC
#define DRV_EN_Pin GPIO_PIN_15
#define DRV_EN_GPIO_Port GPIOA
#define DRV_CS_Pin GPIO_PIN_11
#define DRV_CS_GPIO_Port GPIOC
#define DRV_FAULT_Pin GPIO_PIN_6
#define DRV_FAULT_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
