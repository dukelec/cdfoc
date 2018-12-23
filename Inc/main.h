/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
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

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CDCTL_INT_N_Pin GPIO_PIN_0
#define CDCTL_INT_N_GPIO_Port GPIOC
#define CDCTL_INT_N_EXTI_IRQn EXTI0_IRQn
#define S_NSS_Pin GPIO_PIN_1
#define S_NSS_GPIO_Port GPIOC
#define S_MISO_Pin GPIO_PIN_2
#define S_MISO_GPIO_Port GPIOC
#define S_MOSI_Pin GPIO_PIN_3
#define S_MOSI_GPIO_Port GPIOC
#define CDCTL_NSS_Pin GPIO_PIN_4
#define CDCTL_NSS_GPIO_Port GPIOA
#define CDCTL_SCK_Pin GPIO_PIN_5
#define CDCTL_SCK_GPIO_Port GPIOA
#define CDCTL_MISO_Pin GPIO_PIN_6
#define CDCTL_MISO_GPIO_Port GPIOA
#define CDCTL_MOSI_Pin GPIO_PIN_7
#define CDCTL_MOSI_GPIO_Port GPIOA
#define CDCTL_RST_N_Pin GPIO_PIN_5
#define CDCTL_RST_N_GPIO_Port GPIOC
#define ADC2_IN9_PWR_DET_Pin GPIO_PIN_1
#define ADC2_IN9_PWR_DET_GPIO_Port GPIOB
#define S_SCK_Pin GPIO_PIN_10
#define S_SCK_GPIO_Port GPIOB
#define DRV_C_N_Pin GPIO_PIN_13
#define DRV_C_N_GPIO_Port GPIOB
#define DRV_B_N_Pin GPIO_PIN_14
#define DRV_B_N_GPIO_Port GPIOB
#define DRV_A_N_Pin GPIO_PIN_15
#define DRV_A_N_GPIO_Port GPIOB
#define DRV_CAL_Pin GPIO_PIN_6
#define DRV_CAL_GPIO_Port GPIOC
#define DRV_EN_Pin GPIO_PIN_7
#define DRV_EN_GPIO_Port GPIOC
#define DRV_NSS_Pin GPIO_PIN_8
#define DRV_NSS_GPIO_Port GPIOC
#define DRV_FAULT_N_Pin GPIO_PIN_9
#define DRV_FAULT_N_GPIO_Port GPIOC
#define RS485_RE_Pin GPIO_PIN_2
#define RS485_RE_GPIO_Port GPIOD
#define DRV_SCK_Pin GPIO_PIN_3
#define DRV_SCK_GPIO_Port GPIOB
#define DRV_MISO_Pin GPIO_PIN_4
#define DRV_MISO_GPIO_Port GPIOB
#define DRV_MOSI_Pin GPIO_PIN_5
#define DRV_MOSI_GPIO_Port GPIOB
#define DBG_TX_Pin GPIO_PIN_6
#define DBG_TX_GPIO_Port GPIOB
#define DBG_RX_Pin GPIO_PIN_7
#define DBG_RX_GPIO_Port GPIOB
#define LED_GRN_Pin GPIO_PIN_8
#define LED_GRN_GPIO_Port GPIOB
#define LED_RED_Pin GPIO_PIN_9
#define LED_RED_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
