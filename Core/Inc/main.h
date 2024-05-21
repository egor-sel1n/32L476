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
#include "stm32l4xx_hal.h"

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
#define D5_Pin GPIO_PIN_9
#define D5_GPIO_Port GPIOC
#define GAS_RX_Pin GPIO_PIN_10
#define GAS_RX_GPIO_Port GPIOA
#define LAMP_Pin GPIO_PIN_12
#define LAMP_GPIO_Port GPIOA
#define PAG_MUX_Pin GPIO_PIN_15
#define PAG_MUX_GPIO_Port GPIOA
#define PAG_CLKOUT_Pin GPIO_PIN_10
#define PAG_CLKOUT_GPIO_Port GPIOC
#define PAG_TXRXCLK_Pin GPIO_PIN_11
#define PAG_TXRXCLK_GPIO_Port GPIOC
#define PAG_TXRXDATA_Pin GPIO_PIN_12
#define PAG_TXRXDATA_GPIO_Port GPIOC
#define PAG_SWD_Pin GPIO_PIN_2
#define PAG_SWD_GPIO_Port GPIOD
#define PAG_SCK_Pin GPIO_PIN_3
#define PAG_SCK_GPIO_Port GPIOB
#define PAG_MISO_Pin GPIO_PIN_4
#define PAG_MISO_GPIO_Port GPIOB
#define PAG_MOSI_Pin GPIO_PIN_5
#define PAG_MOSI_GPIO_Port GPIOB
#define PAG_SLE_Pin GPIO_PIN_8
#define PAG_SLE_GPIO_Port GPIOB
#define PAG_CE_Pin GPIO_PIN_9
#define PAG_CE_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
