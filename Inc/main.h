/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#define CS2_Pin GPIO_PIN_13
#define CS2_GPIO_Port GPIOC
#define ENKODER_Pin GPIO_PIN_1
#define ENKODER_GPIO_Port GPIOB
#define ENKODER_EXTI_IRQn EXTI1_IRQn
#define ECHO_Pin GPIO_PIN_2
#define ECHO_GPIO_Port GPIOB
#define ECHO_EXTI_IRQn EXTI2_IRQn
#define TRIG_Pin GPIO_PIN_10
#define TRIG_GPIO_Port GPIOB
#define SM_LED_Pin GPIO_PIN_12
#define SM_LED_GPIO_Port GPIOB
#define MOSI_Pin GPIO_PIN_6
#define MOSI_GPIO_Port GPIOB
#define CS1_Pin GPIO_PIN_8
#define CS1_GPIO_Port GPIOB
#define SCK_Pin GPIO_PIN_9
#define SCK_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
