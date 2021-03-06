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
#include "stm32f0xx_hal.h"

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
void setDebugLEDs(uint8_t LED1, uint8_t LED2);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define BOOT_Pin GPIO_PIN_8
#define BOOT_GPIO_Port GPIOB
#define DEBUG1_Pin GPIO_PIN_1
#define DEBUG1_GPIO_Port GPIOF
#define LEDS0_Pin GPIO_PIN_0
#define LEDS0_GPIO_Port GPIOA
#define LEDS1_Pin GPIO_PIN_1
#define LEDS1_GPIO_Port GPIOA
#define LEDS2_Pin GPIO_PIN_2
#define LEDS2_GPIO_Port GPIOA
#define LEDS3_Pin GPIO_PIN_3
#define LEDS3_GPIO_Port GPIOA
#define NHDD_Pin GPIO_PIN_4
#define NHDD_GPIO_Port GPIOA
#define NHDD_EXTI_IRQn EXTI4_15_IRQn
#define NPOWER_Pin GPIO_PIN_5
#define NPOWER_GPIO_Port GPIOA
#define NPOWER_EXTI_IRQn EXTI4_15_IRQn
#define LEDS4_Pin GPIO_PIN_6
#define LEDS4_GPIO_Port GPIOA
#define LEDSTRIPES_Pin GPIO_PIN_7
#define LEDSTRIPES_GPIO_Port GPIOA
#define DEBUG2_Pin GPIO_PIN_1
#define DEBUG2_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
