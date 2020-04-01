/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
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
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
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
#define Btn_Pin GPIO_PIN_13
#define Btn_GPIO_Port GPIOC
#define STEPPER3_Pin GPIO_PIN_9
#define STEPPER3_GPIO_Port GPIOF
#define STEPPER2_Pin GPIO_PIN_3
#define STEPPER2_GPIO_Port GPIOA
#define STEPPER1_Pin GPIO_PIN_5
#define STEPPER1_GPIO_Port GPIOA
#define STEPPER4_Pin GPIO_PIN_6
#define STEPPER4_GPIO_Port GPIOA
#define SERVO1_Pin GPIO_PIN_9
#define SERVO1_GPIO_Port GPIOE
#define SERVO2_Pin GPIO_PIN_11
#define SERVO2_GPIO_Port GPIOE
#define EN4_Pin GPIO_PIN_10
#define EN4_GPIO_Port GPIOD
#define DIR4_Pin GPIO_PIN_11
#define DIR4_GPIO_Port GPIOD
#define EN3_Pin GPIO_PIN_12
#define EN3_GPIO_Port GPIOD
#define DIR3_Pin GPIO_PIN_13
#define DIR3_GPIO_Port GPIOD
#define EN2_Pin GPIO_PIN_14
#define EN2_GPIO_Port GPIOD
#define DIR2_Pin GPIO_PIN_15
#define DIR2_GPIO_Port GPIOD
#define EN1_Pin GPIO_PIN_2
#define EN1_GPIO_Port GPIOG
#define DIR1_Pin GPIO_PIN_3
#define DIR1_GPIO_Port GPIOG
#define SBUS_TX_Pin GPIO_PIN_10
#define SBUS_TX_GPIO_Port GPIOC
#define SBUS_RX_Pin GPIO_PIN_0
#define SBUS_RX_GPIO_Port GPIOD
#define Led_Pin GPIO_PIN_7
#define Led_GPIO_Port GPIOB
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
