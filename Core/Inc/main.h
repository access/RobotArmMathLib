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
#define JOINT6_DIR_Pin GPIO_PIN_4
#define JOINT6_DIR_GPIO_Port GPIOE
#define JOINT6_STEP_Pin GPIO_PIN_5
#define JOINT6_STEP_GPIO_Port GPIOE
#define SPI_CS_Pin GPIO_PIN_2
#define SPI_CS_GPIO_Port GPIOA
#define JOINT2_DIR_Pin GPIO_PIN_4
#define JOINT2_DIR_GPIO_Port GPIOA
#define JOINT2_STEP_Pin GPIO_PIN_5
#define JOINT2_STEP_GPIO_Port GPIOA
#define JOINT3_STEP_Pin GPIO_PIN_6
#define JOINT3_STEP_GPIO_Port GPIOA
#define JOINT3_DIR_Pin GPIO_PIN_7
#define JOINT3_DIR_GPIO_Port GPIOA
#define JOINT1_STEP_Pin GPIO_PIN_9
#define JOINT1_STEP_GPIO_Port GPIOE
#define JOINT1_DIR_Pin GPIO_PIN_10
#define JOINT1_DIR_GPIO_Port GPIOE
#define JOINT7_DIR_Pin GPIO_PIN_13
#define JOINT7_DIR_GPIO_Port GPIOB
#define JOINT7_STEP_Pin GPIO_PIN_14
#define JOINT7_STEP_GPIO_Port GPIOB
#define JOINT4_DIR_Pin GPIO_PIN_11
#define JOINT4_DIR_GPIO_Port GPIOD
#define JOINT4_STEP_Pin GPIO_PIN_12
#define JOINT4_STEP_GPIO_Port GPIOD
#define LED3_Pin GPIO_PIN_13
#define LED3_GPIO_Port GPIOD
#define LED2_Pin GPIO_PIN_14
#define LED2_GPIO_Port GPIOD
#define LED1_Pin GPIO_PIN_15
#define LED1_GPIO_Port GPIOD
#define JOINT5_STEP_Pin GPIO_PIN_6
#define JOINT5_STEP_GPIO_Port GPIOC
#define JOINT5_DIR_Pin GPIO_PIN_7
#define JOINT5_DIR_GPIO_Port GPIOC
#define LED5_Pin GPIO_PIN_5
#define LED5_GPIO_Port GPIOD
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
