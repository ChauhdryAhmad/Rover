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
#define LEFT_Pin GPIO_PIN_0
#define LEFT_GPIO_Port GPIOB
#define LEFT_EXTI_IRQn EXTI0_IRQn
#define RIGHT_Pin GPIO_PIN_1
#define RIGHT_GPIO_Port GPIOB
#define RIGHT_EXTI_IRQn EXTI1_IRQn
#define Relay1_Pin GPIO_PIN_14
#define Relay1_GPIO_Port GPIOB
#define Relay2_Pin GPIO_PIN_15
#define Relay2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
typedef struct {
	char buf[100];
	int head;
	int tail;
} rxque;

typedef struct {
	char buf[100];
	uint16_t cp;
	uint16_t state;
} srcom;

typedef struct {
	uint32_t sCount;
	uint32_t eCount;
	int diff;
	uint8_t state;
	double rpm;
	short speed;
	double distance;
	uint32_t count;
	short base_start;
	int prev_diff;
	volatile short stop_counter;
} wheel;

typedef struct{
	short cvalue;
	short svalue;
	short error;
	short prev_error;
	short integral;
	short imax;
	short imin;
	double coffp;
	double coffi;
	double coffd;
	short outmax;
	short outmin;
	short output;
	uint8_t side;
	short prev_cvalue;
}PID;
/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
