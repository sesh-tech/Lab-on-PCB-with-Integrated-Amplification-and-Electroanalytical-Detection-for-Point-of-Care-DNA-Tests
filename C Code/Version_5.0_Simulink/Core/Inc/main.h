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
#define electro_Chem_Result_Pin GPIO_PIN_0
#define electro_Chem_Result_GPIO_Port GPIOF
#define air_Heater_Voltage_Pin_Pin GPIO_PIN_0
#define air_Heater_Voltage_Pin_GPIO_Port GPIOA
#define main_Heater_Voltage_Pin_Pin GPIO_PIN_1
#define main_Heater_Voltage_Pin_GPIO_Port GPIOA
#define USART2_TX_Pin GPIO_PIN_2
#define USART2_TX_GPIO_Port GPIOA
#define USART2_RX_Pin GPIO_PIN_3
#define USART2_RX_GPIO_Port GPIOA
#define electro_Chem_Offset_Pin_Pin GPIO_PIN_4
#define electro_Chem_Offset_Pin_GPIO_Port GPIOA
#define electro_Chem_Set_Pin_Pin GPIO_PIN_5
#define electro_Chem_Set_Pin_GPIO_Port GPIOA
#define air_Heater_Current_Pin_Pin GPIO_PIN_6
#define air_Heater_Current_Pin_GPIO_Port GPIOA
#define main_Heater_Current_Pin_Pin GPIO_PIN_7
#define main_Heater_Current_Pin_GPIO_Port GPIOA
#define fan_Pin_Pin GPIO_PIN_9
#define fan_Pin_GPIO_Port GPIOA
#define T_SWDIO_Pin GPIO_PIN_13
#define T_SWDIO_GPIO_Port GPIOA
#define T_SWCLK_Pin GPIO_PIN_14
#define T_SWCLK_GPIO_Port GPIOA
#define air_Heater_PWM_Pin_Pin GPIO_PIN_15
#define air_Heater_PWM_Pin_GPIO_Port GPIOA
#define electro_Chem_ONOFF_Pin GPIO_PIN_4
#define electro_Chem_ONOFF_GPIO_Port GPIOB
#define main_Heater_PWM_Pin_Pin GPIO_PIN_6
#define main_Heater_PWM_Pin_GPIO_Port GPIOB
#define LD2_Pin GPIO_PIN_8
#define LD2_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
