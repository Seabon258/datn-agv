/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LED_Pin GPIO_PIN_13
#define LED_GPIO_Port GPIOC
#define ENC4_A_Pin GPIO_PIN_0
#define ENC4_A_GPIO_Port GPIOA
#define ENC4_B_Pin GPIO_PIN_1
#define ENC4_B_GPIO_Port GPIOA
#define UART_TX_TO_PI_RX_Pin GPIO_PIN_2
#define UART_TX_TO_PI_RX_GPIO_Port GPIOA
#define UART_RX_FROM_PI_TX_Pin GPIO_PIN_3
#define UART_RX_FROM_PI_TX_GPIO_Port GPIOA
#define LIMIT_TOP_Pin GPIO_PIN_4
#define LIMIT_TOP_GPIO_Port GPIOA
#define STEP_EN_Pin GPIO_PIN_5
#define STEP_EN_GPIO_Port GPIOA
#define VBAT_ADC_Pin GPIO_PIN_6
#define VBAT_ADC_GPIO_Port GPIOA
#define STEP_STEP_Pin GPIO_PIN_0
#define STEP_STEP_GPIO_Port GPIOB
#define STEP_DIR_Pin GPIO_PIN_1
#define STEP_DIR_GPIO_Port GPIOB
#define LIMIT_BOT_Pin GPIO_PIN_10
#define LIMIT_BOT_GPIO_Port GPIOB
#define M1_DIR_IN1_Pin GPIO_PIN_12
#define M1_DIR_IN1_GPIO_Port GPIOB
#define M2_DIR_IN1_Pin GPIO_PIN_13
#define M2_DIR_IN1_GPIO_Port GPIOB
#define M3_DIR_IN1_Pin GPIO_PIN_14
#define M3_DIR_IN1_GPIO_Port GPIOB
#define M4_DIR_IN1_Pin GPIO_PIN_15
#define M4_DIR_IN1_GPIO_Port GPIOB
#define M1_PWM_IN2_Pin GPIO_PIN_8
#define M1_PWM_IN2_GPIO_Port GPIOA
#define M2_PWM_IN2_Pin GPIO_PIN_9
#define M2_PWM_IN2_GPIO_Port GPIOA
#define M3_PWM_IN2_Pin GPIO_PIN_10
#define M3_PWM_IN2_GPIO_Port GPIOA
#define M4_PWM_IN2_Pin GPIO_PIN_11
#define M4_PWM_IN2_GPIO_Port GPIOA
#define ENC1_A_Pin GPIO_PIN_15
#define ENC1_A_GPIO_Port GPIOA
#define ENC1_B_Pin GPIO_PIN_3
#define ENC1_B_GPIO_Port GPIOB
#define ENC2_A_Pin GPIO_PIN_4
#define ENC2_A_GPIO_Port GPIOB
#define ENC2_B_Pin GPIO_PIN_5
#define ENC2_B_GPIO_Port GPIOB
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOB
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOB
#define IMU_SCL_Pin GPIO_PIN_8
#define IMU_SCL_GPIO_Port GPIOB
#define IMU_SDA_Pin GPIO_PIN_9
#define IMU_SDA_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
