/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "stm32f1xx_hal.h"

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
void ReloadMessage();

void ReloadingMessage();

void ModeDisplay();

void BulletsDisplay();

void OutOfAmmoEvent();

//Kich hoat am thanh
void PlaySound(const uint8_t audio_data[],const uint32_t audio_length);

//Ham delay bang timer 1
void Timer_Delay_ms(uint16_t ms);

void StartVibrate(int ms);

void StartCounting(int ms);

//Ham tao rung
void vibrate(int ms);
/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define LASER_Pin GPIO_PIN_1
#define LASER_GPIO_Port GPIOA
#define MODE_TRIGGER_Pin GPIO_PIN_12
#define MODE_TRIGGER_GPIO_Port GPIOB
#define MODE_TRIGGER_EXTI_IRQn EXTI15_10_IRQn
#define LASER_TRIGGER_Pin GPIO_PIN_13
#define LASER_TRIGGER_GPIO_Port GPIOB
#define LASER_TRIGGER_EXTI_IRQn EXTI15_10_IRQn
#define RELOAD_TRIGGER_Pin GPIO_PIN_15
#define RELOAD_TRIGGER_GPIO_Port GPIOB
#define RELOAD_TRIGGER_EXTI_IRQn EXTI15_10_IRQn
#define VIBRATION_MOTOR_Pin GPIO_PIN_9
#define VIBRATION_MOTOR_GPIO_Port GPIOA
#define SPEAKER_Pin GPIO_PIN_15
#define SPEAKER_GPIO_Port GPIOA

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
