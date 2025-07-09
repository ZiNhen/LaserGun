/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "sound.h"
#include "stdbool.h"
#include "ssd1306_fonts.h"
#include "ssd1306.h"
#include "string.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DEBOUNCE_DELAY_MS 200
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
DMA_HandleTypeDef hdma_tim3_ch4_up;

/* USER CODE BEGIN PV */
int mode = 0;
int bullets = 30;
bool out_of_ammo = false;
bool reloading = false;
uint32_t last_button_press_time = 0;
char *modes[3] = {"Mode: Single Shot", "Mode: Auto", "Mode: Burst"};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void ReloadMessage(){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(16, 19);
	ssd1306_WriteString("RELOAD", Font_16x26, White);
	ssd1306_UpdateScreen();
}

void ReloadingMessage(){
	ssd1306_Fill(Black);
	ssd1306_SetCursor(14, 23);
	ssd1306_WriteString("RELOADING", Font_11x18, White);
	ssd1306_UpdateScreen();
}

void ModeDisplay(){
	ssd1306_SetCursor(2, 8);
	ssd1306_WriteString("                    ", Font_7x10, White);
	ssd1306_SetCursor(2, 8);
	ssd1306_WriteString(modes[mode], Font_7x10, White);
	ssd1306_UpdateScreen();
}

void BulletsDisplay(){
	  ssd1306_SetCursor(24, 32);
	  ssd1306_WriteChar((bullets / 10) + 48, Font_16x26, White);
	  ssd1306_WriteChar((bullets % 10) + 48, Font_16x26, White);
	  ssd1306_WriteString("/30", Font_16x26, White);
	  ssd1306_UpdateScreen();
}

void OutOfAmmoEvent(){
	  if (reloading) return;
	  StartCounting(1000);
	  if (out_of_ammo){
		  HAL_GPIO_WritePin(VIBRATION_MOTOR_GPIO_Port, VIBRATION_MOTOR_Pin, 1);
		  PlaySound(out_of_ammo_sound, out_of_ammo_sound_length);
		  ReloadMessage();
	  }
	  else{
		  reloading = true;
		  PlaySound(reload_sound, reload_sound_length);
		  ReloadingMessage();
	  }
}

//Kich hoat am thanh
void PlaySound(const uint8_t audio_data[],const uint32_t audio_length)
{
	HAL_DMA_Abort_IT(&hdma_tim3_ch4_up);
	HAL_DMA_Start_IT(&hdma_tim3_ch4_up,(uint32_t)(audio_data+44),(uint32_t)&(TIM2->CCR1), audio_length);
	HAL_TIM_Base_Start(&htim3);
}

//Ham delay bang timer 4
void Timer_Delay_ms(uint16_t ms){
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim4, ms);// Reset counter
    HAL_TIM_Base_Start(&htim4);        // Bắt đầu timer
    while (__HAL_TIM_GET_COUNTER(&htim4) < ms);
    HAL_TIM_Base_Stop(&htim4);         // Dừng timer sau khi xong
}

void StartVibrate(int ms){
	  HAL_GPIO_WritePin(VIBRATION_MOTOR_GPIO_Port, VIBRATION_MOTOR_Pin, 1);
	  __HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
	__HAL_TIM_SET_COUNTER(&htim1, 0);
    __HAL_TIM_SET_AUTORELOAD(&htim1, ms);// Reset counter
    HAL_TIM_Base_Start_IT(&htim1);
}

void StartCounting(int ms){
		__HAL_TIM_CLEAR_FLAG(&htim1, TIM_FLAG_UPDATE);
		__HAL_TIM_SET_COUNTER(&htim1, 0);
		__HAL_TIM_SET_AUTORELOAD(&htim1, ms);// Reset counter
		HAL_TIM_Base_Start_IT(&htim1);
}

//Ham tao rung
void vibrate(int ms){
	  HAL_GPIO_WritePin(VIBRATION_MOTOR_GPIO_Port, VIBRATION_MOTOR_Pin, 1);
	  Timer_Delay_ms(ms);
	  HAL_GPIO_WritePin(VIBRATION_MOTOR_GPIO_Port, VIBRATION_MOTOR_Pin, 0);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM1)
    {
    	  ssd1306_Fill(Black);
		  ModeDisplay();
		  BulletsDisplay();
      if (!out_of_ammo){
		  HAL_GPIO_WritePin(VIBRATION_MOTOR_GPIO_Port, VIBRATION_MOTOR_Pin, 0);
    	  reloading = false;
      }
      HAL_TIM_Base_Stop_IT(htim);
    }
}

//Ham ngat khi hoan tat truyen du lieu qua DMA
void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma)
{
    if (hdma == &hdma_tim3_ch4_up){
    	HAL_DMA_Abort_IT(&hdma_tim3_ch4_up);
    	HAL_TIM_Base_Stop(&htim3);
    }
}

//Ham ngat khi nhan nut
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t now = HAL_GetTick();

	    if (now - last_button_press_time < DEBOUNCE_DELAY_MS) {
	        return;
	    }

	    last_button_press_time = now;
	if (reloading) return;
	if (GPIO_Pin == LASER_TRIGGER_Pin){
		if(!out_of_ammo){
			switch (mode){
			  case 0:
				  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
				  vibrate(50);
				  PlaySound(shoot_sound, shoot_sound_length);
				  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
				  bullets--;
				  BulletsDisplay();
				  if (bullets == 0){
					  out_of_ammo = true;
					  OutOfAmmoEvent();
				  }
				  break;
			  case 1:
				  //Mode 2: auto shot
				  while(HAL_GPIO_ReadPin(LASER_TRIGGER_GPIO_Port, LASER_TRIGGER_Pin) == 1){
					  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
					  vibrate(50);
					  PlaySound(shoot_sound, shoot_sound_length);
					  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
					  Timer_Delay_ms(50);
					  bullets--;
					  BulletsDisplay();
					  if (bullets == 0){
						  out_of_ammo = true;
						  OutOfAmmoEvent();
						  break;
					  }
				  }
				  break;
			  case 2:
				  //Mode 3: burst
				  for (int i = 0; i < 3; i++){
					  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
					  vibrate(50);
					  PlaySound(shoot_sound, shoot_sound_length);
					  HAL_GPIO_TogglePin(LASER_GPIO_Port, LASER_Pin);
					  Timer_Delay_ms(50);
					  bullets--;
					  BulletsDisplay();
					  if (bullets == 0){
						  out_of_ammo = true;
						  OutOfAmmoEvent();
						  break;
					  }
				  }
			}
		}
		else{
			  OutOfAmmoEvent();
		}
	}
	else if (GPIO_Pin == MODE_TRIGGER_Pin){
		  mode++;
		  if (mode > 2) mode = 0;
		  PlaySound(switch_sound, switch_sound_length);
		  ModeDisplay();
	}
	else if (GPIO_Pin == RELOAD_TRIGGER_Pin){

		if (bullets < 30){
			bullets = 30;
			out_of_ammo = false;
			OutOfAmmoEvent();
		}
	}
	else{
		__NOP();
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  ssd1306_Fill(Black);
  ModeDisplay();
  BulletsDisplay();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 7999;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 255;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_1); //cho phep PWM (gia lap DAC)
  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 4 - 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 125 - 1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */
  // Kích hoạt DMA trên Timer Update Event
  __HAL_TIM_ENABLE_DMA(&htim3, TIM_DMA_UPDATE); //Kich hoat DMA
  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 7999;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LASER_Pin|VIBRATION_MOTOR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LASER_Pin VIBRATION_MOTOR_Pin */
  GPIO_InitStruct.Pin = LASER_Pin|VIBRATION_MOTOR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : MODE_TRIGGER_Pin LASER_TRIGGER_Pin RELOAD_TRIGGER_Pin */
  GPIO_InitStruct.Pin = MODE_TRIGGER_Pin|LASER_TRIGGER_Pin|RELOAD_TRIGGER_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
