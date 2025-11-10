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
#include "start.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define LED_ON()			(HAL_GPIO_WritePin(OUT_StatusLED_GPIO_Port, OUT_StatusLED_Pin, GPIO_PIN_SET))
#define LED_OFF()			(HAL_GPIO_WritePin(OUT_StatusLED_GPIO_Port, OUT_StatusLED_Pin, GPIO_PIN_RESET))
#define LED_TOGGLE()		(HAL_GPIO_TogglePin(OUT_StatusLED_GPIO_Port, OUT_StatusLED_Pin))
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

TIM_HandleTypeDef htim1;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */
void start_save_data(START_DATA *data);
void start_load_data(START_DATA *data);
void start_state_changed(char new_state);
void start_programming_done(char new_stop_cmd);
void delay_no_timer(void); // Simple blocking delay
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */
  start_init(start_save_data,
             start_load_data,
             start_programming_done,
             start_state_changed);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	// Non-blocking flash for the final STOPPED state
	if (start_get_state() == START_STATE_STOPPED)
	{
	    LED_ON();
	    HAL_Delay(50); // Use HAL_Delay, it's safe in the main loop
	    LED_OFF();
	    HAL_Delay(50);
	}
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
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
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
  htim1.Init.Prescaler = 48 - 1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65536 - 1;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OUT_StatusLED_GPIO_Port, OUT_StatusLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, OUT_Start_Pin|OUT_Kill_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : OUT_StatusLED_Pin */
  GPIO_InitStruct.Pin = OUT_StatusLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OUT_StatusLED_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : IN_IR_Pin */
  GPIO_InitStruct.Pin = IN_IR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN_IR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OUT_Start_Pin OUT_Kill_Pin */
  GPIO_InitStruct.Pin = OUT_Start_Pin|OUT_Kill_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
/**
 * @brief  Delay function that does not use SysTick.
 * Used for simple blocking delays in indication functions.
 * Calibrated for 48MHz clock.
 */
void delay_no_timer(void) {
	volatile uint32_t delay = 480000; // Approx 100ms
	while(delay--) {
		__NOP();
	}

	// --- Start Module Callback Implementations ---

	/**
	 * @brief  Save start data to non-volatile memory.
	 * @note   STM32C0 has no EEPROM. You must implement Flash Emulated EEPROM here
	 * to properly save the `stop_cmd` across power cycles.
	 */
	void start_save_data(START_DATA *data) {
		/*
		 * TODO: Implement saving `data->stop_cmd` and `data->state` to Flash memory.
		 */
	    (void)data; // Suppress unused parameter warning
	}

	/**
	 * @brief  Load start data from non-volatile memory.
	 * @note   See `start_save_data`. This function should read from Flash.
	 */
	void start_load_data(START_DATA *data) {
		/*
		 * TODO: Implement loading from Flash memory.
		 *
		 * If read fails or data is invalid:
		 * data->state = START_STATE_POWER_ON;
		 * data->stop_cmd = START_DEFAULT_STOP_CMD;
		 */

	    // For now, just set defaults
		data->state = START_STATE_POWER_ON;
		data->stop_cmd = START_DEFAULT_STOP_CMD;
	}

	/**
	 * @brief  Called when the start module state changes.
	 * This is where we control the robot's output pins.
	 */
	void start_state_changed(char new_state) {
		switch(new_state) {
		case START_STATE_POWER_ON:
			LED_OFF();
			HAL_GPIO_WritePin(OUT_Start_GPIO_Port, OUT_Start_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_Kill_GPIO_Port, OUT_Kill_Pin, GPIO_PIN_RESET);
			break;

		case START_STATE_STARTED:
			LED_ON();
			HAL_GPIO_WritePin(OUT_Start_GPIO_Port, OUT_Start_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(OUT_Kill_GPIO_Port, OUT_Kill_Pin, GPIO_PIN_RESET);
			break;

		case START_STATE_STOPPED_SAFE:
			LED_TOGGLE(); // Start flashing
			HAL_GPIO_WritePin(OUT_Start_GPIO_Port, OUT_Start_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_Kill_GPIO_Port, OUT_Kill_Pin, GPIO_PIN_SET); // Set Kill pin
			break;

		case START_STATE_STOPPED:
			LED_OFF(); // LED will be flashed by main loop
			HAL_GPIO_WritePin(OUT_Start_GPIO_Port, OUT_Start_Pin, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(OUT_Kill_GPIO_Port, OUT_Kill_Pin, GPIO_PIN_SET);
			// Main loop `while(1)` will now take over flashing the LED
			break;
		}
	}

	/**
	 * @brief  Called when a new STOP command is successfully programmed.
	 * Indicate this to the user (e.g., flash LED).
	 */
	void start_programming_done(char new_stop_cmd) {
		(void)new_stop_cmd; // Suppress warning

		// Flash LED twice quickly
		LED_OFF();
		delay_no_timer();
		LED_ON();
		delay_no_timer();
		LED_OFF();
		delay_no_timer();
		LED_ON();
		delay_no_timer();
		LED_OFF();
	}


	// --- HAL Interrupt Callback Implementations ---

	/**
	 * @brief  External Line Interrupts
	 * @param  GPIO_Pin: The pin that triggered the interrupt
	 */
	void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
	{
	  if (GPIO_Pin == IN_IR_Pin)
	  {
	    // Call the ported IR state machine from ir_rc5.c
	    process_ir_pin_interrupt();
	  }
	}

	/**
	  * @brief  Timer Period Elapsed Callback
	  * @param  htim: timer handle
	  */
	void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
	{
	  if (htim->Instance == TIM1)
	  {
	    // Call the ported IR timer timeout function from ir_rc5.c
	    process_ir_timer_timeout();
	  }
	}
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
