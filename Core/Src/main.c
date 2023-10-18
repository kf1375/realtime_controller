/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "cmsis_os.h"

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

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

RTC_HandleTypeDef hrtc;

UART_HandleTypeDef huart4;

/* Definitions for LEDTask */
osThreadId_t LEDTaskHandle;
const osThreadAttr_t LEDTask_attributes = {
  .name = "LEDTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
/* Definitions for buttonTask */
osThreadId_t buttonTaskHandle;
const osThreadAttr_t buttonTask_attributes = {
  .name = "buttonTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* Definitions for SleepTask */
osThreadId_t SleepTaskHandle;
const osThreadAttr_t SleepTask_attributes = {
  .name = "SleepTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for LowPowerTask */
osThreadId_t LowPowerTaskHandle;
const osThreadAttr_t LowPowerTask_attributes = {
  .name = "LowPowerTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
/* Definitions for LEDSemaphore */
osSemaphoreId_t LEDSemaphoreHandle;
const osSemaphoreAttr_t LEDSemaphore_attributes = {
  .name = "LEDSemaphore"
};
/* USER CODE BEGIN PV */
uint8_t buttonPressCount = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_UART4_Init(void);
static void MX_RTC_Init(void);
void StartLEDTask(void *argument);
void SartButtonTask(void *argument);
void StartSleepTask(void *argument);
void StartLowPowerTask(void *argument);

/* USER CODE BEGIN PFP */
void JumpToBootloader(void);

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
  MX_ADC1_Init();
  MX_UART4_Init();
  MX_RTC_Init();
  /* USER CODE BEGIN 2 */
  uint32_t buttonPressTime = 0;  // Time the button was pressed

  /* Loop for checking button state during power on */
  while(1)
  {
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	  {
		  buttonPressTime = HAL_GetTick();
		  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET); // Wait for the button to be released
	  }
	  if((HAL_GetTick() - buttonPressTime) >= 5000)
      {
    	  JumpToBootloader();
      }
	  break;
  }


  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* creation of LEDSemaphore */
  LEDSemaphoreHandle = osSemaphoreNew(1, 1, &LEDSemaphore_attributes);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of LEDTask */
  LEDTaskHandle = osThreadNew(StartLEDTask, NULL, &LEDTask_attributes);

  /* creation of buttonTask */
  buttonTaskHandle = osThreadNew(SartButtonTask, NULL, &buttonTask_attributes);

  /* creation of SleepTask */
  SleepTaskHandle = osThreadNew(StartSleepTask, NULL, &SleepTask_attributes);

  /* creation of LowPowerTask */
  LowPowerTaskHandle = osThreadNew(StartLowPowerTask, NULL, &LowPowerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_OCTOBER;
  sDate.Date = 0x1;
  sDate.Year = 0x1;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* Function to perform jump to system memory boot from user application */
void JumpToBootloader(void) {
	void (*SysMemBootJump)(void);

	/* Set system memory address. For STM32F446, system memory is on 0x1FFF0000 */
	volatile uint32_t addr = 0x1FFF0000;

	/* Disable RCC, set it to default (after reset) settings */
	HAL_RCC_DeInit();

	/* Disable systick timer and reset it to default values */
	SysTick->CTRL = 0;
	SysTick->LOAD = 0;
	SysTick->VAL = 0;

	/* Disable all interrupts */
	__disable_irq();

	/* Remap system memory to address 0x0000 0000 in address space */
	__HAL_SYSCFG_REMAPMEMORY_SYSTEMFLASH();

	/* Set jump memory location for system memory. Use address with 4 bytes offset which specifies jump location where program starts */
	SysMemBootJump = (void (*)(void)) (*((uint32_t *)(addr + 4)));

	/* Set main stack pointer */
	__set_MSP(*(uint32_t *)addr);

	/* call our function to jump to set location. This will start system memory execution */
	SysMemBootJump();

	/*
	 * Connect USB<->UART converter to dedicated USART1 or USART3 pins and test bootloader
	 */
}

/* Button Pin External Interrupt Callback */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	/*
	 * When the button pressed the device switched
	 * to Normal Operation mode
	 */

	/* Check for exact pin */
    if(GPIO_Pin == B1_Pin)
    {
    	/* Notify buttonTask and default LEDTask */
    	osThreadFlagsSet(buttonTaskHandle, 1);
    	osThreadFlagsSet(LEDTaskHandle, 1);
    }
}

/* Real Time Clock Wake Up Timer Callaback */
void HAL_RTCEx_WakeUpTimerEventCallback(RTC_HandleTypeDef *hrtc)
{
	/* Reconfigure SysClock after wake up */
	SystemClock_Config();
	/* Resume HAL Tick Timer which is TIM5 in this project */
	HAL_ResumeTick();
	/* Deactivate RTC Wake Up Timer Interrrupt */
	HAL_RTCEx_DeactivateWakeUpTimer(hrtc);
	/* Notify LowPowerTask to perform its job every 10s */
	osThreadFlagsSet(LowPowerTaskHandle, 1);
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartLEDTask */
/**
  * @brief  Function implementing the LEDTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartLEDTask */
void StartLEDTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Initilize Task state in start-up */
  osThreadFlagsSet(LEDTaskHandle, 1);
  /* Infinite loop */
  for(;;)
  {
	  /* Waiting for notification */
	  osThreadFlagsWait(1, osFlagsNoClear, osWaitForever);
	  /* Toggling the LED Every 600ms */
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
	  /* Set Threads Flag to make loop  */
	  osThreadFlagsSet(LEDTaskHandle, 1);
	  osDelay(600);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_SartButtonTask */
/**
* @brief Function implementing the buttonTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_SartButtonTask */
void SartButtonTask(void *argument)
{
  /* USER CODE BEGIN SartButtonTask */

  /* Infinite loop */
  for(;;)
  {
	  /* Waiting for notification */
	  osThreadFlagsWait(1, osFlagsWaitAll, osWaitForever);
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	  {
		  /* Suspend low power task */
		  osThreadSuspend(LowPowerTaskHandle);
		  /* Deactivate RTC Wake Up Timer Interrrupt */
	      HAL_RTCEx_DeactivateWakeUpTimer(&hrtc);
		  /* Resume default LED task */
		  if(osThreadGetState(LEDTaskHandle) == osThreadBlocked)
		  {
			  osThreadResume(LEDTaskHandle);
		  }
		  uint32_t pressDuration = 0;
		  // Calculate Button Press Duration
		  while(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		  {
			  osDelay(50);
			  pressDuration += 50;
		  }
		  /* Check Press Situatiom */
		  if(pressDuration >= 5000) {
			  osThreadSuspend(LEDTaskHandle);
			  // Long press (5 seconds)
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  /* 5 time 5Hz blinks */
			  for(int i = 0; i < 5; i++)
			  {
				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				  osDelay(100);
				  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
				  osDelay(100);
			  }
			  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
			  /* Notify SleepTask */
			  osThreadFlagsSet(SleepTaskHandle, 1);
		  }
		  else if(pressDuration >= 250 && pressDuration <= 1000)
		  {
			  // Short press (between 250ms and 1 second)
			  buttonPressCount++;
			  if (buttonPressCount == 3)
			  {
				  /* Read ADC and send it over UART4 */
				  buttonPressCount = 0;
				  // Start the adc
				  HAL_ADC_Start(&hadc1);
				  // polling for new adc convertion
				  HAL_ADC_PollForConversion(&hadc1, 1);
				  // get the adc value
				  uint32_t adc_val = HAL_ADC_GetValue(&hadc1);
				  // stop adc
				  HAL_ADC_Stop(&hadc1);
				  // sending adc value over UART4
				  HAL_UART_Transmit(&huart4, (uint8_t *) &adc_val, 4, 100);
			  }
		  }
	  }
  }
  /* USER CODE END SartButtonTask */
}

/* USER CODE BEGIN Header_StartSleepTask */
/**
* @brief Function implementing the SleepTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSleepTask */
void StartSleepTask(void *argument)
{
  /* USER CODE BEGIN StartSleepTask */
  /* Infinite loop */
  for(;;)
  {
	  /*
	  * Before going to sleep mode all normal task should switch to
	  * IDLE state
	  */

	  /* Waiting for notification */
	  osThreadFlagsWait(1, osFlagsWaitAll, osWaitForever);

	  /* Perform pre sleep configuration */
	  osThreadFlagsSet(LowPowerTaskHandle, 0);
	  osThreadFlagsSet(buttonTaskHandle, 0);
	  osThreadResume(LowPowerTaskHandle);
	  // Ensure that led is off
	  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
	  // Deactivate Systick
	  HAL_SuspendTick();
	  // Activate RTC Wake Up Interrupt
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x4E20, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	  /* Enter Sleep Mode */
	  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  /* USER CODE END StartSleepTask */
}

/* USER CODE BEGIN Header_StartLowPowerTask */
/**
* @brief Function implementing the LowPowerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLowPowerTask */
void StartLowPowerTask(void *argument)
{
  /* USER CODE BEGIN StartLowPowerTask */
  /* Infinite loop */
  for(;;)
  {
	  /* Waiting for notification */
	  osThreadFlagsWait(1, osFlagsWaitAll, osWaitForever);
	  /* 3 time 5Hz blinks */
	  for(int i = 0; i < 3; i++)
	  {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  osDelay(100);
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  osDelay(100);
	  }
	  /* Reactivate Wake Up Timer to blink LED every 10s */
	  HAL_SuspendTick();
	  HAL_RTCEx_SetWakeUpTimer_IT(&hrtc, 0x4E20, RTC_WAKEUPCLOCK_RTCCLK_DIV16);
	  HAL_PWR_EnterSLEEPMode(PWR_LOWPOWERREGULATOR_ON, PWR_SLEEPENTRY_WFI);
  }
  /* USER CODE END StartLowPowerTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM5 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

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
