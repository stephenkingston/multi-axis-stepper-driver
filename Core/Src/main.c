/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdbool.h>
#include "DWT_Delay.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NO_OF_RAMP_STEPS 32
#define CLOCKWISE -1
#define ANTICLOCKWISE 1
#define MIN_INTERVAL 15.0
#define MAX_INTERVAL 1000.0
#define F_CPU 72000000UL

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
uint8_t rampingUp = 0;
uint8_t rampingDown = 0;
uint8_t rampUpCount = 0;
uint8_t rampDownCount = 0;

static const float sine[32] = {
		1.0000,
		0.9986,
		0.9945,
		0.9877,
		0.9781,
		0.9659,
		0.9510,
		0.9336,
		0.9135,
		0.8910,
		0.8660,
		0.8387,
		0.8090,
		0.7771,
		0.7431,
		0.7071,
		0.6691,
		0.6293,
		0.5878,
		0.5446,
		0.5000,
		0.4540,
		0.4067,
		0.3584,
		0.3090,
		0.2588,
		0.2079,
		0.1564,
		0.1045,
		0.0523,
		0.0175,
		0.0000
};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void setNextInterruptInterval(StepperMotor* motor);
void ramp(uint8_t rampCount);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void setDirectionS0(int direction)
{
	if (direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	}
}

void motionComplete(StepperMotor* motor)
{
	motor->currentCount = 0;
	motor->targetCount = 0;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	motor[0].newCommandAvailable = 0;

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
  MX_USB_DEVICE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  motor[0].setDirection = setDirectionS0;
  motor[0].absolutePosition = 10;
  int absolutePositionRequested = 50;

  if (motor[0].absolutePosition > absolutePositionRequested)
  {
	  motor[0].direction = CLOCKWISE;
	  motor[0].setDirection(CLOCKWISE);
  }
  else if (motor[0].absolutePosition < absolutePositionRequested)
  {
	  motor[0].direction = ANTICLOCKWISE;
	  motor[0].setDirection(ANTICLOCKWISE);
  }
  motor[0].targetCount = abs(absolutePositionRequested - motor[0].absolutePosition);
  motor[0].currentCount = 0;


  HAL_TIM_Base_Start_IT(&htim2);
  //DWT_Delay(2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	 if (motor[0].pulseFlag == 1)
	 {
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		 motor[0].pulseFlag = 0;
		 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
		 motor[0].currentCount ++;
		 motor[0].absolutePosition += motor[0].direction;
		 if (motor[0].newCommandAvailable)
		 {
			 //Set Direction
			 if (motor[0].absolutePosition > motor[0].newAbsoluteTarget)
			 {
				 motor[0].newDirection = CLOCKWISE;
			 }
			 else if (motor[0].absolutePosition < motor[0].newAbsoluteTarget)
			 {
				 motor[0].newDirection = ANTICLOCKWISE;
			 }

			 //Define motion
			 if (motor[0].direction == motor[0].newDirection)
			 {
				 if(rampingDown == 1)
				 {
					//Stop ramp-down and initiate ramp up with the same number of rampDown steps done
					 motor[0].targetCount = abs(motor[0].absolutePosition - motor[0].newAbsoluteTarget);
					 motor[0].currentCount = rampDownCount;
					 motor[0].newCommandAvailable = 0;
				 }
				 else
				 {
					 motor[0].targetCount += abs(motor[0].absolutePosition - motor[0].newAbsoluteTarget);
					 //Same direction
					 motor[0].newCommandAvailable = 0;
				 }
			 }
			 else
			 {
				 if(rampingDown == 1)
				 {
					 	//Ignore and allow the ramp-down to continue
				 }
				 if (rampingUp == 1)
				 {
					 //Start ramp-down sequence with the same number of steps that has currently been ramped up
					 motor[0].targetCount = 2*rampUpCount;
					 motor[0].currentCount = rampUpCount;
				 }
				 else if (rampingUp == 0 && rampingDown == 0)
				 {
					 motor[0].targetCount = abs(motor[0].absolutePosition - motor[0].newAbsoluteTarget);
					 motor[0].direction = motor[0].newDirection;
					 motor[0].newCommandAvailable = 0;
				 }
			 }
		 }
		 else if (motor[0].currentCount <= NO_OF_RAMP_STEPS || motor[0].targetCount - motor[0].currentCount <= NO_OF_RAMP_STEPS)
		 {
			 setNextInterruptInterval(&motor[0]);
		 }
	 }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

void setNextInterruptInterval(StepperMotor* motor)
{
	if (abs(motor->targetCount) > 64)
	{
	//Calculate next delay
		if (motor->currentCount < NO_OF_RAMP_STEPS)
		{
			rampUpCount = abs(motor->currentCount);
			ramp(rampUpCount);
			rampingUp = 1;
			rampingDown = 0;
		}
		else if (motor->targetCount - motor->currentCount < NO_OF_RAMP_STEPS)
		{
			rampDownCount = abs(motor->targetCount - motor->currentCount);
			ramp(rampDownCount);
			rampingDown = 1;
			rampingUp = 0;
		}
		else if (motor->targetCount == motor->currentCount)
		{
			motionComplete(motor);
		}
		else
		{
			rampingUp = 0;
			rampingDown = 0;
			rampUpCount = 0;
			rampDownCount = 0;
		}
	}
	else
	{
		if (motor[0].currentCount < (int)(abs(motor->targetCount)/2.0))
		{
			rampUpCount = abs((motor->currentCount));
			ramp(rampUpCount);
			rampingUp = 1;
			rampingDown = 0;
		}
		else if (motor[0].currentCount > (int)(abs(motor->targetCount)/2.0))
		{
			rampDownCount = abs((motor->targetCount - motor->currentCount));
			ramp(rampDownCount);
			rampingDown = 1;
			rampingUp = 0;
		}
		else if (motor->targetCount == motor->currentCount)
		{
			motionComplete(motor);
		}
		else
		{
			rampingUp = 0;
			rampingDown = 0;
			rampUpCount = 0;
			rampDownCount = 0;
		}
	}
}

void ramp(uint8_t rampCount)
{
	uint16_t nextCompareValue = (uint16_t)((MIN_INTERVAL/1000000.0)*(F_CPU/PRESCALER) +
			((MAX_INTERVAL - MIN_INTERVAL)/1000000.0)*(F_CPU/PRESCALER)*sine[rampCount]);

	__HAL_TIM_SET_AUTORELOAD(&htim2, nextCompareValue);
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 256;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 3;
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
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  htim3.Init.Prescaler = 256;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 286;
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
  htim4.Init.Prescaler = PRESCALER;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 286;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
