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

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

#define NO_OF_RAMP_STEPS 64
#define CLOCKWISE -1
#define ANTICLOCKWISE 1
#define MIN_INTERVAL 100.0
#define MAX_INTERVAL 1000.0
#define F_CPU 72000000UL
#define ACTIVATED 1
#define DEACTIVATED 0
#define NUM_OF_STEPPER_MOTORS 3

#define ABSOLUTE_UPPER_LIMIT 533 //30 degrees on both sides
#define ABSOLUTE_LOWER_LIMIT 0
#define INITIAL_POSITION 267 //(ABSOLUTE_UPPER_LIMIT / 2) + 1

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

static const int TIME_CONSTANT[NO_OF_RAMP_STEPS] = {
		1000,
		1999,
		2996,
		3992,
		4985,
		5975,
		6962,
		7945,
		8923,
		9896,
		10863,
		11825,
		12779,
		13726,
		14666,
		15598,
		16520,
		17434,
		18338,
		19232,
		20115,
		20987,
		21847,
		22695,
		23531,
		24354,
		25163,
		25959,
		26741,
		27507,
		28259,
		28996,
		29716,
		30421,
		31108,
		31779,
		32433,
		33069,
		33687,
		34287,
		34869,
		35432,
		35975,
		36499,
		37004,
		37489,
		37954,
		38398,
		38822,
		39225,
		39607,
		39969,
		40309,
		40627,
		40925,
		41200,
		41454,
		41686,
		41896,
		42084,
		42251,
		42395,
		42517,
		42617
};

static const float cosine[NO_OF_RAMP_STEPS] = {
		0.9997,
		0.9988,
		0.9973,
		0.9952,
		0.9925,
		0.9892,
		0.9853,
		0.9808,
		0.9757,
		0.97,
		0.9638,
		0.9569,
		0.9495,
		0.9415,
		0.933,
		0.9239,
		0.9142,
		0.904,
		0.8932,
		0.8819,
		0.8701,
		0.8577,
		0.8448,
		0.8315,
		0.8176,
		0.8032,
		0.7883,
		0.773,
		0.7572,
		0.7409,
		0.7242,
		0.7071,
		0.6895,
		0.6715,
		0.6532,
		0.6344,
		0.6152,
		0.5957,
		0.5758,
		0.5556,
		0.535,
		0.5141,
		0.4929,
		0.4714,
		0.4496,
		0.4275,
		0.4052,
		0.3827,
		0.3599,
		0.3369,
		0.3137,
		0.2903,
		0.2667,
		0.243,
		0.2191,
		0.1951,
		0.171,
		0.1467,
		0.1224,
		0.098,
		0.0736,
		0.0491,
		0.0245,
		0
};

volatile uint8_t previousMotionComplete = 1;
volatile uint8_t speedFactor = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
/* USER CODE BEGIN PFP */
void setNextInterruptInterval(StepperMotor* motor);
void ramp(uint8_t rampCount, TIM_HandleTypeDef* timerHandle, volatile float scaleFactor);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void microsecondDelay()
{
	for (int i = 0; i < 2; i++)
	{}
}

void setDirectionS0(int direction)
{
	if (direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, RESET);
	}
}

void setDirectionS1(int direction)
{
	if (direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_3, RESET);
	}
}

void setDirectionS2(int direction)
{
	if (direction == ANTICLOCKWISE)
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, SET);
	}
	else
	{
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, RESET);
	}
}

void sendPulseS0()
{
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
	 microsecondDelay();
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_0);
}

void sendPulseS1()
{
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
	 microsecondDelay();
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_2);
}

void sendPulseS2()
{
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
	 microsecondDelay();
	 HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_4);
}


void motionComplete(StepperMotor* motor)
{
	motor->currentCount = 0;
	motor->targetCount = 0;
	motor->rampingUp = 0;
	motor->rampingDown = 0;
	motor->rampUpCount = 0;
	motor->rampDownCount = 0;
	motor->estDurationOfMovement = 0;
}

void programInit()
{
	//Timer Initializations
	motor[0].timerHandle = &htim2;
	motor[1].timerHandle = &htim3;
	motor[2].timerHandle = &htim4;

	//Direction-set function pointers
	motor[0].setDirection = setDirectionS0;
	motor[1].setDirection = setDirectionS1;
	motor[2].setDirection = setDirectionS2;

	//Set pulse pointers
	motor[0].sendPulse = sendPulseS0;
	motor[1].sendPulse = sendPulseS1;
	motor[2].sendPulse = sendPulseS2;

	newCommandAvailable = 0;
	previousMotionComplete = 1;

	for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
	{
		motionComplete(&motor[i]);
		motor[i].scaleFactor = 1;
		motor[i].direction = CLOCKWISE;

		motor[i].absolutePosition = INITIAL_POSITION;
		motor[i].pulseFlag = 0;
		motor[i].estDurationOfMovement = 0;
	}
}


void setNextInterruptInterval(StepperMotor* motor)
{
	if (abs(motor->targetCount) > 2*NO_OF_RAMP_STEPS)
	{
		//Calculate next delay
		if (motor->currentCount < NO_OF_RAMP_STEPS)
		{
			motor->rampUpCount = abs(motor->currentCount);
			ramp(motor->rampUpCount, motor->timerHandle, motor->scaleFactor);

			motor->rampingUp = ACTIVATED;
			motor->rampingDown = DEACTIVATED;
		}
		else if (motor->targetCount - motor->currentCount < NO_OF_RAMP_STEPS)
		{
			motor->rampDownCount = abs(motor->targetCount - motor->currentCount);
			ramp(motor->rampDownCount, motor->timerHandle, motor->scaleFactor);
			motor->rampingDown = ACTIVATED;
			motor->rampingUp = DEACTIVATED;
		}
		else if (motor->targetCount == motor->currentCount)
		{
			motionComplete(motor);
		}
	}
	else
	{
		if (motor->currentCount < (int)(abs(motor->targetCount)/2.0))
		{
			motor->rampUpCount = abs((motor->currentCount));
			ramp(motor->rampUpCount, motor->timerHandle, motor->scaleFactor);
			motor->rampingUp = ACTIVATED;
			motor->rampingDown = DEACTIVATED;
		}
		else if (motor->targetCount == motor->currentCount)
		{
			motionComplete(motor);
		}
		else if (motor->currentCount >= (int)(abs(motor->targetCount)/2.0))
		{
			motor->rampDownCount = abs((motor->targetCount - motor->currentCount));
			ramp(motor->rampDownCount, motor->timerHandle, motor->scaleFactor);
			motor->rampingDown = ACTIVATED;
			motor->rampingUp = DEACTIVATED;
		}
	}
}

void ramp(uint8_t rampCount, TIM_HandleTypeDef* timerHandle, volatile float scaleFactor)
{
	uint16_t nextCompareValue = (uint16_t)(((MIN_INTERVAL/1000000.0)*(F_CPU/PRESCALER) +
			((MAX_INTERVAL - MIN_INTERVAL)/1000000.0)*(F_CPU/PRESCALER)*cosine[rampCount])*scaleFactor*speedFactor);

	__HAL_TIM_SET_AUTORELOAD(timerHandle, nextCompareValue);
}

uint8_t max(volatile int* values)
{
	uint8_t max;
	if (values[0] > values[1])
		max = 0;
	else
		max = 1;
	if (values[2] > values[max])
		max = 2;

	return max;
}

void setScaleFactors()
{

	volatile int values[3] = {motor[0].estDurationOfMovement, motor[1].estDurationOfMovement, motor[2].estDurationOfMovement};
	uint8_t maxMotorIndex = max(&values[0]);

	/*
	 * Perform check to ensure that Timer Compare Register does not overflow after scaling.
	 * Disables scaling in such a scenario - likely when scaling factor is too large -
	 * if 65000 < Initial Period * ScalingFactor
	 */

	for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
	{
		if (motor[i].estDurationOfMovement != 0)
		{
			motor[i].scaleFactor = (float)motor[maxMotorIndex].estDurationOfMovement/(float)motor[i].estDurationOfMovement;

			if ((MAX_INTERVAL/1000000.0)*(F_CPU/PRESCALER)*motor[i].scaleFactor > 65000)
				motor[i].scaleFactor = 1;
		}
	}
	MX_TIM2_Init();
	MX_TIM3_Init();
	MX_TIM4_Init();
}

float getDurationOfUninterruptedMovement(int numOfSteps)
{
	float duration = 0;
	if (numOfSteps > 64)
	{
		duration = (2.0 * (float)TIME_CONSTANT[NO_OF_RAMP_STEPS - 1] / 1000.0) + (((float)numOfSteps - (2 *NO_OF_RAMP_STEPS)) * (float)MIN_INTERVAL/1000.0);
	}
	else
	{
		duration = 2.0 * TIME_CONSTANT[(numOfSteps/2) - 1] / 1000;
	}

	return duration;
}

void runMotor(StepperMotor* motor)
{
	 motor->sendPulse();
	 motor->pulseFlag = 0;
	 motor->currentCount ++;
	 motor->absolutePosition += motor->direction;

	 if (motor->currentCount == motor->targetCount)
	 {
		 motionComplete(motor);
	 }
	 else if (motor->currentCount <= NO_OF_RAMP_STEPS ||
			 abs(motor->targetCount - motor->currentCount) <= NO_OF_RAMP_STEPS)
	 {
		 setNextInterruptInterval(motor);
	 }
}

void copyNewCommands()
{
	for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
	{
		motor[i].newAbsoluteTarget = motor[i].newAbsoluteTargetUSB;
	}
}

void configForNewCommand(StepperMotor* motor)
{
		//Define motion

		int16_t newTarget = abs(motor->absolutePosition - motor->newAbsoluteTarget);
		motor->estDurationOfMovement = getDurationOfUninterruptedMovement(newTarget);
		motor->targetCount = newTarget;
}

uint8_t performDataValidation()
{
	//Prevents motors from moving if incoming data is invalid or outside limits set by ABSOLUTE_UPPER_LIMIT and ABSOLUTE_LOWER_LIMIT
	volatile uint8_t valid;
	valid = 1;

	//Set Direction
	for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
	{
		if (motor[i].absolutePosition > motor[i].newAbsoluteTarget)
		{
			motor[i].newDirection = CLOCKWISE;
			motor[i].direction = CLOCKWISE;
			motor[i].setDirection(CLOCKWISE);
		}
		else if (motor->absolutePosition < motor->newAbsoluteTarget)
		{
			motor[i].newDirection = ANTICLOCKWISE;
			motor[i].direction = ANTICLOCKWISE;
			motor[i].setDirection(ANTICLOCKWISE);
		}

		if ((motor[i].direction) * abs(motor[i].absolutePosition - motor[i].newAbsoluteTarget) +  motor[i].absolutePosition > ABSOLUTE_UPPER_LIMIT
				|| (motor[i].direction) * abs(motor[i].absolutePosition - motor[i].newAbsoluteTarget) +  motor[i].absolutePosition < ABSOLUTE_LOWER_LIMIT)
		{
			valid = 0;
			break;
		}
	}

	return valid;
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

	programInit();
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

  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if (newCommandAvailable == 1 && previousMotionComplete == 1)
	  {
		  	newCommandAvailable = DEACTIVATED;
		  	previousMotionComplete = DEACTIVATED;

		  	copyNewCommands();

		  	//Perform check to ensure incoming data is valid
		  	uint8_t check = performDataValidation();
		  	if (check == 1)
		  	{
			  	for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
			  	{
			  		configForNewCommand(&motor[i]);
			  	}

				setScaleFactors();
				HAL_TIM_Base_Start_IT(&htim2);
				HAL_TIM_Base_Start_IT(&htim3);
				HAL_TIM_Base_Start_IT(&htim4);

		  	}
      }

		for (uint8_t i = 0; i < NUM_OF_STEPPER_MOTORS; i++)
		{
			 if (motor[i].pulseFlag == 1)
			 {
				 runMotor(&motor[i]);
			 }
		}

		if (motor[0].targetCount == 0 && motor[1].targetCount == 0 && motor[2].targetCount == 0)
		{
			previousMotionComplete = ACTIVATED;
		}


		/*if (previousMotionComplete == 1 && temp == 0)
		{
			  motor[0].newAbsoluteTargetUSB = 30;
			  motor[1].newAbsoluteTargetUSB = 30;
			  motor[2].newAbsoluteTargetUSB = -10;

			 newCommandAvailable = ACTIVATED;
			 temp = 1;
		}*/
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
  htim2.Init.Prescaler = PRESCALER;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = (uint16_t)(((MIN_INTERVAL/1000000.0)*(F_CPU/PRESCALER) +
			((MAX_INTERVAL - MIN_INTERVAL)/1000000.0)*(F_CPU/PRESCALER)*cosine[1])*motor[0].scaleFactor);
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim3.Init.Prescaler = PRESCALER;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = (uint16_t)(((MIN_INTERVAL/1000000.0)*(F_CPU/PRESCALER) +
			((MAX_INTERVAL - MIN_INTERVAL)/1000000.0)*(F_CPU/PRESCALER)*cosine[1])*motor[1].scaleFactor);
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  htim4.Init.Period = (uint16_t)(((MIN_INTERVAL/1000000.0)*(F_CPU/PRESCALER) +
			((MAX_INTERVAL - MIN_INTERVAL)/1000000.0)*(F_CPU/PRESCALER)*cosine[1])*motor[2].scaleFactor);
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3 
                           PA4 PA5 PA6 PA7 
                           PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3 
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7 
                          |GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
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
