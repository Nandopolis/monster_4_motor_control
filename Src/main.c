/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */
#define ARM_MATH_CM3
#include "arm_math.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;

osThreadId ControlTaskHandle;
uint32_t ControlTaskBuffer[ 512 ];
osStaticThreadDef_t ControlTaskControlBlock;
osThreadId SysIdTaskHandle;
uint32_t SysIdTaskBuffer[ 1024 ];
osStaticThreadDef_t SysIdTaskControlBlock;
osThreadId I2CTaskHandle;
uint32_t I2CTaskBuffer[ 128 ];
osStaticThreadDef_t I2CTaskControlBlock;
osSemaphoreId SysIdBinarySemHandle;
osStaticSemaphoreDef_t SysIdBinarySemControlBlock;
osSemaphoreId ControlBinarySemHandle;
osStaticSemaphoreDef_t ControlBinarySemControlBlock;
osSemaphoreId i2cBinarySemHandle;
osStaticSemaphoreDef_t i2cBinarySemControlBlock;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/
enum MOTOR_enum {
	MOTOR_B_R,
	MOTOR_B_L,
	MOTOR_F_R,
	MOTOR_F_L,
	
	MOTOR_MAX,
	SCAN_MAX,
};
enum MONSTER_K {
	K_B,
	K_A,
	
	K_MAX,
};
enum W_PARAMS {
	THETA,
	A,
	B,
	
	PARAMS_MAX,
};
enum PID {
	P,
	I,
	D,
	
	PID_MAX
};
float32_t bemf_mV[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
float32_t d_global[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};	// vel estimation in rad/s
float32_t r_global[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};	// vel desired in rad/s
float32_t w_n[MOTOR_MAX][2] =	{	{0.188, 0.776}, \
																{0.75, 0.2}, \
																{0.75, 0.2}, \
																{0.75, 0.2}	};
float32_t pid_k[MOTOR_MAX][PID_MAX] = {	{1.0, 75.0, 0.005}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}	};
float32_t pid_e[MOTOR_MAX][PID_MAX] = {	{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}, \
																				{0.0, 0.0, 0.0}	};
float32_t y_est[MOTOR_MAX], error;
uint16_t r_motor_mOhm[MOTOR_MAX] = {6500, 5650, 0, 0};
uint16_t monster_k[MOTOR_MAX][K_MAX] = {{2108, 4322}, {0, 0}, {0, 0}, {0, 0}};
uint16_t r_sense_Ohm = 1500;
volatile uint16_t adc_scan[SCAN_MAX] = {0, 0, 0, 0};
volatile uint16_t adc_scan_avg[SCAN_MAX] = {0, 0, 0, 0};
float32_t u_n_global[MOTOR_MAX][4] = {{0.0, 0.0, 0.0, 0.0}, \
																			{0.0, 0.0, 0.0, 0.0}, \
																			{0.0, 0.0, 0.0, 0.0}, \
																			{0.0, 0.0, 0.0, 0.0}};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
void StartControlTask(void const * argument);
void StartSysIdTask(void const * argument);
void StartI2CTask(void const * argument);

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

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
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();

  /* USER CODE BEGIN 2 */
	
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* Create the semaphores(s) */
  /* definition and creation of SysIdBinarySem */
  osSemaphoreStaticDef(SysIdBinarySem, &SysIdBinarySemControlBlock);
  SysIdBinarySemHandle = osSemaphoreCreate(osSemaphore(SysIdBinarySem), 1);

  /* definition and creation of ControlBinarySem */
  osSemaphoreStaticDef(ControlBinarySem, &ControlBinarySemControlBlock);
  ControlBinarySemHandle = osSemaphoreCreate(osSemaphore(ControlBinarySem), 1);

  /* definition and creation of i2cBinarySem */
  osSemaphoreStaticDef(i2cBinarySem, &i2cBinarySemControlBlock);
  i2cBinarySemHandle = osSemaphoreCreate(osSemaphore(i2cBinarySem), 1);

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of ControlTask */
  osThreadStaticDef(ControlTask, StartControlTask, osPriorityHigh, 0, 512, ControlTaskBuffer, &ControlTaskControlBlock);
  ControlTaskHandle = osThreadCreate(osThread(ControlTask), NULL);

  /* definition and creation of SysIdTask */
  osThreadStaticDef(SysIdTask, StartSysIdTask, osPriorityNormal, 0, 1024, SysIdTaskBuffer, &SysIdTaskControlBlock);
  SysIdTaskHandle = osThreadCreate(osThread(SysIdTask), NULL);

  /* definition and creation of I2CTask */
  osThreadStaticDef(I2CTask, StartI2CTask, osPriorityIdle, 0, 128, I2CTaskBuffer, &I2CTaskControlBlock);
  I2CTaskHandle = osThreadCreate(osThread(I2CTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
 

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

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
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
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 5;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_7CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = 3;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = 4;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* I2C1 init function */
static void MX_I2C1_Init(void)
{

  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 30;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_ENABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 8191;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 35999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
        * Free pins are configured automatically as Analog (this feature is enabled through 
        * the Code Generation settings)
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_Pin|INA_B_R_Pin|INB_B_R_Pin|INA_B_L_Pin 
                          |INB_B_L_Pin|INA_F_L_Pin|INB_F_R_Pin|INA_F_R_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(INB_F_L_GPIO_Port, INB_F_L_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA5 PA6 PA7 PA12 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DIAG_F_R_Pin DIAG_F_L_Pin DIAG_B_R_Pin DIAG_B_L_Pin 
                           BUT_Pin */
  GPIO_InitStruct.Pin = DIAG_F_R_Pin|DIAG_F_L_Pin|DIAG_B_R_Pin|DIAG_B_L_Pin 
                          |BUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_Pin INA_B_R_Pin INB_B_R_Pin INA_B_L_Pin 
                           INB_B_L_Pin INA_F_L_Pin INB_F_R_Pin INA_F_R_Pin */
  GPIO_InitStruct.Pin = LED_Pin|INA_B_R_Pin|INB_B_R_Pin|INA_B_L_Pin 
                          |INB_B_L_Pin|INA_F_L_Pin|INB_F_R_Pin|INA_F_R_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : INB_F_L_Pin */
  GPIO_InitStruct.Pin = INB_F_L_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(INB_F_L_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/**
  * @brief  Conversion complete callback in non blocking mode
  * @param  AdcHandle : AdcHandle handle
  * @note   This example shows a simple way to report end of conversion
  *         and get conversion result. You can add your own implementation.
  * @retval None
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *AdcHandle)
{
	volatile static uint8_t cnt = 0;
	volatile static uint16_t adc_scan_sum[SCAN_MAX] = {0, 0, 0, 0};
	
	if (AdcHandle->Instance == ADC1) {
		for (uint8_t i = 0; i < SCAN_MAX; i++)
			adc_scan_sum[i] += adc_scan[i];
		cnt++;
		if (cnt >= 10) {
			for (uint8_t i = 0; i < SCAN_MAX; i++) {
				adc_scan_avg[i] = adc_scan_sum[i] / cnt;
				adc_scan_sum[i] = 0;
			}
			cnt = 0;
			osSemaphoreRelease(ControlBinarySemHandle);
		}
	}
}

GPIO_TypeDef *getPort(uint8_t i) {
	switch(i) {
		case MOTOR_B_R:
			return INA_B_R_GPIO_Port;
		case MOTOR_B_L:
			return INA_B_L_GPIO_Port;
		case MOTOR_F_R:
			return INA_F_R_GPIO_Port;
		case MOTOR_F_L:
			return INA_F_L_GPIO_Port;
		default:
			return 0;
	}
}

uint16_t getPin(uint8_t i) {
	switch(i) {
		case MOTOR_B_R:
			return INA_B_R_Pin;
		case MOTOR_B_L:
			return INA_B_L_Pin;
		case MOTOR_F_R:
			return INA_F_R_Pin;
		case MOTOR_F_L:
			return INA_F_L_Pin;
		default:
			return 0;
	}
}

/* USER CODE END 4 */

/* StartControlTask function */
void StartControlTask(void const * argument)
{

  /* USER CODE BEGIN 5 */
	uint8_t i, j;
	float32_t aux1, aux2, aux3, aux4;
	float32_t u[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
	float32_t r[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
	float32_t a = 0.01 / (0.01 + 0.05);
	uint16_t pwm_value[MOTOR_MAX] = {0, 0, 0, 0};
	float32_t v_adc_mV[SCAN_MAX];
	float32_t v_bat_mV;
	float32_t i_motor_mA[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
	// float32_t bemf_mV[MOTOR_MAX] = {0.0, 0.0, 0.0, 0.0};
	
	HAL_ADCEx_Calibration_Start(&hadc1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
	HAL_TIM_Base_Start_IT(&htim3);
	
	HAL_GPIO_WritePin(INA_B_R_GPIO_Port, INA_B_R_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(INB_B_R_GPIO_Port, INB_B_R_Pin, GPIO_PIN_RESET);
	PWM_B_R = pwm_value[MOTOR_B_R];
	
  /* Infinite loop */
  for(;;)
  {
		osSemaphoreWait(ControlBinarySemHandle, osWaitForever);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		
		// data update
		for (i = 0; i < MOTOR_MAX; i++) {
			u_n_global[i][1] = d_global[i];
			u_n_global[i][0] = u[i];
		}
		
		// data measurement
		for (i = 0; i < SCAN_MAX; i++) {
			v_adc_mV[i] = adc_scan_avg[i] * 3300.0 / 4095;
		}
		v_bat_mV = v_adc_mV[MOTOR_MAX] * 13.16 / 3.3;
		
		// control
		if (HAL_GPIO_ReadPin(BUT_GPIO_Port, BUT_Pin) == GPIO_PIN_SET) 
			r_global[MOTOR_B_R] = 879.0;
		else
			r_global[MOTOR_B_R] = 293.0;
		
		for (i = 0; i < MOTOR_MAX; i++) {
			aux1 = d_global[i];
			
			i_motor_mA[i] = v_adc_mV[i] / r_sense_Ohm;
			i_motor_mA[i] *= monster_k[i][HAL_GPIO_ReadPin(getPort(i), getPin(i))];
			bemf_mV[i] = v_bat_mV * pwm_value[i] / 8191 - i_motor_mA[i];
			bemf_mV[i] -= i_motor_mA[i] * r_motor_mOhm[i] / 1000;
			d_global[i] = bemf_mV[i] * 2 * PI / 60;
			arm_dot_prod_f32(w_n[i], u_n_global[i], 2, &y_est[i]);
			
			r[i] = (1 - a) * r[i] + a * r_global[i];
			
			pid_e[i][P] = r[i] - y_est[i];
			u[i] = pid_k[i][P] * pid_e[i][P];
			pid_e[i][D] = (aux1 - d_global[i]) / 0.01;
			u[i] += pid_k[i][D] * pid_e[i][D];
			pid_e[i][I] += pid_e[i][P] * 0.01;
			aux2 = pid_k[i][I] * pid_e[i][I];
			if (aux2 > 1172.0) {
				pid_e[i][I] = 1172.0;
			}
			if (aux2 < 0.0) {
				pid_e[i][I] = 0.0;
			}
			u[i] += aux2;
			
			aux3 = d_global[i] * 1.0;
			if (u[i] > 1172.0) {
				u[i] = 1172.0;
			}
			if (u[i] < aux3) {
				u[i] = aux3;
			}
			// u[i] = r[i];
			pwm_value[MOTOR_B_R] = (uint16_t)(u[MOTOR_B_R] * 8191 / 1172.0);
		}
		
		PWM_B_R = pwm_value[MOTOR_B_R];

		j++;
		if (j == 5) {
			j = 0;
			osSemaphoreRelease(SysIdBinarySemHandle);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END 5 */ 
}

/* StartSysIdTask function */
void StartSysIdTask(void const * argument)
{
  /* USER CODE BEGIN StartSysIdTask */
	float32_t P_n[MOTOR_MAX][4] = {	{	10.0,	0.0,			\
																		0.0, 	10.0	},	\
																	{	10.0,	0.0,			\
																		0.0, 	10.0	},	\
																	{	10.0,	0.0,			\
																		0.0, 	10.0	},	\
																	{	10.0,	0.0,			\
																		0.0, 	10.0,	}	};
	float32_t u_n[MOTOR_MAX][2], k_n[2], aux41[2], aux14[2], aux44[4];
	float32_t a = 0.999, aux, e_n, y_n, d[MOTOR_MAX];
	float32_t ainv = 1/a;
	uint8_t i, j;

	arm_matrix_instance_f32 mP_n[MOTOR_MAX];
	arm_matrix_instance_f32 mk_n;
	arm_matrix_instance_f32 mu_n[MOTOR_MAX];
	arm_matrix_instance_f32 muT_n[MOTOR_MAX];
	arm_matrix_instance_f32 maux41;
	arm_matrix_instance_f32 maux14;
	arm_matrix_instance_f32 maux44;
	
	for (i = 0; i < MOTOR_MAX; i++) {
		arm_mat_init_f32(&mP_n[i], 2, 2, P_n[i]);
		arm_mat_init_f32(&mu_n[i], 2, 1, u_n[i]);
		arm_mat_init_f32(&muT_n[i], 1, 2, u_n[i]);
	}
	arm_mat_init_f32(&mk_n, 2, 1, k_n);
	arm_mat_init_f32(&maux41, 2, 1, aux41);
	arm_mat_init_f32(&maux14, 1, 2, aux14);
	arm_mat_init_f32(&maux44, 2, 2, aux44);
	
  /* Infinite loop */
  for(;;)
  {
    osSemaphoreWait(SysIdBinarySemHandle, osWaitForever);
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
		for (i = 0; i < MOTOR_MAX; i++) {
			for (j = 0; j < 2; j++) {
				u_n[i][j] = u_n_global[i][j];
			}
			d[i] = d_global[i];
			
			//RLS motor i
			arm_mat_mult_f32(&mP_n[i], &mu_n[i], &maux41);
			arm_dot_prod_f32(u_n[i], aux41, 2, &aux);
			aux = 1/(a + aux);
			arm_scale_f32(aux41, aux, k_n, 2);
			
			arm_dot_prod_f32(w_n[i], u_n[i], 2, &y_n);
			
			e_n = d[i] - y_n;
			
			arm_scale_f32(k_n, e_n, aux41, 2);
			arm_add_f32(w_n[i], aux41, w_n[i], 2);
			
			arm_mat_mult_f32(&muT_n[i], &mP_n[i], &maux14);
			arm_mat_mult_f32(&mk_n, &maux14, &maux44);
			arm_mat_sub_f32(&mP_n[i], &maux44, &maux44);
			arm_mat_scale_f32(&maux44, ainv, &mP_n[i]);
		}
		HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
  }
  /* USER CODE END StartSysIdTask */
}

/* StartI2CTask function */
void StartI2CTask(void const * argument)
{
  /* USER CODE BEGIN StartI2CTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartI2CTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM4 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */
	// TIM3 callback, 1000Hz interrupt (sampling frec)
	if (htim->Instance == TIM3) {
		HAL_ADC_Start_DMA(&hadc1, (uint32_t *)adc_scan, SCAN_MAX);
	}
/* USER CODE END Callback 0 */
  if (htim->Instance == TIM4) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
