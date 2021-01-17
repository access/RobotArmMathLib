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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "RobotArm.h"

#include <time.h>
#include <stdlib.h>
#include <math.h>
#include "string.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>

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
SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim9;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart4;
DMA_HandleTypeDef hdma_uart4_rx;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for UART_Send */
osThreadId_t UART_SendHandle;
const osThreadAttr_t UART_Send_attributes = {
  .name = "UART_Send",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for UART_Get */
osThreadId_t UART_GetHandle;
const osThreadAttr_t UART_Get_attributes = {
  .name = "UART_Get",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for JOINT_RUN */
osThreadId_t JOINT_RUNHandle;
const osThreadAttr_t JOINT_RUN_attributes = {
  .name = "JOINT_RUN",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for JOINT_QUEUE */
osMessageQueueId_t JOINT_QUEUEHandle;
const osMessageQueueAttr_t JOINT_QUEUE_attributes = {
  .name = "JOINT_QUEUE"
};
/* USER CODE BEGIN PV */
/************** TIMER COUNTERS ****************/
int64_t TIM1_COUNTER = 0;
int64_t TIM2_COUNTER = 0;
int64_t TIM3_COUNTER = 0;
int64_t TIM4_COUNTER = 0;
int64_t TIM5_COUNTER = 0;
int64_t TIM6_COUNTER = 0;
int64_t TIM7_COUNTER = 0;
/************** JOINT`s LIMITS ****************/
int32_t JOINT1_STEPS_HOME = 0, JOINT1_STEPS_CURRENT = 0, JOINT1_STEPS_MIN = -15000, JOINT1_STEPS_MAX = 15000;
uint32_t JOINT2_STEPS_HOME = 0, JOINT2_STEPS_CURRENT = 0, JOINT2_STEPS_MIN = 0, JOINT2_STEPS_MAX = 14000;
uint32_t JOINT3_STEPS_HOME = 0, JOINT3_STEPS_CURRENT = 0, JOINT3_STEPS_MIN = 0, JOINT3_STEPS_MAX = 10800;
int32_t JOINT4_STEPS_HOME = 0, JOINT4_STEPS_CURRENT = 0, JOINT4_STEPS_MIN = -7500, JOINT4_STEPS_MAX = 7500;
int32_t JOINT5_STEPS_HOME = 0, JOINT5_STEPS_CURRENT = 0, JOINT5_STEPS_MIN = -4650, JOINT5_STEPS_MAX = 4650;
uint32_t JOINT6_STEPS_HOME = 0, JOINT6_STEPS_CURRENT = 0, JOINT6_STEPS_MIN = 0, JOINT6_STEPS_MAX = 200000;
uint32_t JOINT7_STEPS_HOME = 0, JOINT7_STEPS_CURRENT = 0, JOINT7_STEPS_MIN = 0, JOINT7_STEPS_MAX = 9300;
/*************** MODE WHEN SETTING UP HOME POSITION *********************/
uint8_t SET_HOME_MODE = 0;
/*************** DEMO *********************/
uint8_t DEMO_START = 0, CAP_START = 0;

uint8_t UART4_RxBUFFER[1024] = { 0 };
uint8_t UART4_INPUT_BYTES_BUFFER_SIZE = 100;
uint8_t led3On = 0;
uint16_t pwmValue = 500;
uint8_t j1dir = 0;

// queues
//#define QUEUE_SIZE (uint32_t) 1
uint32_t JOINTCMD_QUEUE_SIZE = 100;

//typedef struct JointCMD_t {
typedef struct JointCMD_t {
	uint8_t joint_number;
	uint8_t speed; // in percents
	uint8_t direction;
	uint64_t step_count;
//char cmd[60];
} JointCMD;

//xQueueHandle xQueue;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM8_Init(void);
static void MX_TIM9_Init(void);
static void MX_TIM12_Init(void);
static void MX_UART4_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void *argument);
void UARTsend(void *argument);
void UARTget(void *argument);
void JOINT_RUN_QUEUE(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#define INC_DWT_DELAY_H_
#define DWT_DELAY_NEWBIE 0

void DWT_Init(void) {
	if (!(CoreDebug->DEMCR & CoreDebug_DEMCR_TRCENA_Msk)) {
		CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
		DWT->CYCCNT = 0;
		DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
	}
}

#if DWT_DELAY_NEWBIE
	void DWT_Delay(uint32_t us) // microseconds
	{
		uint32_t startTick  = DWT->CYCCNT,
		targetTick = DWT->CYCCNT + us * (SystemCoreClock/1000000);

		// Must check if target tick is out of bounds and overflowed
		if (targetTick > startTick) {
			// Not overflowed
			while (DWT->CYCCNT < targetTick);
			} else {
			// Overflowed
			while (DWT->CYCCNT > startTick || DWT->CYCCNT < targetTick);
		}
	}
	#else
void DWT_Delay(uint32_t us) // microseconds
{
	uint32_t startTick = DWT->CYCCNT, delayTicks = us * (SystemCoreClock / 1000000);

	while (DWT->CYCCNT - startTick < delayTicks)
		;
}

#endif
int randint(int n) {
	if ((n - 1) == RAND_MAX) {
		return rand();
	} else {
		int end = RAND_MAX / n; // truncate skew
		end *= n;
		int r;
		while ((r = rand()) >= end)
			;
		return r % n;
	}
}

uint16_t start_delay = 6000;
uint16_t udelay = 3000;

void turn(uint16_t count) {
//	uint16_t lck = 0;
//	uint16_t start_period = 200;
	uint16_t motor_steps_period = 200;

	uint16_t full_period = motor_steps_period * count;
	uint16_t period_counter = 0;
	uint8_t period_chunks_count = full_period / 1000;
	uint16_t period_deceleration = full_period / period_chunks_count;
	uint8_t is_deceleration = 0;
	uint8_t is_accelerated = 0;

	for (uint16_t t = 0; t < count; t++) {
		for (uint16_t i = 0; i < motor_steps_period; i++) {

			//HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOE, JOINT1_STEP_Pin, GPIO_PIN_SET);
			DWT_Delay(udelay);
			HAL_GPIO_WritePin(GPIOE, JOINT1_STEP_Pin, GPIO_PIN_RESET);
			if (i > 100) {
				//HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);

			}

			if (udelay > 4000 && is_accelerated != 1)
				udelay -= log(udelay) / log(udelay) * 6;
			else
				is_accelerated = 1;

			if (is_deceleration == 1 && udelay < start_delay)
				udelay += log(udelay) / 4;

			is_deceleration = period_counter >= period_deceleration * (period_chunks_count - 1) ? 1 : 0;
			period_counter++;
		}
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
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM8_Init();
  MX_TIM9_Init();
  MX_TIM12_Init();
  MX_UART4_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
	//DWT_Init();
  InitRobotArm();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* creation of JOINT_QUEUE */
  JOINT_QUEUEHandle = osMessageQueueNew (32, sizeof(uint32_t), &JOINT_QUEUE_attributes);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	//osMessageQDef(JOINT_QUEUEHandle,QUEUE_SIZE,uint32_t);
	//JOINT_QUEUEHandle=osMessageCreate(osMessageQ(JOINT_QUEUEHandle), NULL);
	//JOINT_QUEUECommand = x
	//xQueue = xQueueCreate(JOINTCMD_QUEUE_SIZE, sizeof(JointCMD));
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of UART_Send */
  UART_SendHandle = osThreadNew(UARTsend, NULL, &UART_Send_attributes);

  /* creation of UART_Get */
  UART_GetHandle = osThreadNew(UARTget, NULL, &UART_Get_attributes);

  /* creation of JOINT_RUN */
  JOINT_RUNHandle = osThreadNew(JOINT_RUN_QUEUE, NULL, &JOINT_RUN_attributes);

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
	while (1) {

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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 84-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 3000;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
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
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 2000;
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
  sConfigOC.Pulse = 1500;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 84-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 2000;
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
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 2000;
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
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 84-1;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 2000;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

}

/**
  * @brief TIM9 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM9_Init(void)
{

  /* USER CODE BEGIN TIM9_Init 0 */

  /* USER CODE END TIM9_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM9_Init 1 */

  /* USER CODE END TIM9_Init 1 */
  htim9.Instance = TIM9;
  htim9.Init.Prescaler = 84-1;
  htim9.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim9.Init.Period = 2000;
  htim9.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim9.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim9, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim9) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim9, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM9_Init 2 */

  /* USER CODE END TIM9_Init 2 */
  HAL_TIM_MspPostInit(&htim9);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 2000;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 1000;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */
  HAL_TIM_MspPostInit(&htim12);

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
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, JOINT6_DIR_Pin|JOINT1_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, SPI_CS_Pin|JOINT2_DIR_Pin|JOINT3_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JOINT7_DIR_GPIO_Port, JOINT7_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, JOINT4_DIR_Pin|LED3_Pin|LED2_Pin|LED1_Pin
                          |LED5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(JOINT5_DIR_GPIO_Port, JOINT5_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : JOINT6_DIR_Pin JOINT1_DIR_Pin */
  GPIO_InitStruct.Pin = JOINT6_DIR_Pin|JOINT1_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : SPI_CS_Pin JOINT2_DIR_Pin JOINT3_DIR_Pin */
  GPIO_InitStruct.Pin = SPI_CS_Pin|JOINT2_DIR_Pin|JOINT3_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : JOINT7_DIR_Pin */
  GPIO_InitStruct.Pin = JOINT7_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JOINT7_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : JOINT4_DIR_Pin LED3_Pin LED2_Pin LED1_Pin
                           LED5_Pin */
  GPIO_InitStruct.Pin = JOINT4_DIR_Pin|LED3_Pin|LED2_Pin|LED1_Pin
                          |LED5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : JOINT5_DIR_Pin */
  GPIO_InitStruct.Pin = JOINT5_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(JOINT5_DIR_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void speedInit(TIM_HandleTypeDef tim) {
	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_OC_InitTypeDef sConfigOC = { 0 };

	//htim12.Instance = TIM12;
	htim12.Init.Prescaler = 84 - 1;
	htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
	if (tim.Instance == TIM1) {

	}
	htim12.Init.Period = 2000;
	htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
	if (HAL_TIM_Base_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_TIM_PWM_Init(&htim12) != HAL_OK) {
		Error_Handler();
	}
	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = 1000;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	if (HAL_TIM_PWM_ConfigChannel(&htim12, &sConfigOC, TIM_CHANNEL_1) != HAL_OK) {
		Error_Handler();
	}
	HAL_TIM_MspPostInit(&htim12);
}

void pwm_setvalue(uint16_t value) {
	pwmValue = value;
	TIM_OC_InitTypeDef sConfigOC;

	sConfigOC.OCMode = TIM_OCMODE_PWM1;
	sConfigOC.Pulse = value;
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
	sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
	HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM1) { // joint 1
		/************ LIMITATION COUNTERS JOINT 1 ***************/
		if (HAL_GPIO_ReadPin(GPIOE, JOINT1_DIR_Pin)) { // get direction, IF GPIO_PIN_SET
			JOINT1_STEPS_CURRENT++;
		} else {
			JOINT1_STEPS_CURRENT--;
		}
		if ((JOINT1_STEPS_CURRENT > JOINT1_STEPS_MAX || JOINT1_STEPS_CURRENT < JOINT1_STEPS_MIN) && SET_HOME_MODE != 1) {
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOE, JOINT1_DIR_Pin, GPIO_PIN_RESET); // 1 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
			return;
		}
		/***********************************************/
		TIM1_COUNTER--;
		if (TIM1_COUNTER <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim1, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOE, JOINT1_DIR_Pin, GPIO_PIN_RESET); // 1 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM2) { // joint 2
		/************ LIMITATION COUNTERS JOINT 2 ***************/
		if (HAL_GPIO_ReadPin(GPIOA, JOINT2_DIR_Pin)) { // get direction, IF GPIO_PIN_SET
			JOINT2_STEPS_CURRENT++;
		} else {
			JOINT2_STEPS_CURRENT--;
		}
		if ((JOINT2_STEPS_CURRENT > JOINT2_STEPS_MAX || JOINT2_STEPS_CURRENT < JOINT2_STEPS_MIN) && SET_HOME_MODE != 1) {
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOA, JOINT2_DIR_Pin, GPIO_PIN_RESET); // 1 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
			return;
		}
		/***********************************************/
		TIM2_COUNTER--;
		if (TIM2_COUNTER <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim2, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOA, JOINT2_DIR_Pin, GPIO_PIN_RESET); // 2 : SET=IN, RESET=OUT
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM3) { // joint 3
		/************ LIMITATION COUNTERS JOINT 3 ***************/
		if (HAL_GPIO_ReadPin(GPIOA, JOINT3_DIR_Pin)) { // get direction, IF GPIO_PIN_SET
			JOINT3_STEPS_CURRENT--;
		} else {
			JOINT3_STEPS_CURRENT++;
		}
		if ((JOINT3_STEPS_CURRENT > JOINT3_STEPS_MAX || JOINT3_STEPS_CURRENT < JOINT3_STEPS_MIN) && SET_HOME_MODE != 1) {
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOA, JOINT3_DIR_Pin, GPIO_PIN_RESET); // 3 : SET=IN, RESET=OUT
			return;
		}
		/***********************************************/
		TIM3_COUNTER--;
		if (TIM3_COUNTER <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim3, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOA, JOINT3_DIR_Pin, GPIO_PIN_RESET); // 3 :  SET=IN, RESET=OUT
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM4) { // joint 4
		/************ LIMITATION COUNTERS JOINT 4 ***************/
		if (HAL_GPIO_ReadPin(GPIOD, JOINT4_DIR_Pin)) { // get direction, IF GPIO_PIN_SET
			JOINT4_STEPS_CURRENT--;
		} else {
			JOINT4_STEPS_CURRENT++;
		}
		if ((JOINT4_STEPS_CURRENT > JOINT4_STEPS_MAX || JOINT4_STEPS_CURRENT < JOINT4_STEPS_MIN) && SET_HOME_MODE != 1) {
			HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim4, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOD, JOINT4_DIR_Pin, GPIO_PIN_RESET); // 4 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
			return;
		}
		/***********************************************/
		TIM4_COUNTER--;
		if (TIM4_COUNTER <= 0) {
			HAL_TIM_PWM_Stop_IT(&htim4, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim4, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOD, JOINT4_DIR_Pin, GPIO_PIN_RESET); // 4 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM8) { // joint 5
		/************ LIMITATION COUNTERS JOINT 5 ***************/
		if (HAL_GPIO_ReadPin(GPIOC, JOINT5_DIR_Pin)) { // get direction, IF GPIO_PIN_SET
			JOINT5_STEPS_CURRENT--;
		} else {
			JOINT5_STEPS_CURRENT++;
		}
		if ((JOINT5_STEPS_CURRENT > JOINT5_STEPS_MAX || JOINT5_STEPS_CURRENT < JOINT5_STEPS_MIN) && SET_HOME_MODE != 1) {
			HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOC, JOINT5_DIR_Pin, GPIO_PIN_RESET); // 5 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
			return;
		}
		/***********************************************/
		TIM5_COUNTER--;
		if (TIM5_COUNTER <= 0) {
			//HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
			HAL_TIM_PWM_Stop_IT(&htim8, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim8, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOC, JOINT5_DIR_Pin, GPIO_PIN_RESET); // 5 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM9) { // joint 6
		TIM6_COUNTER--;
		if (TIM6_COUNTER <= 0) {
			//HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
			HAL_TIM_PWM_Stop_IT(&htim9, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim9, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOE, JOINT6_DIR_Pin, GPIO_PIN_RESET); // 6 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		}
	}
	//--------------------------------------------------------------------------------------------------------------------
	if (htim->Instance == TIM12) { // joint 7
		TIM7_COUNTER--;
		if (TIM7_COUNTER <= 0) {
			//HAL_GPIO_TogglePin(GPIOD, LED2_Pin);
			HAL_TIM_PWM_Stop_IT(&htim12, TIM_CHANNEL_1);
			HAL_TIMEx_PWMN_Stop_IT(&htim12, TIM_CHANNEL_1);
			HAL_GPIO_WritePin(GPIOB, JOINT7_DIR_Pin, GPIO_PIN_RESET); // 7 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		}
	}
}

char* constructMessageUART(uint8_t _msg[]) {
	char *tmp = malloc(sizeof(char) * (1024 + 1));
	// filled part
	for (uint16_t i = 0; i < 12; i++) {
		tmp[i] = (char) _msg[i];
	}
	return tmp;
}

void uartSend(UART_HandleTypeDef *huart, char _out[]) {
	HAL_UART_Transmit(huart, (uint8_t*) _out, strlen(_out), 1000);
	//HAL_UART_Receive_DMA(&huart, UART4_rxBuffer, 1024);
}

// move JOINT
void moveJoint(uint8_t jointNumber, uint32_t steps, uint8_t dir) {
	switch (jointNumber) {
	case 1:
		TIM1_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOE, JOINT1_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 1 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
		HAL_TIM_PWM_Start_IT(&htim1, TIM_CHANNEL_1); // 1
		break;
	case 2:
		TIM2_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOA, JOINT2_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 2 : SET=IN, RESET=OUT
		HAL_TIM_PWM_Start_IT(&htim2, TIM_CHANNEL_1); // 2
		break;
	case 3:
		TIM3_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOA, JOINT3_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 3 :  SET=IN, RESET=OUT
		HAL_TIM_PWM_Start_IT(&htim3, TIM_CHANNEL_1); // 3
		break;
	case 4:
		TIM4_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOD, JOINT4_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 4 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		HAL_TIM_PWM_Start_IT(&htim4, TIM_CHANNEL_1); // 4
		break;
	case 5:
		TIM5_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOC, JOINT5_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 5 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		HAL_TIM_PWM_Start_IT(&htim8, TIM_CHANNEL_1); // 5
		break;
	case 6:
		TIM6_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOE, JOINT6_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 6 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		HAL_TIM_PWM_Start_IT(&htim9, TIM_CHANNEL_1); // 6
		break;
	case 7:
		TIM7_COUNTER = steps;
		HAL_GPIO_WritePin(GPIOB, JOINT7_DIR_Pin, dir == 1 ? GPIO_PIN_SET : GPIO_PIN_RESET); // 7 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
		HAL_TIM_PWM_Start_IT(&htim12, TIM_CHANNEL_1); // 7
		break;
	default:
		break;
	}
}

char* extractNumber(char _in[]) {
	char *line = _in;
	for (int i = 0, j; line[i] != '\0'; ++i) {

		// enter the loop if the character is not an alphabet
		// and not the null character
		while (!(line[i] >= '0' && line[i] <= '9') && !(line[i] == '\0')) {
			for (j = i; line[j] != '\0'; ++j) {

				// if jth element of line is not an alphabet,
				// assign the value of (j+1)th element to the jth element
				line[j] = line[j + 1];
			}
			line[j] = '\0';
		}
	}
	return line;
}

// UART Rx callback
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	//led3On = led3On > 0 ? 0 : 1;
	HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_SET);

	//uartSend(&huart4, "\r\nuart_get: ");

	//osDelay(1000);
	char *uartInput = (char*) UART4_RxBUFFER;
	char *str = (char*) uartInput;
	char delim[] = ":";
//	char PACKET_DELIMITER = ':';

	switch (uartInput[0]) {
	// J - JOINT: "J1", "J2" ...
	// J7:100:0:@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@@
	case 'J': {
		uartSend(&huart4, uartInput);
		uartSend(&huart4, "\r\n");

		uint8_t jointNumber = 0;
		uint64_t jointSteps = 0;
		uint8_t jointDirection = 0;
		// ---------------- split incoming data ---------------------------------------
//		int init_size = strlen(str);
		char *ptr = strtok(str, delim);
		uint32_t counter = 0;
		while (ptr != NULL) {
			switch (counter) {
			case 0: // J7 - joint
			{
				char jn[] = { ptr[1] }; //"0";
				//jn[0]=ptr[1];
				//jn[0]=(char)ptr[1];
				jointNumber = atoi(jn);
				break;
			}
			case 1: // 100 - steps
				jointSteps = atoi(ptr);
				break;
			case 2: // 0 - direction
				jointDirection = atoi(ptr);
				break;
			default:
				break;
			}
			ptr = strtok(NULL, delim);
			counter++;
		}
		// ------------------------------------------------------------------------------------------------
		// debug via uart
		char strnumber[20];
		itoa(jointNumber, strnumber, 10);
		uartSend(&huart4, "\r\njointNumber:");
		uartSend(&huart4, strnumber);

		itoa(jointSteps, strnumber, 10);
		uartSend(&huart4, "\r\njointSteps:");
		uartSend(&huart4, strnumber);

		itoa(jointDirection, strnumber, 10);
		uartSend(&huart4, "\r\njointDirection:");
		uartSend(&huart4, strnumber);
		// ------------------------------------------------------------------------------------------------
		moveJoint(jointNumber, jointSteps, jointDirection);
		break;
	}
	case 'S': // SET HOME MODE ENABLE
	{
		uartSend(&huart4, uartInput);
		uartSend(&huart4, "\r\n");
		char *ptr = strtok(str, delim);
		if (strcmp(ptr, "SETHOME") == 0) {
			uartSend(&huart4, ptr);
			uartSend(&huart4, "\r\n");
			ptr = strtok(NULL, delim);
			SET_HOME_MODE = atoi(ptr);
//			char strnumber[20];
//			itoa(SET_HOME_MODE, strnumber, 10);
//
//			uartSend(&huart4, strnumber);
//			uartSend(&huart4, "\r\n");
		}
		break;
	}
	case 'R': // RESET COUNTERS
	{
		uartSend(&huart4, uartInput);
		uartSend(&huart4, "\r\n");
		char *ptr = strtok(str, delim);
		if (strcmp(ptr, "RESETCOUNTERS") == 0) {
			uartSend(&huart4, ptr);
			uartSend(&huart4, "\r\n");
			JOINT1_STEPS_CURRENT = 0;
			JOINT2_STEPS_CURRENT = 0;
			JOINT3_STEPS_CURRENT = 0;
			JOINT4_STEPS_CURRENT = 0;
			JOINT5_STEPS_CURRENT = 0;
			JOINT6_STEPS_CURRENT = 0;
			JOINT7_STEPS_CURRENT = 0;
		}
		break;
	}
	case 'D': // DEMO
	{
		char *ptr = strtok(str, delim);
		if (strcmp(ptr, "DEMO") == 0) {
			uartSend(&huart4, "DEMO");
			DEMO_START = 1;
		}
		break;
	}
	case 'C': // CAP
	{
		char *ptr = strtok(str, delim);
		if (strcmp(ptr, "CAP") == 0) {
			uartSend(&huart4, "CAP");
			CAP_START = 1;
		}
		break;
	}
	case 'H': {

		break;
	}
	case 'M':

		break;
	default:
		break;
	}

	//HAL_UART_Receive_IT(&huart4, UART4_rxBuffer, 12);
	//uartSend(&huart4, uartInput); // print full line

	//uartSend(&huart4, (UART4_RxBUFFER));
	HAL_UART_Receive_DMA(&huart4, UART4_RxBUFFER, UART4_INPUT_BYTES_BUFFER_SIZE);
	//HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_RESET);
}

void demoPlay() {
	moveJoint(6, 20000, 0);
	moveJoint(3, 7000, 0);
	moveJoint(2, 7000, 1);

	osDelay(2000);

	moveJoint(5, 3500, 1);
	moveJoint(4, 2000, 1);

	moveJoint(1, 6000, 0);
	osDelay(5000);
	//moveJoint(1, 2000, 1);

	osDelay(4000);

	moveJoint(5, 3500, 0);
	moveJoint(4, 2000, 0);

	osDelay(4500);				 // TOP
	//moveJoint(1, 2000, 1);

	moveJoint(5, 3500, 0);
	moveJoint(4, 2000, 0);

	moveJoint(3, 7000, 1);
	moveJoint(2, 7000, 0);

	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
	osDelay(1000);
	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
	osDelay(1000);

	moveJoint(6, 500, 1);
	osDelay(1000);

	moveJoint(1, 6000, 1);
	moveJoint(5, 3500, 1);
	moveJoint(4, 2000, 1);

	moveJoint(6, 500, 0);
	osDelay(1000);
	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
	osDelay(1000);

	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
	osDelay(1000);
	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
	osDelay(1000);

	moveJoint(6, 500, 1);
	osDelay(1000);
	moveJoint(6, 500, 0);
}

void capPlay() {
	moveJoint(3, 1000, 0);
	moveJoint(2, 10000, 1);

	osDelay(5000);
	moveJoint(5, 1000, 0);
	moveJoint(1, 800, 0);

	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);

	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);

	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);

	osDelay(1000);
	moveJoint(5, 250, 0);
	osDelay(1000);
	moveJoint(5, 250, 0);

	osDelay(1500);
	//moveJoint(5, 1000, 0);
	moveJoint(3, 2000, 0);
	moveJoint(2, 500, 1);

	moveJoint(5, 500, 1);
	osDelay(500);
	//moveJoint(5, 500, 0);
	osDelay(500);
	moveJoint(5, 500, 1);
	osDelay(500);
	moveJoint(5, 500, 1);
	osDelay(500);
	moveJoint(5, 500, 1);
	osDelay(500);
	moveJoint(5, 500, 1);
	osDelay(500);
	moveJoint(5, 500, 0);
	osDelay(500);

// get cup
	osDelay(1000);
	moveJoint(2, 1500, 0);
	osDelay(1500);
	moveJoint(1, 800, 1);
	moveJoint(3, 500, 1);
	moveJoint(5, 500, 0);
// fill water
	osDelay(1500);
	for (uint8_t i = 0; i < 55; ++i) {
		moveJoint(6, 1, 0);
		//moveJoint(2, 20, 0);
		osDelay(40);
	}

	/************ BACK ***************/
	osDelay(10000);
	moveJoint(2, 11000-500, 0);
	moveJoint(3, 1000, 1);

	for (uint8_t i = 0; i < 55; ++i) {
		moveJoint(6, 1, 1);
		//moveJoint(2, 5, 0);
		osDelay(40);
	}


	moveJoint(5, 1000, 0);
	osDelay(1000);

	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	//moveJoint(1, 200, 0);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);

	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);

	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 250, 1);
	osDelay(1000);
	moveJoint(5, 1250, 1);

	moveJoint(3, 2000, 1);
	//moveJoint(5, 3000, 1);

//	moveJoint(4, 2000, 1);
//
//	moveJoint(1, 6000, 0);
//	osDelay(5000);
//	//moveJoint(1, 2000, 1);
//
//	osDelay(4000);
//
//	moveJoint(5, 3500, 0);
//	moveJoint(4, 2000, 0);
//
//	osDelay(4500);				 // TOP

}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
	/* Infinite loop */
	/************** RESET ALL DIR FOR EMI DISABLE ***************************/
	HAL_GPIO_WritePin(GPIOE, JOINT1_DIR_Pin, GPIO_PIN_RESET); // 1 : SET=CLOCKWISE, RESET=ANTI-CLOCKWISE
	HAL_GPIO_WritePin(GPIOA, JOINT2_DIR_Pin, GPIO_PIN_RESET); // 2 : SET=IN, RESET=OUT
	HAL_GPIO_WritePin(GPIOA, JOINT3_DIR_Pin, GPIO_PIN_RESET); // 3 :  SET=IN, RESET=OUT
	HAL_GPIO_WritePin(GPIOD, JOINT4_DIR_Pin, GPIO_PIN_RESET); // 4 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
	HAL_GPIO_WritePin(GPIOC, JOINT5_DIR_Pin, GPIO_PIN_RESET); // 5 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
	HAL_GPIO_WritePin(GPIOE, JOINT6_DIR_Pin, GPIO_PIN_RESET); // 6 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE
	HAL_GPIO_WritePin(GPIOB, JOINT7_DIR_Pin, GPIO_PIN_RESET); // 7 :  SET=ANTI-CLOCKWISE, RESET=CLOCKWISE

	uartSend(&huart4, "START");
	HAL_UART_Receive_DMA(&huart4, UART4_RxBUFFER, UART4_INPUT_BYTES_BUFFER_SIZE);

	for (;;) {
		HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_SET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOD, LED1_Pin, GPIO_PIN_RESET);
		osDelay(500);
		HAL_GPIO_WritePin(GPIOD, LED3_Pin, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOD, LED2_Pin, GPIO_PIN_RESET);
	}
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_UARTsend */
/**
 * @brief Function implementing the UART_Send thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UARTsend */
void UARTsend(void *argument)
{
  /* USER CODE BEGIN UARTsend */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END UARTsend */
}

/* USER CODE BEGIN Header_UARTget */
/**
 * @brief Function implementing the UART_Get thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_UARTget */
void UARTget(void *argument)
{
  /* USER CODE BEGIN UARTget */
	/* Infinite loop */
	for (;;) {
		osDelay(1);
	}
  /* USER CODE END UARTget */
}

/* USER CODE BEGIN Header_JOINT_RUN_QUEUE */
/**
 * @brief Function implementing the JOINT_RUN thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_JOINT_RUN_QUEUE */
void JOINT_RUN_QUEUE(void *argument)
{
  /* USER CODE BEGIN JOINT_RUN_QUEUE */
	/* Infinite loop */
	for (;;) {
		if (DEMO_START == 1) {
			DEMO_START = 0;
			demoPlay();
		}
		if (CAP_START == 1) {
			CAP_START = 0;
			capPlay();
		}
		osDelay(100);
	}
  /* USER CODE END JOINT_RUN_QUEUE */
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
//	if (htim->Instance == TIM2) {
//		HAL_GPIO_WritePin(GPIOD, LED2_Pin, GPIO_PIN_SET);
//		TIM1_COUNTER++;
//		if (TIM1_COUNTER > 200) {
//			HAL_TIM_PWM_Stop(&htim2, TIM_CHANNEL_1);
//			HAL_TIMEx_PWMN_Stop(&htim2, TIM_CHANNEL_1);
//		}
//	}
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
