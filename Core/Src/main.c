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
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "Stdbool.h"
#include "Stdint.h"
#include "string.h"
#define False 0
#define True 1
#define Vref_mV 3300


	  	  	 		  	  	 	  	   static uint32_t value1 = 0;
									   static uint32_t value2 = 0;
	  	  	 		  	  	 	  	   static uint32_t value1_conv = 0;
									   static uint32_t value2_conv = 0;

#define Position_L_R 2500
#define Position_U_D 2700

#define Hystersis 200

char ButtonState;
char state=0;
char moveleft=0;
char moveright=0;
char moveup=0;
char movedown=0;
char PositioningPhase=False;

#define Button_UP 0
#define Button_DOWN 1
#define Button_LEFT 2
#define Button_RIGHT 3
#define Button_LED 4
#define Button_LEDOFF 5
#define Button_COUNT 6

uint8_t buttonState[Button_COUNT] = {0};
uint8_t debounceCounter[Button_COUNT] = {0};
char  Message[]="Test";
static void PWM_setduty(uint8_t channel, uint16_t dutycycle);
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
ADC_HandleTypeDef hadc2;

COMP_HandleTypeDef hcomp2;
COMP_HandleTypeDef hcomp3;
COMP_HandleTypeDef hcomp4;
COMP_HandleTypeDef hcomp6;

HRTIM_HandleTypeDef hhrtim1;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

PCD_HandleTypeDef hpcd_USB_FS;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for myTask02 */
osThreadId_t myTask02Handle;
const osThreadAttr_t myTask02_attributes = {
  .name = "myTask02",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Position */
osThreadId_t PositionHandle;
const osThreadAttr_t Position_attributes = {
  .name = "Position",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for Task_5m */
osThreadId_t Task_5mHandle;
const osThreadAttr_t Task_5m_attributes = {
  .name = "Task_5m",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* Definitions for myTask05 */
osThreadId_t myTask05Handle;
const osThreadAttr_t myTask05_attributes = {
  .name = "myTask05",
  .priority = (osPriority_t) osPriorityLow,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_COMP2_Init(void);
static void MX_COMP3_Init(void);
static void MX_COMP4_Init(void);
static void MX_COMP6_Init(void);
static void MX_HRTIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_ADC2_Init(void);
static void MX_TIM3_Init(void);
void StartDefaultTask(void *argument);
void StartTask02(void *argument);
void StartTask03(void *argument);
void StartTask04(void *argument);
void DeadZone(void *argument);

static void MX_NVIC_Init(void);
/* USER CODE BEGIN PFP */
void StartADC(void);
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
  MX_COMP2_Init();
  MX_COMP3_Init();
  MX_COMP4_Init();
  MX_COMP6_Init();
  MX_HRTIM1_Init();
  MX_USART3_UART_Init();
  MX_USB_PCD_Init();
  MX_ADC2_Init();
  MX_TIM3_Init();

  /* Initialize interrupts */
  MX_NVIC_Init();
  /* USER CODE BEGIN 2 */
HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);
HAL_UART_Transmit(&huart3, (uint8_t *) Message, strlen(Message), 10);
HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
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

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of myTask02 */
  myTask02Handle = osThreadNew(StartTask02, NULL, &myTask02_attributes);

  /* creation of Position */
  PositionHandle = osThreadNew(StartTask03, NULL, &Position_attributes);

  /* creation of Task_5m */
  Task_5mHandle = osThreadNew(StartTask04, NULL, &Task_5m_attributes);

  /* creation of myTask05 */
  myTask05Handle = osThreadNew(DeadZone, NULL, &myTask05_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI48;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV8;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV8;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief NVIC Configuration.
  * @retval None
  */
static void MX_NVIC_Init(void)
{
  /* ADC1_2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(ADC1_2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(ADC1_2_IRQn);
}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV16;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.NbrOfConversion = 2;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc2.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_640CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief COMP2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP2_Init(void)
{

  /* USER CODE BEGIN COMP2_Init 0 */

  /* USER CODE END COMP2_Init 0 */

  /* USER CODE BEGIN COMP2_Init 1 */

  /* USER CODE END COMP2_Init 1 */
  hcomp2.Instance = COMP2;
  hcomp2.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp2.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp2.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp2.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp2.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp2.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP2_Init 2 */

  /* USER CODE END COMP2_Init 2 */

}

/**
  * @brief COMP3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP3_Init(void)
{

  /* USER CODE BEGIN COMP3_Init 0 */

  /* USER CODE END COMP3_Init 0 */

  /* USER CODE BEGIN COMP3_Init 1 */

  /* USER CODE END COMP3_Init 1 */
  hcomp3.Instance = COMP3;
  hcomp3.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp3.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp3.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp3.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp3.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp3.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP3_Init 2 */

  /* USER CODE END COMP3_Init 2 */

}

/**
  * @brief COMP4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP4_Init(void)
{

  /* USER CODE BEGIN COMP4_Init 0 */

  /* USER CODE END COMP4_Init 0 */

  /* USER CODE BEGIN COMP4_Init 1 */

  /* USER CODE END COMP4_Init 1 */
  hcomp4.Instance = COMP4;
  hcomp4.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp4.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp4.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp4.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp4.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp4.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP4_Init 2 */

  /* USER CODE END COMP4_Init 2 */

}

/**
  * @brief COMP6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_COMP6_Init(void)
{

  /* USER CODE BEGIN COMP6_Init 0 */

  /* USER CODE END COMP6_Init 0 */

  /* USER CODE BEGIN COMP6_Init 1 */

  /* USER CODE END COMP6_Init 1 */
  hcomp6.Instance = COMP6;
  hcomp6.Init.InputPlus = COMP_INPUT_PLUS_IO1;
  hcomp6.Init.InputMinus = COMP_INPUT_MINUS_1_4VREFINT;
  hcomp6.Init.OutputPol = COMP_OUTPUTPOL_NONINVERTED;
  hcomp6.Init.Hysteresis = COMP_HYSTERESIS_NONE;
  hcomp6.Init.BlankingSrce = COMP_BLANKINGSRC_NONE;
  hcomp6.Init.TriggerMode = COMP_TRIGGERMODE_NONE;
  if (HAL_COMP_Init(&hcomp6) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN COMP6_Init 2 */

  /* USER CODE END COMP6_Init 2 */

}

/**
  * @brief HRTIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_HRTIM1_Init(void)
{

  /* USER CODE BEGIN HRTIM1_Init 0 */

  /* USER CODE END HRTIM1_Init 0 */

  HRTIM_TimeBaseCfgTypeDef pTimeBaseCfg = {0};
  HRTIM_TimerCtlTypeDef pTimerCtl = {0};
  HRTIM_TimerCfgTypeDef pTimerCfg = {0};
  HRTIM_OutputCfgTypeDef pOutputCfg = {0};

  /* USER CODE BEGIN HRTIM1_Init 1 */

  /* USER CODE END HRTIM1_Init 1 */
  hhrtim1.Instance = HRTIM1;
  hhrtim1.Init.HRTIMInterruptResquests = HRTIM_IT_NONE;
  hhrtim1.Init.SyncOptions = HRTIM_SYNCOPTION_NONE;
  if (HAL_HRTIM_Init(&hhrtim1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_DLLCalibrationStart(&hhrtim1, HRTIM_CALIBRATIONRATE_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_PollForDLLCalibration(&hhrtim1, 10) != HAL_OK)
  {
    Error_Handler();
  }
  pTimeBaseCfg.Period = 0xFFDF;
  pTimeBaseCfg.RepetitionCounter = 0x00;
  pTimeBaseCfg.PrescalerRatio = HRTIM_PRESCALERRATIO_MUL32;
  pTimeBaseCfg.Mode = HRTIM_MODE_CONTINUOUS;
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCtl.UpDownMode = HRTIM_TIMERUPDOWNMODE_UP;
  pTimerCtl.DualChannelDacEnable = HRTIM_TIMER_DCDE_DISABLED;
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.InterruptRequests = HRTIM_TIM_IT_NONE;
  pTimerCfg.DMARequests = HRTIM_TIM_DMA_NONE;
  pTimerCfg.DMASrcAddress = 0x0000;
  pTimerCfg.DMADstAddress = 0x0000;
  pTimerCfg.DMASize = 0x1;
  pTimerCfg.HalfModeEnable = HRTIM_HALFMODE_DISABLED;
  pTimerCfg.InterleavedMode = HRTIM_INTERLEAVED_MODE_DISABLED;
  pTimerCfg.StartOnSync = HRTIM_SYNCSTART_DISABLED;
  pTimerCfg.ResetOnSync = HRTIM_SYNCRESET_DISABLED;
  pTimerCfg.DACSynchro = HRTIM_DACSYNC_NONE;
  pTimerCfg.PreloadEnable = HRTIM_PRELOAD_DISABLED;
  pTimerCfg.UpdateGating = HRTIM_UPDATEGATING_INDEPENDENT;
  pTimerCfg.BurstMode = HRTIM_TIMERBURSTMODE_MAINTAINCLOCK;
  pTimerCfg.RepetitionUpdate = HRTIM_UPDATEONREPETITION_DISABLED;
  pTimerCfg.PushPull = HRTIM_TIMPUSHPULLMODE_DISABLED;
  pTimerCfg.FaultEnable = HRTIM_TIMFAULTENABLE_NONE;
  pTimerCfg.FaultLock = HRTIM_TIMFAULTLOCK_READWRITE;
  pTimerCfg.DeadTimeInsertion = HRTIM_TIMDEADTIMEINSERTION_DISABLED;
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_A_B_C_DELAYEDPROTECTION_DISABLED;
  pTimerCfg.UpdateTrigger = HRTIM_TIMUPDATETRIGGER_NONE;
  pTimerCfg.ResetTrigger = HRTIM_TIMRESETTRIGGER_NONE;
  pTimerCfg.ResetUpdate = HRTIM_TIMUPDATEONRESET_DISABLED;
  pTimerCfg.ReSyncUpdate = HRTIM_TIMERESYNC_UPDATE_UNCONDITIONAL;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_D_E_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pTimerCfg.DelayedProtectionMode = HRTIM_TIMER_F_DELAYEDPROTECTION_DISABLED;
  if (HAL_HRTIM_WaveformTimerConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCfg) != HAL_OK)
  {
    Error_Handler();
  }
  pOutputCfg.Polarity = HRTIM_OUTPUTPOLARITY_HIGH;
  pOutputCfg.SetSource = HRTIM_OUTPUTSET_NONE;
  pOutputCfg.ResetSource = HRTIM_OUTPUTRESET_NONE;
  pOutputCfg.IdleMode = HRTIM_OUTPUTIDLEMODE_NONE;
  pOutputCfg.IdleLevel = HRTIM_OUTPUTIDLELEVEL_INACTIVE;
  pOutputCfg.FaultLevel = HRTIM_OUTPUTFAULTLEVEL_NONE;
  pOutputCfg.ChopperModeEnable = HRTIM_OUTPUTCHOPPERMODE_DISABLED;
  pOutputCfg.BurstModeEntryDelayed = HRTIM_OUTPUTBURSTMODEENTRY_REGULAR;
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_A, HRTIM_OUTPUT_TA1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, HRTIM_OUTPUT_TE1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF1, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_C, HRTIM_OUTPUT_TC2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformOutputConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, HRTIM_OUTPUT_TF2, &pOutputCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_E, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_TimeBaseConfig(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimeBaseCfg) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_HRTIM_WaveformTimerControl(&hhrtim1, HRTIM_TIMERINDEX_TIMER_F, &pTimerCtl) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN HRTIM1_Init 2 */

  /* USER CODE END HRTIM1_Init 2 */
  HAL_HRTIM_MspPostInit(&hhrtim1);

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
  htim3.Init.Prescaler = 32;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10000;
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
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV4;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.Sof_enable = DISABLE;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, BUCKBOOST_LOAD_1_Pin|BUCKBOOST_LOAD_2_Pin|Button_R_L_SEL_0_Pin|Button_R_L_PWM_Pin
                          |BUCKBOOST_USBPD_EN_Pin|USBPD_1A_PROTECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, Button_R_L_IN_A_Pin|Button_R_L_IN_B_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, Button_LED_IN_B_Pin|LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LD3_Pin|Button_U_D_PWM_Pin|Button_U_D_SEL_0_Pin|Button_U_D_IN_B_Pin
                          |Button_U_D_IN_A_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(USBPD_550mA_PROTECT_GPIO_Port, USBPD_550mA_PROTECT_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : Button_OK_Pin Button_LEFT_Pin Button_DOWN_Pin */
  GPIO_InitStruct.Pin = Button_OK_Pin|Button_LEFT_Pin|Button_DOWN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUCKBOOST_LOAD_1_Pin BUCKBOOST_LOAD_2_Pin Button_R_L_SEL_0_Pin Button_R_L_PWM_Pin
                           BUCKBOOST_USBPD_EN_Pin USBPD_1A_PROTECT_Pin */
  GPIO_InitStruct.Pin = BUCKBOOST_LOAD_1_Pin|BUCKBOOST_LOAD_2_Pin|Button_R_L_SEL_0_Pin|Button_R_L_PWM_Pin
                          |BUCKBOOST_USBPD_EN_Pin|USBPD_1A_PROTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_R_L_IN_A_Pin Button_R_L_IN_B_Pin */
  GPIO_InitStruct.Pin = Button_R_L_IN_A_Pin|Button_R_L_IN_B_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pin : USBPD_VIN_Pin */
  GPIO_InitStruct.Pin = USBPD_VIN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USBPD_VIN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BUCKBOOST_VIN_Pin BUCKBOOST_I_IN_AVG_Pin BUCKBOOST_VOUT_Pin */
  GPIO_InitStruct.Pin = BUCKBOOST_VIN_Pin|BUCKBOOST_I_IN_AVG_Pin|BUCKBOOST_VOUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_LED_IN_B_Pin LD2_Pin */
  GPIO_InitStruct.Pin = Button_LED_IN_B_Pin|LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LD3_Pin Button_U_D_PWM_Pin Button_U_D_SEL_0_Pin Button_U_D_IN_B_Pin
                           Button_U_D_IN_A_Pin */
  GPIO_InitStruct.Pin = LD3_Pin|Button_U_D_PWM_Pin|Button_U_D_SEL_0_Pin|Button_U_D_IN_B_Pin
                          |Button_U_D_IN_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Button_RIGHT_Pin Button_UP_Pin */
  GPIO_InitStruct.Pin = Button_RIGHT_Pin|Button_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Deadzone_Pin */
  GPIO_InitStruct.Pin = Button_Deadzone_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_Deadzone_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : USBPD_550mA_PROTECT_Pin */
  GPIO_InitStruct.Pin = USBPD_550mA_PROTECT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(USBPD_550mA_PROTECT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

static void PWM_setduty(uint8_t channel, uint16_t dutycycle)
{
	uint16_t calcduty=(htim3.Init.Period/100)*dutycycle;
	__HAL_TIM_SET_COMPARE(&htim3,channel,calcduty);

}

void StartADC(void)
{
			HAL_ADC_Start(&hadc2);
			   while(HAL_ADC_PollForConversion(&hadc2, 650000) != HAL_OK)
			  {

			  }
			   value1 = HAL_ADC_GetValue(&hadc2);
			   value1_conv= __HAL_ADC_CALC_DATA_TO_VOLTAGE(Vref_mV,value1,ADC_RESOLUTION_12B);
			   HAL_ADC_Start(&hadc2);
			   while(HAL_ADC_PollForConversion(&hadc2, 650000) != HAL_OK)
			  {

			  }
			   value2 = HAL_ADC_GetValue(&hadc2);
			   value2_conv= __HAL_ADC_CALC_DATA_TO_VOLTAGE(Vref_mV,value2,ADC_RESOLUTION_12B);
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
  for(;;)
  {

	  if(buttonState[Button_LEFT]==1 && buttonState[Button_RIGHT]==0 &&buttonState[Button_DOWN]==0&&buttonState[Button_UP]==0)
	  {
		  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 1);
		  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
		  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 1);
		  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 1);
	  }
	  else if(buttonState[Button_LEFT]==0 && buttonState[Button_RIGHT]==0)
	  {
		  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 1);
		  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
		  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 1);
		  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 0);
	  }
	  if(buttonState[Button_RIGHT]==1&&buttonState[Button_LEFT]==0&&buttonState[Button_DOWN]==0&&buttonState[Button_UP]==0)
	  {
		  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 0);
		  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 1);
		  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 0);
		  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 1);
	  }
	  else if(buttonState[Button_RIGHT]==0&&buttonState[Button_LEFT]==0)
	  	  {
	  		  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 0);
	  		  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 0);
	  	  }
	  if(buttonState[Button_UP]==1 && buttonState[Button_DOWN]==0 && buttonState[Button_RIGHT]==0&&buttonState[Button_LEFT]==0)
	 	  {
	 		  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 1);
	 		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
	 		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 1);
	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 1);
	 	  }
	 	  else if(buttonState[Button_UP]==0 && buttonState[Button_DOWN]==0&& buttonState[Button_RIGHT]==0&&buttonState[Button_LEFT]==0)
	 	  {
	 		  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 1);
	 		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
	 		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 1);
	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 0);
	 	  }
	 	  if(buttonState[Button_DOWN]==1&&buttonState[Button_UP]==0 && buttonState[Button_RIGHT]==0&&buttonState[Button_LEFT]==0)
	 	  {
	 		  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 0);
	 		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 1);
	 		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 0);
	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 1);
	 	  }
	 	  else if(buttonState[Button_DOWN]==0&&buttonState[Button_UP]==0&& buttonState[Button_RIGHT]==0&&buttonState[Button_LEFT]==0)
	 	  	  {
	 	  		  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 0);
	 	  		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
	 	  		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 0);
	 	  		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 0);
		  		  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 0);
		  		  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
		  		  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 0);
		  		  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 0);
	 	  	  }
    osDelay(5);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartTask02 */
/**
* @brief Function implementing the myTask02 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask02 */
void StartTask02(void *argument)
{
  /* USER CODE BEGIN StartTask02 */
  /* Infinite loop */
  for(;;)
  {
	  if(HAL_GPIO_ReadPin(Button_UP_GPIO_Port, Button_UP_Pin) == 0)
	  	  {
	  		  debounceCounter[Button_UP]++;
	  		  if(debounceCounter[Button_UP] >= 5u)
	  		  {
	  			  debounceCounter[Button_UP] = 0;
	  			  buttonState[Button_UP] = True;
	  		  }
	  	  }
	  	  else {
	  		  debounceCounter[Button_UP] = 0;
	  		  buttonState[Button_UP] = False;
	  	  }
	  	  if(HAL_GPIO_ReadPin(Button_DOWN_GPIO_Port, Button_DOWN_Pin) == 0)
	  	  	  {
	  	  		  debounceCounter[Button_DOWN]++;
	  	  		  if(debounceCounter[Button_DOWN] >= 5u)
	  	  		  {
	  	  			  debounceCounter[Button_DOWN] = 0;
	  	  			  buttonState[Button_DOWN] = True;
	  	  		  }
	  	  	  }
	  	  	  else {
	  	  		  debounceCounter[Button_DOWN] = 0;
	  	  		  buttonState[Button_DOWN] = False;
	  	  	  }
	  	  if(HAL_GPIO_ReadPin(Button_RIGHT_GPIO_Port, Button_RIGHT_Pin) == 0)
	  	  	  {
	  	  		  debounceCounter[Button_RIGHT]++;
	  	  		  if(debounceCounter[Button_RIGHT] >= 5u)
	  	  		  {
	  	  			  debounceCounter[Button_RIGHT] = 0;
	  	  			  buttonState[Button_RIGHT] = True;
	  	  		  }
	  	  	  }
	  	  	  else {
	  	  		  debounceCounter[Button_RIGHT] = 0;
	  	  		  buttonState[Button_RIGHT] = False;
	  	  	  }
	  	  if(HAL_GPIO_ReadPin(Button_LEFT_GPIO_Port, Button_LEFT_Pin) == 0)
	  	  	  {
	  	  		  debounceCounter[Button_LEFT]++;
	  	  		  if(debounceCounter[Button_LEFT] >= 5u)
	  	  		  {
	  	  			  debounceCounter[Button_LEFT] = 0;
	  	  			  buttonState[Button_LEFT] = True;
	  	  		  }
	  	  	  }
	  	  	  else {
	  	  		  debounceCounter[Button_LEFT] = 0;
	  	  		  buttonState[Button_LEFT] = False;
	  	  	  }
	  	  if(HAL_GPIO_ReadPin(Button_OK_GPIO_Port, Button_OK_Pin) == 0)
	  	  	  	  {
	  	  	  		  debounceCounter[Button_LED]++;
	  	  	  		  if(debounceCounter[Button_LED] >= 5u)
	  	  	  		  {
	  	  	  			  debounceCounter[Button_LED] = 0;
	  	  	  			  buttonState[Button_LED] = True;
	  	  	  		  }
	  	  	  	  }
	  	  	  	  else {
	  	  	  		  debounceCounter[Button_LED] = 0;
	  	  	  		  buttonState[Button_LED] = False;
	  	  	  	  }
	  	  if(HAL_GPIO_ReadPin(Button_Deadzone_GPIO_Port, Button_Deadzone_Pin) == 1)
	  	  	  	  {
	  	  	  		  debounceCounter[Button_LEDOFF]++;
	  	  	  		  if(debounceCounter[Button_LEDOFF] >= 50u)
	  	  	  		  {
	  	  	  			  debounceCounter[Button_LEDOFF] = 0;
	  	  	  			  buttonState[Button_LEDOFF]++;
	  	  	  			  if(buttonState[Button_LEDOFF]>=3)
	  	  	  			  {
	  	  	  				  buttonState[Button_LEDOFF]=0;
	  	  	  			  }
	  	  	  		  }
	  	  	  	  }
	  	  	  	  else {
	  	  	  		  debounceCounter[Button_LEDOFF] = 0;
	  	  	  	  }
				StartADC();


    osDelay(10);
  }
  /* USER CODE END StartTask02 */
}

/* USER CODE BEGIN Header_StartTask03 */
/**
* @brief Function implementing the Position thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask03 */
void StartTask03(void *argument)
{
  /* USER CODE BEGIN StartTask03 */
  /* Infinite loop */
  for(;;)
  {
	  if(PositioningPhase==True)
	  {

		  while(value2_conv>(Position_L_R+Hystersis) || value2_conv<(Position_L_R-Hystersis) )
		  {
			  StartADC();
			  if(value2_conv>Position_L_R)
			  {
				  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 1);
				  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 1);
			  }
			  else if(value2_conv<Position_L_R)
			  {
				  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 1);
				  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 1);
				  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 1);
			  }
			  else
			  {
				  HAL_GPIO_WritePin(Button_R_L_IN_A_GPIO_Port, Button_R_L_IN_A_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_IN_B_GPIO_Port, Button_R_L_IN_B_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_SEL_0_GPIO_Port, Button_R_L_SEL_0_Pin, 0);
				  HAL_GPIO_WritePin(Button_R_L_PWM_GPIO_Port, Button_R_L_PWM_Pin, 0);
			  }


		  }

		  while(value1_conv>(Position_U_D+Hystersis) || value2_conv<(Position_U_D-Hystersis) )
		  {
			  StartADC();
			  if(value1_conv>Position_U_D)
			  			  {
				  	  	  	  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 1);
				  	 		  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
				  	 		  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 1);
				  	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 1);
			  			  }
			  			  else if(value1_conv<Position_U_D)
			  			  {
			  				  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 1);
			  				  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
			  				  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 1);
				  	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 0);
			  			  }
			  			  else
								{
			  				  HAL_GPIO_WritePin(Button_U_D_IN_A_GPIO_Port, Button_U_D_IN_A_Pin, 0);
			  				  HAL_GPIO_WritePin(Button_U_D_IN_B_GPIO_Port, Button_U_D_IN_B_Pin, 0);
			  				  HAL_GPIO_WritePin(Button_U_D_SEL_0_GPIO_Port, Button_U_D_SEL_0_Pin, 0);
				  	 		  HAL_GPIO_WritePin(Button_U_D_PWM_GPIO_Port, Button_U_D_PWM_Pin, 0);
								}

		  }
		  PositioningPhase=False;
	  }
    osDelay(1);
  }
  /* USER CODE END StartTask03 */
}

/* USER CODE BEGIN Header_StartTask04 */
/**
* @brief Function implementing the Task_5m thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartTask04 */
void StartTask04(void *argument)
{
  /* USER CODE BEGIN StartTask04 */
  /* Infinite loop */
  for(;;)
  {
	  switch (buttonState[Button_LED]) {
	 	  	  	  case 0:
	 	  	  		  HAL_GPIO_WritePin(Button_LED_IN_B_GPIO_Port, Button_LED_IN_B_Pin, 0);
	 	  	  		  break;
	 	  			case 1:
	 	  				HAL_GPIO_TogglePin(Button_LED_IN_B_GPIO_Port, Button_LED_IN_B_Pin);
	 	  				break;
	 	  			default:
	 	  				HAL_GPIO_WritePin(Button_LED_IN_B_GPIO_Port, Button_LED_IN_B_Pin, 0);
	 	  				break;
	 	  		}
    osDelay(500);
  }
  /* USER CODE END StartTask04 */
}

/* USER CODE BEGIN Header_DeadZone */
/**
* @brief Function implementing the myTask05 thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_DeadZone */
void DeadZone(void *argument)
{
  /* USER CODE BEGIN DeadZone */
  /* Infinite loop */
  for(;;)
  {
	  switch (buttonState[Button_LEDOFF]) {
	  case 0:
		  PWM_setduty(TIM_CHANNEL_4, 0);
		  break;
	  case 1:
		  PWM_setduty(TIM_CHANNEL_4, 50);
		  break;
	  case 2:
		  PWM_setduty(TIM_CHANNEL_4, 100);
		  break;
	  default:
		 break;
	  }
    osDelay(5);
  }
  /* USER CODE END DeadZone */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
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
#ifdef USE_FULL_ASSERT
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
