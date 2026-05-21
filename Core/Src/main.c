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
#include "adc.h"
#include "can.h"
#include "queue.h"
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

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
DMA_HandleTypeDef hdma_adc2;

FDCAN_HandleTypeDef hfdcan1;

/* Definitions for xReadTemp */
osThreadId_t xReadTempHandle;
const osThreadAttr_t xReadTemp_attributes = {
  .name = "xReadTemp",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 128 * 4
};
/* Definitions for xSendCAN */
osThreadId_t xSendCANHandle;
const osThreadAttr_t xSendCAN_attributes = {
  .name = "xSendCAN",
  .priority = (osPriority_t) osPriorityAboveNormal,
  .stack_size = 128 * 4
};
/* USER CODE BEGIN PV */

/* =================== Periféricos manuais (não-IOC) ===================== */
/* TIM6: gatilho periódico do ADC2 a 10 Hz (TRGO=Update).
 * IWDG: watchdog independente de hardware (~2s).
 * Ambos vivem em USER CODE para sobreviver à regeneração via .ioc.       */
TIM_HandleTypeDef  htim6;
IWDG_HandleTypeDef hiwdg;

/* ==================== Mutex for Temperature Buffer ===================== */
/**
 * @brief Thread-safe protection for the tempBuffer[] array.
 * synchronizes access between xReadTemp (producer) and xSendCAN (consumer).
 */
osMutexId_t tempBufferMutexHandle;
const osMutexAttr_t tempBufferMutex_attributes = {.name = "tempBufferMutex"};

/* ===================== Buffers de aquisição ============================== */
uint16_t rawAdcBuffer[numberOfThermistors];          // Buffer DMA p/ amostras brutas do ADC
float tempBuffer[numberOfThermistors];               // Temperaturas processadas (compartilhado via mutex)
extern uint16_t filteredAdcBuffer[numberOfThermistors]; // Definido em adc.c

/* CAN1 TX globals (FDCAN1TxData / FDCAN1TxHeader) intentionally NOT externed:
 * cada TX site agora constrói header + payload locais (ver can.c::initCan1TxHeader). */

/* =================== Fila de recepção CAN ================================ */
osMessageQueueId_t canRxQueueHandle;                 // Handle da fila (extern declarado em can.h)

/* ===================== Flags de status do sistema ======================== */
// int thermistorFault = 0;                         // Variável legada (não usada — bloco de detecção foi reescrito).
thermStatus readStatus = 0;                         // Status da última verificação de conexão

/* ================ Variáveis de status/debug CAN ========================= */
CAN_TxStatus_t lastTxStatus = CAN_TX_OK;            // Resultado do último burst TX
#ifdef testLoopback
CAN_RxMsg_t lastRxMsg;                               // Última mensagem recebida (debug)
uint32_t rxLastId = 0;                               // Último ID recebido (debug)
#endif
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_ADC2_Init(void);
void xReadTempFunction(void *argument);
void xSendCANFunction(void *argument);

/* USER CODE BEGIN PFP */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/**
 * @brief Inicializa TIM6 como gerador de TRGO periódico p/ trigger do ADC.
 *        f_TIM6 = APB1Timer / ((PSC+1)(ARR+1)) = 85MHz / (8500 * 1000) = 10 Hz
 *        TRGO=Update sai uma vez por ciclo, dispara a sequência de 16 conversões.
 *        Sem IRQ — só o canal interno TRGO importa.
 */
static void MX_TIM6_Init_User(void)
{
	__HAL_RCC_TIM6_CLK_ENABLE();

	htim6.Instance               = TIM6;
	htim6.Init.Prescaler         = 8499;   /* 85MHz / 8500 = 10 kHz */
	htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
	htim6.Init.Period            = 999;    /* 10 kHz / 1000 = 10 Hz */
	htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim6) != HAL_OK) {
		Error_Handler();
	}

	TIM_MasterConfigTypeDef sMasterConfig = {0};
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode     = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief Inicializa o IWDG (Independent Watchdog).
 *        LSI nominal = 32 kHz. Prescaler 64 → 500 Hz. Reload 999 → ~2s timeout.
 *        Refresh deve ser chamado pelo xSendCAN a cada 100ms (margem 20×).
 */
static void MX_IWDG_Init_User(void)
{
	hiwdg.Instance       = IWDG;
	hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
	hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
	hiwdg.Init.Reload    = 999;
	if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
		Error_Handler();
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
  MX_FDCAN1_Init();
  MX_ADC2_Init();
  /* USER CODE BEGIN 2 */
  HAL_ADCEx_Calibration_Start(&hadc2, ADC_SINGLE_ENDED);

  /* Inicializa periféricos manuais (não-IOC): TIM6 (gatilho ADC) + IWDG (watchdog). */
  MX_TIM6_Init_User();
  MX_IWDG_Init_User();

  /* Arma a watchdog de RX da Master: a partir daqui, se passar
   * CAN_RX_MASTER_TIMEOUT_MS sem nenhum frame da Master, dispara SDC. */
  lastMasterRxTick = HAL_GetTick();
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  tempBufferMutexHandle = osMutexNew(&tempBufferMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  canRxQueueHandle =
      osMessageQueueNew(CAN_RX_QUEUE_SIZE, sizeof(CAN_RxMsg_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of xReadTemp */
  xReadTempHandle = osThreadNew(xReadTempFunction, NULL, &xReadTemp_attributes);

  /* creation of xSendCAN */
  xSendCANHandle = osThreadNew(xSendCANFunction, NULL, &xSendCAN_attributes);

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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV4;
  RCC_OscInitStruct.PLL.PLLN = 85;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
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
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.GainCompensation = 0;
  hadc2.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.NbrOfConversion = 16;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc2.Init.OversamplingMode = ENABLE;
  hadc2.Init.Oversampling.Ratio         = ADC_OVERSAMPLING_RATIO_16;
  hadc2.Init.Oversampling.RightBitShift = ADC_RIGHTBITSHIFT_4;
  hadc2.Init.Oversampling.TriggeredMode = ADC_TRIGGEREDMODE_SINGLE_TRIGGER;
  hadc2.Init.Oversampling.OversamplingStopReset = ADC_REGOVERSAMPLING_CONTINUED_MODE;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
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
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_3;
  sConfig.Rank = ADC_REGULAR_RANK_3;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = ADC_REGULAR_RANK_4;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_5;
  sConfig.Rank = ADC_REGULAR_RANK_5;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_6;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_7;
  sConfig.Rank = ADC_REGULAR_RANK_7;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_8;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_9;
  sConfig.Rank = ADC_REGULAR_RANK_9;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = ADC_REGULAR_RANK_10;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = ADC_REGULAR_RANK_11;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_12;
  sConfig.Rank = ADC_REGULAR_RANK_12;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_13;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_14;
  sConfig.Rank = ADC_REGULAR_RANK_14;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_15;
  sConfig.Rank = ADC_REGULAR_RANK_15;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_17;
  sConfig.Rank = ADC_REGULAR_RANK_16;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */
  /* Override IOC: trigger externo TIM6_TRGO + single-shot.
   * TIM6 não está no .ioc (vive em USER CODE), então não pode aparecer no
   * dropdown de trigger do CubeMX — patcheamos aqui após HAL_ADC_Init().
   * Re-chamar HAL_ADC_Init reconfigura os registradores sem perder canais. */
  hadc2.Init.ContinuousConvMode   = DISABLE;
  hadc2.Init.ExternalTrigConv     = ADC_EXTERNALTRIG_T6_TRGO;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  if (HAL_ADC_Init(&hadc2) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 5;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 26;
  hfdcan1.Init.NominalTimeSeg2 = 7;
  hfdcan1.Init.DataPrescaler = 1;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 1;
  hfdcan1.Init.DataTimeSeg2 = 1;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
  FDCAN_FilterTypeDef sFilterConfig;

  sFilterConfig.IdType = FDCAN_STANDARD_ID;
  sFilterConfig.FilterIndex = 0;
  sFilterConfig.FilterType = FDCAN_FILTER_MASK;
  sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  sFilterConfig.FilterID1 = 0;
  sFilterConfig.FilterID2 = 0;

  if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
    Error_Handler();
  }

  HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);
  HAL_FDCAN_Start(&hfdcan1);

  /* Note: FDCAN1TxHeader global removida — todo TX site constroi header local
   * via can.c::initCan1TxHeader() (todos os 9 campos populados). */
  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7|userLED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC7 userLED_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_7|userLED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* ====================== INTERRUPT CALLBACKS (ISR) ====================== */

/**
 * @brief  ADC DMA Completion Callback.
 * Transfers raw 16-channel samples to rawAdcBuffer[] and signals xReadTemp.
 * Minimal ISR footprint: 1 FreeRTOS notification.
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;
  vTaskNotifyGiveFromISR(xReadTempHandle, &xHigherPriorityTaskWoken);
  portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/**
 * @brief  FDCAN RX FIFO 0 Callback.
 * Implements "Deferred Interrupt Processing":
 * 1. Reads hardware message to local stack struct.
 * 2. Pushes struct to the FreeRTOS message queue.
 * 3. xSendCAN thread drains the queue during its period.
 */
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t RxFifo0ITs) {
  if (hfdcan->Instance == FDCAN1) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
      CAN_RxMsg_t rxMsg;
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;

      if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &rxMsg.header,
                                 rxMsg.data) == HAL_OK) {
        /* Watchdog de comunicação: registra tick do último frame recebido.
         * Qualquer frame conta (Master heartbeat 0x00A em particular).
         * xSendCAN dispara SDC se passar de CAN_RX_MASTER_TIMEOUT_MS sem RX. */
        if (rxMsg.header.Identifier == idMaster) {
          lastMasterRxTick = HAL_GetTick();
        }

        xQueueSendFromISR((QueueHandle_t)canRxQueueHandle, &rxMsg,
                          &xHigherPriorityTaskWoken);
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
      }
    }
  }
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_xReadTempFunction */
/**
 * @brief  Thread de leitura de temperatura (Prioridade: Normal)
 *
 * Responsável pela aquisição e processamento dos 16 canais de temperatura.
 * Fluxo a cada iteração (~100ms):
 *   1. Inicia conversão DMA do ADC2 (16 canais simultaneamente)
 *   2. Dorme até o DMA completar (Task Notification do callback)
 *   3. Aplica filtro de mediana (remove spikes de ruído EMI)
 *   4. Aplica filtro IIR (suaviza oscilações residuais)
 *   5. Verifica integridade do sensor (curto/aberto)
 *   6. Converte ADC → Volts → °C e grava no buffer compartilhado (mutex)
 */
/* USER CODE END Header_xReadTempFunction */
void xReadTempFunction(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Main thermal acquisition loop (~10Hz) — timing agora controlado por TIM6_TRGO
   * (sem osDelay no loop; o bloqueio fica no Take aguardando DMA Complete). */
  static bool filtersInitialized = false;
  uint8_t   adcTimeoutCount = 0;

  /* Arma o ADC para o primeiro trigger e liga o gerador TIM6 (TRGO=Update, 10Hz). */
  if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)rawAdcBuffer, numberOfThermistors) != HAL_OK) {
    Error_Handler();
  }
  if (HAL_TIM_Base_Start(&htim6) != HAL_OK) {
    Error_Handler();
  }

  for (;;) {
    /* Wait for completion signal from HAL_ADC_ConvCpltCallback.
     * Timeout 500ms (5× o período) detecta TIM6/DMA travado — depois de 3 timeouts
     * consecutivos dispara SDC. xSendCAN ainda refresca o IWDG enquanto isso. */
    if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500)) == 0) {
      adcTimeoutCount++;
      if (adcTimeoutCount >= 3) {
        Error_Handler();  /* ADC freeze: trigger/DMA não está completando */
      }
      /* Re-arma DMA — talvez tenha tido OVR ou estado inconsistente */
      HAL_ADC_Stop_DMA(&hadc2);
      HAL_ADC_Start_DMA(&hadc2, (uint32_t *)rawAdcBuffer, numberOfThermistors);
      continue;
    }
    adcTimeoutCount = 0;

    HAL_ADC_Stop_DMA(&hadc2);

    /* Prime DSP filters on first runtime iteration */
    if (!filtersInitialized) {
      initTemperatureFilters(rawAdcBuffer);
      filtersInitialized = true;
    }

    /* Process all 16 thermistor channels */
    bool thermFaultThisCycle = false;
    for (int i = 0; i < numberOfThermistors; i++) {
      /* Step 1: Median Filter (5-tap window) - spike elimination */
      uint16_t medianADC = applyMedianFilter(rawAdcBuffer[i], i);

      /* Step 2: IIR Filter (alpha=1/8) - signal smoothing. Função já escreve em
       * filteredAdcBuffer[i] internamente — não reatribuir (era store duplicado). */
      (void)applyIIRFilter(medianADC, i);

      /* Step 3: Diagnostic verification (Short/Open protection) */
      readStatus = checkThermistorConnection(filteredAdcBuffer[i]);
      if (readStatus != OK) {
        thermFaultThisCycle = true;
      }
    }

    /* Step 4: Physical unit conversion sob um único mutex acquire/release.
     * NOTA (race conhecida): granularizar por iteração permitiria xSendCAN
     * (prioridade maior) preemptar no meio e tirar snapshot Frankenstein.
     * Mantemos UM acquire em bloco — burst CAN sempre vê um estado coerente. */
    if (osMutexAcquire(tempBufferMutexHandle, osWaitForever) == osOK) {
      for (int i = 0; i < numberOfThermistors; i++) {
        tempBuffer[i] = convertVoltageToTemperature(
                          convertBitsToVoltage(filteredAdcBuffer[i]));
      }
#ifdef slave2
      /* HACK: termistor 1 do slave 2 fisicamente quebrado — espelha do canal 0 */
      tempBuffer[1] = tempBuffer[0];
#endif
#ifdef slave3
      /* HACK: termistores 1, 12, 13 do slave 3 fisicamente quebrados — derivados */
      tempBuffer[1]  = tempBuffer[0]  + 0.12f;
      tempBuffer[12] = tempBuffer[10] + 0.67f;
      tempBuffer[13] = tempBuffer[0]  + 0.0910f;
#endif
      osMutexRelease(tempBufferMutexHandle);
    }

    /* Step 5: Se algum termistor desconectado, reporta na CAN e dispara SDC.
     * Religado conforme decisão de safety — README promete essa proteção. */
    if (thermFaultThisCycle) {
      sendReadingErrorInfoIntoCAN();
      osDelay(5);          /* dá tempo do frame entrar na FIFO antes do halt */
      Error_Handler();     /* Trigger SDC: open/short = bateria desprotegida */
    }

    /* Re-arma DMA para o próximo trigger do TIM6 — sem osDelay aqui. */
    if (HAL_ADC_Start_DMA(&hadc2, (uint32_t *)rawAdcBuffer, numberOfThermistors) != HAL_OK) {
      Error_Handler();
    }
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_xSendCANFunction */
/**
 * @brief  Thread de envio CAN e processamento de recepção (Prioridade: Acima do Normal)
 *
 * Responsável por:
 *   1. Drenar a fila de mensagens CAN recebidas (se testLoopback ativo)
 *   2. Copiar o buffer de temperaturas (protegido por mutex)
 *   3. Enviar as 16 temperaturas via CAN (8 quadros por burst)
 *   4. Piscar LED de heartbeat (GPIO PC7)
 *
 * Prioridade acima do normal para garantir que o envio CAN não
 * seja atrasado pela leitura de temperatura (que é mais lenta).
 */
/* USER CODE END Header_xSendCANFunction */
void xSendCANFunction(void *argument)
{
  /* USER CODE BEGIN xSendCANFunction */
  /* Main reporting loop (~10Hz update rate, period precision via osDelayUntil) */
  float localTempBuffer[numberOfThermistors];
  CAN_RxMsg_t rxMsg;
  uint32_t nextWake = osKernelGetTickCount();

  for (;;) {

    /* ====== Watchdog hardware refresh ====== */
    /* IWDG reload = 2s; chamado a cada 100ms tem 20× de margem. Se essa task
     * congelar (deadlock, stack overflow, etc.), IWDG reseta a CPU. */
    HAL_IWDG_Refresh(&hiwdg);

    /* ====== RX Watchdog: Master ainda viva? ====== */
    /* Se passou >CAN_RX_MASTER_TIMEOUT_MS sem frame da Master, a rede está morta —
     * sem essa decisão central de comando, o slave decide pelo SDC defensivo. */
    if ((HAL_GetTick() - lastMasterRxTick) > CAN_RX_MASTER_TIMEOUT_MS) {
      Error_Handler();  /* SDC: master perdida */
    }

    /* ====== CAN Reception Handling (Loopback/Testing) ====== */
#ifdef testLoopback
    while (osMessageQueueGet(canRxQueueHandle, &rxMsg, NULL, 0) == osOK) {
      lastRxMsg = rxMsg;
      rxLastId = rxMsg.header.Identifier;
      HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_8);
    }
#else
    /* Drena a queue mesmo em produção para não acumular — não processa payload */
    while (osMessageQueueGet(canRxQueueHandle, &rxMsg, NULL, 0) == osOK) { }
#endif

    /* ====== Temperature Reporting Block ====== */
    /* Copy latest thermal data to local buffer using quick mutex lock */
    if (osMutexAcquire(tempBufferMutexHandle, osWaitForever) == osOK) {
      memcpy(localTempBuffer, tempBuffer, sizeof(localTempBuffer));
      osMutexRelease(tempBufferMutexHandle);
    }

    /* Transmit burst to Master — baseID derivado uma vez via SLAVE_ID. */
    lastTxStatus = sendTemperatureToMaster(localTempBuffer, (uint16_t)idSlaveBurst0);

    /* Heartbeat LED: Toggles every 100ms to indicate OS health */
    HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_7);

    /* osDelayUntil dá período preciso (compensa jitter de execução). */
    nextWake += 100;
    osDelayUntil(nextWake);
  }
  /* USER CODE END xSendCANFunction */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, SET);
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
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
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
