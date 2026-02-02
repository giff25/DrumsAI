#include "main.h"
#include "cmsis_os.h"
#include "stm32f446xx.h"
#include "stm32f4xx.h"
#include "stm32f4xx_hal_adc.h"
#include "stm32f4xx_hal_adc_ex.h"
#include <stdio.h>
#include <string.h>

/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  */

typedef StaticTask_t        osStaticThreadDef_t;
typedef StaticTimer_t       osStaticTimerDef_t;
typedef StaticEventGroup_t  osStaticEventGroupDef_t;

/* ADC / DMA sizing */
#define ADC_CHANNELS_TOTAL     9
#define ADC_SAMPLES_PER_HALF  1
#define ADC_DMA_HALF_SIZE     (ADC_CHANNELS_TOTAL * ADC_SAMPLES_PER_HALF)
#define ADC_DMA_FULL_SIZE     (2 * ADC_DMA_HALF_SIZE)

/* ADC handles */
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

/* DMA handles */
DMA_HandleTypeDef hdma_adc1;
DMA_HandleTypeDef hdma_adc2;
DMA_HandleTypeDef hdma_adc3;

/* Scheduler task */
osThreadId_t SchedulerHandle;
uint32_t SchedulerBuffer[200];
osStaticThreadDef_t SchedulerControlBlock;

const osThreadAttr_t Scheduler_attributes = {
  .name       = "Scheduler",
  .cb_mem     = &SchedulerControlBlock,
  .cb_size    = sizeof(SchedulerControlBlock),
  .stack_mem  = SchedulerBuffer,
  .stack_size = sizeof(SchedulerBuffer),
  .priority   = osPriorityNormal,
};

/* ADC acquisition task */
osThreadId_t AquireSensorDatHandle;
uint32_t AquireSensorDatBuffer[500];
osStaticThreadDef_t AquireSensorDatControlBlock;

const osThreadAttr_t AquireSensorDat_attributes = {
  .name       = "AquireSensorDat",
  .cb_mem     = &AquireSensorDatControlBlock,
  .cb_size    = sizeof(AquireSensorDatControlBlock),
  .stack_mem  = AquireSensorDatBuffer,
  .stack_size = sizeof(AquireSensorDatBuffer),
  .priority   = osPriorityRealtime1,
};

/* ADC processing task */
osThreadId_t ProcessADCHandle;
uint32_t ADCSignal[2000];
osStaticThreadDef_t ProcessADCControlBlock;

const osThreadAttr_t ProcessADC_attributes = {
  .name       = "ProcessADC",
  .cb_mem     = &ProcessADCControlBlock,
  .cb_size    = sizeof(ProcessADCControlBlock),
  .stack_mem  = ADCSignal,
  .stack_size = sizeof(ADCSignal),
  .priority   = osPriorityHigh5,
};

/* Periodic ADC request timer */
osTimerId_t ADC_request_PeriodHandle;
osStaticTimerDef_t myTimer01ControlBlock;

const osTimerAttr_t ADC_request_Period_attributes = {
  .name    = "ADC_request_Period",
  .cb_mem = &myTimer01ControlBlock,
  .cb_size = sizeof(myTimer01ControlBlock),
};

/* Mutex protecting sensor resources */
osMutexId_t SensorZHandle;

const osMutexAttr_t SensorZ_attributes = {
  .name = "SensorZ"
};

/* Event flags indicating DMA buffer readiness */
osEventFlagsId_t SensorZAvailableHandle;
osStaticEventGroupDef_t SensorZAvailableControlBlock;

const osEventFlagsAttr_t SensorZAvailable_attributes = {
  .name    = "SensorZAvailable",
  .cb_mem = &SensorZAvailableControlBlock,
  .cb_size = sizeof(SensorZAvailableControlBlock),
};

/* Double-buffered ADC DMA buffer */
uint16_t adcDmaBuffer[ADC_DMA_FULL_SIZE];

/* Function prototypes */
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC3_Init(void);

void GetSignal(void *argument);
void Callback01(void *argument);

/**
  * @brief  Application entry point
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();

  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_ADC2_Init();
  MX_ADC3_Init();

  /* Start ADCs in continuous circular DMA mode */
  HAL_ADCEx_MultiModeStart_DMA(
      &hadc1,
      (uint32_t *)adcDmaBuffer,
      ADC_DMA_FULL_SIZE
  );

  osKernelInitialize();

  SensorZHandle = osMutexNew(&SensorZ_attributes);

  ADC_request_PeriodHandle =
      osTimerNew(Callback01, osTimerPeriodic, NULL,
                 &ADC_request_Period_attributes);

  ProcessADCHandle =
      osThreadNew(GetSignal, NULL, &ProcessADC_attributes);

  SensorZAvailableHandle =
      osEventFlagsNew(&SensorZAvailable_attributes);

  osKernelStart();

  while (1) {}
}

/**
  * @brief  DMA half-transfer complete callback
  */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
    osEventFlagsSet(SensorZAvailableHandle, 0x01);
}

/**
  * @brief  DMA full-transfer complete callback
  */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
  if (hadc->Instance == ADC1)
    osEventFlagsSet(SensorZAvailableHandle, 0x02);
}


/**
  * @brief  ADC processing task
  */
void GetSignal(void *argument)
{
  uint32_t flags;
  uint16_t recvBuff[9];

  for (;;)
  {
    flags = osEventFlagsWait(
        SensorZAvailableHandle,
        0x01 | 0x02,
        osFlagsWaitAny,
        osWaitForever
    );

    if (flags & 0x01)
      memcpy(recvBuff, adcDmaBuffer,
             ADC_DMA_HALF_SIZE * sizeof(uint16_t));
    else if (flags & 0x02)
      memcpy(recvBuff,
             &adcDmaBuffer[ADC_DMA_HALF_SIZE],
             ADC_DMA_HALF_SIZE * sizeof(uint16_t));
  }
}

/**
  * @brief Periodic timer callback
  */
void Callback01(void *argument) {}

/**
  * @brief System Clock Configuration
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;

  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
    Error_Handler();

  RCC_ClkInitStruct.ClockType =
      RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;

  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
    Error_Handler();
}

/**
  * @brief ADC1 initialization (master, triple mode)
  */
static void MX_ADC1_Init(void)
{
  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 3;
  hadc1.Init.DMAContinuousRequests = ENABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  HAL_ADC_Init(&hadc1);

  multimode.Mode = ADC_TRIPLEMODE_REGSIMULT;
  multimode.DMAAccessMode = ADC_DMAACCESSMODE_2;
  multimode.TwoSamplingDelay = ADC_TWOSAMPLINGDELAY_5CYCLES;

  HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode);

  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  sConfig.Channel = ADC_CHANNEL_4;  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_8;  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);

  sConfig.Channel = ADC_CHANNEL_10; sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc1, &sConfig);
}

/**
  * @brief ADC2 initialization (slave)
  */
static void MX_ADC2_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ENABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 3;
  hadc2.Init.DMAContinuousRequests = ENABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  HAL_ADC_Init(&hadc2);

  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  sConfig.Channel = ADC_CHANNEL_1;  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_6;  sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);

  sConfig.Channel = ADC_CHANNEL_11; sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

/**
  * @brief ADC3 initialization (slave)
  */
static void MX_ADC3_Init(void)
{
  ADC_ChannelConfTypeDef sConfig = {0};

  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 3;
  hadc3.Init.DMAContinuousRequests = ENABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SEQ_CONV;

  HAL_ADC_Init(&hadc3);

  sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;

  sConfig.Channel = ADC_CHANNEL_0;  sConfig.Rank = 1;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_12; sConfig.Rank = 2;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);

  sConfig.Channel = ADC_CHANNEL_13; sConfig.Rank = 3;
  HAL_ADC_ConfigChannel(&hadc3, &sConfig);
}

/**
  * @brief DMA controller initialization
  */
static void MX_DMA_Init(void)
{
  __HAL_RCC_DMA2_CLK_ENABLE();

  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);

  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
}

/**
  * @brief GPIO initialization
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = USART_TX_Pin | USART_RX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief Fatal error handler
  */
void Error_Handler(void)
{
  __disable_irq();
  while (1) {}
}