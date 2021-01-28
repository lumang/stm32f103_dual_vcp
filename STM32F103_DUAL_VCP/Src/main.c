/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f1xx_hal.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include "SEGGER_RTT.h"
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
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart3_rx;
DMA_HandleTypeDef hdma_usart3_tx;

DMA_HandleTypeDef hdma_memtomem_dma1_channel1;

volatile uint32_t u32LEDcounter;
uint32_t u32LEDblinkEnable;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
ctx_t ctx;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

//  SCB->VTOR = 0x08004000; //uncomment this line for YAB use, set IROM1 Start address coordinately.
  
  int i;
  uart_ctx_t * uart_ctx;
  memset(&ctx, 0, sizeof(ctx_t));
  
  ctx.uart1.name = "UART1";
  ctx.uart1.huart = &huart1;
  ctx.uart1.hdma_rx = &hdma_usart1_rx;
  ctx.uart1.hdma_tx = &hdma_usart1_tx;
  ctx.uart1.irq_num = USART1_IRQn;
  ctx.uart2.name = "UART2";
  ctx.uart2.huart = &huart2;
  ctx.uart2.hdma_rx = &hdma_usart2_rx;
  ctx.uart2.hdma_tx = &hdma_usart2_tx;
  ctx.uart2.irq_num = USART2_IRQn;
  ctx.uart3.name = "UART3";
  ctx.uart3.huart = &huart3;
  ctx.uart3.hdma_rx = &hdma_usart3_rx;
  ctx.uart3.hdma_tx = &hdma_usart3_tx;
  ctx.uart3.irq_num = USART3_IRQn;
  
  ctx.memcpy_dma = &hdma_memtomem_dma1_channel1;
  
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
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_2);
  HAL_NVIC_SetPriority(USB_HP_CAN1_TX_IRQn  , 0, 2);
  HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn , 0, 3);
  HAL_NVIC_SetPriority(USART1_IRQn          , 0, 1);
  HAL_NVIC_SetPriority(USART2_IRQn          , 0, 1);
  HAL_NVIC_SetPriority(USART3_IRQn          , 0, 1);
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn   , 0, 2); // DMA for memory copy.
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn   , 1, 1); // UART3 Tx
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn   , 1, 0); // UART3 Rx
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn   , 1, 1); // UART1 Tx
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn   , 1, 0); // UART1 Rx
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn   , 1, 0); // UART2 Rx
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn   , 1, 1); // UART2 Tx

  __HAL_UART_DISABLE(&huart1);
  __HAL_UART_DISABLE(&huart2);
  __HAL_UART_DISABLE(&huart3);
  
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
    // __wfe();
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
    for (i = 0; i < 2; i++) {
      uart_ctx = (i == 0) ? &ctx.uart1 : &ctx.uart2;

      if (uart_ctx->buf_idx != uart_ctx->buf.idx) {
        int buf_idx = uart_ctx->buf_idx;
        while (CDC_Transmit_FS((uint8_t *)uart_ctx->buf.data[buf_idx], uart_ctx->buf.len[buf_idx], 2 * i) == USBD_BUSY) {
          /* Until data out. */
        }
        uart_ctx->buf_idx = buf_idx ? 0 : 1;
        // SEGGER_RTT_printf(0, "mloop: buf=%d\n", buf_idx);
      } 

      if (uart_ctx->buf.rest_len > 0) {
        int tx_len = uart_ctx->buf.rest_len;
        uart_ctx->buf.rest_len = 0;
        uart_ctx->buf_idx = 0;
        while (CDC_Transmit_FS((uint8_t *)uart_ctx->buf.data_rest, tx_len, 2 * i) == USBD_BUSY) {
          /* Until data out. */
        }
        // SEGGER_RTT_printf(0, "rest: len=%d\n", tx_len);
      }
    }
    //LED
    if (u32LEDblinkEnable)
    {
      if (u32LEDcounter & 0x00000080)
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET); //LED on
      }
      else
      {
        HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET); //LED off
        u32LEDblinkEnable = 0;
      }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL_DIV1_5;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/**
  * @brief USART1 Initialization Function
  */
static void MX_USART1_UART_Init(void)
{
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  */
static void MX_USART2_UART_Init(void)
{
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART3 Initialization Function
  */
static void MX_USART3_UART_Init(void)
{
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* Configure DMA request hdma_memtomem_dma1_channel1 on DMA1_Channel1 */
  hdma_memtomem_dma1_channel1.Instance = DMA1_Channel1;
  hdma_memtomem_dma1_channel1.Init.Direction = DMA_MEMORY_TO_MEMORY;
  hdma_memtomem_dma1_channel1.Init.PeriphInc = DMA_PINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.MemInc = DMA_MINC_ENABLE;
  hdma_memtomem_dma1_channel1.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
  hdma_memtomem_dma1_channel1.Init.Mode = DMA_NORMAL;
  hdma_memtomem_dma1_channel1.Init.Priority = DMA_PRIORITY_MEDIUM;
  if (HAL_DMA_Init(&hdma_memtomem_dma1_channel1) != HAL_OK)
  {
    Error_Handler();
  }  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel3_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel3_IRQn);
  /* DMA1_Channel4_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel4_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel4_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
}

/* USER CODE BEGIN 4 */

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * const uart_ctx = (huart == &huart1) ? &ctx.uart1 : &ctx.uart2;
  
  if (uart_ctx->buf.idx != 0) {
    uart_ctx->buf.idx = 0;
  }

  if (uart_ctx->buf_idx == 1) {
    // SEGGER_RTT_printf(0, "rxhalf: %s; [X]\n", uart_ctx->name);
    return;
  }

  // In Rx Half callback, the length of received data is half length of double buffer.
  uart_ctx->buf.len[0] = DBL_BUF_LEN; 
  // Set index of double buffer to next.
  uart_ctx->buf.idx = 1;
  // SEGGER_RTT_printf(0, "rxhalf: %s; \n", uart_ctx->name);
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * const uart_ctx = (huart == &huart1) ? &ctx.uart1 : &ctx.uart2;

  if (uart_ctx->buf.idx != 1) {
    uart_ctx->buf.idx = 1;
  }
  if (uart_ctx->buf_idx == 0) {
    // SEGGER_RTT_printf(0, "rxcmpl: %s; [X]\n", uart_ctx->name);
    return;
  }

  // In Rx callback, the length of received data is half length of double buffer.
  uart_ctx->buf.len[1] = DBL_BUF_LEN; 
  // Set index of double buffer to next.
  uart_ctx->buf.idx = 0;
  // SEGGER_RTT_printf(0, "rxcmpl: %s; \n", uart_ctx->name);
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  uart_ctx_t * const uart_ctx = (huart == &huart1) ? &ctx.uart1 : &ctx.uart2;
  
//  SEGGER_RTT_printf(0, "uart error: %s; \n", uart_ctx->name);
  HAL_UART_DMAStop(huart);
  HAL_UART_Receive_DMA(huart, (uint8_t *)uart_ctx->buf.data[0], DBL_BUF_TOTAL_LEN);
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
//    SEGGER_RTT_printf(0, "_Error_Handler: %s #%d\n", file, line);
    HAL_Delay(1000);
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
