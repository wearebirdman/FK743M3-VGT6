/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */
#include "FreeRTOS.h"
#include "cmsis_os2.h"

#include <stdarg.h>
#include <stdio.h>

/* 接收缓冲：DMA1 无法访问 DTCM，必须放 AXI SRAM；32 字节对齐匹配缓存行，整段独占无共享 */
__attribute__((section(".dma_buffer"), aligned(32)))
static uint8_t s_RxBuf[UART_RX_BUF_MAX];
static volatile uint16_t s_RxLen;         /* 本帧有效字符串长度（ISR 写、任务读） */

static osMutexId_t s_UartTxMutexHandle;   /* 打印互斥锁 */
static osSemaphoreId_t s_UartRxSemHandle; /* 接收就绪信号量 */
/* USER CODE END 0 */

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */

  /** Initializes the peripherals clock
  */
    PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_USART1;
    PeriphClkInitStruct.Usart16ClockSelection = RCC_USART16CLKSOURCE_D2PCLK2;
    if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
    {
      Error_Handler();
    }

    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 DMA Init */
    /* USART1_RX Init */
    hdma_usart1_rx.Instance = DMA1_Stream0;
    hdma_usart1_rx.Init.Request = DMA_REQUEST_USART1_RX;
    hdma_usart1_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
    hdma_usart1_rx.Init.PeriphInc = DMA_PINC_DISABLE;
    hdma_usart1_rx.Init.MemInc = DMA_MINC_ENABLE;
    hdma_usart1_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
    hdma_usart1_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
    hdma_usart1_rx.Init.Mode = DMA_NORMAL;
    hdma_usart1_rx.Init.Priority = DMA_PRIORITY_LOW;
    hdma_usart1_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
    if (HAL_DMA_Init(&hdma_usart1_rx) != HAL_OK)
    {
      Error_Handler();
    }

    __HAL_LINKDMA(uartHandle,hdmarx,hdma_usart1_rx);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 DMA DeInit */
    HAL_DMA_DeInit(uartHandle->hdmarx);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
/* 启动串口运行环境：创建 RTOS 同步对象，挂接 DMA 空闲不定长接收
 * 需在调度器启动后调用（由 UART 接收任务入口调用） */
void Uart_Init(void)
{
  if (s_UartTxMutexHandle == NULL)
    s_UartTxMutexHandle = osMutexNew(NULL);
  if (s_UartRxSemHandle == NULL)
    s_UartRxSemHandle = osSemaphoreNew(1U, 0U, NULL);

  __HAL_UART_CLEAR_OREFLAG(&huart1); /* 清启动前可能残留的溢出标志 */
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_RxBuf, UART_RX_BUF_MAX);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);
}

/* printf 风格调试发送（互斥锁整行原子输出）
 * fmt: 格式串
 * ...: 可变参数 */
void Uart_Printf(const char *fmt, ...)
{
  char buf[UART_PRINTF_BUF_MAX];
  va_list args;
  int total;

  va_start(args, fmt);
  total = vsnprintf(buf, sizeof(buf), fmt, args);
  va_end(args);

  if (total <= 0)
    return;

  if (total > (int)(sizeof(buf) - 1))
    total = (int)(sizeof(buf) - 1); /* 超长截断，只发实际写入部分 */

  if (s_UartTxMutexHandle != NULL)
    osMutexAcquire(s_UartTxMutexHandle, osWaitForever);
  HAL_UART_Transmit(&huart1, (uint8_t *)buf, (uint16_t)total, HAL_MAX_DELAY);
  if (s_UartTxMutexHandle != NULL)
    osMutexRelease(s_UartTxMutexHandle);
}

/* DMA 空闲不定长接收回调（ISR）：整帧规范化为标准字符串后通知接收任务
 * 规范化 = 剥离尾部 \r/\n 结束符 + 补 '\0' + 记录有效长度，随后立即重挂 DMA
 * 调试口低吞吐场景；若任务未及时消费，下一帧会覆盖本缓冲 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
  if (huart == &huart1)
  {
    uint16_t len = size;

    SCB_InvalidateDCache_by_Addr(s_RxBuf, UART_RX_BUF_MAX); /* DMA 写入不经缓存，读前失效才能取到真实帧数据 */

    while (len > 0U && (s_RxBuf[len - 1U] == '\r' || s_RxBuf[len - 1U] == '\n'))
      len--; /* 剥离尾部结束符 */

    if (len >= UART_RX_BUF_MAX)
      len = UART_RX_BUF_MAX - 1U; /* 缓冲占满且无结束符：截尾保证可补 '\0' */
    s_RxBuf[len] = '\0';
    s_RxLen = len;

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, s_RxBuf, UART_RX_BUF_MAX);
    __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    if (len > 0U) /* 纯结束符/空帧不通知 */
      osSemaphoreRelease(s_UartRxSemHandle);
  }
}

/* 阻塞等待一条规范化消息（无结束符、'\0' 结尾）
 * len: 输出有效字符串长度（不含 '\0'）
 * 返回: 消息缓冲指针；调用方应尽快拷走（缓冲会被下一帧覆盖） */
uint8_t *Uart_RxWaitFrame(uint16_t *len)
{
  osSemaphoreAcquire(s_UartRxSemHandle, osWaitForever);
  *len = s_RxLen;
  return s_RxBuf;
}
/* USER CODE END 1 */

