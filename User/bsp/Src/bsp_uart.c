/**
 * @file bsp_uart.c
 * @brief UART Driver with FreeRTOS Semaphore for DMA Safety (Multi-Instance)
 */

#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <stdarg.h>
#include <string.h>

/* [FreeRTOS Includes] */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* --- Configuration --- */
#define UART_RX_BUF_SIZE    (256)   // 必须是2的幂
#define UART_TX_TIMEOUT_MS  (100)   // 信号量等待超时时间

/* --- Hardware Resources --- */
extern UART_HandleTypeDef huart3;   // DEBUG
extern UART_HandleTypeDef huart1;   // ELRS (新增)

/* --- Internal Context --- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t             rx_buffer[UART_RX_BUF_SIZE];
    volatile uint16_t   rx_tail_pos;
    AraCallback_t       rx_cplt_cb;
} UartContext_t;

/* 多实例上下文数组 */
static UartContext_t uart_ctx[BSP_UART_NUM];

/* [IPC Resources - 仅供 DEBUG 发送使用] */
static SemaphoreHandle_t tx_sem = NULL; // 发送完成信号量
static char tx_buf[128];                // 发送缓冲区 (由信号量保护)

/* --- Helper Functions --- */
static uint16_t GetDmaHead(BspUart_Dev_t dev) {
    if (dev >= BSP_UART_NUM || !uart_ctx[dev].huart->hdmarx) return 0;

    uint16_t counter = __HAL_DMA_GET_COUNTER(uart_ctx[dev].huart->hdmarx);
    uint16_t head = UART_RX_BUF_SIZE - counter;
    if (head >= UART_RX_BUF_SIZE) head = 0;

    return head;
}

/* --- API Implementation --- */

void BSP_UART_Init(void)
{
    // 1. 绑定硬件句柄
    uart_ctx[BSP_UART_DEBUG].huart = &huart3;
    uart_ctx[BSP_UART_ELRS].huart  = &huart1;

    // 初始化状态
    for (int i = 0; i < BSP_UART_NUM; i++) {
        uart_ctx[i].rx_tail_pos = 0;
        uart_ctx[i].rx_cplt_cb = NULL;
    }

    // 2. 启动 DMA 接收 (Circular)
    HAL_UART_Receive_DMA(uart_ctx[BSP_UART_DEBUG].huart, uart_ctx[BSP_UART_DEBUG].rx_buffer, UART_RX_BUF_SIZE);
    HAL_UART_Receive_DMA(uart_ctx[BSP_UART_ELRS].huart,  uart_ctx[BSP_UART_ELRS].rx_buffer,  UART_RX_BUF_SIZE);

    // 3. 创建二值信号量 (供 Printf 使用)
    tx_sem = xSemaphoreCreateBinary();

    // 4. 初始状态必须 Give (允许第一次发送)
    if (tx_sem != NULL) {
        xSemaphoreGive(tx_sem);
    }
}

void BSP_UART_Printf(const char *format, ...)
{
    va_list args;
    int len;

    UART_HandleTypeDef *debug_huart = uart_ctx[BSP_UART_DEBUG].huart;

    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && tx_sem != NULL)
    {
        // --- 模式 A: RTOS 运行中 (使用 DMA + 信号量) ---
        if (xSemaphoreTake(tx_sem, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS)) == pdTRUE) {
            va_start(args, format);
            len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);
            va_end(args);

            if (len > 0) {
                if (HAL_UART_Transmit_DMA(debug_huart, (uint8_t*)tx_buf, len) != HAL_OK) {
                    xSemaphoreGive(tx_sem);
                }
            } else {
                xSemaphoreGive(tx_sem);
            }
        }
    }
    else
    {
        // --- 模式 B: 初始化阶段或 ISR 中 (使用阻塞发送) ---
        while(HAL_UART_GetState(debug_huart) == HAL_UART_STATE_BUSY_TX);

        va_start(args, format);
        len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);
        va_end(args);

        if (len > 0) {
            HAL_UART_Transmit(debug_huart, (uint8_t*)tx_buf, len, 100);
        }
    }
}

AraStatus_t BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_UART_NUM || p_data == NULL) return ARA_ERR_PARAM;
    if (HAL_UART_GetState(uart_ctx[dev].huart) == HAL_UART_STATE_BUSY_TX) return ARA_BUSY;

    if (HAL_UART_Transmit_DMA(uart_ctx[dev].huart, p_data, len) != HAL_OK) return ARA_ERR_IO;
    return ARA_OK;
}

uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_UART_NUM || p_data == NULL) return 0;

    UartContext_t *ctx = &uart_ctx[dev];

    uint16_t head = GetDmaHead(dev);
    uint16_t tail = ctx->rx_tail_pos;
    uint16_t bytes_available = (head >= tail) ? (head - tail) : ((UART_RX_BUF_SIZE - tail) + head);

    if (bytes_available == 0) return 0;

    uint16_t read_len = (len < bytes_available) ? len : bytes_available;
    uint16_t cnt = 0;

    while (cnt < read_len) {
        p_data[cnt++] = ctx->rx_buffer[tail++];
        if (tail >= UART_RX_BUF_SIZE) tail = 0;
    }

    ctx->rx_tail_pos = tail;
    return read_len;
}

void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, AraCallback_t cb)
{
    if (dev < BSP_UART_NUM) {
        uart_ctx[dev].rx_cplt_cb = cb;
    }
}

/* --- ISR Callbacks --- */

// 1. 接收完成 (RingBuffer 通知)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 遍历查找是哪个外设触发了中断
    for (int i = 0; i < BSP_UART_NUM; i++) {
        if (huart == uart_ctx[i].huart && uart_ctx[i].rx_cplt_cb) {
            uart_ctx[i].rx_cplt_cb();
            break;
        }
    }
}

// 2. 发送完成 (信号量释放)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 只有 DEBUG 端口配置了发送信号量
    if (huart == uart_ctx[BSP_UART_DEBUG].huart) {
        if (tx_sem != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(tx_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}