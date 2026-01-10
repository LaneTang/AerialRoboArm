/**
 * @file bsp_uart.c
 * @brief UART Driver Implementation (DMA RingBuffer + Blocking Printf)
 * @author ARA Project Coder
 */

#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include <stdio.h>
#include <string.h>

/* --- Configuration --- */
#define UART_RX_BUF_SIZE    (256)   // Must be power of 2 for efficiency if optimizing
#define UART_TX_TIMEOUT     (100)   // ms for blocking printf

/* --- Hardware Resources --- */
// External handles defined in main.c
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;
// Note: hdma_usart1_rx is needed to read CNDTR (Counter)

/* --- Internal Context --- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_buffer[UART_RX_BUF_SIZE];
    uint16_t rx_tail_pos;  // Software Read Index
    AraCallback_t rx_cplt_cb;
} UartContext_t;

static UartContext_t ctx_debug = {
        .huart = &huart3,
        .rx_tail_pos = 0,
        .rx_cplt_cb = NULL
};

/* --- Helper: Get DMA Head Position --- */
static uint16_t GetDmaHead(void)
{
    /* * CNDTR counts DOWN from Size to 0.
     * Head Index = Size - CNDTR
     */
    return UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(ctx_debug.huart->hdmarx);
}

/* --- API Implementation --- */

void BSP_UART_Init(void)
{
    // Start DMA Reception in Circular Mode
    // Data keeps overwriting old data if not read fast enough (Ring Buffer)
    HAL_UART_Receive_DMA(ctx_debug.huart, ctx_debug.rx_buffer, UART_RX_BUF_SIZE);
}

void BSP_UART_Printf(const char *format, ...)
{
    static char tx_buf[128]; // Local static buffer (Non-reentrant without lock)

    /* * CRITICAL SECTION START
     * In a strict OS environment, use a Mutex here.
     * For L1/BSP, we assume single-threaded debugging or accept minor collisions.
     */

    va_list args;
    va_start(args, format);

    // Format string
    int len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);

    va_end(args);

    if (len > 0) {
        // Blocking Transmit to ensure full message goes out
        HAL_UART_Transmit(ctx_debug.huart, (uint8_t*)tx_buf, (uint16_t)len, UART_TX_TIMEOUT);
    }
}

AraStatus_t BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_UART_NUM) return ARA_ERR_PARAM;
    // Currently only supporting Debug Port (USART1)
    if (dev != BSP_UART_DEBUG) return ARA_ERR_PARAM;

    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(ctx_debug.huart, p_data, len);

    if (status == HAL_BUSY) return ARA_BUSY;
    if (status != HAL_OK)   return ARA_ERR_IO;

    return ARA_OK;
}

uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    if (dev != BSP_UART_DEBUG) return 0;

    uint16_t head = GetDmaHead();
    uint16_t tail = ctx_debug.rx_tail_pos;
    uint16_t bytes_available = 0;

    // Calculate available bytes
    if (head >= tail) {
        bytes_available = head - tail;
    } else {
        bytes_available = (UART_RX_BUF_SIZE - tail) + head;
    }

    if (bytes_available == 0) {
        return 0;
    }

    // Cap read length to available data
    uint16_t read_len = (len < bytes_available) ? len : bytes_available;
    uint16_t cnt = 0;

    // Copy data (handling wrap-around)
    while (cnt < read_len) {
        p_data[cnt++] = ctx_debug.rx_buffer[tail++];

        if (tail >= UART_RX_BUF_SIZE) {
            tail = 0;
        }
    }

    // Update global tail
    ctx_debug.rx_tail_pos = tail;

    return read_len;
}

void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, AraCallback_t cb)
{
    if (dev == BSP_UART_DEBUG) {
        ctx_debug.rx_cplt_cb = cb;
    }
}

/* --- ISR Hooks --- */
/**
 * @brief Rx Transfer Completed Callback
 * @note  In Circular Mode, this is called when buffer is half-full (HT) or full (TC).
 * Usually used to wake up the processing task.
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == ctx_debug.huart) {
        if (ctx_debug.rx_cplt_cb) {
            ctx_debug.rx_cplt_cb();
        }
    }
}