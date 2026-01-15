/**
 * @file bsp_uart.c
 * @brief UART Driver with FreeRTOS Semaphore for DMA Safety
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
extern UART_HandleTypeDef huart3;

/* --- Internal Context --- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t  rx_buffer[UART_RX_BUF_SIZE];
    volatile uint16_t rx_tail_pos;
    AraCallback_t rx_cplt_cb;
} UartContext_t;

static UartContext_t ctx_debug = {
        .huart = &huart3,
        .rx_tail_pos = 0,
        .rx_cplt_cb = NULL
};

/* [IPC Resources] */
static SemaphoreHandle_t tx_sem = NULL; // 发送完成信号量
static char tx_buf[128];                // 发送缓冲区 (由信号量保护，无需双缓冲)

/* --- Helper Functions --- */
static uint16_t GetDmaHead(void) {
    if (!ctx_debug.huart->hdmarx) return 0;
    uint16_t counter = __HAL_DMA_GET_COUNTER(ctx_debug.huart->hdmarx);
    uint16_t head = UART_RX_BUF_SIZE - counter;
    if (head >= UART_RX_BUF_SIZE) head = 0;
    return head;
}

/* --- API Implementation --- */

void BSP_UART_Init(void)
{
    // 1. 启动 DMA 接收 (Circular)
    HAL_UART_Receive_DMA(ctx_debug.huart, ctx_debug.rx_buffer, UART_RX_BUF_SIZE);

    // 2. 创建二值信号量
    tx_sem = xSemaphoreCreateBinary();

    // 3. 初始状态必须 Give (允许第一次发送)
    if (tx_sem != NULL) {
        xSemaphoreGive(tx_sem);
    }
}

void BSP_UART_Printf(const char *format, ...)
{
    va_list args;
    int len;
    bool use_dma = false;

    // 格式化字符串 (先写入 buffer，注意：此时假设没有多任务竞争，或已被信号量保护)
    // 严谨做法：如果是抢占式内核，这里的 vsnprintf 应该也在临界区或被信号量包含
    // 但为了代码结构清晰，我们先获取锁，再格式化

    // [检查环境] 调度器运行了吗？
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING && tx_sem != NULL)
    {
        // --- 模式 A: RTOS 运行中 (使用 DMA + 信号量) ---
        // 尝试获取信号量，等待上一帧发完
        if (xSemaphoreTake(tx_sem, pdMS_TO_TICKS(UART_TX_TIMEOUT_MS)) == pdTRUE) {
            use_dma = true;

            // 安全区域：现在 tx_buf 归我了
            va_start(args, format);
            len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);
            va_end(args);

            if (len > 0) {
                // 启动 DMA 发送
                // 注意：这里不 Give 信号量！等中断里 Give
                if (HAL_UART_Transmit_DMA(ctx_debug.huart, (uint8_t*)tx_buf, len) != HAL_OK) {
                    // 如果启动失败（及其罕见），必须归还信号量，否则下次死锁
                    xSemaphoreGive(tx_sem);
                }
            } else {
                // 长度为0，归还信号量
                xSemaphoreGive(tx_sem);
            }
        } else {
            // 获取信号量超时 (上一帧卡死了)
            // 可选：HAL_UART_AbortTransmit(ctx_debug.huart);
            // 这里选择直接丢弃本次打印，保护系统不卡死
        }
    }
    else
    {
        // --- 模式 B: 初始化阶段或 ISR 中 (使用阻塞发送) ---
        // 此时没有任务切换能力，只能死等

        // 1. 等待上一次可能的 DMA 结束
        while(HAL_UART_GetState(ctx_debug.huart) == HAL_UART_STATE_BUSY_TX);

        va_start(args, format);
        len = vsnprintf(tx_buf, sizeof(tx_buf), format, args);
        va_end(args);

        if (len > 0) {
            HAL_UART_Transmit(ctx_debug.huart, (uint8_t*)tx_buf, len, 100);
        }
    }
}

AraStatus_t BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    // 注意：此函数未集成信号量，仅用于 Echo 测试或特定逻辑
    // 如果与 Printf 混用，可能会破坏信号量逻辑。
    // 建议 Echo 测试也改用阻塞，或者复用 Printf 逻辑。
    // 为保持兼容性，这里仅做基础保护：

    if (dev != BSP_UART_DEBUG || p_data == NULL) return ARA_ERR_PARAM;
    if (HAL_UART_GetState(ctx_debug.huart) == HAL_UART_STATE_BUSY_TX) return ARA_BUSY;

    if (HAL_UART_Transmit_DMA(ctx_debug.huart, p_data, len) != HAL_OK) return ARA_ERR_IO;
    return ARA_OK;
}

uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len)
{
    if (dev != BSP_UART_DEBUG || p_data == NULL) return 0;

    uint16_t head = GetDmaHead();
    uint16_t tail = ctx_debug.rx_tail_pos;
    uint16_t bytes_available = (head >= tail) ? (head - tail) : ((UART_RX_BUF_SIZE - tail) + head);

    if (bytes_available == 0) return 0;

    uint16_t read_len = (len < bytes_available) ? len : bytes_available;
    uint16_t cnt = 0;

    while (cnt < read_len) {
        p_data[cnt++] = ctx_debug.rx_buffer[tail++];
        if (tail >= UART_RX_BUF_SIZE) tail = 0;
    }

    ctx_debug.rx_tail_pos = tail;
    return read_len;
}

void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, AraCallback_t cb)
{
    if (dev == BSP_UART_DEBUG) ctx_debug.rx_cplt_cb = cb;
}

/* --- ISR Callbacks --- */

// 1. 接收完成 (RingBuffer 通知)
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == ctx_debug.huart && ctx_debug.rx_cplt_cb) {
        ctx_debug.rx_cplt_cb();
    }
}

// 2. [关键] 发送完成 (信号量释放)
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == ctx_debug.huart) {
        // 唤醒因等待 tx_sem 而阻塞的任务 (Printf)
        if (tx_sem != NULL) {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            xSemaphoreGiveFromISR(tx_sem, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}