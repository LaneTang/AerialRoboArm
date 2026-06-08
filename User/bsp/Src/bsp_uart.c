/**
 * @file bsp_uart.c
 * @brief UART Driver: FSUS interrupt-RX / blocking-TX on USART2,
 *        DMA ring buffers on USART1/3, DEBUG printf over DMA.
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
#define UART_DEBUG_RX_BUF_SIZE   (128U)
#define UART_ELRS_RX_BUF_SIZE    (1024U)
#define UART_FSUS_RX_BUF_SIZE    (256U)
#define UART_TX_TIMEOUT_MS       (100U)

/* Half-duplex notification bits (legacy ST3215 HD state machine) */
#define HD_NOTIFY_RX_DONE        (1U << 0)
#define HD_NOTIFY_RX_ERROR       (1U << 1)

/* --- Hardware Resources --- */
extern UART_HandleTypeDef huart3;   // DEBUG
extern UART_HandleTypeDef huart1;   // ELRS
extern UART_HandleTypeDef huart2;   // FSUS (USART2, full-duplex)

/* --- Internal Context --- */
typedef struct {
    UART_HandleTypeDef *huart;
    uint8_t            *rx_buffer;
    uint16_t            rx_buffer_size;
    volatile uint16_t   rx_tail_pos;
    AraCallback_t       rx_cplt_cb;
} UartContext_t;

static uint8_t s_debug_rx_buffer[UART_DEBUG_RX_BUF_SIZE];
static uint8_t s_elrs_rx_buffer[UART_ELRS_RX_BUF_SIZE];

/* 多实例上下文数组 */
static UartContext_t uart_ctx[BSP_UART_NUM];

/* [IPC Resources - 仅供 DEBUG 发送使用] */
static SemaphoreHandle_t tx_sem = NULL; // 发送完成信号量

/* =============================================================================
 * FSUS interrupt-mode RX ring buffer (USART2)
 *
 * ISR pushes one byte at a time; task reads via BSP_UART_Fsus_Recv.
 * ============================================================================= */

static uint8_t  s_fsus_rx_buf[UART_FSUS_RX_BUF_SIZE];
static volatile uint16_t s_fsus_rx_head = 0U;  /* ISR writes */
static uint16_t s_fsus_rx_tail = 0U;           /* task reads */
static uint8_t  s_fsus_rx_byte;                /* 1-byte IT target */
static char tx_buf[128];                // 发送缓冲区 (由信号量保护)

/* Bring-up diagnostics: TX/RX byte counters visible to upper layers. */
static volatile uint32_t s_fsus_tx_bytes = 0U;
static volatile uint32_t s_fsus_rx_bytes = 0U;
static volatile uint32_t s_elrs_rx_bytes = 0U;
static volatile uint32_t s_uart_rx_dma_recoveries = 0U;

/* =============================================================================
 * Half-Duplex transaction state (ST3215 / USART2)
 *
 * One outstanding transaction at a time. Three observable states:
 *   IDLE          - no transaction
 *   TX_INFLIGHT   - HAL_UART_Transmit_DMA in progress; on TC we kick RX
 *   RX_INFLIGHT   - HAL_UART_Receive_DMA in progress; on RC/error we notify
 * The waiting task blocks in HD_WaitRx on a direct-to-task notification.
 * ============================================================================= */

typedef enum {
    HD_STATE_IDLE = 0,
    HD_STATE_TX_INFLIGHT,
    HD_STATE_RX_INFLIGHT,
} HdState_t;

typedef struct {
    volatile HdState_t  state;
    uint8_t            *rx_buf;
    uint16_t            expect_rx_len;
    volatile uint16_t   rx_received_len;
    TaskHandle_t        waiting_task;
    volatile uint8_t    last_error;   /* 0 = ok, non-zero = HAL error code */
} HdCtx_t;

static HdCtx_t s_hd_ctx;

/* --- Helper Functions --- */
static uint16_t GetDmaHead(BspUart_Dev_t dev) {
    if (dev >= BSP_UART_NUM) return 0;

    UartContext_t *ctx = &uart_ctx[dev];
    if (ctx->huart == NULL || ctx->huart->hdmarx == NULL ||
        ctx->rx_buffer == NULL || ctx->rx_buffer_size == 0U) {
        return 0;
    }

    uint16_t counter = __HAL_DMA_GET_COUNTER(ctx->huart->hdmarx);
    uint16_t head = (uint16_t)(ctx->rx_buffer_size - counter);
    if (head >= ctx->rx_buffer_size) head = 0;

    return head;
}

static void RestartCircularDmaRx(BspUart_Dev_t dev)
{
    if (dev >= BSP_UART_NUM) return;

    UartContext_t *ctx = &uart_ctx[dev];
    if (ctx->huart == NULL || ctx->rx_buffer == NULL ||
        ctx->rx_buffer_size == 0U) {
        return;
    }

    ctx->rx_tail_pos = 0U;
    (void)HAL_UART_Receive_DMA(ctx->huart,
                               ctx->rx_buffer,
                               ctx->rx_buffer_size);
    s_uart_rx_dma_recoveries++;
}

/* --- API Implementation --- */

void BSP_UART_Init(void)
{
    /* 1. Bind hardware handles. */
    uart_ctx[BSP_UART_DEBUG].huart  = &huart3;
    uart_ctx[BSP_UART_ELRS].huart   = &huart1;
    uart_ctx[BSP_UART_ST3215].huart = &huart2;   /* FSUS, full-duplex IT RX */

    uart_ctx[BSP_UART_DEBUG].rx_buffer       = s_debug_rx_buffer;
    uart_ctx[BSP_UART_DEBUG].rx_buffer_size  = sizeof(s_debug_rx_buffer);
    uart_ctx[BSP_UART_ELRS].rx_buffer        = s_elrs_rx_buffer;
    uart_ctx[BSP_UART_ELRS].rx_buffer_size   = sizeof(s_elrs_rx_buffer);

    /* Initialize per-port software state. */
    for (int i = 0; i < BSP_UART_NUM; i++) {
        uart_ctx[i].rx_tail_pos = 0;
        uart_ctx[i].rx_cplt_cb = NULL;
    }

    /* 2. Start Circular DMA RX for DEBUG / ELRS ring buffers. */
    HAL_UART_Receive_DMA(uart_ctx[BSP_UART_DEBUG].huart,
                         uart_ctx[BSP_UART_DEBUG].rx_buffer,
                         uart_ctx[BSP_UART_DEBUG].rx_buffer_size);
    HAL_UART_Receive_DMA(uart_ctx[BSP_UART_ELRS].huart,
                         uart_ctx[BSP_UART_ELRS].rx_buffer,
                         uart_ctx[BSP_UART_ELRS].rx_buffer_size);

    /* 3. Start interrupt-based RX on USART2 (FSUS).
     *     One byte per interrupt, pushed to s_fsus_rx_buf in the ISR. */
    HAL_UART_Receive_IT(&huart2, &s_fsus_rx_byte, 1);

    /* 4. Create the DEBUG TX semaphore used by BSP_UART_Printf. */
    tx_sem = xSemaphoreCreateBinary();

    /* 5. Allow the first DEBUG TX. */
    if (tx_sem != NULL) {
        xSemaphoreGive(tx_sem);
    }

    /* 6. Half-duplex context init (legacy, kept for reference). */
    memset(&s_hd_ctx, 0, sizeof(s_hd_ctx));
    s_hd_ctx.state = HD_STATE_IDLE;
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
    if (ctx->rx_buffer == NULL || ctx->rx_buffer_size == 0U) return 0;

    /* ST3215 RX is event-driven, not ringbuffer; reject ringbuffer reads on it */
    if (dev == BSP_UART_ST3215) return 0;

    uint16_t head = GetDmaHead(dev);
    uint16_t tail = ctx->rx_tail_pos;
    uint16_t bytes_available = (head >= tail) ? (head - tail) : ((ctx->rx_buffer_size - tail) + head);

    if (bytes_available == 0) return 0;

    uint16_t read_len = (len < bytes_available) ? len : bytes_available;
    uint16_t cnt = 0;

    while (cnt < read_len) {
        p_data[cnt++] = ctx->rx_buffer[tail++];
        if (tail >= ctx->rx_buffer_size) tail = 0;
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

/* =============================================================================
 * Half-Duplex API: real implementation (demo_v7 phase 2)
 *
 * Wire-level sequence per transaction:
 *   1. HalfDuplex_Transact():
 *        - capture rx_buf / expect_rx_len / waiting_task
 *        - state = TX_INFLIGHT
 *        - HAL_UART_Transmit_DMA(huart2, tx_buf, tx_len)
 *   2. HAL_UART_TxCpltCallback(huart2):
 *        - state = RX_INFLIGHT
 *        - HAL_UART_Receive_DMA(huart2, rx_buf, expect_rx_len)
 *   3. HAL_UART_RxCpltCallback(huart2):
 *        - state = IDLE
 *        - rx_received_len = expect_rx_len
 *        - xTaskNotifyFromISR(waiting_task, HD_NOTIFY_RX_DONE, ...)
 *      OR
 *      HAL_UART_ErrorCallback(huart2):
 *        - state = IDLE, last_error = code
 *        - xTaskNotifyFromISR(waiting_task, HD_NOTIFY_RX_ERROR, ...)
 *   4. HD_WaitRx():
 *        - xTaskNotifyWait(timeout)
 *        - return ARA_OK / ARA_ERR_IO / ARA_TIMEOUT
 *      Caller MUST call HD_AbortRx after a TIMEOUT.
 *
 * Single ST3215 servo, single waiting task: no per-transaction queueing.
 * ============================================================================= */

AraStatus_t BSP_UART_HalfDuplex_Transact(BspUart_Dev_t  dev,
                                         const uint8_t *tx_buf,
                                         uint16_t       tx_len,
                                         uint8_t       *rx_buf,
                                         uint16_t       expect_rx_len)
{
    if ((dev != BSP_UART_ST3215) || (tx_buf == NULL) || (rx_buf == NULL) ||
        (tx_len == 0U) || (expect_rx_len == 0U)) {
        return ARA_ERR_PARAM;
    }
    if (expect_rx_len > UART_FSUS_RX_BUF_SIZE) {
        return ARA_ERR_PARAM;
    }

    UART_HandleTypeDef *huart = uart_ctx[dev].huart;
    if (huart == NULL) {
        return ARA_ERR_IO;
    }

    /* Reject overlapping transactions. Caller is expected to fully consume
     * (WaitRx + AbortRx if needed) before issuing the next one. */
    if (s_hd_ctx.state != HD_STATE_IDLE) {
        return ARA_BUSY;
    }

    /* Snapshot the transaction context BEFORE starting TX so the TC ISR
     * has all the info it needs to kick off the RX leg. */
    s_hd_ctx.rx_buf          = rx_buf;
    s_hd_ctx.expect_rx_len   = expect_rx_len;
    s_hd_ctx.rx_received_len = 0U;
    s_hd_ctx.waiting_task    = xTaskGetCurrentTaskHandle();
    s_hd_ctx.last_error      = 0U;
    s_hd_ctx.state           = HD_STATE_TX_INFLIGHT;

    /* Clear any stale notification on the calling task to avoid a previous
     * aborted transact triggering an immediate spurious wake-up. */
    (void)ulTaskNotifyValueClear(s_hd_ctx.waiting_task,
                                 HD_NOTIFY_RX_DONE | HD_NOTIFY_RX_ERROR);

    /* HDSEL: HAL_HalfDuplex_Init already sets USART_CR3_HDSEL. The peripheral
     * automatically tristates RX while transmitting. We just kick TX DMA. */
    HAL_StatusTypeDef hr = HAL_UART_Transmit_DMA(huart, (uint8_t *)tx_buf, tx_len);
    if (hr != HAL_OK) {
        s_hd_ctx.state = HD_STATE_IDLE;
        return ARA_ERR_IO;
    }
    return ARA_OK;
}

AraStatus_t BSP_UART_HD_WaitRx(BspUart_Dev_t dev,
                               uint32_t      timeout_ms,
                               uint16_t     *out_rx_len)
{
    if (dev != BSP_UART_ST3215) {
        return ARA_ERR_PARAM;
    }
    if (out_rx_len != NULL) {
        *out_rx_len = 0U;
    }

    /* Defend against caller pairing without a preceding Transact. */
    if (s_hd_ctx.state == HD_STATE_IDLE && s_hd_ctx.last_error == 0U &&
        s_hd_ctx.rx_received_len == 0U) {
        return ARA_ERR_PARAM;
    }

    uint32_t notify_bits = 0U;
    BaseType_t got = xTaskNotifyWait(0U, /* don't clear on entry */
                                     HD_NOTIFY_RX_DONE | HD_NOTIFY_RX_ERROR,
                                     &notify_bits,
                                     pdMS_TO_TICKS(timeout_ms));
    if (got != pdTRUE) {
        return ARA_TIMEOUT;
    }
    if (notify_bits & HD_NOTIFY_RX_ERROR) {
        return ARA_ERR_IO;
    }
    if (notify_bits & HD_NOTIFY_RX_DONE) {
        if (out_rx_len != NULL) {
            *out_rx_len = s_hd_ctx.rx_received_len;
        }
        return ARA_OK;
    }
    return ARA_TIMEOUT;
}

void BSP_UART_HD_AbortRx(BspUart_Dev_t dev)
{
    if (dev != BSP_UART_ST3215) return;
    UART_HandleTypeDef *huart = uart_ctx[dev].huart;
    if (huart == NULL) return;

    /* Stop whichever DMA leg is currently armed. HAL_UART_Abort handles
     * both TX and RX paths and tears the DMA down cleanly. */
    (void)HAL_UART_Abort(huart);

    s_hd_ctx.state           = HD_STATE_IDLE;
    s_hd_ctx.rx_buf          = NULL;
    s_hd_ctx.expect_rx_len   = 0U;
    s_hd_ctx.rx_received_len = 0U;
    s_hd_ctx.waiting_task    = NULL;
    s_hd_ctx.last_error      = 0U;
}

/* --- ISR Callbacks --- */

// 1. 接收完成
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    /* FSUS (USART2): push each received byte into the FSUS ring buffer. */
    if (huart == uart_ctx[BSP_UART_ST3215].huart) {
        uint16_t next = (s_fsus_rx_head + 1U) % UART_FSUS_RX_BUF_SIZE;
        if (next != s_fsus_rx_tail) {
            s_fsus_rx_buf[s_fsus_rx_head] = s_fsus_rx_byte;
            s_fsus_rx_head = next;
            s_fsus_rx_bytes++;
        }
        HAL_UART_Receive_IT(huart, &s_fsus_rx_byte, 1);
        return;
    }

    if (huart == uart_ctx[BSP_UART_ELRS].huart) {
        s_elrs_rx_bytes += UART_ELRS_RX_BUF_SIZE / 2U;
    }

    /* DEBUG / ELRS: legacy DMA ringbuffer-cplt notification path. */
    for (int i = 0; i < BSP_UART_NUM; i++) {
        if (huart == uart_ctx[i].huart && uart_ctx[i].rx_cplt_cb) {
            uart_ctx[i].rx_cplt_cb();
            break;
        }
    }
}

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart == uart_ctx[BSP_UART_ELRS].huart) {
        s_elrs_rx_bytes += UART_ELRS_RX_BUF_SIZE / 2U;
    }
}

// 2. 发送完成
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    /* DEBUG: release the printf semaphore. */
    if (huart == uart_ctx[BSP_UART_DEBUG].huart) {
        if (tx_sem != NULL) {
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(tx_sem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
        return;
    }

    /* FSUS uses blocking TX — no callback action needed for USART2. */
}

// 3. 错误回调
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
    /* FSUS (USART2): re-arm interrupt RX after an error. */
    if (huart == uart_ctx[BSP_UART_ST3215].huart) {
        HAL_UART_Receive_IT(huart, &s_fsus_rx_byte, 1);
        return;
    }

    /* DEBUG: re-give the semaphore so a parity/overrun does not deadlock printf. */
    if (huart == uart_ctx[BSP_UART_DEBUG].huart) {
        if (tx_sem != NULL) {
            BaseType_t hpw = pdFALSE;
            xSemaphoreGiveFromISR(tx_sem, &hpw);
            portYIELD_FROM_ISR(hpw);
        }
        RestartCircularDmaRx(BSP_UART_DEBUG);
        return;
    }

    /* ELRS (USART1): STM32 HAL treats every UART error in DMA RX mode as
     * blocking and aborts the RX DMA before calling this callback. Re-arm
     * circular RX here so a transient FE/NE/ORE cannot permanently stall RC. */
    if (huart == uart_ctx[BSP_UART_ELRS].huart) {
        RestartCircularDmaRx(BSP_UART_ELRS);
        return;
    }
}

/* =============================================================================
 * FSUS transaction API (USART2, full-duplex, interrupt RX + blocking TX)
 * ============================================================================= */

void BSP_UART_Fsus_Send(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return;
    }
    if (HAL_UART_Transmit(&huart2, (uint8_t *)data, len, 100) == HAL_OK) {
        s_fsus_tx_bytes += len;
    }
}

uint16_t BSP_UART_Fsus_Recv(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }
    uint16_t count = 0U;
    while ((count < len) && (s_fsus_rx_tail != s_fsus_rx_head)) {
        data[count++] = s_fsus_rx_buf[s_fsus_rx_tail];
        s_fsus_rx_tail = (s_fsus_rx_tail + 1U) % UART_FSUS_RX_BUF_SIZE;
    }
    return count;
}

void BSP_UART_Fsus_Flush(void)
{
    s_fsus_rx_tail = s_fsus_rx_head;
}

uint32_t BSP_UART_Fsus_GetTxBytes(void) { return s_fsus_tx_bytes; }
uint32_t BSP_UART_Fsus_GetRxBytes(void) { return s_fsus_rx_bytes; }

uint32_t BSP_UART_Elrs_GetRxBytes(void)
{
    return s_elrs_rx_bytes;
}

uint32_t BSP_UART_RxDma_GetRecoveries(void)
{
    return s_uart_rx_dma_recoveries;
}
