/**
  ******************************************************************************
  * @file    bsp_uart.c
  * @brief   BSP UART 实现 (带回调机制)
  ******************************************************************************
  */

#include "bsp_uart.h"
#include "usart.h" // 包含 huart1, huart2
#include <string.h>

/* --- 内部管理结构体 --- */
typedef struct {
    UART_HandleTypeDef *huart;
    DMA_HandleTypeDef  *hdma_rx;

    // RX RingBuffer
    uint8_t  rx_buffer[UART_RX_BUF_SIZE];
    uint16_t rx_write_index;
    uint16_t rx_read_index;

    // TX Mutex
    osMutexId_t tx_mutex;

    // 接收完成回调 (Hook)
    BspUart_Callback_t rx_cplt_cb;
} BspUart_Handle_t;

extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart2_rx;

/* --- 静态实例 --- */
static BspUart_Handle_t uart_devs[BSP_UART_NUM] = {
        // [0] ELRS (USART1)
        {
                .huart = &huart1,
                .hdma_rx = &hdma_usart1_rx,
                .rx_read_index = 0,
                .tx_mutex = NULL,
                .rx_cplt_cb = NULL // 初始为空
        },
        // [1] DEBUG (USART2)
        {
                .huart = &huart2,
                .hdma_rx = &hdma_usart2_rx,
                .rx_read_index = 0,
                .tx_mutex = NULL,
                .rx_cplt_cb = NULL
        }
};

static const osMutexAttr_t uart_tx_mutex_attr = {
        .name = "UART_TX_Mutex",
        .attr_bits = osMutexPrioInherit,
};

/* --- 初始化 --- */
void BSP_UART_Init(void) {
    for (int i = 0; i < BSP_UART_NUM; i++) {
        uart_devs[i].tx_mutex = osMutexNew(&uart_tx_mutex_attr);

        // 开启 IDLE 中断
        __HAL_UART_ENABLE_IT(uart_devs[i].huart, UART_IT_IDLE);

        // 启动 DMA Circular 接收
        HAL_UART_Receive_DMA(uart_devs[i].huart, uart_devs[i].rx_buffer, UART_RX_BUF_SIZE);
    }
}

/* --- 注册回调函数 --- */
void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, BspUart_Callback_t cb) {
    if (dev < BSP_UART_NUM) {
        uart_devs[dev].rx_cplt_cb = cb;
    }
}

/* --- 核心：中断处理分发器 --- */
// 请在 stm32f1xx_it.c 的 USARTx_IRQHandler 中调用此函数
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart) {
    BspUart_Handle_t *dev = NULL;

    // 1. 匹配设备
    if (huart->Instance == USART1) {
        dev = &uart_devs[BSP_UART_ELRS];
    } else if (huart->Instance == USART2) {
        dev = &uart_devs[BSP_UART_DEBUG];
    }

    if (dev == NULL) return;

    // 2. 检查 IDLE 中断
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // 3. 执行上层注册的回调 (如果存在)
        if (dev->rx_cplt_cb != NULL) {
            dev->rx_cplt_cb();
        }
    }
}

/* --- 发送逻辑 (保持不变) --- */
HAL_StatusTypeDef BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len) {
    BspUart_Handle_t *h = &uart_devs[dev];
    if (osMutexAcquire(h->tx_mutex, 10) != osOK) return HAL_BUSY;
    HAL_StatusTypeDef status = HAL_UART_Transmit_DMA(h->huart, p_data, len);
    osMutexRelease(h->tx_mutex);
    return status;
}

HAL_StatusTypeDef BSP_UART_Send_Poll(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len) {
    return HAL_UART_Transmit(uart_devs[dev].huart, p_data, len, 100);
}

/* --- 接收逻辑 (保持不变) --- */
static uint16_t Get_DMA_Write_Index(BspUart_Handle_t *h) {
    return (UART_RX_BUF_SIZE - __HAL_DMA_GET_COUNTER(h->hdma_rx));
}

uint16_t BSP_UART_Available(BspUart_Dev_t dev) {
    BspUart_Handle_t *h = &uart_devs[dev];
    uint16_t write_idx = Get_DMA_Write_Index(h);

    if (write_idx >= h->rx_read_index) {
        return write_idx - h->rx_read_index;
    } else {
        return (UART_RX_BUF_SIZE - h->rx_read_index) + write_idx;
    }
}

uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len) {
    BspUart_Handle_t *h = &uart_devs[dev];
    uint16_t available = BSP_UART_Available(dev);
    uint16_t write_idx = Get_DMA_Write_Index(h);

    if (len > available) len = available;
    if (len == 0) return 0;

    if (write_idx >= h->rx_read_index) {
        memcpy(p_data, &h->rx_buffer[h->rx_read_index], len);
        h->rx_read_index += len;
    } else {
        uint16_t tail_len = UART_RX_BUF_SIZE - h->rx_read_index;
        if (len <= tail_len) {
            memcpy(p_data, &h->rx_buffer[h->rx_read_index], len);
            h->rx_read_index += len;
        } else {
            memcpy(p_data, &h->rx_buffer[h->rx_read_index], tail_len);
            memcpy(p_data + tail_len, &h->rx_buffer[0], len - tail_len);
            h->rx_read_index = len - tail_len;
        }
    }
    return len;
}

void BSP_UART_FlushRx(BspUart_Dev_t dev) {
    BspUart_Handle_t *h = &uart_devs[dev];
    h->rx_read_index = Get_DMA_Write_Index(h);
}