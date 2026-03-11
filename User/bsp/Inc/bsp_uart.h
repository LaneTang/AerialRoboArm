/**
 * @file bsp_uart.h
 * @brief UART Driver with DMA RingBuffer
 */

#ifndef BSP_UART_H
#define BSP_UART_H

#include "ara_def.h"
#include <stdarg.h>

/* --- Device Definition --- */
typedef enum {
    BSP_UART_DEBUG = 0,
    BSP_UART_ELRS,      // <--- 新增此行，用于绑定接收机串口
    BSP_UART_NUM
} BspUart_Dev_t;

/* --- API --- */

void BSP_UART_Init(void);

/* Debug / Logging */
void BSP_UART_Printf(const char *format, ...);

/* DMA Send (Non-blocking) */
AraStatus_t BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);

/* RingBuffer Read */
uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);

/* Async Receive Callback */
void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, AraCallback_t cb);

#endif // BSP_UART_H