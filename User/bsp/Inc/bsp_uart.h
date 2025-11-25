/**
  ******************************************************************************
  * @file    bsp_uart.h
  * @brief   BSP UART 驱动 (DMA Circular RX + RingBuffer + DMA TX)
  * @author  ARA Project Team
  ******************************************************************************
  */

#ifndef __BSP_UART_H
#define __BSP_UART_H

#include "stm32f1xx_hal.h" // 根据芯片型号修改
#include "cmsis_os.h"

/* --- 配置参数 --- */
#define UART_RX_BUF_SIZE    512   // 接收缓冲区大小 (字节)，建议 2^n
#define UART_TX_BUF_SIZE    256   // 发送缓冲区大小 (DMA暂存)

/* --- 设备定义 --- */
typedef enum {
    BSP_UART_ELRS = 0,  // 遥控器输入 (高波特率)
    BSP_UART_DEBUG,     // 调试/日志 (HC05)
    BSP_UART_NUM
} BspUart_Dev_t;

/* 当接收到 IDLE (一包数据结束) 时调用此回调 */
typedef void (*BspUart_Callback_t)(void);

/* --- 初始化 --- */
void BSP_UART_Init(void);

/* --- 发送接口 (TX) --- */
/**
 * @brief  [DMA] 发送数据 (非阻塞)
 * @note   适合在 FOC 循环或高频任务中打印日志，不会卡顿
 */
HAL_StatusTypeDef BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);
/**
 * @brief  [阻塞] 发送数据
 * @note   适合初始化阶段或系统 Fault 时的紧急打印
 */
HAL_StatusTypeDef BSP_UART_Send_Poll(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);


/* --- 接收接口 (RX - RingBuffer) --- */
/**
 * @brief  查询缓冲区中可读字节数
 */
uint16_t BSP_UART_Available(BspUart_Dev_t dev);
/**
 * @brief  从 RingBuffer 读取数据
 * @param  p_data: 目标缓冲区
 * @param  len:    希望读取的长度
 * @return 实际读取到的长度
 */
uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);
/**
 * @brief  清空接收缓冲区
 */
void BSP_UART_FlushRx(BspUart_Dev_t dev);

/**
 * @brief  为指定串口注册“接收完成”回调函数
 * @param  dev: 串口设备号
 * @param  cb:  上层驱动提供的函数指针 (例如 Drv_Elrs_RxHandler)
 */
void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, BspUart_Callback_t cb);

/* --- 中断处理入口 (供 stm32f1xx_it.c 调用) --- */
void BSP_UART_IRQHandler(UART_HandleTypeDef *huart);

#endif // __BSP_UART_H