/**
  ******************************************************************************
  * @file    bsp_i2c.h
  * @brief   BSP I2C 抽象层头文件 (支持 阻塞/DMA 双模式 + FreeRTOS Mutex)
  * @author  ARA Project Team
  ******************************************************************************
  */

#ifndef __BSP_I2C_H
#define __BSP_I2C_H

#include "stm32f1xx_hal.h"  // 根据你的芯片型号修改，如 stm32f4xx_hal.h
#include "cmsis_os.h"       // FreeRTOS CMSIS v2 接口

/* 设备定义 */
typedef enum {
    BSP_I2C_MOTION = 0, // 对应 I2C1: 动力域 (AS5600 + Laser)
    BSP_I2C_FUNC,       // 对应 I2C2: 功能域 (JY60)
    BSP_I2C_NUM         // 设备数量
} BspI2c_Dev_t;

/* 初始化 */
void BSP_I2C_Init(void);

/* ============================================================ */
/* 阻塞模式 API (Blocking)                 */
/* 适用于: 初始化、低频配置、不允许并行的原子操作           */
/* ============================================================ */

/**
 * @brief  [阻塞] 读取 I2C 设备寄存器
 * @note   包含 Mutex 保护，会阻塞当前任务直到传输完成或超时
 */
HAL_StatusTypeDef BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

/**
 * @brief  [阻塞] 写入 I2C 设备寄存器
 */
HAL_StatusTypeDef BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

/**
 * @brief  [阻塞] 检查设备是否在线
 */
HAL_StatusTypeDef BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr);


/* ============================================================ */
/* DMA 模式 API (Zero-Wait)                 */
/* 适用于: 高频周期性任务 (FOC loop, 传感器采集)          */
/* ============================================================ */

/**
 * @brief  [DMA] 读取 I2C 设备寄存器 (Yield CPU)
 * @note   1. 获取锁
 * 2. 启动 DMA
 * 3. 挂起当前任务 (FreeRTOS Blocked) 释放 CPU
 * 4. DMA 中断唤醒任务
 * 5. 释放锁
 */
HAL_StatusTypeDef BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

/**
 * @brief  [DMA] 写入 I2C 设备寄存器 (Yield CPU)
 */
HAL_StatusTypeDef BSP_I2C_WriteMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

#endif //__BSP_I2C_H