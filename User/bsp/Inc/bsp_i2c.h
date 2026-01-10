/**
 * @file bsp_i2c.h
 * @brief I2C Wrapper with Thread-Safe Blocking & DMA Callback support
 */

#ifndef BSP_I2C_H
#define BSP_I2C_H

#include "ara_def.h"

/* --- Device Definition --- */
typedef enum {
    BSP_I2C_MOTION = 0, // AS5600 Encoder Bus
    BSP_I2C_NUM
} BspI2c_Dev_t;

/* --- API --- */

void BSP_I2C_Init(void);

/* === Blocking API (Initialization / Low Speed) === */

AraStatus_t BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
AraStatus_t BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);
AraStatus_t BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr);

/* === DMA / Async API (High Speed Loop) === */

/**
 * @brief Register callback for DMA completion
 * @param cb Function to call when DMA Transaction completes (called from ISR context!)
 */
void BSP_I2C_SetRxCpltCallback(BspI2c_Dev_t dev, AraCallback_t cb);

/**
 * @brief  Start DMA Read Transaction
 * @return ARA_OK if started, ARA_BUSY if I2C is in use
 */
AraStatus_t BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);

#endif // BSP_I2C_H