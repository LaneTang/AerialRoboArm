/**
 * @file drv_as5600.h
 * @brief AS5600 Driver (L2) - Async DMA Version
 * @note  Requires BSP_I2C_V2.1 with Callback support
 */

#ifndef DRV_AS5600_H
#define DRV_AS5600_H

#include "ara_def.h"
#include "bsp_i2c.h"

/* --- Configuration & Context --- */
typedef struct {
    /* Hardware Handle */
    BspI2c_Dev_t i2c_dev;
    uint16_t     dev_addr;      // 7-bit Address (0x36)

    /* DMA Buffer (Zero-Copy target) */
    /* Note: Must be 2-byte aligned if Architecture requires,
       but F103 handles byte alignment okay for this small buffer */
    uint8_t      dma_buffer[2];

    /* State */
    bool         initialized;
} DrvAS5600_Context_t;

/* --- API --- */

/**
 * @brief Initialize AS5600 Driver and Register Async Callback
 * * @param p_ctx       Driver Context
 * @param i2c_dev     BSP I2C Device ID
 * @param on_complete ISR Callback function provided by L4 Task.
 * (Usually sends a Task Notification/Semaphore)
 * @return ARA_OK or Error code
 */
AraStatus_t DrvAS5600_Init(DrvAS5600_Context_t *p_ctx,
                           BspI2c_Dev_t i2c_dev,
                           AraCallback_t on_complete);

/**
 * @brief Trigger Non-blocking DMA Read of Raw Angle
 * * @note  Calls BSP_I2C_ReadMem_DMA.
 * When finished, the 'on_complete' callback will fire.
 */
AraStatus_t DrvAS5600_TriggerUpdate(DrvAS5600_Context_t *p_ctx);

/**
 * @brief Parse the DMA buffer to get Raw Angle
 * * @note  Should be called ONLY after 'on_complete' has fired.
 * @return 12-bit Raw Angle (0 - 4095)
 */
uint16_t DrvAS5600_GetRawAngle(DrvAS5600_Context_t *p_ctx);

#endif // DRV_AS5600_H