/**
 * @file drv_as5600.c
 * @brief AS5600 Driver Implementation (DMA Async) - Fixed Address Logic
 * @author ARA Project Coder
 */

#include "drv_as5600.h"

/* --- Constants --- */
#define AS5600_DEFAULT_ADDR     (0x36) // 7-bit Address
#define AS5600_REG_RAW_ANGLE    (0x0C)
#define AS5600_MASK_12BIT       (0x0FFF)

/* --- API Implementation --- */

AraStatus_t DrvAS5600_Init(DrvAS5600_Context_t *p_ctx,
                           BspI2c_Dev_t i2c_dev,
                           AraCallback_t on_complete)
{
    if (p_ctx == NULL) {
        return ARA_ERR_PARAM;
    }

    // 1. Initialize Context
    p_ctx->i2c_dev = i2c_dev;

    // [Fix] Do NOT shift here. BSP layer handles the 8-bit conversion.
    // Use raw 7-bit address (0x36).
    p_ctx->dev_addr = AS5600_DEFAULT_ADDR;

    // Clear buffer
    p_ctx->dma_buffer[0] = 0;
    p_ctx->dma_buffer[1] = 0;
    p_ctx->initialized = false; // Mark false until check passes

    // 2. Register Callback with BSP Layer
    if (on_complete != NULL) {
        BSP_I2C_SetRxCpltCallback(i2c_dev, on_complete);
    }

    // 3. Verify device presence (Blocking check)
    // This is the moment of truth. If this passes, the sensor is online.
    AraStatus_t status = BSP_I2C_IsDeviceReady(p_ctx->i2c_dev, p_ctx->dev_addr);

    if (status != ARA_OK) {
        return ARA_ERR_DISCONNECTED; // Return error explicitly
    }

    // Only mark initialized if hardware ACKed
    p_ctx->initialized = true;
    return ARA_OK;
}

AraStatus_t DrvAS5600_TriggerUpdate(DrvAS5600_Context_t *p_ctx)
{
    if (p_ctx == NULL || !p_ctx->initialized) {
        return ARA_ERR_PARAM; // Changed from PARAM to RESOURCE to indicate state error
    }

    /* Start DMA Read */
    return BSP_I2C_ReadMem_DMA(
            p_ctx->i2c_dev,
            p_ctx->dev_addr,
            AS5600_REG_RAW_ANGLE,
            p_ctx->dma_buffer,
            2
    );
}

uint16_t DrvAS5600_GetRawAngle(DrvAS5600_Context_t *p_ctx)
{
    if (p_ctx == NULL) {
        return 0;
    }

    /* Parse Big Endian Data */
    uint16_t raw_angle;
    raw_angle  = ((uint16_t)p_ctx->dma_buffer[0] << 8);
    raw_angle |= (uint16_t)p_ctx->dma_buffer[1];

    return (raw_angle & AS5600_MASK_12BIT);
}