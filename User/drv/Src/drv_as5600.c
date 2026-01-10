/**
 * @file drv_as5600.c
 * @brief AS5600 Driver Implementation (DMA Async)
 * @author ARA Project Coder
 */

#include "drv_as5600.h"

/* --- Constants --- */
#define AS5600_DEFAULT_ADDR     (0x36)
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
    p_ctx->initialized = true;

    // Set default I2C address if not manually configured before Init
    if (p_ctx->dev_addr == 0) {
        p_ctx->dev_addr = (AS5600_DEFAULT_ADDR << 1); // Shift for HAL (8-bit address)
    }

    // Clear buffer
    p_ctx->dma_buffer[0] = 0;
    p_ctx->dma_buffer[1] = 0;

    // 2. Register Callback with BSP Layer
    // When DMA finishes, BSP will call 'on_complete' directly.
    if (on_complete != NULL) {
        BSP_I2C_SetRxCpltCallback(i2c_dev, on_complete);
    }

    // 3. Optional: Verify device presence (Blocking check)
    // This ensures we don't start FOC loop with a disconnected sensor.
    AraStatus_t status = BSP_I2C_IsDeviceReady(p_ctx->i2c_dev, p_ctx->dev_addr);

    if (status != ARA_OK) {
        p_ctx->initialized = false;
        return ARA_ERR_DISCONNECTED;
    }

    return ARA_OK;
}

AraStatus_t DrvAS5600_TriggerUpdate(DrvAS5600_Context_t *p_ctx)
{
    if (p_ctx == NULL || !p_ctx->initialized) {
        return ARA_ERR_PARAM;
    }

    /* * Start DMA Read
     * Target: Internal Register 0x0C (Raw Angle High)
     * Length: 2 Bytes (0x0C, 0x0D)
     * Dest:   p_ctx->dma_buffer
     *
     * Note: AS5600 supports auto-increment for register pointers.
     */
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

    /*
     * Parse Buffer (Big Endian logic for AS5600 I2C)
     * Buffer[0] = Register 0x0C (Bits 11:8)
     * Buffer[1] = Register 0x0D (Bits 7:0)
     */
    uint16_t raw_angle;

    raw_angle  = ((uint16_t)p_ctx->dma_buffer[0] << 8);
    raw_angle |= (uint16_t)p_ctx->dma_buffer[1];

    // Mask valid 12 bits
    return (raw_angle & AS5600_MASK_12BIT);
}