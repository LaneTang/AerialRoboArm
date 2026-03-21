/**
 * @file drv_elrs.h
 * @brief ELRS CRSF Protocol Driver (L2)
 * @note  Hardware-agnostic parsing of CRSF frames. Relies on BSP_UART_Read.
 * DO NOT use floating point. Processes data from UART RingBuffer.
 */

#ifndef DRV_ELRS_H
#define DRV_ELRS_H

#include "ara_def.h"
#include "bsp_uart.h"

/* =========================================================
 * 1. Protocol Constants
 * ========================================================= */
#define DRV_ELRS_MAX_CHANNELS    16
#define DRV_ELRS_MAX_FRAME_LEN   64
#define DRV_ELRS_LINK_TIMEOUT_MS 100    // Loss of signal threshold


/* =========================================================
 * 2. Driver Context Structure
 * ========================================================= */

/**
 * @brief ELRS Driver Context. Simulator for Object-Oriented design.
 * @note  All state variables are encapsulated here. No global variables.
 */
typedef struct {
    /* --- Hardware Bindings --- */
    BspUart_Dev_t uart_dev;

    /* --- CRSF Parsing State Machine --- */
    uint8_t  rx_buf[DRV_ELRS_MAX_FRAME_LEN];
    uint8_t  rx_len;
    uint8_t  rx_expected_len;

    /* --- Output Data --- */
    uint16_t channels[DRV_ELRS_MAX_CHANNELS]; // Raw 11-bit values (172 to 1811 typical)

    /* --- Link Monitoring --- */
    uint32_t last_frame_tick;   // System tick of the last valid CRSF frame
    bool     is_link_up;        // true = Active, false = Disconnected
} DrvElrs_Context_t;


/* =========================================================
 * 3. Driver API
 * ========================================================= */

/**
 * @brief Initialize ELRS Context and Hardware link
 * @param p_ctx    Pointer to the driver context
 * @param uart_dev The BSP UART device enum (e.g., BSP_UART_ELRS)
 * @return ARA_OK on success, ARA_ERR_PARAM if pointers are null
 */
AraStatus_t DrvElrs_Init(DrvElrs_Context_t *p_ctx, BspUart_Dev_t uart_dev);

/**
 * @brief Update ELRS State Machine (Non-blocking)
 * @note  MUST be called periodically by L4 Task. Reads available bytes from
 * BSP RingBuffer, parses frames, and checks for link timeout.
 * @param p_ctx            Pointer to the driver context
 * @param current_tick_ms  Current FreeRTOS tick count (ms)
 * @return ARA_OK               - Link is healthy (new frame parsed or maintaining link)
 * ARA_ERR_DISCONNECTED - Link timeout exceeded DRV_ELRS_LINK_TIMEOUT_MS
 * ARA_ERR_CRC          - A corrupted frame was dropped (Non-fatal, just logs it)
 */
AraStatus_t DrvElrs_Update(DrvElrs_Context_t *p_ctx, uint32_t current_tick_ms);

/**
 * @brief Get the status of the radio link
 * @param p_ctx Pointer to the driver context
 * @return true if link is up, false if link is lost
 */
bool DrvElrs_IsLinkUp(const DrvElrs_Context_t *p_ctx);

/**
 * @brief Get the raw value of a specific channel
 * @param p_ctx  Pointer to the driver context
 * @param ch_idx Channel index (0 to 15)
 * @return 11-bit raw value (0-2047). Returns 0 if ch_idx is out of bounds or link is down.
 */
uint16_t DrvElrs_GetChannel(const DrvElrs_Context_t *p_ctx, uint8_t ch_idx);

#endif /* DRV_ELRS_H */