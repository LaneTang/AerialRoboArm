/**
 * @file drv_elrs.h
 * @brief ELRS CRSF Protocol Driver (L2)
 * @note  Hardware-agnostic parsing of CRSF frames. Relies on BSP_UART_Read.
 *        DO NOT use floating point. Processes data from UART RingBuffer.
 */

#ifndef DRV_ELRS_H
#define DRV_ELRS_H

#include "ara_def.h"
#include "bsp_uart.h"

/* =========================================================
 * 1. Protocol Constants
 * ========================================================= */
#define DRV_ELRS_MAX_CHANNELS    16U
#define DRV_ELRS_MAX_FRAME_LEN   64U
#define DRV_ELRS_LINK_TIMEOUT_MS 100U  /**< Loss-of-link watchdog threshold. */

/* =========================================================
 * 2. Driver Context Structure
 * ========================================================= */

/**
 * @brief ELRS Driver Context.
 * @note  All parser and link states are encapsulated here.
 */
typedef struct {
    /* --- Hardware Binding --- */
    BspUart_Dev_t uart_dev;

    /* --- CRSF Parser State --- */
    uint8_t  rx_buf[DRV_ELRS_MAX_FRAME_LEN];
    uint8_t  rx_len;
    uint8_t  rx_expected_len;

    /* --- Latest Valid Channel Snapshot --- */
    uint16_t channels[DRV_ELRS_MAX_CHANNELS];

    /* --- Link Monitoring --- */
    uint32_t last_frame_tick;
    bool     is_link_up;
} DrvElrs_Context_t;

/* =========================================================
 * 3. Driver API
 * ========================================================= */

/**
 * @brief  Initialize the ELRS driver context.
 * @param  p_ctx Pointer to driver context.
 * @param  uart_dev Bound UART device.
 * @return ARA_OK on success, otherwise ARA_ERR_PARAM.
 */
AraStatus_t DrvElrs_Init(DrvElrs_Context_t *p_ctx, BspUart_Dev_t uart_dev);

/**
 * @brief  Update CRSF parser and link watchdog.
 * @param  p_ctx Pointer to driver context.
 * @param  current_tick_ms Current system tick in milliseconds.
 * @return ARA_OK when link is healthy,
 *         ARA_ERR_CRC when a corrupted frame was dropped but link may still be up,
 *         ARA_ERR_DISCONNECTED when link timeout is exceeded,
 *         or ARA_ERR_PARAM on invalid input.
 */
AraStatus_t DrvElrs_Update(DrvElrs_Context_t *p_ctx, uint32_t current_tick_ms);

/**
 * @brief  Query current radio-link status.
 * @param  p_ctx Pointer to driver context.
 * @return true if link is up, otherwise false.
 */
bool DrvElrs_IsLinkUp(const DrvElrs_Context_t *p_ctx);

/**
 * @brief  Get one raw channel value.
 * @param  p_ctx Pointer to driver context.
 * @param  ch_idx Channel index in [0, DRV_ELRS_MAX_CHANNELS-1].
 * @return Raw CRSF value, or 0 on invalid access / link down.
 */
uint16_t DrvElrs_GetChannel(const DrvElrs_Context_t *p_ctx, uint8_t ch_idx);

/**
 * @brief  Copy the latest valid channel snapshot to caller buffer.
 * @param  p_ctx Pointer to driver context.
 * @param  p_out_channels Output buffer for all channels.
 * @param  out_count Number of elements available in @p p_out_channels.
 * @return ARA_OK on success,
 *         ARA_ERR_PARAM on invalid input,
 *         ARA_ERR_DISCONNECTED if link is currently down.
 */
AraStatus_t DrvElrs_CopyChannels(const DrvElrs_Context_t *p_ctx,
                                 uint16_t *p_out_channels,
                                 uint8_t out_count);

#endif /* DRV_ELRS_H */