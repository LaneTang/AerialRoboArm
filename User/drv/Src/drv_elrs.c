/**
 * @file drv_elrs.c
 * @brief ELRS CRSF Protocol Driver (L2) Implementation
 */

#include "drv_elrs.h"
#include <string.h>

/* =========================================================
 * Internal Protocol Constants
 * ========================================================= */
#define CRSF_SYNC_BYTE       0xC8U
#define CRSF_TYPE_RC_CH      0x16U
#define CRSF_PAYLOAD_LEN_RC  22U

/* =========================================================
 * Internal Helper Functions
 * ========================================================= */

/**
 * @brief  Compute CRC8 using CRSF polynomial 0xD5.
 * @param  data Pointer to input bytes.
 * @param  len Number of bytes.
 * @return Computed CRC8 value.
 */
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0U;
    uint8_t i;
    uint8_t b;

    for (i = 0U; i < len; i++) {
        crc ^= data[i];
        for (b = 0U; b < 8U; b++) {
            if ((crc & 0x80U) != 0U) {
                crc = (uint8_t)((crc << 1) ^ 0xD5U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief  Unpack 11-bit CRSF channels from 22-byte payload.
 * @param  p_ctx Pointer to driver context.
 * @param  payload Pointer to RC payload.
 */
static void unpack_channels(DrvElrs_Context_t *p_ctx, const uint8_t *payload)
{
    uint32_t bits = 0U;
    uint8_t  bit_count = 0U;
    uint8_t  ch = 0U;
    uint8_t  i;

    for (i = 0U; i < CRSF_PAYLOAD_LEN_RC; i++) {
        bits |= ((uint32_t)payload[i]) << bit_count;
        bit_count = (uint8_t)(bit_count + 8U);

        while ((bit_count >= 11U) && (ch < DRV_ELRS_MAX_CHANNELS)) {
            p_ctx->channels[ch] = (uint16_t)(bits & 0x7FFU);
            ch++;
            bits >>= 11U;
            bit_count = (uint8_t)(bit_count - 11U);
        }
    }
}

/* =========================================================
 * Driver API Implementation
 * ========================================================= */

AraStatus_t DrvElrs_Init(DrvElrs_Context_t *p_ctx, BspUart_Dev_t uart_dev)
{
    if (p_ctx == NULL) {
        return ARA_ERR_PARAM;
    }

    memset(p_ctx, 0, sizeof(DrvElrs_Context_t));
    p_ctx->uart_dev = uart_dev;

    return ARA_OK;
}

AraStatus_t DrvElrs_Update(DrvElrs_Context_t *p_ctx, uint32_t current_tick_ms)
{
    AraStatus_t ret_status = ARA_OK;
    uint8_t rx_byte;

    if (p_ctx == NULL) {
        return ARA_ERR_PARAM;
    }

    while (BSP_UART_Read(p_ctx->uart_dev, &rx_byte, 1U) == 1) {

        if (p_ctx->rx_len == 0U) {
            if (rx_byte == CRSF_SYNC_BYTE) {
                p_ctx->rx_buf[p_ctx->rx_len++] = rx_byte;
            }
        } else {
            p_ctx->rx_buf[p_ctx->rx_len++] = rx_byte;

            if (p_ctx->rx_len == 2U) {
                p_ctx->rx_expected_len = (uint8_t)(p_ctx->rx_buf[1] + 2U);

                if ((p_ctx->rx_expected_len > DRV_ELRS_MAX_FRAME_LEN) ||
                    (p_ctx->rx_expected_len < 4U))
                {
                    p_ctx->rx_len = 0U;
                    p_ctx->rx_expected_len = 0U;
                }
            } else if ((p_ctx->rx_expected_len > 0U) &&
                       (p_ctx->rx_len >= p_ctx->rx_expected_len))
            {
                uint8_t len = p_ctx->rx_expected_len;
                uint8_t crc_calc = crsf_crc8(&p_ctx->rx_buf[2], (uint8_t)(len - 3U));
                uint8_t crc_recv = p_ctx->rx_buf[len - 1U];

                if (crc_calc == crc_recv) {
                    if ((p_ctx->rx_buf[2] == CRSF_TYPE_RC_CH) &&
                        (p_ctx->rx_buf[1] == (CRSF_PAYLOAD_LEN_RC + 2U)))
                    {
                        unpack_channels(p_ctx, &p_ctx->rx_buf[3]);
                        p_ctx->last_frame_tick = current_tick_ms;
                        p_ctx->is_link_up = true;
                    }
                } else {
                    ret_status = ARA_ERR_CRC;
                }

                p_ctx->rx_len = 0U;
                p_ctx->rx_expected_len = 0U;
            }
        }
    }

    if (p_ctx->is_link_up) {
        if ((uint32_t)(current_tick_ms - p_ctx->last_frame_tick) > DRV_ELRS_LINK_TIMEOUT_MS) {
            p_ctx->is_link_up = false;
        }
    }

    if (!p_ctx->is_link_up) {
        return ARA_ERR_DISCONNECTED;
    }

    return ret_status;
}

bool DrvElrs_IsLinkUp(const DrvElrs_Context_t *p_ctx)
{
    if (p_ctx == NULL) {
        return false;
    }

    return p_ctx->is_link_up;
}

uint16_t DrvElrs_GetChannel(const DrvElrs_Context_t *p_ctx, uint8_t ch_idx)
{
    if ((p_ctx == NULL) || (!p_ctx->is_link_up) || (ch_idx >= DRV_ELRS_MAX_CHANNELS)) {
        return 0U;
    }

    return p_ctx->channels[ch_idx];
}

AraStatus_t DrvElrs_CopyChannels(const DrvElrs_Context_t *p_ctx,
                                 uint16_t *p_out_channels,
                                 uint8_t out_count)
{
    if ((p_ctx == NULL) || (p_out_channels == NULL) || (out_count < DRV_ELRS_MAX_CHANNELS)) {
        return ARA_ERR_PARAM;
    }

    if (!p_ctx->is_link_up) {
        return ARA_ERR_DISCONNECTED;
    }

    memcpy(p_out_channels,
           p_ctx->channels,
           sizeof(uint16_t) * DRV_ELRS_MAX_CHANNELS);

    return ARA_OK;
}