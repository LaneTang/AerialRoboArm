/**
 * @file drv_elrs.c
 * @brief ELRS CRSF Protocol Driver (L2) Implementation
 */

#include "drv_elrs.h"
#include <string.h>

/* =========================================================
 * Internal Protocol Constants
 * ========================================================= */
#define CRSF_SYNC_BYTE       0xC8
#define CRSF_TYPE_RC_CH      0x16
#define CRSF_PAYLOAD_LEN_RC  22

/* =========================================================
 * Internal Helper Functions
 * ========================================================= */

/**
 * @brief Compute CRC8 using CRSF Polynomial 0xD5
 * @note  Loop-based to save RAM (avoids 256-byte LUT). Fast enough for 64-byte max length.
 */
static uint8_t crsf_crc8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0xD5;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

/**
 * @brief Unpack 11-bit CRSF channels from 22-byte payload
 */
static void unpack_channels(DrvElrs_Context_t *p_ctx, const uint8_t *payload)
{
    uint32_t bits = 0;
    uint8_t  bit_count = 0;
    uint8_t  ch = 0;

    for (uint8_t i = 0; i < CRSF_PAYLOAD_LEN_RC; i++) {
        bits |= ((uint32_t)payload[i]) << bit_count;
        bit_count += 8;

        while (bit_count >= 11 && ch < DRV_ELRS_MAX_CHANNELS) {
            p_ctx->channels[ch++] = bits & 0x7FF;
            bits >>= 11;
            bit_count -= 11;
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

    /* Initialize context variables to zero/false */
    memset(p_ctx, 0, sizeof(DrvElrs_Context_t));

    p_ctx->uart_dev = uart_dev;

    return ARA_OK;
}

AraStatus_t DrvElrs_Update(DrvElrs_Context_t *p_ctx, uint32_t current_tick_ms)
{
    if (p_ctx == NULL) {
        return ARA_ERR_PARAM;
    }

    AraStatus_t ret_status = ARA_OK;
    uint8_t rx_byte;

    /* * Drain the BSP UART RingBuffer.
     * Process all available bytes sequentially through the state machine.
     */
    while (BSP_UART_Read(p_ctx->uart_dev, &rx_byte, 1) == 1) {

        /* State 1: Wait for Sync Byte */
        if (p_ctx->rx_len == 0) {
            if (rx_byte == CRSF_SYNC_BYTE) {
                p_ctx->rx_buf[p_ctx->rx_len++] = rx_byte;
            }
        }
            /* State 2: Reading Frame */
        else {
            p_ctx->rx_buf[p_ctx->rx_len++] = rx_byte;

            /* Extract length at 2nd byte */
            if (p_ctx->rx_len == 2) {
                p_ctx->rx_expected_len = p_ctx->rx_buf[1] + 2; // Payload len + Sync + Len byte

                /* Sanity check on length */
                if (p_ctx->rx_expected_len > DRV_ELRS_MAX_FRAME_LEN ||
                    p_ctx->rx_expected_len < 4) {
                    p_ctx->rx_len = 0; // Abort and reset
                }
            }
                /* Frame completely received */
            else if (p_ctx->rx_expected_len > 0 && p_ctx->rx_len >= p_ctx->rx_expected_len) {

                uint8_t len = p_ctx->rx_expected_len;
                uint8_t crc_calc = crsf_crc8(&p_ctx->rx_buf[2], len - 3); // CRC starts from Type
                uint8_t crc_recv = p_ctx->rx_buf[len - 1];

                if (crc_calc == crc_recv) {
                    /* Process only RC Channel packets */
                    if (p_ctx->rx_buf[2] == CRSF_TYPE_RC_CH) {
                        unpack_channels(p_ctx, &p_ctx->rx_buf[3]);
                        p_ctx->last_frame_tick = current_tick_ms;
                        p_ctx->is_link_up = true;
                    }
                } else {
                    /* CRC Error detected */
                    ret_status = ARA_ERR_CRC;
                }

                /* Reset state machine for the next frame */
                p_ctx->rx_len = 0;
                p_ctx->rx_expected_len = 0;
            }
        }
    }

    /* Link Timeout Watchdog */
    if (p_ctx->is_link_up) {
        if ((current_tick_ms - p_ctx->last_frame_tick) > DRV_ELRS_LINK_TIMEOUT_MS) {
            p_ctx->is_link_up = false;
        }
    }

    /* Highest priority error check: Disconnection */
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
    if (p_ctx == NULL || !p_ctx->is_link_up || ch_idx >= DRV_ELRS_MAX_CHANNELS) {
        return 0; /* Safe default */
    }
    return p_ctx->channels[ch_idx];
}