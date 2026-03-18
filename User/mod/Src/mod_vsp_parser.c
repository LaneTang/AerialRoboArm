/**
 * @file mod_vsp_parser.c
 * @brief ARA-VSP (Vision Simplex Protocol) Parser Module Implementation (L3)
 * @note  Strictly C99/C11. No dynamic allocation, no floating-point.
 * Implements a non-blocking state machine for byte-by-byte parsing.
 */

#include "mod_vsp_parser.h"

/* =========================================================
 * Internal Helper Functions (Static)
 * ========================================================= */

/**
 * @brief Calculate the ARA-VSP Checksum
 * @param p_buf Pointer to the start of the frame buffer
 * @return Calculated 8-bit checksum (Sum of bytes [2] to [7])
 */
static uint8_t Calculate_Checksum(const uint8_t *p_buf)
{
    uint8_t sum = 0;
    /* Sum bytes from Seq_ID [2] up to Target_Dist High Byte [7] */
    for (uint8_t i = 2; i <= 7; i++) {
        sum += p_buf[i];
    }
    return sum;
}

/**
 * @brief Reset the state machine back to hunting for the first header
 */
static void Reset_Parser(ModVspParser_Context_t *p_ctx)
{
    p_ctx->state = VSP_STATE_WAIT_H1;
    p_ctx->byte_index = 0;
}

/* =========================================================
 * Module API Implementation
 * ========================================================= */

void ModVspParser_Init(ModVspParser_Context_t *p_ctx)
{
    if (p_ctx == NULL) {
        return;
    }

    /* Initialize state machine and clear buffers/stats */
    p_ctx->state = VSP_STATE_WAIT_H1;
    p_ctx->byte_index = 0;
    p_ctx->last_seq_id = 0;
    p_ctx->valid_frame_cnt = 0;
    p_ctx->crc_error_cnt = 0;

    for (uint8_t i = 0; i < VSP_FRAME_LENGTH; i++) {
        p_ctx->buffer[i] = 0;
    }
}

AraStatus_t ModVspParser_FeedByte(ModVspParser_Context_t *p_ctx,
                                  uint8_t in_byte,
                                  AraVisionData_t *p_out_data)
{
    /* Safety check */
    if (p_ctx == NULL || p_out_data == NULL) {
        return ARA_ERR_PARAM;
    }

    /* State Machine Execution */
    switch (p_ctx->state) {
        case VSP_STATE_WAIT_H1:
            if (in_byte == VSP_FRAME_HEADER_1) {
                p_ctx->buffer[0] = in_byte;
                p_ctx->byte_index = 1;
                p_ctx->state = VSP_STATE_WAIT_H2;
            }
            break;

        case VSP_STATE_WAIT_H2:
            if (in_byte == VSP_FRAME_HEADER_2) {
                p_ctx->buffer[1] = in_byte;
                p_ctx->byte_index = 2;
                p_ctx->state = VSP_STATE_PAYLOAD;
            } else if (in_byte == VSP_FRAME_HEADER_1) {
                /* Edge case: Received 0xAA 0xAA. The second 0xAA might be the start of a valid frame */
                p_ctx->buffer[0] = in_byte;
                p_ctx->byte_index = 1;
            } else {
                /* Invalid Header 2, reset state machine */
                Reset_Parser(p_ctx);
            }
            break;

        case VSP_STATE_PAYLOAD:
            p_ctx->buffer[p_ctx->byte_index++] = in_byte;

            /* Check if the full frame has been received */
            if (p_ctx->byte_index >= VSP_FRAME_LENGTH) {
                /* Frame complete, verify Checksum */
                uint8_t calc_crc = Calculate_Checksum(p_ctx->buffer);
                uint8_t rx_crc   = p_ctx->buffer[8];

                if (calc_crc == rx_crc) {
                    /* --- Extraction & Decoding Phase (Little-Endian) --- */

                    /* 1. Sequence ID */
                    p_out_data->seq_id = p_ctx->buffer[2];

                    /* 2. Perception Flags (Bitmasking) */
                    uint8_t flags = p_ctx->buffer[3];
                    p_out_data->is_tracking   = (flags & VSP_FLAG_IS_TRACKING)  ? true : false;
                    p_out_data->is_grabbable  = (flags & VSP_FLAG_IS_GRABBABLE) ? true : false;
                    p_out_data->pc_estop_req  = (flags & VSP_FLAG_ESTOP)        ? true : false;

                    /* 3. Target Roll (int16_t, signed reconstruction) */
                    p_out_data->target_roll = (int16_t)((p_ctx->buffer[5] << 8) | p_ctx->buffer[4]);

                    /* 4. Target Distance (uint16_t, unsigned reconstruction) */
                    p_out_data->target_dist_mm = (uint16_t)((p_ctx->buffer[7] << 8) | p_ctx->buffer[6]);

                    /* Update Statistics & Reset for next frame */
                    p_ctx->valid_frame_cnt++;
                    p_ctx->last_seq_id = p_out_data->seq_id;
                    Reset_Parser(p_ctx);

                    return ARA_OK; /* Frame successfully parsed and output populated */

                } else {
                    /* CRC Mismatch */
                    p_ctx->crc_error_cnt++;
                    Reset_Parser(p_ctx);
                    return ARA_ERR_CRC;
                }
            }
            break;

        default:
            /* Should never reach here, but fail gracefully if memory corrupted */
            Reset_Parser(p_ctx);
            break;
    }

    /* Return BUSY to indicate the parser is still accumulating bytes */
    return ARA_BUSY;
}