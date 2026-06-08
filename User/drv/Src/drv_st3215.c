/**
 * @file drv_st3215.c
 * @brief L2 Hardware Driver: ST3215-HS Serial Bus Servo (protocol layer).
 *
 * Pure C, no RTOS, no HAL, no BSP. All wire-format byte sequences match
 * the Feetech SMS_STS application layer with End=0 (little-endian on wire).
 */

#include "drv_st3215.h"

#include <string.h>

/* =============================================================================
 * Internal helpers
 * ============================================================================= */

/** @brief Write a 16-bit value to buf as low-byte-first (SMS_STS End=0). */
static inline void st3215_pack_word_le(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/** @brief Read a 16-bit value from buf as low-byte-first. */
static inline uint16_t st3215_unpack_word_le(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/**
 * @brief Encode a signed position into the 16-bit sign-bit-15 wire format
 *        used by SMS_STS WritePosEx.
 *        Negative values encode as |value| with bit 15 set.
 */
static uint16_t st3215_encode_signed_pos(int16_t pos)
{
    if (pos < 0) {
        uint16_t mag = (uint16_t)(-(int32_t)pos);
        return (uint16_t)(mag | 0x8000U);
    }
    return (uint16_t)pos;
}

/** @brief Decode a 16-bit value with sign bit at position bit_pos into int16_t. */
static int16_t st3215_decode_signed(uint16_t raw, uint8_t bit_pos)
{
    const uint16_t sign_mask = (uint16_t)(1U << bit_pos);
    if ((raw & sign_mask) != 0U) {
        return (int16_t)(-(int32_t)(raw & (uint16_t)~sign_mask));
    }
    return (int16_t)raw;
}

/**
 * @brief Build a frame: FF FF ID LEN INST [params...] CHECKSUM.
 *
 * LEN field = INST byte + params bytes + CHECKSUM byte - 1 (per SCS spec):
 *   - PING     (no params)          : LEN = 2
 *   - READ/WRITE (params present)   : LEN = param_len + 2
 *
 * MemAddr is treated as the first byte of the caller-supplied params.
 */
static uint8_t st3215_build_frame(uint8_t       *tx_buf,
                                  uint8_t        servo_id,
                                  uint8_t        inst,
                                  const uint8_t *params,
                                  uint8_t        param_len)
{
    if (tx_buf == NULL) {
        return 0U;
    }

    const uint8_t length_field = (param_len == 0U)
                                     ? 2U
                                     : (uint8_t)(param_len + 2U);

    uint8_t idx = 0U;
    tx_buf[idx++] = ST3215_FRAME_HEADER_BYTE;
    tx_buf[idx++] = ST3215_FRAME_HEADER_BYTE;
    tx_buf[idx++] = servo_id;
    tx_buf[idx++] = length_field;
    tx_buf[idx++] = inst;

    /* Checksum covers: ID + LEN + INST + all params */
    uint16_t sum = (uint16_t)servo_id + length_field + inst;
    for (uint8_t i = 0U; i < param_len; i++) {
        tx_buf[idx++] = params[i];
        sum += params[i];
    }
    tx_buf[idx++] = (uint8_t)(~sum);
    return idx;
}

/* =============================================================================
 * Context lifecycle
 * ============================================================================= */

AraStatus_t DrvSt3215_Init(DrvSt3215_Context_t *ctx, uint8_t servo_id)
{
    if (ctx == NULL) {
        return ARA_ERR_PARAM;
    }
    if ((servo_id == 0U) || (servo_id == ST3215_BROADCAST_ID) || (servo_id > 253U)) {
        return ARA_ERR_PARAM;
    }
    memset(ctx, 0, sizeof(*ctx));
    ctx->servo_id = servo_id;
    return ARA_OK;
}

void DrvSt3215_NoteIoOk(DrvSt3215_Context_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return;
    }
    /* Sentinel 0 means "never connected". Force to 1 if caller passes 0. */
    ctx->last_ok_ms      = (now_ms == 0U) ? 1U : now_ms;
    ctx->fail_count      = 0U;
    ctx->last_error_bits = 0U;
}

void DrvSt3215_NoteIoFail(DrvSt3215_Context_t *ctx, uint8_t error_bits)
{
    if (ctx == NULL) {
        return;
    }
    ctx->fail_count      += 1U;
    ctx->last_error_bits  = error_bits;
}

bool DrvSt3215_IsOnline(const DrvSt3215_Context_t *ctx, uint32_t now_ms)
{
    if (ctx == NULL) {
        return false;
    }
    if (ctx->last_ok_ms == 0U) {
        return false; /* never connected */
    }
    return ((uint32_t)(now_ms - ctx->last_ok_ms) < ST3215_OFFLINE_MS);
}

/* =============================================================================
 * Encoding API
 * ============================================================================= */

uint8_t DrvSt3215_EncodePing(uint8_t *tx_buf, uint8_t servo_id)
{
    return st3215_build_frame(tx_buf, servo_id, ST3215_INST_PING, NULL, 0U);
}

uint8_t DrvSt3215_EncodeRead(uint8_t *tx_buf,
                             uint8_t servo_id,
                             uint8_t reg_addr,
                             uint8_t read_len)
{
    if (read_len == 0U) {
        return 0U;
    }
    uint8_t params[2];
    params[0] = reg_addr;
    params[1] = read_len;
    return st3215_build_frame(tx_buf, servo_id, ST3215_INST_READ, params, 2U);
}

uint8_t DrvSt3215_EncodeReadFeedback(uint8_t *tx_buf, uint8_t servo_id)
{
    return DrvSt3215_EncodeRead(tx_buf,
                                servo_id,
                                ST3215_REG_PRESENT_POSITION_L,
                                ST3215_FEEDBACK_PAYLOAD_LEN);
}

uint8_t DrvSt3215_EncodeWriteByte(uint8_t *tx_buf,
                                  uint8_t servo_id,
                                  uint8_t reg_addr,
                                  uint8_t value)
{
    uint8_t params[2];
    params[0] = reg_addr;
    params[1] = value;
    return st3215_build_frame(tx_buf, servo_id, ST3215_INST_WRITE, params, 2U);
}

uint8_t DrvSt3215_EncodeWriteWord(uint8_t *tx_buf,
                                  uint8_t servo_id,
                                  uint8_t reg_addr,
                                  uint16_t value)
{
    uint8_t params[3];
    params[0] = reg_addr;
    st3215_pack_word_le(&params[1], value);
    return st3215_build_frame(tx_buf, servo_id, ST3215_INST_WRITE, params, 3U);
}

uint8_t DrvSt3215_EncodeWritePos(uint8_t *tx_buf,
                                 uint8_t  servo_id,
                                 int16_t  target_pos,
                                 uint16_t speed,
                                 uint8_t  acc)
{
    /* Mirror SMS_STS::WritePosEx: write 7 bytes from register ACC (41).
     *  [0]   MemAddr = ACC = 41
     *  [1]   ACC value
     *  [2-3] GOAL_POSITION (signed via bit15)
     *  [4-5] GOAL_TIME = 0
     *  [6-7] GOAL_SPEED  (unsigned in WritePosEx)
     */
    uint8_t params[8];
    params[0] = ST3215_REG_ACC;
    params[1] = acc;
    st3215_pack_word_le(&params[2], st3215_encode_signed_pos(target_pos));
    st3215_pack_word_le(&params[4], 0U);
    st3215_pack_word_le(&params[6], speed);
    return st3215_build_frame(tx_buf, servo_id, ST3215_INST_WRITE, params, 8U);
}

/* =============================================================================
 * Parsing API
 * ============================================================================= */

/**
 * @brief Common preamble check: header + ID + LEN field validation.
 * @return PARSE_OK if header/id/len consistent; otherwise specific error.
 */
static St3215_ParseResult_t st3215_check_preamble(const uint8_t *rx_buf,
                                                  uint8_t        rx_len,
                                                  uint8_t        expect_id,
                                                  uint8_t        expect_payload_len)
{
    if (rx_buf == NULL) {
        return ST3215_PARSE_BAD_HEADER;
    }
    /* Minimum: FF FF ID LEN ERROR [payload] CHECKSUM = 5 + payload + 1 */
    if (rx_len < (uint8_t)(5U + expect_payload_len + 1U)) {
        return ST3215_PARSE_BAD_LENGTH;
    }
    if ((rx_buf[0] != ST3215_FRAME_HEADER_BYTE) ||
        (rx_buf[1] != ST3215_FRAME_HEADER_BYTE)) {
        return ST3215_PARSE_BAD_HEADER;
    }
    if (rx_buf[2] != expect_id) {
        return ST3215_PARSE_BAD_ID;
    }
    /* LEN field = ERROR + payload + CHECKSUM = payload_len + 2 */
    if (rx_buf[3] != (uint8_t)(expect_payload_len + 2U)) {
        return ST3215_PARSE_BAD_LENGTH;
    }
    return ST3215_PARSE_OK;
}

/**
 * @brief Verify checksum over [ID..last_payload_byte] vs trailing byte.
 * @param covered_len  ID + LEN + ERROR + payload bytes count.
 */
static bool st3215_verify_checksum(const uint8_t *rx_buf, uint8_t covered_len)
{
    uint16_t sum = 0U;
    for (uint8_t i = 0U; i < covered_len; i++) {
        sum += rx_buf[2U + i];
    }
    const uint8_t expected = (uint8_t)(~sum);
    return (expected == rx_buf[2U + covered_len]);
}

St3215_ParseResult_t DrvSt3215_ParseAck(const uint8_t *rx_buf,
                                        uint8_t        rx_len,
                                        uint8_t        expect_id,
                                        uint8_t       *out_error_bits)
{
    St3215_ParseResult_t pre = st3215_check_preamble(rx_buf, rx_len, expect_id, 0U);
    if (pre != ST3215_PARSE_OK) {
        return pre;
    }
    /* Covered bytes: ID + LEN + ERROR = 3 */
    if (!st3215_verify_checksum(rx_buf, 3U)) {
        return ST3215_PARSE_BAD_CHECKSUM;
    }
    const uint8_t error_bits = rx_buf[4];
    if (out_error_bits != NULL) {
        *out_error_bits = error_bits;
    }
    return (error_bits == 0U) ? ST3215_PARSE_OK : ST3215_PARSE_SERVO_ERROR;
}

St3215_ParseResult_t DrvSt3215_ParseFeedback(const uint8_t     *rx_buf,
                                             uint8_t            rx_len,
                                             uint8_t            expect_id,
                                             uint32_t           now_ms,
                                             St3215_Feedback_t *out,
                                             uint8_t           *out_error_bits)
{
    if (out == NULL) {
        return ST3215_PARSE_BAD_HEADER; /* contract violation */
    }
    St3215_ParseResult_t pre =
        st3215_check_preamble(rx_buf, rx_len, expect_id, ST3215_FEEDBACK_PAYLOAD_LEN);
    if (pre != ST3215_PARSE_OK) {
        return pre;
    }
    /* Covered: ID + LEN + ERROR + 15 payload = 18 bytes */
    if (!st3215_verify_checksum(rx_buf, 18U)) {
        return ST3215_PARSE_BAD_CHECKSUM;
    }
    const uint8_t error_bits = rx_buf[4];
    if (out_error_bits != NULL) {
        *out_error_bits = error_bits;
    }
    if (error_bits != 0U) {
        return ST3215_PARSE_SERVO_ERROR;
    }

    /* Payload starts at rx_buf[5], 15 bytes covering registers 56..70. */
    const uint8_t *p = &rx_buf[5];

    const uint16_t pos_raw  = st3215_unpack_word_le(&p[0]);
    const uint16_t spd_raw  = st3215_unpack_word_le(&p[2]);
    const uint16_t load_raw = st3215_unpack_word_le(&p[4]);
    const uint16_t cur_raw  = st3215_unpack_word_le(&p[13]);

    out->position     = st3215_decode_signed(pos_raw, 15U);
    out->speed        = st3215_decode_signed(spd_raw, 15U);
    out->load         = st3215_decode_signed(load_raw, 10U);
    out->voltage_dv   = p[6];
    out->temp_c       = p[7];
    /* p[8], p[9] = registers 64, 65 (reserved) */
    out->moving       = (p[10] != 0U);
    /* p[11], p[12] = registers 67, 68 (reserved) */
    out->current      = st3215_decode_signed(cur_raw, 15U);
    out->timestamp_ms = now_ms;

    return ST3215_PARSE_OK;
}

/* =============================================================================
 * Utility
 * ============================================================================= */

uint8_t DrvSt3215_Checksum(const uint8_t *data, uint8_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0xFFU;
    }
    uint16_t sum = 0U;
    for (uint8_t i = 0U; i < len; i++) {
        sum += data[i];
    }
    return (uint8_t)(~sum);
}

St3215_MotionStatus_t DrvSt3215_ClassifyMotion(
    const St3215_Feedback_t          *fb,
    int16_t                           target_pos,
    const St3215_MotionThresholds_t  *thresholds)
{
    if (fb == NULL) {
        return ST3215_MOTION_UNKNOWN;
    }

    const int16_t pos_tol  = (thresholds != NULL) ? thresholds->pos_tol_steps
                                                  : ST3215_DEFAULT_POS_TOL_STEPS;
    const int16_t spd_tol  = (thresholds != NULL) ? thresholds->spd_tol_steps
                                                  : ST3215_DEFAULT_SPD_TOL_STEPS;
    const int16_t load_ovl = (thresholds != NULL) ? thresholds->load_overload_abs
                                                  : ST3215_DEFAULT_LOAD_OVERLOAD;

    int32_t err = (int32_t)target_pos - (int32_t)fb->position;
    if (err < 0) {
        err = -err;
    }
    const int16_t spd_abs  = (fb->speed < 0) ? (int16_t)-fb->speed : fb->speed;
    const int16_t load_abs = (fb->load  < 0) ? (int16_t)-fb->load  : fb->load;

    /* ARRIVED takes priority over OVERLOAD when the servo is genuinely
     * stationary at target. A robotic arm holding a payload against
     * gravity can sit at the target with load >= load_ovl indefinitely;
     * classifying that as OVERLOAD would let an upper-layer FSM strip
     * torque and drop the payload. The strict ARRIVED predicate
     * (!moving && err < pos_tol && spd_abs < spd_tol) ensures we only
     * pre-empt OVERLOAD when motion has truly settled at target. */
    if (!fb->moving && (err < pos_tol) && (spd_abs < spd_tol)) {
        return ST3215_MOTION_ARRIVED;
    }

    /* Overload protection: high load AND not at-rest at target. */
    if (load_abs >= load_ovl) {
        if (!fb->moving && (err >= pos_tol)) {
            return ST3215_MOTION_STALLED;
        }
        return ST3215_MOTION_OVERLOAD;
    }

    if (fb->moving) {
        return ST3215_MOTION_MOVING;
    }
    /* Stopped but not yet at target: stalled. */
    return ST3215_MOTION_STALLED;
}

St3215_IoResult_t DrvSt3215_ClassifyIoResult(
    const DrvSt3215_Context_t *ctx,
    St3215_ParseResult_t       parse_result,
    bool                       upper_timeout,
    uint32_t                   now_ms)
{
    /* Offline judgement requires at least one prior success. */
    if ((ctx != NULL) && (ctx->last_ok_ms != 0U) &&
        ((uint32_t)(now_ms - ctx->last_ok_ms) > ST3215_OFFLINE_MS)) {
        return ST3215_IO_OFFLINE;
    }

    if (upper_timeout) {
        return ST3215_IO_TIMEOUT;
    }

    switch (parse_result) {
    case ST3215_PARSE_OK:           return ST3215_IO_OK;
    case ST3215_PARSE_SERVO_ERROR:  return ST3215_IO_SERVO_ERROR;
    case ST3215_PARSE_BAD_HEADER:   /* fallthrough */
    case ST3215_PARSE_BAD_LENGTH:   /* fallthrough */
    case ST3215_PARSE_BAD_ID:       /* fallthrough */
    case ST3215_PARSE_BAD_CHECKSUM: /* fallthrough */
    default:                         return ST3215_IO_BAD_FRAME;
    }
}
