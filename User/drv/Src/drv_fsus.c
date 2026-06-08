/**
 * @file drv_fsus.c
 * @brief L2 Hardware Driver: Fashion Star UART Servo protocol layer.
 *
 * Pure C, no RTOS, no HAL, no BSP. All wire-format byte sequences match
 * the Fashion Star Bus Servo SDK v0.0.2.
 */

#include "drv_fsus.h"
#include <string.h>

/* =============================================================================
 * Internal helpers
 * ============================================================================= */

/** Write a 16-bit value as little-endian.
 *  FSUS 协议的多字节字段都是低字节在前，所以 0x1234 会发成 34 12。
 */
static inline void fsus_pack_u16_le(uint8_t *buf, uint16_t value)
{
    buf[0] = (uint8_t)(value & 0xFFU);
    buf[1] = (uint8_t)((value >> 8) & 0xFFU);
}

/** Write a signed 16-bit value as little-endian. */
static inline void fsus_pack_s16_le(uint8_t *buf, int16_t value)
{
    fsus_pack_u16_le(buf, (uint16_t)value);
}

/** Read a 16-bit little-endian value. */
static inline uint16_t fsus_unpack_u16_le(const uint8_t *buf)
{
    return (uint16_t)((uint16_t)buf[0] | ((uint16_t)buf[1] << 8));
}

/** Read a signed 16-bit little-endian value. */
static inline int16_t fsus_unpack_s16_le(const uint8_t *buf)
{
    return (int16_t)fsus_unpack_u16_le(buf);
}

/** Read a signed 32-bit little-endian value (for multi-turn angle). */
static inline int32_t fsus_unpack_s32_le(const uint8_t *buf)
{
    return (int32_t)((uint32_t)buf[0]
                   | ((uint32_t)buf[1] << 8)
                   | ((uint32_t)buf[2] << 16)
                   | ((uint32_t)buf[3] << 24));
}

/**
 * @brief Build and checksum a full FSUS request frame.
 *
 * Frame: HEADER_LO HEADER_HI CMD SIZE_L [0xFF SIZE_HI SIZE_LO] CONTENT CK
 *        0x12      0x4C      命令 内容长度       命令参数       校验
 *
 * 这个函数只负责“把上层语义命令变成总线上的字节流”，不碰 UART。
 * 真正发出去的动作在 task_motion.c -> BSP_UART_Fsus_Send()。
 *
 * @param tx_buf     Output buffer (caller-sized >= FSUS_TX_BUF_SIZE).
 * @param cmd_id     FSUS command ID.
 * @param content    Command content bytes (may be NULL when size == 0).
 * @param size       Content byte count.
 * @return Total bytes written, or 0 on error.
 */
static uint16_t fsus_build_frame(uint8_t       *tx_buf,
                                 uint8_t        cmd_id,
                                 const uint8_t *content,
                                 uint16_t       size)
{
    if (tx_buf == NULL) {
        return 0U;
    }
    if ((size > 0U) && (content == NULL)) {
        return 0U;
    }
    if (size > (FSUS_TX_BUF_SIZE - 7U)) {  /* header(2)+cid(1)+size(3 max)+ck(1) */
        return 0U;
    }

    uint16_t idx = 0U;

    /* Request header: every FSUS request starts with 0x12 0x4C. */
    tx_buf[idx++] = FSUS_HEADER_REQ_LO;
    tx_buf[idx++] = FSUS_HEADER_REQ_HI;

    /* CMD tells the servo how to interpret CONTENT, e.g. set angle or monitor. */
    tx_buf[idx++] = cmd_id;

    /* Size field: 1 byte normally, or 0xFF escape + 2 bytes for >= 255. */
    if (size >= 255U) {
        tx_buf[idx++] = 0xFFU;
        tx_buf[idx++] = (uint8_t)(size & 0xFFU);
        tx_buf[idx++] = (uint8_t)((size >> 8) & 0xFFU);
    } else {
        tx_buf[idx++] = (uint8_t)size;
    }

    /* CONTENT is command-specific: often starts with servo_id, then parameters. */
    if (size > 0U) {
        (void)memcpy(&tx_buf[idx], content, size);
        idx += size;
    }

    /* Checksum lets the servo reject a frame corrupted on the UART bus. */
    tx_buf[idx] = DrvFsus_Checksum(tx_buf, idx);
    idx++;

    return idx;
}

/* =============================================================================
 * Public: Checksum
 * ============================================================================= */

uint8_t DrvFsus_Checksum(const uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0xFFU;
    }
    uint16_t sum = 0U;
    for (uint16_t i = 0U; i < len; i++) {
        sum += data[i];
    }
    /* FSUS checksum is the low 8 bits of the byte sum (no complement). */
    return (uint8_t)sum;
}

/* =============================================================================
 * Public: Encoding
 * ============================================================================= */

uint16_t DrvFsus_EncodePing(uint8_t *tx_buf, uint8_t servo_id)
{
    if (servo_id > 254U) {
        return 0U;
    }
    uint8_t content[1] = { servo_id };
    return fsus_build_frame(tx_buf, FSUS_CMD_PING, content, sizeof(content));
}

uint16_t DrvFsus_EncodeSetAngleByVelocity(uint8_t *tx_buf,
                                          uint8_t  servo_id,
                                          float    angle_deg,
                                          float    velocity_deg_per_s,
                                          uint16_t t_acc_ms,
                                          uint16_t t_dec_ms,
                                          uint16_t power_mw)
{
    if (servo_id > 254U) {
        return 0U;
    }

    /* Clamp inputs per FSUS spec before encoding. The servo will run its
     * internal closed loop; these fields are the target trajectory constraints,
     * not raw motor PWM duty. */
    if (angle_deg > FSUS_ANGLE_MAX_DEG)         angle_deg = FSUS_ANGLE_MAX_DEG;
    if (angle_deg < FSUS_ANGLE_MIN_DEG)         angle_deg = FSUS_ANGLE_MIN_DEG;
    if (velocity_deg_per_s > FSUS_VELOCITY_MAX_DEG_PER_S) velocity_deg_per_s = FSUS_VELOCITY_MAX_DEG_PER_S;
    if (velocity_deg_per_s < FSUS_VELOCITY_MIN_DEG_PER_S) velocity_deg_per_s = FSUS_VELOCITY_MIN_DEG_PER_S;
    if (t_acc_ms < FSUS_T_ACC_DEC_MIN_MS)       t_acc_ms = FSUS_T_ACC_DEC_MIN_MS;
    if (t_dec_ms < FSUS_T_ACC_DEC_MIN_MS)       t_dec_ms = FSUS_T_ACC_DEC_MIN_MS;

    /* Build content:
     *   ID       : which servo on the bus should execute this command
     *   angle    : target angle in 0.1 deg, signed, little-endian
     *   velocity : max move speed in 0.1 deg/s
     *   t_acc/dec: acceleration/deceleration time in ms
     *   power    : execution power limit in mW
     *
     * After receiving this, the servo's own MCU reads its encoder and drives
     * its motor driver until the output shaft follows the requested trajectory. */
    uint8_t  content[11];
    uint16_t off = 0U;

    content[off++] = servo_id;

    /* Convert human-friendly units into the integer units used on the wire. */
    fsus_pack_s16_le(&content[off], (int16_t)(angle_deg * 10.0f));
    off += 2U;
    fsus_pack_u16_le(&content[off], (uint16_t)(velocity_deg_per_s * 10.0f));
    off += 2U;
    fsus_pack_u16_le(&content[off], t_acc_ms);
    off += 2U;
    fsus_pack_u16_le(&content[off], t_dec_ms);
    off += 2U;
    fsus_pack_u16_le(&content[off], power_mw);
    off += 2U;

    return fsus_build_frame(tx_buf, FSUS_CMD_SET_ANGLE_BY_VELOCITY, content, off);
}

uint16_t DrvFsus_EncodeQueryAngle(uint8_t *tx_buf, uint8_t servo_id)
{
    if (servo_id > 254U) {
        return 0U;
    }
    uint8_t content[1] = { servo_id };
    return fsus_build_frame(tx_buf, FSUS_CMD_QUERY_ANGLE, content, sizeof(content));
}

uint16_t DrvFsus_EncodeServoMonitor(uint8_t *tx_buf, uint8_t servo_id)
{
    if (servo_id > 254U) {
        return 0U;
    }
    /* Query one servo for telemetry. The response is parsed by
     * DrvFsus_ParseServoMonitor() into voltage/current/power/temp/status/angle. */
    uint8_t content[1] = { servo_id };
    return fsus_build_frame(tx_buf, FSUS_CMD_SERVO_MONITOR, content, sizeof(content));
}

uint16_t DrvFsus_EncodeStop(uint8_t *tx_buf,
                            uint8_t  servo_id,
                            uint8_t  mode,
                            uint16_t power_mw)
{
    if (servo_id > 254U) {
        return 0U;
    }
    if (mode > 2U) {
        return 0U;
    }

    /* Content: ID(1) + mode_byte(1) + power(2) = 4 bytes.
     * mode=unlock releases torque; hold/damping keep some controlled output.
     * Mode byte = mode | 0x10 (per FSUS spec). */
    uint8_t content[4] = {
        servo_id,
        (uint8_t)(mode | 0x10U),
        (uint8_t)(power_mw & 0xFFU),
        (uint8_t)((power_mw >> 8) & 0xFFU)
    };

    return fsus_build_frame(tx_buf, FSUS_CMD_STOP_ON_CONTROL_MODE, content, sizeof(content));
}

/* =============================================================================
 * Public: Parsing
 * ============================================================================= */

/**
 * @brief Verify response header: must be 0x05 0x1C.
 *
 * Request frames start with 0x12 0x4C; response frames start with 0x05 0x1C.
 * This split helps task_motion.c resync if there is stale or noisy UART data.
 */
static FsusParseResult_t fsus_check_header(const uint8_t *rx_buf, uint16_t rx_len)
{
    if ((rx_buf == NULL) || (rx_len < 5U)) {
        return FSUS_PARSE_BAD_FRAME;
    }
    if ((rx_buf[0] != FSUS_HEADER_RESP_LO) ||
        (rx_buf[1] != FSUS_HEADER_RESP_HI)) {
        return FSUS_PARSE_BAD_HEADER;
    }
    return FSUS_PARSE_OK;
}

/**
 * @brief Validate checksum over the complete frame.
 * @param rx_buf  Start of frame (header byte 0).
 * @param rx_len  Total bytes in frame (including checksum).
 * @return true when checksum matches.
 */
static bool fsus_verify_response_checksum(const uint8_t *rx_buf, uint16_t rx_len)
{
    if ((rx_buf == NULL) || (rx_len < 6U)) {
        return false;
    }
    /* Checksum is over bytes [0 .. rx_len-2], stored at [rx_len-1]. */
    uint8_t ck = DrvFsus_Checksum(rx_buf, (uint16_t)(rx_len - 1U));
    return (ck == rx_buf[rx_len - 1U]);
}

FsusParseResult_t DrvFsus_ParsePing(const uint8_t *rx_buf,
                                    uint16_t       rx_len,
                                    uint8_t        expect_id)
{
    FsusParseResult_t pre = fsus_check_header(rx_buf, rx_len);
    if (pre != FSUS_PARSE_OK) {
        return pre;
    }
    /* PING response: HDR(2) + CMD(1)=1 + SIZE(1)=1 + content[ID(1)] + CK(1) = 6 bytes */
    if (rx_len != 6U) {
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_buf[2] != FSUS_CMD_PING) {
        return FSUS_PARSE_BAD_FRAME;
    }
    if (rx_buf[3] != 1U) {  /* size == 1 */
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_buf[4] != expect_id) {
        return FSUS_PARSE_BAD_ID;
    }
    if (!fsus_verify_response_checksum(rx_buf, rx_len)) {
        return FSUS_PARSE_BAD_CHECKSUM;
    }
    return FSUS_PARSE_OK;
}

FsusParseResult_t DrvFsus_ParseQueryAngle(const uint8_t *rx_buf,
                                          uint16_t       rx_len,
                                          uint8_t        expect_id,
                                          float         *out_angle)
{
    if (out_angle == NULL) {
        return FSUS_PARSE_BAD_FRAME;
    }
    FsusParseResult_t pre = fsus_check_header(rx_buf, rx_len);
    if (pre != FSUS_PARSE_OK) {
        return pre;
    }
    /* QueryAngle response: HDR(2) + CMD(1)=10 + SIZE(1)=3 + content[ID(1)+angle_s16(2)] + CK(1) = 8 */
    if (rx_len != 8U) {
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_buf[2] != FSUS_CMD_QUERY_ANGLE) {
        return FSUS_PARSE_BAD_FRAME;
    }
    if (rx_buf[3] != 3U) {
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_buf[4] != expect_id) {
        return FSUS_PARSE_BAD_ID;
    }
    if (!fsus_verify_response_checksum(rx_buf, rx_len)) {
        return FSUS_PARSE_BAD_CHECKSUM;
    }

    int16_t raw = fsus_unpack_s16_le(&rx_buf[5]);
    *out_angle = (float)raw / 10.0f;

    return FSUS_PARSE_OK;
}

FsusParseResult_t DrvFsus_ParseServoMonitor(const uint8_t *rx_buf,
                                            uint16_t       rx_len,
                                            uint8_t        expect_id,
                                            uint32_t       now_ms,
                                            FsusFeedback_t *out)
{
    if (out == NULL) {
        return FSUS_PARSE_BAD_FRAME;
    }
    FsusParseResult_t pre = fsus_check_header(rx_buf, rx_len);
    if (pre != FSUS_PARSE_OK) {
        return pre;
    }
    /* ServoMonitor response: HDR(2)+CMD(1)=22+SIZE(1)+content[...]+CK(1)
     *
     * 这一步是总线舵机区别于 PWM 舵机的关键：PWM 舵机通常只能接收脉宽，
     * 而 FSUS 会把内部采样到的状态通过数字帧回传给控制器。
     *
     * Content (20 bytes):
     *   [0]       servo_id
     *   [1-2]     voltage (s16 LE, mV)
     *   [3-4]     current (s16 LE, mA)
     *   [5-6]     power   (s16 LE, mW)
     *   [7-8]     temperature (s16 LE, raw ADC)
     *   [9]       status
     *   [10-13]   angle   (s32 LE, 0.1 deg)
     *   [14-15]   circle_count (s16 LE)
     * Total content = 16 bytes. Total frame = 2+1+1+16+1 = 21 bytes.
     */
    if (rx_buf[2] != FSUS_CMD_SERVO_MONITOR) {
        return FSUS_PARSE_BAD_FRAME;
    }
    /* Check that the expected content length matches. Per FSUS SDK, ServoMonitor
     * response for a single servo returns 16 bytes of content. */
    if (rx_buf[3] != 16U) {
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_len != 21U) {
        return FSUS_PARSE_BAD_SIZE;
    }
    if (rx_buf[4] != expect_id) {
        return FSUS_PARSE_BAD_ID;
    }
    if (!fsus_verify_response_checksum(rx_buf, rx_len)) {
        return FSUS_PARSE_BAD_CHECKSUM;
    }

    const uint8_t *c = &rx_buf[4];  /* content starts at byte 4 */

    out->servo_id     = c[0];
    out->voltage_mv   = fsus_unpack_s16_le(&c[1]);
    out->current_ma   = fsus_unpack_s16_le(&c[3]);
    out->power_mw     = fsus_unpack_s16_le(&c[5]);
    out->temp_raw     = fsus_unpack_s16_le(&c[7]);

    /* status is the servo's compact fault/motion bitfield. Upper layers use
     * it to mark stall/overload instead of guessing from PWM commands. */
    out->status       = c[9];
    {
        /* Angle is returned in 0.1 deg units, signed 32-bit for multi-turn data. */
        int32_t raw_angle = fsus_unpack_s32_le(&c[10]);
        out->angle_deg = (float)raw_angle / 10.0f;
    }
    out->circle_count = fsus_unpack_s16_le(&c[14]);
    out->timestamp_ms = now_ms;

    return FSUS_PARSE_OK;
}
