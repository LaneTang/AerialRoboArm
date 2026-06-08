/**
 * @file drv_fsus.h
 * @brief L2 Hardware Driver: Fashion Star UART Servo (FSUS) protocol.
 *
 * Pure protocol encoding / decoding layer for HX8-U26H-M brushless servo.
 * Strictly C99. NO RTOS, NO HAL, NO BSP dependencies. NO FPU (float in API,
 * integer math internally).
 *
 * Protocol reference: Fashion Star Bus Servo SDK v0.0.2
 *
 * Wire format (little-endian throughout):
 *   Request:  0x4C 0x12  CMD  SIZE  [CONTENT...]  CHECKSUM
 *   Response: 0x05 0x1C  CMD  SIZE  [CONTENT...]  CHECKSUM
 *
 * Checksum = ~(sum of all bytes from header through last content byte) & 0xFF.
 */

#ifndef DRV_FSUS_H
#define DRV_FSUS_H

#include "ara_def.h"

/* =============================================================================
 * 1. Protocol Constants
 * ============================================================================= */

/* --- Frame header (little-endian on wire) --- */
#define FSUS_HEADER_REQ_HI              0x4CU
#define FSUS_HEADER_REQ_LO              0x12U
#define FSUS_HEADER_RESP_HI             0x1CU
#define FSUS_HEADER_RESP_LO             0x05U

/* --- Command IDs --- */
#define FSUS_CMD_PING                   1U
#define FSUS_CMD_SET_ANGLE_BY_VELOCITY  12U
#define FSUS_CMD_QUERY_ANGLE            10U
#define FSUS_CMD_SERVO_MONITOR          22U
#define FSUS_CMD_STOP_ON_CONTROL_MODE   24U

/* --- Stop mode (cmd 24) --- */
#define FSUS_STOP_MODE_UNLOCK           0U   /**< Release torque after stop. */
#define FSUS_STOP_MODE_HOLD             1U   /**< Hold position after stop. */
#define FSUS_STOP_MODE_DAMPING          2U   /**< Damping after stop. */

/* --- Buffer sizing --- */
#define FSUS_TX_BUF_SIZE                64U
#define FSUS_RX_BUF_SIZE                64U

/* --- Angle & velocity limits (from FSUS spec) --- */
#define FSUS_ANGLE_MIN_DEG              (-180.0f)
#define FSUS_ANGLE_MAX_DEG              (180.0f)
#define FSUS_VELOCITY_MIN_DEG_PER_S     (1.0f)
#define FSUS_VELOCITY_MAX_DEG_PER_S     (750.0f)
#define FSUS_T_ACC_DEC_MIN_MS           (20U)

/* --- Timeouts (ms) --- */
#define FSUS_IO_TIMEOUT_MS              (100U)

/* =============================================================================
 * 2. Feedback Structure
 * ============================================================================= */

typedef struct {
    uint8_t  servo_id;
    int16_t  voltage_mv;         /**< Supply voltage, mV. */
    int16_t  current_ma;         /**< Instantaneous current, mA. */
    int16_t  power_mw;           /**< Instantaneous power, mW. */
    int16_t  temp_raw;           /**< Raw ADC reading (thermistor). */
    uint8_t  status;             /**< Bitfield: BIT2=stall, BIT3=overvolt, etc. */
    float    angle_deg;          /**< Present angle, degrees (-180..+180). */
    int16_t  circle_count;       /**< Multi-turn circle count. */
    uint32_t timestamp_ms;       /**< Upper layer timestamp. */
} FsusFeedback_t;

/* =============================================================================
 * 3. Parse / IO Status
 * ============================================================================= */

typedef enum {
    FSUS_PARSE_OK = 0,
    FSUS_PARSE_BAD_HEADER,       /**< Wrong response header. */
    FSUS_PARSE_BAD_ID,           /**< Response servo ID mismatch. */
    FSUS_PARSE_BAD_SIZE,         /**< Size field inconsistent or too large. */
    FSUS_PARSE_BAD_CHECKSUM,     /**< Checksum mismatch. */
    FSUS_PARSE_TIMEOUT,          /**< Upper layer reported RX timeout. */
    FSUS_PARSE_BAD_FRAME         /**< Generic frame error. */
} FsusParseResult_t;

/* =============================================================================
 * 4. Encoding API (pure functions)
 *
 * All Encode_* write protocol bytes into tx_buf and return bytes written.
 * Return 0 on parameter error.
 * ============================================================================= */

/**
 * @brief  Encode a PING request.
 * @param  tx_buf   Output buffer, >= 6 bytes.
 * @param  servo_id Target servo ID (0..254).
 * @return Bytes written (6), or 0 on error.
 */
uint16_t DrvFsus_EncodePing(uint8_t *tx_buf, uint8_t servo_id);

/**
 * @brief  Encode SetAngleByVelocity: move to angle at specified speed.
 * @param  tx_buf   Output buffer, >= FSUS_TX_BUF_SIZE.
 * @param  servo_id Target servo ID.
 * @param  angle_deg       Target angle, degrees (-180..+180).
 * @param  velocity_deg_per_s  Traversal speed, deg/s (1..750).
 * @param  t_acc_ms        Acceleration time, ms (>= 20).
 * @param  t_dec_ms        Deceleration time, ms (>= 20).
 * @param  power_mw        Execution power, mW (0 = servo default).
 * @return Bytes written, or 0 on error.
 */
uint16_t DrvFsus_EncodeSetAngleByVelocity(uint8_t *tx_buf,
                                          uint8_t  servo_id,
                                          float    angle_deg,
                                          float    velocity_deg_per_s,
                                          uint16_t t_acc_ms,
                                          uint16_t t_dec_ms,
                                          uint16_t power_mw);

/**
 * @brief  Encode a QueryAngle request.
 * @param  tx_buf   Output buffer, >= 5 bytes.
 * @param  servo_id Target servo ID.
 * @return Bytes written (5), or 0 on error.
 */
uint16_t DrvFsus_EncodeQueryAngle(uint8_t *tx_buf, uint8_t servo_id);

/**
 * @brief  Encode a ServoMonitor request (reads voltage/current/power/temp/status/angle).
 * @param  tx_buf   Output buffer, >= 5 bytes.
 * @param  servo_id Target servo ID.
 * @return Bytes written (5), or 0 on error.
 */
uint16_t DrvFsus_EncodeServoMonitor(uint8_t *tx_buf, uint8_t servo_id);

/**
 * @brief  Encode a StopOnControlMode command.
 * @param  tx_buf   Output buffer, >= 9 bytes.
 * @param  servo_id Target servo ID.
 * @param  mode     Stop mode: 0=unlock, 1=hold, 2=damping.
 * @param  power_mw Damping power (relevant for mode 2).
 * @return Bytes written, or 0 on error.
 */
uint16_t DrvFsus_EncodeStop(uint8_t *tx_buf,
                            uint8_t  servo_id,
                            uint8_t  mode,
                            uint16_t power_mw);

/* =============================================================================
 * 5. Parsing API (pure functions)
 * ============================================================================= */

/**
 * @brief  Parse a PING response.
 * @param  rx_buf     Received frame, >= 6 bytes.
 * @param  rx_len     Number of bytes in rx_buf.
 * @param  expect_id  Servo ID expected in response.
 * @return FSUS_PARSE_OK on valid echo, error code otherwise.
 */
FsusParseResult_t DrvFsus_ParsePing(const uint8_t *rx_buf,
                                    uint16_t       rx_len,
                                    uint8_t        expect_id);

/**
 * @brief  Parse a QueryAngle response.
 * @param  rx_buf     Received frame.
 * @param  rx_len     Rx byte count.
 * @param  expect_id  Servo ID expected.
 * @param  out_angle  Output: angle in degrees (-180..+180). Unchanged on error.
 * @return FSUS_PARSE_OK or error.
 */
FsusParseResult_t DrvFsus_ParseQueryAngle(const uint8_t *rx_buf,
                                          uint16_t       rx_len,
                                          uint8_t        expect_id,
                                          float         *out_angle);

/**
 * @brief  Parse a ServoMonitor response.
 * @param  rx_buf     Received frame.
 * @param  rx_len     Rx byte count.
 * @param  expect_id  Servo ID expected.
 * @param  now_ms     Timestamp written into out->timestamp_ms on success.
 * @param  out        Output feedback structure. Populated only on OK.
 * @return FSUS_PARSE_OK or error.
 */
FsusParseResult_t DrvFsus_ParseServoMonitor(const uint8_t *rx_buf,
                                            uint16_t       rx_len,
                                            uint8_t        expect_id,
                                            uint32_t       now_ms,
                                            FsusFeedback_t *out);

/* =============================================================================
 * 6. Utility
 * ============================================================================= */

/**
 * @brief  Compute FSUS frame checksum.
 *         Checksum = ~(sum over data[start..start+len-1]) & 0xFF.
 */
uint8_t DrvFsus_Checksum(const uint8_t *data, uint16_t len);

#endif /* DRV_FSUS_H */
