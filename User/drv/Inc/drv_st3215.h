/**
 * @file drv_st3215.h
 * @brief L2 Hardware Driver: Feetech/Waveshare ST3215-HS Serial Bus Servo
 *
 * Pure protocol encoding / decoding layer for the ST3215-HS intelligent joint.
 * Strictly C99. NO RTOS, NO HAL, NO BSP dependencies. NO FPU.
 *
 * Position closed-loop is handled INSIDE the servo. This driver only produces
 * and parses protocol bytes. Actual half-duplex UART transactions are issued
 * by the upper layer (task_motion) through bsp_uart HD API.
 *
 * Protocol reference: Feetech SCServo library (SMS_STS application layer),
 * equivalent byte-for-byte behaviour to SMS_STS::WritePosEx / FeedBack / Ping.
 *
 * Layering contract:
 *   - All Encode_* functions write into a caller-provided buffer and return
 *     the number of bytes written. They never touch hardware.
 *   - All Parse_* functions consume a caller-provided buffer and populate a
 *     caller-provided output structure. They never touch hardware.
 *   - The driver context carries ONLY servo-level static state (id, last_ok
 *     timestamp, fail counter, last feedback cache). It does NOT own any
 *     UART handle.
 */

#ifndef DRV_ST3215_H
#define DRV_ST3215_H

#include "ara_def.h"

/* =============================================================================
 * 1. Protocol Constants (mirrors SCServo INST.h / SMS_STS.h)
 * ============================================================================= */

/* --- Frame constants --- */
#define ST3215_FRAME_HEADER_BYTE        (0xFFU)
#define ST3215_BROADCAST_ID             (0xFEU)

/* --- Instruction codes --- */
#define ST3215_INST_PING                (0x01U)
#define ST3215_INST_READ                (0x02U)
#define ST3215_INST_WRITE               (0x03U)
#define ST3215_INST_REG_WRITE           (0x04U)
#define ST3215_INST_REG_ACTION          (0x05U)
#define ST3215_INST_SYNC_WRITE          (0x83U)

/* --- Buffer sizing --- */
/**
 * @brief Maximum TX frame length this driver will produce.
 * @note  WritePosEx = 14 bytes; ReadFeedback = 8 bytes; Ping = 6 bytes;
 *        Single-register writes = 7 or 8 bytes. 32 bytes covers all cases.
 */
#define ST3215_TX_BUF_SIZE              (32U)

/**
 * @brief Maximum RX frame length this driver will consume.
 * @note  FeedBack response = 21 bytes (4 header + 1 error + 15 payload + 1 crc).
 *        32 bytes covers all cases with margin.
 */
#define ST3215_RX_BUF_SIZE              (32U)

/**
 * @brief Minimal ACK response length (no payload).
 * @note  Layout: FF FF ID LEN=2 ERROR CHECKSUM.
 */
#define ST3215_ACK_FRAME_LEN            (6U)

/**
 * @brief FeedBack response frame length (covers registers 56..70).
 * @note  Layout: FF FF ID LEN=17 ERROR <15 bytes> CHECKSUM.
 */
#define ST3215_FEEDBACK_FRAME_LEN       (21U)

/* --- Baudrate codes (register value for SMS_STS_BAUD_RATE) --- */
#define ST3215_BAUD_CODE_1M             (0U)
#define ST3215_BAUD_CODE_500K           (1U)
#define ST3215_BAUD_CODE_115200         (4U)

/* =============================================================================
 * 2. Register Map (mirrors Feetech SMS_STS.h memory table)
 * ============================================================================= */

/* ---- EEPROM: read only ---- */
#define ST3215_REG_MODEL_L              (3U)
#define ST3215_REG_MODEL_H              (4U)

/* ---- EEPROM: read / write (require LOCK = 0) ---- */
#define ST3215_REG_ID                   (5U)
#define ST3215_REG_BAUD_RATE            (6U)
#define ST3215_REG_MIN_ANGLE_LIMIT_L    (9U)
#define ST3215_REG_MIN_ANGLE_LIMIT_H    (10U)
#define ST3215_REG_MAX_ANGLE_LIMIT_L    (11U)
#define ST3215_REG_MAX_ANGLE_LIMIT_H    (12U)
#define ST3215_REG_CW_DEAD              (26U)
#define ST3215_REG_CCW_DEAD             (27U)
#define ST3215_REG_OFS_L                (31U)
#define ST3215_REG_OFS_H                (32U)
#define ST3215_REG_MODE                 (33U)

/* ---- SRAM: read / write ---- */
#define ST3215_REG_TORQUE_ENABLE        (40U)
#define ST3215_REG_ACC                  (41U)
#define ST3215_REG_GOAL_POSITION_L      (42U)
#define ST3215_REG_GOAL_POSITION_H      (43U)
#define ST3215_REG_GOAL_TIME_L          (44U)
#define ST3215_REG_GOAL_TIME_H          (45U)
#define ST3215_REG_GOAL_SPEED_L         (46U)
#define ST3215_REG_GOAL_SPEED_H         (47U)
#define ST3215_REG_TORQUE_LIMIT_L       (48U)
#define ST3215_REG_TORQUE_LIMIT_H       (49U)
#define ST3215_REG_LOCK                 (55U)

/* ---- SRAM: read only (continuous block consumed by FeedBack) ---- */
#define ST3215_REG_PRESENT_POSITION_L   (56U)
#define ST3215_REG_PRESENT_POSITION_H   (57U)
#define ST3215_REG_PRESENT_SPEED_L      (58U)
#define ST3215_REG_PRESENT_SPEED_H      (59U)
#define ST3215_REG_PRESENT_LOAD_L       (60U)
#define ST3215_REG_PRESENT_LOAD_H       (61U)
#define ST3215_REG_PRESENT_VOLTAGE      (62U)
#define ST3215_REG_PRESENT_TEMPERATURE  (63U)
#define ST3215_REG_MOVING               (66U)
#define ST3215_REG_PRESENT_CURRENT_L    (69U)
#define ST3215_REG_PRESENT_CURRENT_H    (70U)

/**
 * @brief Length of the contiguous SRAM block read by DrvSt3215_EncodeReadFeedback.
 * @note  From PRESENT_POSITION_L(56) to PRESENT_CURRENT_H(70), inclusive. 15 bytes.
 */
#define ST3215_FEEDBACK_PAYLOAD_LEN     (15U)

/* ---- Special values ---- */
#define ST3215_TORQUE_DISABLE           (0U)
#define ST3215_TORQUE_ENABLE            (1U)
/**
 * @brief Writing this value to TORQUE_ENABLE burns current position as the
 *        middle offset (OFS register). Consumes an EEPROM write cycle.
 */
#define ST3215_TORQUE_CALIBRATE_MIDDLE  (128U)

#define ST3215_LOCK_OPEN                (0U)
#define ST3215_LOCK_CLOSED              (1U)

#define ST3215_MODE_POSITION_SERVO      (0U) /**< Single-turn servo, bounded by MIN/MAX_ANGLE_LIMIT. */
#define ST3215_MODE_WHEEL               (1U) /**< Continuous velocity loop, no position. */
#define ST3215_MODE_PWM_OPEN            (2U) /**< Open-loop PWM. */
#define ST3215_MODE_STEP                (3U) /**< Multi-turn step position (MAX_ANGLE_LIMIT = 0). */

/* =============================================================================
 * 3. Error Bitfield (servo ERROR byte in response)
 * ============================================================================= */
#define ST3215_ERRBIT_VOLTAGE           (1U << 0) /**< Voltage out of range. */
#define ST3215_ERRBIT_SENSOR            (1U << 1) /**< Magnetic encoder fault. */
#define ST3215_ERRBIT_TEMPERATURE       (1U << 2) /**< Over-temperature. */
#define ST3215_ERRBIT_CURRENT           (1U << 3) /**< Over-current. */
#define ST3215_ERRBIT_ANGLE             (1U << 4) /**< Position tracking error too large. */
#define ST3215_ERRBIT_OVERLOAD          (1U << 5) /**< Sustained overload. */

/* =============================================================================
 * 4. Physical Constants
 * ============================================================================= */

/**
 * @brief Encoder steps per full mechanical revolution.
 * @note  ST3215-HS uses a 12-bit magnetic encoder mapped to 0..4095 over 360°.
 */
#define ST3215_STEPS_PER_REV            (4096)

/**
 * @brief Convert integer degrees to step units (integer math).
 * @note  Uses 32-bit intermediate to avoid overflow.
 *        Signed degrees are accepted for future multi-turn usage.
 */
#define ST3215_DEG_TO_STEPS(deg_int)    ((int32_t)(deg_int) * ST3215_STEPS_PER_REV / 360)

/**
 * @brief Convert step units to integer degrees.
 */
#define ST3215_STEPS_TO_DEG(step_int)   ((int32_t)(step_int) * 360 / ST3215_STEPS_PER_REV)

/**
 * @brief Target position clamp limits for single-turn servo mode.
 */
#define ST3215_POS_SINGLE_MIN           (0)
#define ST3215_POS_SINGLE_MAX           (ST3215_STEPS_PER_REV - 1)

/* =============================================================================
 * 5. Time Budget (demo_v7 locked to 1 Mbps baudrate)
 * ============================================================================= */

/**
 * @brief Single round-trip IO timeout used by the upper layer when waiting for
 *        a response at 1 Mbps. Round-trip at 1 Mbps is < 500 us; 2 ms leaves
 *        4x safety margin. See Doc/design/04_ST3215_PROTOCOL.md for derivation.
 * @note  This value is a contract between L2 driver and L1 BSP HD API.
 *        L2 does not enforce it (L2 is pure protocol); the upper layer must
 *        pass this value to BSP_UART_HD_WaitRx.
 */
#define ST3215_IO_TIMEOUT_MS_1M         (2U)

/**
 * @brief Cumulative offline judgement threshold. When last successful
 *        exchange is older than this, the servo is considered disconnected.
 * @note  Enforced by ClassifyIoResult against ctx->last_ok_ms.
 */
#define ST3215_OFFLINE_MS               (250U)

/* =============================================================================
 * 6. Status Types
 * ============================================================================= */

/**
 * @brief Result of a single protocol-level IO attempt, as reported by the
 *        driver after encode/transact/parse sequence.
 */
typedef enum {
    ST3215_IO_OK        = 0,    /**< Valid ACK or feedback received. */
    ST3215_IO_TIMEOUT,          /**< Upper layer reported RX timeout; link may still be healthy. */
    ST3215_IO_BAD_FRAME,        /**< Header / length / checksum error. */
    ST3215_IO_SERVO_ERROR,      /**< Frame OK but ERROR byte is non-zero. */
    ST3215_IO_OFFLINE           /**< Cumulative failure exceeded ST3215_OFFLINE_MS. */
} St3215_IoResult_t;

/**
 * @brief Result of a pure-function protocol parse (no timing involved).
 */
typedef enum {
    ST3215_PARSE_OK         = 0,
    ST3215_PARSE_BAD_HEADER,    /**< First two bytes are not 0xFF 0xFF. */
    ST3215_PARSE_BAD_LENGTH,    /**< LEN field inconsistent with rx_len. */
    ST3215_PARSE_BAD_ID,        /**< Response ID mismatch. */
    ST3215_PARSE_BAD_CHECKSUM,  /**< Checksum mismatch. */
    ST3215_PARSE_SERVO_ERROR    /**< Frame OK but ERROR byte is non-zero. */
} St3215_ParseResult_t;

/**
 * @brief High-level motion classification used by manipulator FSM.
 * @note  Computed from the feedback + target + thresholds in a pure function.
 */
typedef enum {
    ST3215_MOTION_UNKNOWN = 0,  /**< No fresh feedback yet. */
    ST3215_MOTION_ARRIVED,      /**< At target, stationary, speed near zero;
                                     pre-empts OVERLOAD so a payload-holding
                                     pose is not mistaken for a fault. */
    ST3215_MOTION_MOVING,       /**< Still traversing. */
    ST3215_MOTION_STALLED,      /**< Stopped before reaching target. */
    ST3215_MOTION_OVERLOAD      /**< |load| exceeds threshold while servo is
                                     moving or off-target. Indicates "monitor",
                                     not "must release torque" — the policy of
                                     stripping torque belongs to the upper
                                     layer, not the driver. */
} St3215_MotionStatus_t;

/* =============================================================================
 * 7. Feedback Data Structure
 * ============================================================================= */

/**
 * @brief Parsed feedback from one ReadFeedback response.
 * @note  All signed fields follow SMS_STS semantics: bit15 (or bit10 for load)
 *        is the sign bit, already decoded into native signed C integers here.
 *        Driver consumers must NOT re-apply the sign mask.
 */
typedef struct {
    int16_t  position;          /**< Present position in step units. Single-turn: 0..4095. */
    int16_t  speed;             /**< Present velocity in step/s, signed. */
    int16_t  load;              /**< Present load, -1000..+1000. Sign = rotation direction. */
    uint8_t  voltage_dv;        /**< Present voltage in 0.1 V units (e.g. 74 = 7.4 V). */
    uint8_t  temp_c;            /**< Present temperature in degrees Celsius. */
    bool     moving;            /**< True if servo reports it is actively moving. */
    int16_t  current;           /**< Present current, signed. One unit ~ 6.5 mA. */
    uint32_t timestamp_ms;      /**< Upper layer timestamp when this feedback was accepted. */
} St3215_Feedback_t;

/* =============================================================================
 * 8. Motion Classification Thresholds
 * ============================================================================= */

/**
 * @brief Default position tolerance for ARRIVED classification, step units.
 *        At 4096 steps / 360 deg this is approximately 1.76 deg.
 */
#define ST3215_DEFAULT_POS_TOL_STEPS    (20)

/**
 * @brief Default speed tolerance for ARRIVED classification, step/s.
 */
#define ST3215_DEFAULT_SPD_TOL_STEPS    (10)

/**
 * @brief Default overload magnitude threshold for STALLED / OVERLOAD detection.
 */
#define ST3215_DEFAULT_LOAD_OVERLOAD    (800)

typedef struct {
    int16_t pos_tol_steps;      /**< |target - position| tolerance for ARRIVED. */
    int16_t spd_tol_steps;      /**< |speed| tolerance for ARRIVED. */
    int16_t load_overload_abs;  /**< |load| threshold for STALLED / OVERLOAD. */
} St3215_MotionThresholds_t;

/* =============================================================================
 * 9. Driver Context
 * ============================================================================= */

/**
 * @brief Per-servo driver context.
 * @note  Does NOT carry a UART handle. The upper layer binds this context to
 *        a specific BSP UART device externally. Protocol logic is hardware-
 *        agnostic.
 *
 *        Life-cycle fields (last_ok_ms, fail_count, last_fb) are updated by
 *        the upper layer through DrvSt3215_NoteIoOk / DrvSt3215_NoteIoFail
 *        helper hooks, so that DrvSt3215_IsOnline can be queried purely from
 *        the context without reaching into timing code.
 */
typedef struct {
    uint8_t           servo_id;       /**< 1..253. 0xFE = broadcast (reserved). */

    /* Link statistics (updated by upper layer via Note helpers) */
    uint32_t          last_ok_ms;     /**< Last successful exchange timestamp. */
    uint32_t          fail_count;     /**< Consecutive failures since last OK. */
    uint8_t           last_error_bits;/**< ERROR byte from the last non-OK response. */

    /* Latest cached feedback (optional, for IsOnline/telemetry helpers) */
    St3215_Feedback_t last_fb;
    bool              last_fb_valid;
} DrvSt3215_Context_t;

/* =============================================================================
 * 10. Context Lifecycle API
 * ============================================================================= */

/**
 * @brief  Initialise driver context to a safe default state.
 * @param  ctx      Pointer to context storage (caller-owned).
 * @param  servo_id Target servo ID (1..253).
 * @retval ARA_OK on success.
 * @retval ARA_ERR_PARAM when ctx is NULL or servo_id is 0 / 0xFE / >253.
 * @note   Non-blocking. Does NOT produce or receive any bytes.
 */
AraStatus_t DrvSt3215_Init(DrvSt3215_Context_t *ctx, uint8_t servo_id);

/**
 * @brief  Notify the driver that a successful exchange just completed.
 * @param  ctx    Driver context.
 * @param  now_ms Current timestamp from upper layer, milliseconds.
 * @note   Resets fail_count. Called by the upper layer after a parse succeeds.
 */
void DrvSt3215_NoteIoOk(DrvSt3215_Context_t *ctx, uint32_t now_ms);

/**
 * @brief  Notify the driver that the most recent exchange failed.
 * @param  ctx         Driver context.
 * @param  error_bits  ERROR byte from the response, or 0 when no response.
 * @note   Increments fail_count. Does NOT change last_ok_ms.
 */
void DrvSt3215_NoteIoFail(DrvSt3215_Context_t *ctx, uint8_t error_bits);

/**
 * @brief  Query whether the servo is considered online at the given time.
 * @param  ctx    Driver context.
 * @param  now_ms Current timestamp, milliseconds.
 * @return true  when (now_ms - last_ok_ms) < ST3215_OFFLINE_MS.
 * @return false when never succeeded OR offline threshold exceeded.
 */
bool DrvSt3215_IsOnline(const DrvSt3215_Context_t *ctx, uint32_t now_ms);

/* =============================================================================
 * 11. Encoding API (pure functions, no hardware)
 *
 * All Encode_* functions write protocol bytes into tx_buf and return the
 * number of bytes written. Zero return means encoding failed (null buffer,
 * buffer too small, invalid parameters).
 * ============================================================================= */

/**
 * @brief  Encode a PING instruction.
 * @param  tx_buf   Output buffer, must hold at least 6 bytes.
 * @param  servo_id Target servo ID.
 * @return Number of bytes written (6), or 0 on error.
 * @note   One-shot initialisation use. Response is an ACK frame.
 */
uint8_t DrvSt3215_EncodePing(uint8_t *tx_buf, uint8_t servo_id);

/**
 * @brief  Encode a READ instruction for a contiguous register block.
 * @param  tx_buf    Output buffer, must hold at least 8 bytes.
 * @param  servo_id  Target servo ID.
 * @param  reg_addr  Starting register address.
 * @param  read_len  Number of bytes to read (1..64).
 * @return Number of bytes written (8), or 0 on error.
 */
uint8_t DrvSt3215_EncodeRead(uint8_t *tx_buf,
                             uint8_t servo_id,
                             uint8_t reg_addr,
                             uint8_t read_len);

/**
 * @brief  Convenience: encode the FeedBack read (registers 56..70, 15 bytes).
 * @param  tx_buf   Output buffer, must hold at least 8 bytes.
 * @param  servo_id Target servo ID.
 * @return Number of bytes written (8), or 0 on error.
 * @note   Runtime-path use. The response is a 21-byte FeedBack frame.
 */
uint8_t DrvSt3215_EncodeReadFeedback(uint8_t *tx_buf, uint8_t servo_id);

/**
 * @brief  Encode a single-byte WRITE.
 * @param  tx_buf    Output buffer, must hold at least 8 bytes.
 * @param  servo_id  Target servo ID.
 * @param  reg_addr  Target register.
 * @param  value     Byte to write.
 * @return Number of bytes written (8), or 0 on error.
 * @note   Used during bring-up: TORQUE_ENABLE, LOCK, MODE, ACC.
 */
uint8_t DrvSt3215_EncodeWriteByte(uint8_t *tx_buf,
                                  uint8_t servo_id,
                                  uint8_t reg_addr,
                                  uint8_t value);

/**
 * @brief  Encode a 16-bit WRITE (little-endian, matching SMS_STS End=0).
 * @param  tx_buf    Output buffer, must hold at least 9 bytes.
 * @param  servo_id  Target servo ID.
 * @param  reg_addr  Target register (low byte).
 * @param  value     Word to write.
 * @return Number of bytes written (9), or 0 on error.
 * @note   Used for MIN/MAX_ANGLE_LIMIT and TORQUE_LIMIT during bring-up.
 */
uint8_t DrvSt3215_EncodeWriteWord(uint8_t *tx_buf,
                                  uint8_t servo_id,
                                  uint8_t reg_addr,
                                  uint16_t value);

/**
 * @brief  Encode WritePosEx: set target position with speed and acceleration.
 *         Writes 7 bytes starting at register ACC(41).
 * @param  tx_buf      Output buffer, must hold at least 14 bytes.
 * @param  servo_id    Target servo ID.
 * @param  target_pos  Target position in step units. Signed: bit15 of the
 *                     over-the-wire word is set for negative values
 *                     (this driver applies the sign encoding internally).
 * @param  speed       Target speed, step/s. 0 means use the servo's maximum.
 * @param  acc         Acceleration (0..254, 100 step/s^2 per unit).
 * @return Number of bytes written (14), or 0 on error.
 * @note   Runtime hot-path. Upper layer SHOULD dedup by target deadband and
 *         emit this at most on target change plus a periodic keepalive.
 */
uint8_t DrvSt3215_EncodeWritePos(uint8_t *tx_buf,
                                 uint8_t  servo_id,
                                 int16_t  target_pos,
                                 uint16_t speed,
                                 uint8_t  acc);

/* =============================================================================
 * 12. Parsing API (pure functions, no hardware)
 *
 * All Parse_* functions consume rx_buf and populate the output structure. The
 * upper layer is responsible for ensuring rx_buf holds a complete frame
 * (typically via BSP_UART_HD_WaitRx which returns the received byte count).
 * ============================================================================= */

/**
 * @brief  Parse a bare ACK frame (WRITE / PING response, 6 bytes).
 * @param  rx_buf           Received frame, length at least 6.
 * @param  rx_len           Number of bytes in rx_buf.
 * @param  expect_id        Servo ID the caller issued the request to.
 * @param  out_error_bits   Output: ERROR byte (0 when no fault). May be NULL.
 * @return ST3215_PARSE_OK                 when frame is structurally valid and
 *                                         ERROR byte is zero.
 * @return ST3215_PARSE_SERVO_ERROR        when frame is valid but ERROR byte
 *                                         is non-zero. out_error_bits is set.
 * @return ST3215_PARSE_BAD_*              when the frame is malformed.
 */
St3215_ParseResult_t DrvSt3215_ParseAck(const uint8_t *rx_buf,
                                        uint8_t        rx_len,
                                        uint8_t        expect_id,
                                        uint8_t       *out_error_bits);

/**
 * @brief  Parse a FeedBack response (21 bytes) into St3215_Feedback_t.
 * @param  rx_buf           Received frame.
 * @param  rx_len           Number of bytes in rx_buf. Must be >= 21.
 * @param  expect_id        Servo ID the caller issued the request to.
 * @param  now_ms           Timestamp written into out->timestamp_ms on success.
 * @param  out              Output feedback structure. Populated only on
 *                          PARSE_OK. Left untouched on any error.
 * @param  out_error_bits   Output: ERROR byte. May be NULL.
 * @return ST3215_PARSE_OK / PARSE_SERVO_ERROR / PARSE_BAD_*.
 * @note   On PARSE_SERVO_ERROR the payload is NOT decoded: the upper layer
 *         should treat the response as unreliable.
 */
St3215_ParseResult_t DrvSt3215_ParseFeedback(const uint8_t     *rx_buf,
                                             uint8_t            rx_len,
                                             uint8_t            expect_id,
                                             uint32_t           now_ms,
                                             St3215_Feedback_t *out,
                                             uint8_t           *out_error_bits);

/* =============================================================================
 * 13. Utility API
 * ============================================================================= */

/**
 * @brief  Compute SMS_STS-style checksum over a byte range.
 *         Sum then bitwise invert.
 * @param  data Pointer to payload (starting at ID, excluding the two 0xFF
 *              header bytes).
 * @param  len  Number of bytes covered by the sum.
 * @return Inverted 8-bit sum.
 */
uint8_t DrvSt3215_Checksum(const uint8_t *data, uint8_t len);

/**
 * @brief  Classify motion state from feedback and desired target.
 * @param  fb           Latest feedback (must be valid).
 * @param  target_pos   Commanded position in step units.
 * @param  thresholds   Tolerances. Pass NULL to use ST3215_DEFAULT_* macros.
 * @return One of St3215_MotionStatus_t values.
 * @note   Pure function. Does NOT consult servo ERROR byte; overload in this
 *         classification is load-based, not fault-bit-based. Fault handling
 *         remains the upper layer's responsibility.
 */
St3215_MotionStatus_t DrvSt3215_ClassifyMotion(
    const St3215_Feedback_t           *fb,
    int16_t                            target_pos,
    const St3215_MotionThresholds_t   *thresholds);

/**
 * @brief  Translate a parse / IO outcome into a user-facing IoResult, taking
 *         the driver context's offline bookkeeping into account.
 * @param  ctx           Driver context.
 * @param  parse_result  Outcome of the most recent parse attempt, OR
 *                       ST3215_PARSE_OK when the upper layer reports a
 *                       transport-level timeout.
 * @param  upper_timeout true when the upper layer reported no bytes received.
 * @param  now_ms        Current timestamp, milliseconds.
 * @return Mapped St3215_IoResult_t.
 */
St3215_IoResult_t DrvSt3215_ClassifyIoResult(
    const DrvSt3215_Context_t *ctx,
    St3215_ParseResult_t       parse_result,
    bool                       upper_timeout,
    uint32_t                   now_ms);

#endif /* DRV_ST3215_H */
