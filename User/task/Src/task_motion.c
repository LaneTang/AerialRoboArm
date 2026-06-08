/**
 * @file task_motion.c
 * @brief FSUS servo proxy with Mock fallback.
 */

#include "task_motion.h"
#include "bsp_uart.h"
#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"

#include <string.h>

/* =============================================================================
 * Internal state
 * ============================================================================= */

/* Last-sent target tracking for change detection. */
static float    s_last_target_deg     = -999.0f;
static bool     s_torque_active       = false;

#if TASK_MOTION_USE_MOCK
/* Mock-mode servo simulator */
static float    s_mock_pos_deg        = 0.0f;
static float    s_mock_target_deg     = 0.0f;
static bool     s_mock_torque_on      = false;
static uint32_t s_mock_last_tick_ms   = 0U;

#define MOCK_SLEW_DEG_PER_SEC   (130.0f)
#endif

/* =============================================================================
 * Helpers
 * ============================================================================= */

static float map_angle_to_fsus(float deg)
{
    /* Whole stack now operates in the FSUS-native signed -180..+180 domain.
     * This helper only enforces hardware range; no wrap/translation needed. */
    if (deg > FSUS_ANGLE_MAX_DEG)       return FSUS_ANGLE_MAX_DEG;
    if (deg < FSUS_ANGLE_MIN_DEG)       return FSUS_ANGLE_MIN_DEG;
    return deg;
}

/**
 * @brief FSUS transaction: send, then sync on response header 0x05 0x1C.
 * @param tx_buf    Encoded request frame.
 * @param tx_len    Request length.
 * @param rx_buf    Response buffer (>= FSUS_RX_BUF_SIZE).
 * @param timeout_ms Max wait.
 * @return Total response bytes received, or 0 on timeout.
 */
static uint16_t fsus_transact(const uint8_t *tx_buf, uint16_t tx_len,
                              uint8_t *rx_buf, uint32_t timeout_ms)
{
    /* Drop stale bytes before every request. A half-old response in the ring
     * buffer would otherwise look like a valid but unrelated servo reply. */
    BSP_UART_Fsus_Flush();
    BSP_UART_Fsus_Send(tx_buf, tx_len);

    uint32_t deadline = HAL_GetTick() + timeout_ms;
    uint16_t total = 0U;

    /* 1. Collect enough bytes to identify the response and read SIZE. */
    while (total < 5U) {
        total += BSP_UART_Fsus_Recv(&rx_buf[total], (uint16_t)(5U - total));
        if (total >= 5U) break;
        if (HAL_GetTick() > deadline) return 0U;
        osDelay(1U);   /* yield so lower-prio tasks (housekeeping/IWDG) run */
    }

    /* 2. Sync to response header 0x05 0x1C. If noise or stale bytes are
     * present, slide one byte at a time until the frame boundary is aligned. */
    while (rx_buf[0] != 0x05U || rx_buf[1] != 0x1CU) {
        /* Shift buffer left by 1, read one more byte. */
        for (uint16_t i = 0U; i < total - 1U; i++) {
            rx_buf[i] = rx_buf[i + 1U];
        }
        total--;
        while (BSP_UART_Fsus_Recv(&rx_buf[total], 1U) == 0U) {
            if (HAL_GetTick() > deadline) return 0U;
            osDelay(1U);
        }
        total++;
    }

    /* 3. Read remaining bytes. SIZE is the content length.
     *     Frame length = header(2) + cmd(1) + size(1) + content(size) + ck(1)
     *                  = 5 + content_size. */
    uint8_t content_size = rx_buf[3];
    uint16_t frame_len = (uint16_t)(5U + content_size);
    if (frame_len > FSUS_RX_BUF_SIZE) return 0U;

    while (total < frame_len) {
        total += BSP_UART_Fsus_Recv(&rx_buf[total], (uint16_t)(frame_len - total));
        if (total >= frame_len) break;
        if (HAL_GetTick() > deadline) return 0U;
        osDelay(1U);
    }
    return total;
}

/* =============================================================================
 * Real-servo helpers
 * ============================================================================= */

#if !TASK_MOTION_USE_MOCK

static FsusParseResult_t do_stop(uint8_t mode, uint16_t power_mw)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint8_t rx[FSUS_RX_BUF_SIZE];

    uint16_t tx_len = DrvFsus_EncodeStop(tx, TASK_MOTION_SERVO_ID, mode, power_mw);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }

    /* Stop has no defined response; this is a fire-and-forget control frame.
     * In unlock mode it releases the servo's holding torque. */
    BSP_UART_Fsus_Flush();
    BSP_UART_Fsus_Send(tx, tx_len);
    return FSUS_PARSE_OK;
}

static FsusParseResult_t do_set_angle(float    angle_deg,
                                       float    velocity_deg_per_s,
                                       uint16_t t_acc_ms,
                                       uint16_t t_dec_ms,
                                       uint16_t power_mw)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint8_t rx[FSUS_RX_BUF_SIZE];

    /* FSUS expects no response for SetAngleByVelocity by default.
     * Send and return OK — feedback is obtained via ServoMonitor. */
    /* Encode a motion target. This updates the servo's internal target
     * trajectory; the servo then closes the loop with its own encoder. */
    uint16_t tx_len = DrvFsus_EncodeSetAngleByVelocity(tx, TASK_MOTION_SERVO_ID,
                                                       angle_deg, velocity_deg_per_s,
                                                       t_acc_ms, t_dec_ms, power_mw);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }
    BSP_UART_Fsus_Flush();
    BSP_UART_Fsus_Send(tx, tx_len);
    return FSUS_PARSE_OK;
}

static FsusParseResult_t do_read_feedback(uint32_t        tick_ms,
                                          FsusFeedback_t *out)
{
    uint8_t tx[FSUS_TX_BUF_SIZE];
    uint8_t rx[FSUS_RX_BUF_SIZE];

    /* Ask the servo to report the state measured by its internal controller:
     * angle, voltage, current, power, temperature and status bits. */
    uint16_t tx_len = DrvFsus_EncodeServoMonitor(tx, TASK_MOTION_SERVO_ID);
    if (tx_len == 0U) {
        return FSUS_PARSE_BAD_FRAME;
    }

    uint16_t rx_len = fsus_transact(tx, tx_len, rx, FSUS_IO_TIMEOUT_MS);
    if (rx_len < 5U) {
        return FSUS_PARSE_TIMEOUT;
    }
    /* Convert raw UART bytes into a typed feedback struct for the FSM. */
    return DrvFsus_ParseServoMonitor(rx, rx_len, TASK_MOTION_SERVO_ID, tick_ms, out);
}

#endif /* !TASK_MOTION_USE_MOCK */

/* =============================================================================
 * Public API
 * ============================================================================= */

void TaskMotion_Init(void)
{
    s_last_target_deg = -999.0f;
    s_torque_active   = false;

#if TASK_MOTION_USE_MOCK
    s_mock_pos_deg       = 0.0f;
    s_mock_target_deg    = 0.0f;
    s_mock_torque_on     = false;
    s_mock_last_tick_ms  = 0U;
#endif
}

#if TASK_MOTION_USE_MOCK

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state)
{
    if ((cmd == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));

    if (s_mock_last_tick_ms == 0U) {
        s_mock_last_tick_ms = tick_ms;
    }
    uint32_t dt_ms = tick_ms - s_mock_last_tick_ms;
    if (dt_ms > 500U) dt_ms = 500U;
    s_mock_last_tick_ms = tick_ms;

    s_mock_torque_on = cmd->torque_on;
    if (cmd->torque_on) {
        s_mock_target_deg = map_angle_to_fsus(cmd->target_angle_deg);
    }

    /* First-order tracker */
    float err       = s_mock_target_deg - s_mock_pos_deg;
    float max_delta = MOCK_SLEW_DEG_PER_SEC * (float)(int32_t)dt_ms / 1000.0f;
    if (max_delta < 0.1f) max_delta = 0.1f;

    float delta;
    if (err > max_delta)        delta =  max_delta;
    else if (err < -max_delta)  delta = -max_delta;
    else                        delta =  err;

    s_mock_pos_deg += delta;

    /* Simulated feedback */
    FsusFeedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.servo_id      = TASK_MOTION_SERVO_ID;
    fb.angle_deg     = s_mock_pos_deg;
    fb.voltage_mv    = 7400;
    fb.current_ma    = 0;
    fb.power_mw      = 0;
    fb.temp_raw      = 3000;
    fb.status        = 0;
    fb.circle_count  = 0;
    fb.timestamp_ms  = tick_ms;

    state->last_write_result = FSUS_PARSE_OK;
    state->last_read_result  = FSUS_PARSE_OK;
    state->feedback          = fb;
    state->feedback_valid    = true;
    state->servo_online      = true;
    state->is_moving         = (delta > 0.5f || delta < -0.5f);
    state->is_stalled        = false;
    state->is_overload       = false;
}

#else /* ============ Real-servo path (FSUS) ============ */

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state)
{
    if ((cmd == NULL) || (state == NULL)) {
        return;
    }
    memset(state, 0, sizeof(*state));

#if TASK_MOTION_SKIP_PING_FOR_BENCH
    /* Bench mode short-circuit: HX8 / UC01 not powered. Pretend the main
     * arm is online and silently swallow any motion command so the rest of
     * the FSM (arbiter, manipulator) is allowed to drive PA0/PA1 PTK
     * end-effectors. We do NOT touch the FSUS bus here. */
    (void)cmd;
    state->last_write_result = FSUS_PARSE_OK;
    state->last_read_result  = FSUS_PARSE_OK;
    state->feedback.servo_id     = TASK_MOTION_SERVO_ID;
    state->feedback.angle_deg    = 0.0f;
    state->feedback.timestamp_ms = tick_ms;
    state->feedback_valid    = true;
    state->servo_online      = true;
    state->is_moving         = false;
    state->is_stalled        = false;
    state->is_overload       = false;
    return;
#endif

    float target_deg = map_angle_to_fsus(cmd->target_angle_deg);

    /* --- Torque transition ---
     * Protocol-level meaning:
     *   false -> Stop(unlock), release holding torque
     *   true  -> send position command, servo holds/follows target internally */
    if (cmd->torque_on != s_torque_active) {
        if (cmd->torque_on) {
            /* Enable: send motion command to lock at target. */
            state->last_write_result = do_set_angle(target_deg,
                                                    cmd->velocity_deg_per_s,
                                                    cmd->t_acc_ms,
                                                    cmd->t_dec_ms,
                                                    cmd->power_mw);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_torque_active   = true;
                s_last_target_deg = target_deg;
            }
        } else {
            /* Disable: stop and unlock. */
            state->last_write_result = do_stop(FSUS_STOP_MODE_UNLOCK, 0U);
            s_torque_active   = false;
            s_last_target_deg = -999.0f;
        }
    } else if (cmd->torque_on) {
        /* --- Target change detection ---
         * Avoid spamming the bus with the same target every control tick.
         * The servo keeps running its internal closed loop after one command. */
        float diff = target_deg - s_last_target_deg;
        if (diff < 0.0f) diff = -diff;
        bool changed = (diff > 0.5f) || cmd->force_update;

        if (changed) {
            state->last_write_result = do_set_angle(target_deg,
                                                    cmd->velocity_deg_per_s,
                                                    cmd->t_acc_ms,
                                                    cmd->t_dec_ms,
                                                    cmd->power_mw);
            if (state->last_write_result == FSUS_PARSE_OK) {
                s_last_target_deg = target_deg;
            }
        } else {
            state->last_write_result = FSUS_PARSE_OK;
        }
    } else {
        state->last_write_result = FSUS_PARSE_OK;
    }

    /* --- Always read feedback ---
     * Command frames tell the servo what to do; monitor frames tell us what
     * actually happened inside the servo controller. */
    state->last_read_result = do_read_feedback(tick_ms, &state->feedback);
    if (state->last_read_result == FSUS_PARSE_OK) {
        state->feedback_valid = true;
        state->servo_online   = true;

        /* Motion classification from servo status byte. */
        uint8_t st = state->feedback.status;
        state->is_stalled  = ((st >> 2) & 0x01U) != 0U;  /* BIT2 = stall */
        state->is_overload = (st != 0U);                   /* any fault */
        state->is_moving   = !state->is_stalled;           /* rough */
    }
}

#endif /* TASK_MOTION_USE_MOCK */
