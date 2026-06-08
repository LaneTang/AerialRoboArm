/**
 * @file task_arbiter.h
 * @brief Pure-function dual-link arbitration (L4).
 *
 * The arbiter takes RC intent, Vision intent, and current servo state, and
 * decides on the single unified motion intent the system should pursue
 * this tick. It is intentionally a pure function:
 *   - no global state read/write
 *   - no DataHub touch
 *   - no RTOS calls
 *
 * This makes it trivially unit-testable on the PC and keeps arbitration
 * rules visible in one place.
 *
 * Priority ladder (high to low):
 *   1. Operator E-Stop via SD (latched)
 *   2. RC link loss (latched)
 *   3. Servo offline (not latched; bring-up owns recovery)
 *   4. Latched fault reset via SB, only after SD is released
 *   5. MANUAL mode (RC intent)
 *   6. AUTO mode with fresh/stale vision
 *   7. IDLE default
 */

#ifndef TASK_ARBITER_H
#define TASK_ARBITER_H

#include "ara_def.h"
#include "datahub.h"

/* ============================================================================
 * Output codes for observability (packed into ArbiterOutput.reason_code)
 * ========================================================================== */

typedef enum {
    ARB_REASON_BOOT                = 0,
    ARB_REASON_ESTOP_OPERATOR      = 1,
    ARB_REASON_ESTOP_RC_LOSS       = 2,
    ARB_REASON_MANUAL_RC           = 3,
    ARB_REASON_AUTO_VISION_FRESH   = 4,
    ARB_REASON_AUTO_VISION_STALE   = 5,
    ARB_REASON_IDLE_DEFAULT        = 6,
    ARB_REASON_SERVO_OFFLINE       = 7,
    ARB_REASON_FAULT_PENDING_RESET = 8
} ArbiterReason_t;

/* ============================================================================
 * Arbitration input / output
 * ========================================================================== */

typedef struct {
    const RcControlData_t  *rc;
    const VisionIntent_t   *vision;
    /** Opaque handle-style, read: is the servo currently reachable. */
    bool                    servo_online;
    uint32_t                tick_ms;
    AraSysMode_t            prev_mode;
    /** Vision freshness window in ms. Past this, AUTO falls back to park. */
    uint32_t                vision_stale_ms;
    /** Caller-owned latch state from previous tick. When true, arbiter
     *  refuses to leave ERROR until rc->sys_reset_pulse is observed. */
    bool                    fault_latched;
} ArbiterInput_t;

typedef struct {
    AraSysMode_t      mode;
    bool              estop;
    /** True when the arbiter wants torque applied to the servo. */
    bool              torque_request;
    /** Target joint angle in degrees. */
    int16_t           target_angle_deg;
    /** Target traversal speed, step/s. 0 means servo default. */
    uint16_t          target_speed;
    /** Acceleration register value (0..254). */
    uint8_t           target_acc;
    AraGripperCmd_t   gripper_cmd;
    uint8_t           roll_degree;
    uint8_t           gripper_angle;
    AraLedPattern_t   led_pattern;
    ArbiterReason_t   reason_code;
    /** Latch state caller should persist for next tick. Mirrors the
     *  two-step reset semantic: SD-active / RC-loss set this true; only
     *  rc->sys_reset_pulse can clear it (see fault_reset_consumed). */
    bool              fault_latched_next;
    /** True for exactly the tick that consumed sys_reset_pulse and left
     *  the latched ERROR state. Caller uses this to trigger side effects
     *  (e.g. reseed CH1 incremental accumulator) that arbiter cannot do
     *  itself while remaining a pure function. */
    bool              fault_reset_consumed;
} ArbiterOutput_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Default vision staleness window: a vision sample older than this
 *        is ignored in AUTO mode and the system parks.
 */
#define ARBITER_DEFAULT_VISION_STALE_MS   (200U)

/**
 * @brief Sentinel meaning "this output's target_angle_deg is intentionally
 *        unset; the consumer must keep its previous target". Currently
 *        emitted on the AUTO/vision-stale path where the arbiter cannot
 *        know what the joint should hold at, but does not want to imply
 *        a real 0-degree command. INT16_MIN is well outside any plausible
 *        joint range so a debug log of -32768 immediately identifies it.
 *        Consumers (manipulator FSM) detect this and fall back to their
 *        own last_target instead of forwarding the value to motion.
 */
#define ARBITER_TARGET_ANGLE_HOLD         (INT16_MIN)

/**
 * @brief Pure function. Consumes ArbiterInput, populates ArbiterOutput.
 *        Does not touch any globals.
 */
void TaskArbiter_Decide(const ArbiterInput_t *in, ArbiterOutput_t *out);

#endif /* TASK_ARBITER_H */
