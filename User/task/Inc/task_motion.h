/**
 * @file task_motion.h
 * @brief FSUS servo proxy (L4).
 *
 * Motion is a thin agent over the FSUS bus:
 *   - Encode target / read feedback via drv_fsus.
 *   - Issue transactions via bsp_uart FSUS API (blocking TX + interrupt RX).
 *
 * MOCK mode simulates a first-order tracker producing plausible feedback.
 * Set TASK_MOTION_USE_MOCK=0 when the HX8-U26H-M is physically connected.
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "drv_fsus.h"

/* ============================================================================
 * Compile-time configuration
 * ============================================================================= */

#ifndef TASK_MOTION_USE_MOCK
#define TASK_MOTION_USE_MOCK            (0)
#endif

/**
 * @brief When set, the bring-up FSM skips the FSUS ping and pretends the
 *        main-arm servo is online. Use this **only** when bench-testing the
 *        PA0/PA1 PTK end-effector servos without the HX8 / UC01 powered up.
 *        With this on, do NOT command the main arm — fault detection is
 *        disabled. Turn back to 0 before any flight bring-up.
 */
#ifndef TASK_MOTION_SKIP_PING_FOR_BENCH
#define TASK_MOTION_SKIP_PING_FOR_BENCH (1)
#endif

#define TASK_MOTION_SERVO_ID            (0U)    /**< Default FSUS servo ID. */

#define TASK_MOTION_ANGLE_MIN_DEG       (-180)  /**< FSUS native: signed, [-180,+180]. */
#define TASK_MOTION_ANGLE_MAX_DEG       (180)

#define TASK_MOTION_DEFAULT_VELOCITY    (750.0f)/**< deg/s, FSUS protocol max. */
#define TASK_MOTION_DEFAULT_T_ACC_MS    (20U)   /**< Acceleration time, protocol min. */
#define TASK_MOTION_DEFAULT_T_DEC_MS    (20U)   /**< Deceleration time, protocol min. */
#define TASK_MOTION_DEFAULT_POWER_MW    (0U)    /**< 0 = servo auto-calc. */

/* ============================================================================
 * Command / state types
 * ============================================================================= */

typedef struct {
    bool     torque_on;              /**< false = send Stop(unlock). */
    float    target_angle_deg;       /**< Commanded angle, -180..+180. */
    float    velocity_deg_per_s;     /**< Traverse speed. */
    uint16_t t_acc_ms;               /**< Accel time, >= 20. */
    uint16_t t_dec_ms;               /**< Decel time, >= 20. */
    uint16_t power_mw;               /**< Execution power, 0 = servo default. */
    bool     force_update;           /**< Bypass change detection. */
} MotionCmd_t;

typedef struct {
    /* --- This-tick IO outcome --- */
    FsusParseResult_t   last_write_result;
    FsusParseResult_t   last_read_result;

    /* --- Latest feedback --- */
    FsusFeedback_t      feedback;
    bool                feedback_valid;

    /* --- Motion status (simplified) --- */
    bool                is_moving;      /**< Estimated from angle delta. */
    bool                is_stalled;     /**< BIT2 in servo status byte. */
    bool                is_overload;    /**< Any fault bit set. */

    /* --- Link health --- */
    bool                servo_online;
} MotionState_t;

/* ============================================================================
 * API
 * ============================================================================= */

void TaskMotion_Init(void);

void TaskMotion_Update(const MotionCmd_t *cmd,
                       uint32_t           tick_ms,
                       MotionState_t     *state);

#endif /* TASK_MOTION_H */
