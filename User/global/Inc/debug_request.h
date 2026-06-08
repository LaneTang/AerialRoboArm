/**
 * @file debug_request.h
 * @brief Single-slot Console -> ControlTask request channel.
 *
 * The DataHub is single-writer (ControlTask). To preserve that invariant,
 * Console-driven actions (mode switches, mock vision triggers, parameter
 * tweaks) MUST go through this side channel instead of writing DataHub
 * directly.
 *
 * Producer: any task (typically DebugTask via Console parser).
 * Consumer: ControlTask (via DebugRequest_TakePending at the top of each tick).
 *
 * Slot semantics:
 *   - Single slot. A new Post overwrites any pending request.
 *   - The slot is cleared on TakePending. Each request is therefore consumed
 *     at most once.
 *   - Atomicity is provided by taskENTER_CRITICAL(). Execution time is
 *     deterministic and below 1us on F103@72MHz.
 *
 * No queueing. The design assumes operator pace and that ControlTask polls
 * once per 20ms tick, so request loss only happens when the operator hits
 * two keys faster than 20ms - acceptable for a debug channel.
 */

#ifndef DEBUG_REQUEST_H
#define DEBUG_REQUEST_H

#include "ara_def.h"
#include "datahub.h"   /* AraSysMode_t for SwitchMode requests */

/* ============================================================================
 * Request types
 * ========================================================================== */

typedef enum {
    DBG_REQ_NONE = 0,

    /** Request a system mode change. arg1 carries an AraSysMode_t value. */
    DBG_REQ_SWITCH_MODE,

    /** Toggle E-Stop assertion. arg1 != 0 asserts, == 0 releases. */
    DBG_REQ_SET_ESTOP,

    /** Trigger a mock vision target. arg1 = target angle in degrees,
     *  arg2 = duration_ms. ControlTask forwards to TaskVision Mock injector. */
    DBG_REQ_MOCK_VISION_TARGET,

    /** Clear mock vision target. */
    DBG_REQ_MOCK_VISION_CLEAR,

    /** Force-mode bring-up: bypass Arbiter/Manipulator and drive
     *  TaskMotion_Update directly with a fixed target angle.
     *
     *  arg1 (int16_t cast)  = target angle in degrees
     *  arg2:
     *      >= 0  enable force mode and lock to arg1
     *       < 0  disable force mode (servo torque is released next tick)
     *
     *  While force mode is active, Arbiter and Manipulator are NOT
     *  consulted. This is intentional: force mode is a bring-up tool
     *  for verifying the USART2 / ST3215 / mechanical loop without
     *  any policy layer in between.
     */
    DBG_REQ_FORCE_GOTO_ANGLE,

    /** Calibrate current servo position as logical zero offset. */
    DBG_REQ_CALIBRATE_ZERO,

    /** Clear ControlTask FAULT phase, retry SERVO_PING. */
    DBG_REQ_CLEAR_FAULT,

    /** Force-mode for PTK end-effector servos (PA0 gripper / PA1 roll).
     *  Bypasses arbiter for the named PTK channel only; HX8 main arm
     *  is unaffected.
     *
     *  arg1 = channel (0 = PA0 gripper, 1 = PA1 roll)
     *  arg2:
     *      0..180 lock the named PTK channel to that degree
     *        < 0  release (return to RC arbiter control)
     */
    DBG_REQ_FORCE_PTK_ANGLE,

    /** Last enum sentinel for bounds checking. */
    DBG_REQ_TYPE_COUNT
} DebugReqType_t;

/* ============================================================================
 * Request payload
 * ========================================================================== */

typedef struct {
    DebugReqType_t type;
    int32_t        arg1;
    int32_t        arg2;
    uint32_t       posted_tick_ms;
} DebugRequest_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief  Initialise the request slot to empty. Call before scheduler start.
 */
void DebugRequest_Init(void);

/**
 * @brief  Post a new request. Overwrites any pending unread request.
 * @param  type   Request type (DBG_REQ_*).
 * @param  arg1   Type-specific argument.
 * @param  arg2   Type-specific argument.
 * @note   Safe to call from any task. NOT safe to call from ISR (uses
 *         taskENTER_CRITICAL, not portENTER_CRITICAL_FROM_ISR).
 */
void DebugRequest_Post(DebugReqType_t type, int32_t arg1, int32_t arg2);

/**
 * @brief  Atomically take and clear the pending request.
 * @param  out  Output buffer. Populated only when return is true.
 * @return true  when a request was waiting and copied into out.
 * @return false when no request was pending. out is left untouched.
 */
bool DebugRequest_TakePending(DebugRequest_t *out);

#endif /* DEBUG_REQUEST_H */
