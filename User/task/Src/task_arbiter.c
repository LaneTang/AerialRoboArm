/**
 * @file task_arbiter.c
 * @brief Pure-function dual-link arbitration implementation.
 *
 * Two-step reset semantics:
 *   - Any fresh fault (SD active, RC loss) emits ERROR and asks caller to
 *     keep fault_latched_next=true for the next tick.
 *   - While fault_latched is true, the only way out is rc->sys_reset_pulse,
 *     which must arrive AFTER the underlying fault has cleared (SD released,
 *     RC link up). On the consuming tick, mode goes to IDLE (forced — SA is
 *     ignored on that tick) and fault_reset_consumed is set so caller can
 *     trigger a reseed of any cross-tick accumulators.
 *   - servo_online is intentionally NOT latched — bring-up phase machine
 *     owns that recovery path.
 */

#include "task_arbiter.h"

void TaskArbiter_Decide(const ArbiterInput_t *in, ArbiterOutput_t *out)
{
    if ((in == NULL) || (out == NULL)) {
        return;
    }
    if ((in->rc == NULL) || (in->vision == NULL)) {
        return;
    }

    /* Safe defaults */
    out->mode             = ARA_MODE_IDLE;
    out->estop            = false;
    out->torque_request   = false;
    out->target_angle_deg = 0;
    out->target_speed     = 0U;
    out->target_acc       = 50U;
    out->gripper_cmd      = GRIPPER_CMD_STOP;
    out->roll_degree      = 90U;
    out->gripper_angle    = 90U;
    out->led_pattern      = LED_PATTERN_IDLE_SLOW_BLINK;
    out->reason_code      = ARB_REASON_IDLE_DEFAULT;
    out->fault_latched_next   = in->fault_latched;
    out->fault_reset_consumed = false;

    const RcControlData_t *rc = in->rc;
    const VisionIntent_t  *vs = in->vision;

    const uint32_t vision_stale_ms = (in->vision_stale_ms != 0U)
                                         ? in->vision_stale_ms
                                         : ARBITER_DEFAULT_VISION_STALE_MS;

    const bool sd_active = rc->is_link_up && (rc->estop_state == ESTOP_ACTIVE);
    const bool rc_lost   = !rc->is_link_up;

    /* ----- Rule 1: operator E-Stop absolute (latches) ----- */
    if (sd_active) {
        out->mode               = ARA_MODE_ERROR;
        out->estop              = true;
        out->torque_request     = false;
        out->led_pattern        = LED_PATTERN_ERROR_SOS;
        out->reason_code        = ARB_REASON_ESTOP_OPERATOR;
        out->fault_latched_next = true;
        return;
    }

    /* ----- Rule 2: RC link loss (latches) ----- */
    if (rc_lost) {
        out->mode               = ARA_MODE_ERROR;
        out->estop              = true;
        out->torque_request     = false;
        out->led_pattern        = LED_PATTERN_ERROR_SOS;
        out->reason_code        = ARB_REASON_ESTOP_RC_LOSS;
        out->fault_latched_next = true;
        return;
    }

    /* ----- Rule 3: servo offline (does NOT latch) ----- */
    if (!in->servo_online) {
        out->mode           = ARA_MODE_ERROR;
        out->estop          = true;
        out->torque_request = false;
        out->led_pattern    = LED_PATTERN_ERROR_SOS;
        out->reason_code    = ARB_REASON_SERVO_OFFLINE;
        /* fault_latched_next mirrors current fault_latched — bring-up phase
         * owns servo recovery, we don't escalate this into a latch. */
        return;
    }

    /* ----- Rule 4: latched but underlying fault cleared ----- */
    if (in->fault_latched) {
        if (rc->sys_reset_pulse && (rc->estop_state == ESTOP_RELEASED)) {
            /* Consume the pulse: leave latched ERROR, forced IDLE this tick.
             * Caller will react to fault_reset_consumed (e.g. reseed
             * incremental accumulator). Next tick fault_latched=false and
             * normal SA-driven arbitration resumes. */
            out->mode                 = ARA_MODE_IDLE;
            out->estop                = false;
            out->torque_request       = false;
            out->led_pattern          = LED_PATTERN_IDLE_SLOW_BLINK;
            out->reason_code          = ARB_REASON_IDLE_DEFAULT;
            out->fault_latched_next   = false;
            out->fault_reset_consumed = true;
            return;
        }

        /* Latched, no reset yet — hold ERROR with distinct LED. */
        out->mode               = ARA_MODE_ERROR;
        out->estop              = true;
        out->torque_request     = false;
        out->led_pattern        = LED_PATTERN_FAULT_PENDING_RESET;
        out->reason_code        = ARB_REASON_FAULT_PENDING_RESET;
        out->fault_latched_next = true;
        return;
    }

    /* ----- Rule 5: MANUAL mode (RC up, operator present) ----- */
    if (rc->req_mode == ARA_MODE_MANUAL) {
        out->mode             = ARA_MODE_MANUAL;
        out->torque_request   = true;
        out->target_angle_deg = rc->incremental_angle_deg;
        out->target_speed     = 0U;
        out->target_acc       = 50U;
        out->gripper_cmd      = rc->gripper_cmd;
        out->roll_degree      = rc->roll_degree;
        out->gripper_angle    = rc->gripper_angle;
        out->led_pattern      = LED_PATTERN_MANUAL_HEARTBEAT;
        out->reason_code      = ARB_REASON_MANUAL_RC;
        return;
    }

    /* ----- Rule 6: AUTO mode ----- */
    if (rc->req_mode == ARA_MODE_AUTO) {
        const bool vision_fresh = vs->target_present &&
            ((uint32_t)(in->tick_ms - vs->last_update_tick_ms) < vision_stale_ms);

        if (vision_fresh) {
            out->mode             = ARA_MODE_AUTO;
            out->torque_request   = true;
            out->target_angle_deg = vs->target_angle_deg;
            out->target_speed     = (vs->target_speed != 0U) ? vs->target_speed : 0U;
            out->target_acc       = 50U;
            out->gripper_cmd      = GRIPPER_CMD_STOP;
            out->roll_degree      = 90U;
            out->gripper_angle    = 90U;
            out->led_pattern      = LED_PATTERN_AUTO_SOLID;
            out->reason_code      = ARB_REASON_AUTO_VISION_FRESH;
            return;
        }

        /* Vision stale: park-hold at current position. */
        out->mode             = ARA_MODE_AUTO;
        out->torque_request   = true;
        out->target_angle_deg = ARBITER_TARGET_ANGLE_HOLD;
        out->target_speed     = 0U;
        out->target_acc       = 50U;
        out->gripper_cmd      = GRIPPER_CMD_STOP;
        out->led_pattern      = LED_PATTERN_AUTO_SOLID;
        out->reason_code      = ARB_REASON_AUTO_VISION_STALE;
        return;
    }

    /* ----- Rule 7: default IDLE ----- */
    out->mode           = ARA_MODE_IDLE;
    out->torque_request = false;
    out->led_pattern    = LED_PATTERN_IDLE_SLOW_BLINK;
    out->reason_code    = ARB_REASON_IDLE_DEFAULT;
}
