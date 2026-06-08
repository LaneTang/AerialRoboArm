/**
 * @file task_manipulator.c
 * @brief Manipulator FSM implementation for demo_v7.
 *
 * Stripped-down state machine. Heavy auto-flow logic from demo_v6 is gone;
 * AUTO is now driven directly by Arbiter (which itself is driven by Vision
 * Mock). This file just maps Arbiter intent to MotionCmd and tracks the
 * high-level demo state for observability.
 */

#include "task_manipulator.h"

#include <string.h>

static TaskManipulator_Context_t s_ctx;

static void enter_state(ManipulatorState_t s, uint32_t tick_ms)
{
    s_ctx.current_state  = s;
    s_ctx.state_enter_ms = tick_ms;
}

void TaskManipulator_Init(void)
{
    memset(&s_ctx, 0, sizeof(s_ctx));
    (void)ModActuator_Init(&s_ctx.actuator);
    s_ctx.current_state         = MANIP_STATE_BOOT;
    s_ctx.last_target_angle_deg = 0.0f;
}

ManipulatorState_t TaskManipulator_GetState(void)
{
    return s_ctx.current_state;
}

void TaskManipulator_Update(const ArbiterOutput_t *arb,
                            const MotionState_t   *mot_state,
                            uint32_t               tick_ms,
                            MotionCmd_t           *out_cmd)
{
    if ((arb == NULL) || (mot_state == NULL) || (out_cmd == NULL)) {
        return;
    }

    /* Default safe command */
    out_cmd->torque_on          = false;
    out_cmd->target_angle_deg   = s_ctx.last_target_angle_deg;
    out_cmd->velocity_deg_per_s = 0.0f;
    out_cmd->t_acc_ms           = 0U;
    out_cmd->t_dec_ms           = 0U;
    out_cmd->power_mw           = 0U;
    out_cmd->force_update       = false;

    /* Error / E-Stop short-circuits everything. */
    if (arb->estop || (arb->mode == ARA_MODE_ERROR)) {
        if (s_ctx.current_state != MANIP_STATE_ERROR_SAFE) {
            enter_state(MANIP_STATE_ERROR_SAFE, tick_ms);
        }
        out_cmd->torque_on = false;
        return;
    }

    /* Recovery from ERROR_SAFE: arbiter went non-error, transition to IDLE. */
    if (s_ctx.current_state == MANIP_STATE_ERROR_SAFE) {
        enter_state(MANIP_STATE_IDLE, tick_ms);
    }

    /* Boot waits for servo online. Arbiter rejects offline -> ERROR_SAFE. So
     * when we land here with non-error mode, servo is online. */
    if (s_ctx.current_state == MANIP_STATE_BOOT) {
        enter_state(MANIP_STATE_IDLE, tick_ms);
    }

    (void)ModActuator_SetRoll(&s_ctx.actuator, arb->roll_degree);
    (void)ModActuator_SetGripperAngle(&s_ctx.actuator, arb->gripper_angle);

    switch (arb->mode) {
    case ARA_MODE_MANUAL:
        if (s_ctx.current_state != MANIP_STATE_MANUAL) {
            enter_state(MANIP_STATE_MANUAL, tick_ms);
        }
        out_cmd->torque_on = arb->torque_request;
        if (arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD) {
            out_cmd->target_angle_deg   = (float)arb->target_angle_deg;
            s_ctx.last_target_angle_deg = (float)arb->target_angle_deg;
        } else {
            out_cmd->target_angle_deg   = s_ctx.last_target_angle_deg;
        }
        out_cmd->velocity_deg_per_s = TASK_MOTION_DEFAULT_VELOCITY;
        out_cmd->t_acc_ms           = TASK_MOTION_DEFAULT_T_ACC_MS;
        out_cmd->t_dec_ms           = TASK_MOTION_DEFAULT_T_DEC_MS;
        out_cmd->power_mw           = TASK_MOTION_DEFAULT_POWER_MW;
        break;

    case ARA_MODE_AUTO:
        out_cmd->torque_on          = arb->torque_request;
        out_cmd->velocity_deg_per_s = TASK_MOTION_DEFAULT_VELOCITY;
        out_cmd->t_acc_ms           = TASK_MOTION_DEFAULT_T_ACC_MS;
        out_cmd->t_dec_ms           = TASK_MOTION_DEFAULT_T_DEC_MS;
        out_cmd->power_mw           = TASK_MOTION_DEFAULT_POWER_MW;

        if ((arb->reason_code == ARB_REASON_AUTO_VISION_FRESH) &&
            (arb->target_angle_deg != ARBITER_TARGET_ANGLE_HOLD)) {
            out_cmd->target_angle_deg   = (float)arb->target_angle_deg;
            s_ctx.last_target_angle_deg = (float)arb->target_angle_deg;
            if (!mot_state->is_moving) {
                if (s_ctx.current_state != MANIP_STATE_AUTO_HOLD) {
                    enter_state(MANIP_STATE_AUTO_HOLD, tick_ms);
                }
            } else {
                if (s_ctx.current_state != MANIP_STATE_AUTO_GO) {
                    enter_state(MANIP_STATE_AUTO_GO, tick_ms);
                }
            }
        } else {
            out_cmd->target_angle_deg = s_ctx.last_target_angle_deg;
            if (s_ctx.current_state != MANIP_STATE_AUTO_HOLD) {
                enter_state(MANIP_STATE_AUTO_HOLD, tick_ms);
            }
        }
        break;

    case ARA_MODE_INIT:
    case ARA_MODE_IDLE:
    default:
        if (s_ctx.current_state != MANIP_STATE_IDLE) {
            enter_state(MANIP_STATE_IDLE, tick_ms);
        }
        out_cmd->torque_on = false;
        break;
    }
}
