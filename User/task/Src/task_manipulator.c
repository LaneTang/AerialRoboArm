/**
 * @file task_manipulator.c
 * @brief Demo-oriented manipulator brain and state machine runnable (L4).
 * @note  Strictly C99. No floating-point math. No RTOS dependencies.
 *        This revision aligns the manipulator with the current demo target:
 *        - Startup park to +15 deg after Motion finishes its own calibration path.
 *        - MANUAL mode directly controls all actuators from RC semantics.
 *        - AUTO mode executes a deterministic fixed demo flow:
 *            +120 deg -> hold within 3 deg for 2s -> gripper close -> wait 1s
 *            -> retract to +30 deg -> done.
 */

#include "task_manipulator.h"
#include "mod_actuator.h"
#include <stddef.h>

/* ============================================================================
 * 1. Demo Configuration Constants
 * ========================================================================== */

/**
 * @brief Calibrated raw-angle value corresponding to the joint mechanical 0 deg.
 * @note  Replace this with the measured AS5600 raw value from bench calibration.
 */
#define MANIP_DEMO_JOINT_ZERO_RAW              (850U)

/**
 * @brief Direction of positive joint rotation in raw-angle domain.
 * @note  Use +1 if positive mechanical angle increases raw counts,
 *        or -1 if positive mechanical angle decreases raw counts.
 */
#define MANIP_DEMO_POSITIVE_DIR                (1)

/**
 * @brief Demo startup / idle park angle in mechanical degree.
 */
#define MANIP_DEMO_PARK_DEG                    (15U)

/**
 * @brief Demo AUTO pick angle in mechanical degree.
 */
#define MANIP_DEMO_PICK_DEG                    (120U)

/**
 * @brief Demo AUTO retract angle in mechanical degree.
 */
#define MANIP_DEMO_RETRACT_DEG                 (30U)

/**
 * @brief Joint reach tolerance used by AUTO hold verification.
 */
#define MANIP_DEMO_REACH_TOL_DEG               (3U)

/**
 * @brief Startup park settle duration in 50Hz ticks.
 * @note  25 ticks * 20ms = 500ms.
 */
#define MANIP_STARTUP_PARK_SETTLE_TICKS        (25U)

/**
 * @brief AUTO hold verification duration in 50Hz ticks.
 * @note  100 ticks * 20ms = 2s.
 */
#define MANIP_AUTO_HOLD_VERIFY_TICKS           (100U)

/**
 * @brief AUTO gripper close wait duration in 50Hz ticks.
 * @note  50 ticks * 20ms = 1s.
 */
#define MANIP_AUTO_GRAB_WAIT_TICKS             (50U)

/**
 * @brief AUTO retract settle duration in 50Hz ticks.
 * @note  25 ticks * 20ms = 500ms.
 */
#define MANIP_AUTO_RETRACT_SETTLE_TICKS        (25U)

/**
 * @brief Manual-mode joint increment step in mechanical degree.
 */
#define MANIP_MANUAL_EXT_STEP_DEG              (4U)

/**
 * @brief Manual-mode gripper step in percent.
 */
#define MANIP_MANUAL_GRIPPER_STEP_PERCENT      (8U)

/**
 * @brief Roll servo safe home degree used in startup / idle / error-safe.
 * @note  Adjust this according to the real mechanical neutral on bench.
 */
#define MANIP_DEMO_ROLL_HOME_DEG               (0U)

/**
 * @brief Roll servo target degree used in AUTO demo.
 */
#define MANIP_DEMO_ROLL_AUTO_TARGET_DEG        (90U)

/**
 * @brief Gripper open percentage used in startup / idle / error-safe.
 */
#define MANIP_DEMO_GRIPPER_HOME_PERCENT        (0U)

/**
 * @brief Gripper pre-open percentage used in AUTO pick / hold stages.
 */
#define MANIP_DEMO_GRIPPER_PREOPEN_PERCENT     (45U)

/**
 * @brief Gripper fully-closed percentage used in AUTO grab / retract stages.
 */
#define MANIP_DEMO_GRIPPER_CLOSE_PERCENT       (100U)

/**
 * @brief Manual-mode minimum joint angle in mechanical degree.
 */
#define MANIP_MANUAL_MIN_DEG                   (0U)

/**
 * @brief Manual-mode maximum joint angle in mechanical degree.
 */
#define MANIP_MANUAL_MAX_DEG                   (130U)

/* ============================================================================
 * 2. Static Context & Owned Dependencies
 * ========================================================================== */

/**
 * @brief Singleton manipulator runtime context.
 */
static TaskManipulator_Context_t s_manip_ctx;

/**
 * @brief Internal 50Hz monotonic tick counter.
 */
static uint32_t s_task_tick_counter = 0U;

/**
 * @brief End-effector actuator module instance owned by this runnable.
 */
static ModActuator_Context_t s_actuator_ctx;

/* ============================================================================
 * 3. Private Helper Functions
 * ========================================================================== */

/**
 * @brief  Compute absolute value of a signed 32-bit integer.
 * @param  value Input value.
 * @return Absolute value.
 */
static int32_t Manip_Abs32(int32_t value)
{
    return (value >= 0) ? value : (-value);
}

/**
 * @brief  Wrap an arbitrary signed angle into the AS5600 raw domain [0, 4095].
 * @param  angle Input angle in extended integer domain.
 * @return Wrapped raw-angle value.
 */
static uint16_t Manip_WrapRawAngle(int32_t angle)
{
    while (angle < 0) {
        angle += 4096;
    }

    while (angle >= 4096) {
        angle -= 4096;
    }

    return (uint16_t)angle;
}

/**
 * @brief  Convert a mechanical degree value into raw encoder counts.
 * @param  degree Mechanical angle in degree.
 * @return Equivalent raw-angle offset.
 */
static uint16_t Manip_DegToRaw(uint16_t degree)
{
    return (uint16_t)(((uint32_t)degree * 4096U) / 360U);
}

/**
 * @brief  Convert a mechanical demo angle into an absolute raw-angle target.
 * @param  degree Mechanical angle in degree.
 * @return Absolute AS5600 raw target.
 */
static uint16_t Manip_DemoAngleToRaw(uint16_t degree)
{
    int32_t target;

    target = (int32_t)MANIP_DEMO_JOINT_ZERO_RAW +
             ((int32_t)MANIP_DEMO_POSITIVE_DIR * (int32_t)Manip_DegToRaw(degree));

    return Manip_WrapRawAngle(target);
}

/**
 * @brief  Convert an absolute raw angle into demo mechanical degree.
 * @param  raw_angle Absolute AS5600 raw angle.
 * @return Mechanical degree clamped into the manual demo range.
 * @note   This conversion assumes the demo stroke does not cross the wrap point.
 */
static uint16_t Manip_DemoRawToDegree(uint16_t raw_angle)
{
    int32_t delta_raw;
    int32_t mech_raw;
    int32_t degree;

    delta_raw = (int32_t)raw_angle - (int32_t)MANIP_DEMO_JOINT_ZERO_RAW;
    if (delta_raw > 2048) {
        delta_raw -= 4096;
    } else if (delta_raw < -2048) {
        delta_raw += 4096;
    }

    mech_raw = delta_raw * (int32_t)MANIP_DEMO_POSITIVE_DIR;
    if (mech_raw < 0) {
        mech_raw = 0;
    }

    degree = (mech_raw * 360) / 4096;
    if (degree < (int32_t)MANIP_MANUAL_MIN_DEG) {
        degree = (int32_t)MANIP_MANUAL_MIN_DEG;
    } else if (degree > (int32_t)MANIP_MANUAL_MAX_DEG) {
        degree = (int32_t)MANIP_MANUAL_MAX_DEG;
    }

    return (uint16_t)degree;
}

/**
 * @brief  Compute the shortest signed raw-angle error between target and actual.
 * @param  target Target raw angle.
 * @param  actual Actual raw angle.
 * @return Signed shortest-path error in raw counts.
 */
static int32_t Manip_CalcShortestPathError(uint16_t target, uint16_t actual)
{
    int32_t error = (int32_t)target - (int32_t)actual;

    if (error > 2048) {
        error -= 4096;
    } else if (error < -2048) {
        error += 4096;
    }

    return error;
}

/**
 * @brief  Clamp an unsigned 8-bit value into an inclusive range.
 * @param  value Input value.
 * @param  min_value Minimum allowed value.
 * @param  max_value Maximum allowed value.
 * @return Clamped value.
 */
static uint8_t Manip_ClampU8(uint8_t value, uint8_t min_value, uint8_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief  Clamp an unsigned 16-bit value into an inclusive range.
 * @param  value Input value.
 * @param  min_value Minimum allowed value.
 * @param  max_value Maximum allowed value.
 * @return Clamped value.
 */
static uint16_t Manip_ClampU16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief  Map RC aux knob permille to absolute roll servo degree.
 * @param  aux_permille RC aux knob value in 0~1000 permille.
 * @return Roll servo command in 0~180 degree.
 */
static uint8_t Manip_MapAuxToRollDegree(uint16_t aux_permille)
{
    uint32_t degree;

    if (aux_permille > 1000U) {
        aux_permille = 1000U;
    }

    degree = ((uint32_t)aux_permille * 180U) / 1000U;
    return (uint8_t)degree;
}

/**
 * @brief  Enter a new manipulator state and refresh the state's entry tick.
 * @param  new_state Target state.
 */
static void Manip_SetState(ManipulatorState_t new_state)
{
    s_manip_ctx.current_state = new_state;
    s_manip_ctx.state_enter_tick = s_task_tick_counter;
}

/**
 * @brief  Reset MANUAL-mode cached targets to safe demo defaults.
 */
static void Manip_ResetManualTargets(void)
{
    s_manip_ctx.manual_target_degree = MANIP_DEMO_PARK_DEG;
    s_manip_ctx.manual_gripper_percent = MANIP_DEMO_GRIPPER_HOME_PERCENT;
    s_manip_ctx.manual_roll_degree = MANIP_DEMO_ROLL_HOME_DEG;
}

/**
 * @brief  Reset the AUTO start edge gate.
 * @note   After reset / power-up, AUTO will not start until a non-AUTO request
 *         has been observed at least once. This prevents immediate auto-run when
 *         the RC mode switch is already sitting in AUTO before demo entry.
 */
static void Manip_ResetAutoStartGate(void)
{
    s_manip_ctx.auto_start_armed = false;
}

/**
 * @brief  Seed MANUAL-mode joint target from the latest measured joint angle.
 * @param  p_motion_state Pointer to motion feedback.
 */
static void Manip_SeedManualJointTarget(const DataHub_State_t *p_motion_state)
{
    if (p_motion_state != NULL) {
        s_manip_ctx.manual_target_degree =
                Manip_DemoRawToDegree(p_motion_state->current_foc_angle);
    }
}

/**
 * @brief  Apply a posture command to all three actuators.
 * @param  p_out_cmd Pointer to BLDC downlink command.
 * @param  joint_target_raw BLDC target in raw encoder counts.
 * @param  gripper_percent Gripper target in 0~100%.
 * @param  roll_degree Roll servo target in 0~180 degree.
 */
static void Manip_CommandPose(DataHub_Cmd_t *p_out_cmd,
                              uint16_t joint_target_raw,
                              uint8_t gripper_percent,
                              uint8_t roll_degree)
{
    if (p_out_cmd == NULL) {
        return;
    }

    p_out_cmd->target_foc_angle = joint_target_raw;

    (void)ModActuator_SetGripper(&s_actuator_ctx, gripper_percent);
    (void)ModActuator_SetRoll(&s_actuator_ctx, roll_degree);
}

/**
 * @brief  Check whether the joint has reached the requested raw-angle target.
 * @param  p_motion_state Pointer to motion feedback.
 * @param  target_raw Target raw angle.
 * @param  tolerance_raw Allowed absolute error in raw counts.
 * @return true if target is considered reached, otherwise false.
 */
static bool Manip_IsJointReached(const DataHub_State_t *p_motion_state,
                                 uint16_t target_raw,
                                 uint16_t tolerance_raw)
{
    int32_t error;

    if (p_motion_state == NULL) {
        return false;
    }

    if (p_motion_state->foc_status != ARA_OK) {
        return false;
    }

    error = Manip_CalcShortestPathError(target_raw, p_motion_state->current_foc_angle);
    return (Manip_Abs32(error) <= (int32_t)tolerance_raw);
}

/**
 * @brief  Execute MANUAL-mode direct control.
 * @param  p_rc_intent Pointer to RC semantic intent.
 * @param  p_out_cmd Pointer to motion downlink command.
 */
static void Manip_RunManual(const RcControlData_t *p_rc_intent,
                            DataHub_Cmd_t *p_out_cmd)
{
    uint16_t target_raw;

    if ((p_rc_intent == NULL) || (p_out_cmd == NULL)) {
        return;
    }

    if (p_rc_intent->arm_cmd == ARM_CMD_EXTEND) {
        s_manip_ctx.manual_target_degree =
                (uint16_t)(s_manip_ctx.manual_target_degree + MANIP_MANUAL_EXT_STEP_DEG);
    } else if (p_rc_intent->arm_cmd == ARM_CMD_RETRACT) {
        if (s_manip_ctx.manual_target_degree > MANIP_MANUAL_EXT_STEP_DEG) {
            s_manip_ctx.manual_target_degree =
                    (uint16_t)(s_manip_ctx.manual_target_degree - MANIP_MANUAL_EXT_STEP_DEG);
        } else {
            s_manip_ctx.manual_target_degree = 0U;
        }
    } else {
        /* HOLD: preserve previous target. */
    }

    s_manip_ctx.manual_target_degree =
            Manip_ClampU16(s_manip_ctx.manual_target_degree,
                           MANIP_MANUAL_MIN_DEG,
                           MANIP_MANUAL_MAX_DEG);

    if (p_rc_intent->gripper_cmd == GRIPPER_CMD_OPEN) {
        if (s_manip_ctx.manual_gripper_percent > MANIP_MANUAL_GRIPPER_STEP_PERCENT) {
            s_manip_ctx.manual_gripper_percent =
                    (uint8_t)(s_manip_ctx.manual_gripper_percent - MANIP_MANUAL_GRIPPER_STEP_PERCENT);
        } else {
            s_manip_ctx.manual_gripper_percent = 0U;
        }
    } else if (p_rc_intent->gripper_cmd == GRIPPER_CMD_CLOSE) {
        uint16_t next_percent = (uint16_t)s_manip_ctx.manual_gripper_percent +
                                (uint16_t)MANIP_MANUAL_GRIPPER_STEP_PERCENT;
        if (next_percent > 100U) {
            next_percent = 100U;
        }
        s_manip_ctx.manual_gripper_percent = (uint8_t)next_percent;
    } else {
        /* STOP: preserve previous gripper command. */
    }

    s_manip_ctx.manual_roll_degree = Manip_MapAuxToRollDegree(p_rc_intent->aux_knob_val);
    s_manip_ctx.manual_roll_degree =
            Manip_ClampU8(s_manip_ctx.manual_roll_degree, 0U, 180U);

    target_raw = Manip_DemoAngleToRaw(s_manip_ctx.manual_target_degree);

    Manip_CommandPose(p_out_cmd,
                      target_raw,
                      s_manip_ctx.manual_gripper_percent,
                      s_manip_ctx.manual_roll_degree);
}

/* ============================================================================
 * 4. Public API
 * ========================================================================== */

void TaskManipulator_Init(void)
{
    s_task_tick_counter = 0U;

    s_manip_ctx.current_state = MANIP_STATE_STARTUP_PARK;
    s_manip_ctx.state_enter_tick = 0U;

    Manip_ResetManualTargets();
    Manip_ResetAutoStartGate();

    (void)ModActuator_Init(&s_actuator_ctx);
}

void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const DataHub_State_t *p_motion_state,
                            DataHub_Cmd_t *p_out_cmd)
{
    uint16_t park_target_raw;
    uint16_t pick_target_raw;
    uint16_t retract_target_raw;
    uint16_t reach_tol_raw;

    if ((p_rc_intent == NULL) || (p_motion_state == NULL) || (p_out_cmd == NULL)) {
        return;
    }

    s_task_tick_counter++;

    park_target_raw = Manip_DemoAngleToRaw(MANIP_DEMO_PARK_DEG);
    pick_target_raw = Manip_DemoAngleToRaw(MANIP_DEMO_PICK_DEG);
    retract_target_raw = Manip_DemoAngleToRaw(MANIP_DEMO_RETRACT_DEG);
    reach_tol_raw = Manip_DegToRaw(MANIP_DEMO_REACH_TOL_DEG);

    /* ---------------- Safe default output ---------------- */
    p_out_cmd->sys_mode = ARA_MODE_MANUAL;
    p_out_cmd->emergency_stop = false;
    p_out_cmd->target_foc_angle = park_target_raw;

    /* ------------------------------------------------------------------------
     * STAGE 1: Reset / safety / operator arbitration
     * --------------------------------------------------------------------- */

    if (p_rc_intent->sys_reset_pulse) {
        Manip_ResetManualTargets();
        Manip_ResetAutoStartGate();
        Manip_SetState(MANIP_STATE_STARTUP_PARK);

        p_out_cmd->sys_mode = ARA_MODE_INIT;
        p_out_cmd->emergency_stop = false;
        p_out_cmd->target_foc_angle = park_target_raw;

        (void)ModActuator_SetGripper(&s_actuator_ctx, MANIP_DEMO_GRIPPER_HOME_PERCENT);
        (void)ModActuator_SetRoll(&s_actuator_ctx, MANIP_DEMO_ROLL_HOME_DEG);
        return;
    }

    if ((p_rc_intent->estop_state == ESTOP_ACTIVE) || (!p_rc_intent->is_link_up)) {
        if (s_manip_ctx.current_state != MANIP_STATE_ERROR_SAFE) {
            Manip_SetState(MANIP_STATE_ERROR_SAFE);
        }
    } else if (s_manip_ctx.current_state == MANIP_STATE_ERROR_SAFE) {
        /* Stay latched until an explicit reset pulse arrives. */
    } else {
        /* AUTO start gate:
         * AUTO can only start after the system has first observed a non-AUTO mode.
         * This prevents immediate auto-run when the RC mode switch is already in
         * AUTO before entering the demo.
         */
        if (p_rc_intent->req_mode != ARA_MODE_AUTO) {
            s_manip_ctx.auto_start_armed = true;
        }

        if ((s_manip_ctx.current_state == MANIP_STATE_AUTO_MOVE_PICK) ||
            (s_manip_ctx.current_state == MANIP_STATE_AUTO_HOLD_VERIFY) ||
            (s_manip_ctx.current_state == MANIP_STATE_AUTO_GRAB_CLOSE) ||
            (s_manip_ctx.current_state == MANIP_STATE_AUTO_RETRACT) ||
            (s_manip_ctx.current_state == MANIP_STATE_AUTO_DONE))
        {
            if (p_rc_intent->req_mode == ARA_MODE_MANUAL) {
                Manip_SeedManualJointTarget(p_motion_state);
                Manip_SetState(MANIP_STATE_MANUAL);
            }
        } else if ((s_manip_ctx.current_state == MANIP_STATE_IDLE) &&
                   (p_rc_intent->req_mode == ARA_MODE_MANUAL))
        {
            Manip_SeedManualJointTarget(p_motion_state);
            Manip_SetState(MANIP_STATE_MANUAL);
        } else if ((s_manip_ctx.current_state == MANIP_STATE_MANUAL) &&
                   (p_rc_intent->req_mode == ARA_MODE_AUTO) &&
                   s_manip_ctx.auto_start_armed)
        {
            s_manip_ctx.auto_start_armed = false;
            Manip_SetState(MANIP_STATE_AUTO_MOVE_PICK);
        } else if ((s_manip_ctx.current_state == MANIP_STATE_IDLE) &&
                   (p_rc_intent->req_mode == ARA_MODE_AUTO) &&
                   s_manip_ctx.auto_start_armed)
        {
            s_manip_ctx.auto_start_armed = false;
            Manip_SetState(MANIP_STATE_AUTO_MOVE_PICK);
        }
    }

    /* ------------------------------------------------------------------------
     * STAGE 2: State machine execution
     * --------------------------------------------------------------------- */
    switch (s_manip_ctx.current_state)
    {
        case MANIP_STATE_STARTUP_PARK:
        {
            p_out_cmd->sys_mode = ARA_MODE_MANUAL;
            Manip_CommandPose(p_out_cmd,
                              park_target_raw,
                              MANIP_DEMO_GRIPPER_HOME_PERCENT,
                              MANIP_DEMO_ROLL_HOME_DEG);

            if (Manip_IsJointReached(p_motion_state, park_target_raw, reach_tol_raw)) {
                if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >=
                    MANIP_STARTUP_PARK_SETTLE_TICKS)
                {
                    Manip_SetState(MANIP_STATE_IDLE);
                }
            } else {
                s_manip_ctx.state_enter_tick = s_task_tick_counter;
            }
            break;
        }

        case MANIP_STATE_IDLE:
        {
            p_out_cmd->sys_mode = ARA_MODE_MANUAL;
            Manip_CommandPose(p_out_cmd,
                              park_target_raw,
                              MANIP_DEMO_GRIPPER_HOME_PERCENT,
                              MANIP_DEMO_ROLL_HOME_DEG);
            break;
        }

        case MANIP_STATE_MANUAL:
        {
            p_out_cmd->sys_mode = ARA_MODE_MANUAL;
            Manip_RunManual(p_rc_intent, p_out_cmd);
            break;
        }

        case MANIP_STATE_AUTO_MOVE_PICK:
        {
            p_out_cmd->sys_mode = ARA_MODE_AUTO;
            Manip_CommandPose(p_out_cmd,
                              pick_target_raw,
                              MANIP_DEMO_GRIPPER_PREOPEN_PERCENT,
                              MANIP_DEMO_ROLL_AUTO_TARGET_DEG);

            if (Manip_IsJointReached(p_motion_state, pick_target_raw, reach_tol_raw)) {
                Manip_SetState(MANIP_STATE_AUTO_HOLD_VERIFY);
            }
            break;
        }

        case MANIP_STATE_AUTO_HOLD_VERIFY:
        {
            p_out_cmd->sys_mode = ARA_MODE_AUTO;
            Manip_CommandPose(p_out_cmd,
                              pick_target_raw,
                              MANIP_DEMO_GRIPPER_PREOPEN_PERCENT,
                              MANIP_DEMO_ROLL_AUTO_TARGET_DEG);

            if (!Manip_IsJointReached(p_motion_state, pick_target_raw, reach_tol_raw)) {
                Manip_SetState(MANIP_STATE_AUTO_MOVE_PICK);
            } else if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >=
                       MANIP_AUTO_HOLD_VERIFY_TICKS)
            {
                Manip_SetState(MANIP_STATE_AUTO_GRAB_CLOSE);
            }
            break;
        }

        case MANIP_STATE_AUTO_GRAB_CLOSE:
        {
            p_out_cmd->sys_mode = ARA_MODE_AUTO;
            Manip_CommandPose(p_out_cmd,
                              pick_target_raw,
                              MANIP_DEMO_GRIPPER_CLOSE_PERCENT,
                              MANIP_DEMO_ROLL_AUTO_TARGET_DEG);

            if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >=
                MANIP_AUTO_GRAB_WAIT_TICKS)
            {
                Manip_SetState(MANIP_STATE_AUTO_RETRACT);
            }
            break;
        }

        case MANIP_STATE_AUTO_RETRACT:
        {
            p_out_cmd->sys_mode = ARA_MODE_AUTO;
            Manip_CommandPose(p_out_cmd,
                              retract_target_raw,
                              MANIP_DEMO_GRIPPER_CLOSE_PERCENT,
                              MANIP_DEMO_ROLL_AUTO_TARGET_DEG);

            if (Manip_IsJointReached(p_motion_state, retract_target_raw, reach_tol_raw)) {
                if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >=
                    MANIP_AUTO_RETRACT_SETTLE_TICKS)
                {
                    Manip_SetState(MANIP_STATE_AUTO_DONE);
                }
            } else {
                s_manip_ctx.state_enter_tick = s_task_tick_counter;
            }
            break;
        }

        case MANIP_STATE_AUTO_DONE:
        {
            p_out_cmd->sys_mode = ARA_MODE_AUTO;
            Manip_CommandPose(p_out_cmd,
                              retract_target_raw,
                              MANIP_DEMO_GRIPPER_CLOSE_PERCENT,
                              MANIP_DEMO_ROLL_AUTO_TARGET_DEG);
            break;
        }

        case MANIP_STATE_ERROR_SAFE:
        default:
        {
            p_out_cmd->sys_mode = ARA_MODE_ERROR;
            p_out_cmd->emergency_stop = true;
            p_out_cmd->target_foc_angle = park_target_raw;

            (void)ModActuator_SetGripper(&s_actuator_ctx, MANIP_DEMO_GRIPPER_HOME_PERCENT);
            (void)ModActuator_SetRoll(&s_actuator_ctx, MANIP_DEMO_ROLL_HOME_DEG);
            break;
        }
    }
}