/**
 * @file task_manipulator.c
 * @brief Arm manipulator brain and state machine runnable (L4).
 * @note  Strictly C99. No floating point. No RTOS dependencies.
 *        This revision aligns the manipulator with the current system contract:
 *        - MANUAL mode directly controls all actuators from RC semantics.
 *        - AUTO mode remains a 50Hz vision-driven state machine.
 *        - DataHub / Motion interface remains backward-compatible.
 */

#include "task_manipulator.h"
#include "mod_actuator.h"
#include <stddef.h>

/* ============================================================================
 * 1. Business Parameters
 * ========================================================================== */

/**
 * @brief Z-axis convergence deadband in millimeters.
 */
#define MANIP_EXT_DEADBAND_MM              (5U)

/**
 * @brief Roll convergence deadband in centi-degrees.
 * @note  600 = 6.00 degree.
 */
#define MANIP_ROLL_DEADBAND_CDEG           (600)

/**
 * @brief Debounce duration before grab after convergence.
 * @note  10 ticks * 20ms = 200ms.
 */
#define MANIP_GRAB_DEBOUNCE_TICKS          (10U)

/**
 * @brief Blind wait duration during grab / retract actions.
 * @note  25 ticks * 20ms = 500ms.
 */
#define MANIP_ACTION_WAIT_TICKS            (25U)

/**
 * @brief First-order LPF alpha in Q8 form.
 * @note  0~256. Smaller = smoother but slower.
 */
#define MANIP_LPF_ALPHA_Q8                 (50U)

/**
 * @brief Manual-mode raw-angle step applied every 50Hz tick.
 */
#define MANIP_MANUAL_EXT_STEP_RAW          (40U)

/**
 * @brief Manual-mode gripper percent step applied every 50Hz tick.
 */
#define MANIP_MANUAL_GRIPPER_STEP_PERCENT  (8U)

/**
 * @brief Neutral roll servo angle in degree for AUTO mode.
 */
#define MANIP_ROLL_NEUTRAL_DEG             (90U)

/**
 * @brief Maximum absolute roll command in degree relative to neutral.
 */
#define MANIP_ROLL_MAX_ABS_DEG             (90)

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
 * 3. Z-Axis Mapping LUT
 * ========================================================================== */

/**
 * @brief LUT point for distance-to-raw-angle mapping.
 */
typedef struct {
    uint16_t dist_mm;      /**< Physical target distance in millimeters. */
    uint16_t angle_raw;    /**< Corresponding AS5600 raw-angle target. */
} ManipLutPoint_t;

/**
 * @brief Offline calibration LUT from extension distance to encoder raw angle.
 * @note  Must remain strictly monotonic in both distance and angle.
 */
static const ManipLutPoint_t s_z_to_angle_lut[] = {
        {120U,  850U},   /**< Folded home */
        {150U, 1200U},
        {180U, 1650U},
        {210U, 2100U},
        {240U, 2600U},
        {270U, 3150U},
        {300U, 3700U}    /**< Maximum extension */
};

#define MANIP_LUT_SIZE \
    ((uint8_t)(sizeof(s_z_to_angle_lut) / sizeof(s_z_to_angle_lut[0])))

#define MANIP_HOME_DIST_MM \
    (s_z_to_angle_lut[0].dist_mm)

#define MANIP_HOME_ANGLE_RAW \
    (s_z_to_angle_lut[0].angle_raw)

#define MANIP_MAX_ANGLE_RAW \
    (s_z_to_angle_lut[MANIP_LUT_SIZE - 1U].angle_raw)

/* ============================================================================
 * 4. Private Helper Functions
 * ========================================================================== */

/**
 * @brief  Compute absolute value of a signed 16-bit integer.
 * @param  value Input value.
 * @return Absolute value in unsigned form.
 */
static uint16_t Manip_Abs16(int16_t value)
{
    return (value < 0) ? (uint16_t)(-value) : (uint16_t)value;
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
 * @brief  Enter a new manipulator state and refresh the state's entry tick.
 * @param  new_state Target state.
 */
static void Manip_SetState(ManipulatorState_t new_state)
{
    s_manip_ctx.current_state   = new_state;
    s_manip_ctx.state_enter_tick = s_task_tick_counter;
}

/**
 * @brief  Reset AUTO-tracking filter state to the folded-home reference.
 */
static void Manip_ResetAutoFilters(void)
{
    s_manip_ctx.filtered_ext_mm = MANIP_HOME_DIST_MM;
    s_manip_ctx.filtered_roll   = 0;
}

/**
 * @brief  Reset MANUAL control state to safe defaults.
 */
static void Manip_ResetManualTargets(void)
{
    s_manip_ctx.manual_target_angle_raw = MANIP_HOME_ANGLE_RAW;
    s_manip_ctx.manual_gripper_percent  = 0U;
    s_manip_ctx.manual_roll_degree      = MANIP_ROLL_NEUTRAL_DEG;
}

/**
 * @brief  Map a target extension distance to BLDC raw-angle target.
 * @param  target_mm Target extension distance in millimeters.
 * @return AS5600 raw-angle target.
 */
static uint16_t Manip_MapDistanceToAngle(uint16_t target_mm)
{
    uint8_t i;

    if (target_mm <= s_z_to_angle_lut[0].dist_mm) {
        return s_z_to_angle_lut[0].angle_raw;
    }

    if (target_mm >= s_z_to_angle_lut[MANIP_LUT_SIZE - 1U].dist_mm) {
        return s_z_to_angle_lut[MANIP_LUT_SIZE - 1U].angle_raw;
    }

    for (i = 0U; i < (MANIP_LUT_SIZE - 1U); i++) {
        if ((target_mm >= s_z_to_angle_lut[i].dist_mm) &&
            (target_mm <= s_z_to_angle_lut[i + 1U].dist_mm))
        {
            int32_t x0 = (int32_t)s_z_to_angle_lut[i].dist_mm;
            int32_t x1 = (int32_t)s_z_to_angle_lut[i + 1U].dist_mm;
            int32_t y0 = (int32_t)s_z_to_angle_lut[i].angle_raw;
            int32_t y1 = (int32_t)s_z_to_angle_lut[i + 1U].angle_raw;
            int32_t interpolated;

            interpolated = y0 + ((y1 - y0) * ((int32_t)target_mm - x0)) / (x1 - x0);
            return (uint16_t)interpolated;
        }
    }

    return MANIP_HOME_ANGLE_RAW;
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
 * @brief  Map AUTO roll target in centi-degrees to servo angle.
 * @param  roll_cdeg Signed roll target in centi-degrees.
 * @return Roll servo command in 0~180 degree.
 * @note   A roll target of 0 maps to neutral (90 degree).
 */
static uint8_t Manip_MapAutoRollToServoDegree(int16_t roll_cdeg)
{
    int32_t roll_deg;
    int32_t servo_deg;

    roll_deg = (int32_t)roll_cdeg / 100;

    if (roll_deg > MANIP_ROLL_MAX_ABS_DEG) {
        roll_deg = MANIP_ROLL_MAX_ABS_DEG;
    } else if (roll_deg < (-MANIP_ROLL_MAX_ABS_DEG)) {
        roll_deg = -MANIP_ROLL_MAX_ABS_DEG;
    }

    servo_deg = (int32_t)MANIP_ROLL_NEUTRAL_DEG + roll_deg;

    if (servo_deg < 0) {
        servo_deg = 0;
    } else if (servo_deg > 180) {
        servo_deg = 180;
    }

    return (uint8_t)servo_deg;
}

/**
 * @brief  Apply MANUAL-mode direct control to all actuators.
 * @param  p_rc_intent Pointer to RC semantic intent.
 * @param  p_out_cmd Pointer to motion downlink command.
 */
static void Manip_RunManual(const RcControlData_t *p_rc_intent,
                            DataHub_Cmd_t *p_out_cmd)
{
    if ((p_rc_intent == NULL) || (p_out_cmd == NULL)) {
        return;
    }

    /* ---------------- BLDC extension axis ---------------- */
    if (p_rc_intent->arm_cmd == ARM_CMD_EXTEND) {
        uint32_t next_target = (uint32_t)s_manip_ctx.manual_target_angle_raw +
                               (uint32_t)MANIP_MANUAL_EXT_STEP_RAW;
        if (next_target > (uint32_t)MANIP_MAX_ANGLE_RAW) {
            next_target = (uint32_t)MANIP_MAX_ANGLE_RAW;
        }
        s_manip_ctx.manual_target_angle_raw = (uint16_t)next_target;
    } else if (p_rc_intent->arm_cmd == ARM_CMD_RETRACT) {
        if (s_manip_ctx.manual_target_angle_raw > (MANIP_HOME_ANGLE_RAW + MANIP_MANUAL_EXT_STEP_RAW)) {
            s_manip_ctx.manual_target_angle_raw =
                    (uint16_t)(s_manip_ctx.manual_target_angle_raw - MANIP_MANUAL_EXT_STEP_RAW);
        } else {
            s_manip_ctx.manual_target_angle_raw = MANIP_HOME_ANGLE_RAW;
        }
    } else {
        /* HOLD: preserve previous target */
    }

    s_manip_ctx.manual_target_angle_raw =
            Manip_ClampU16(s_manip_ctx.manual_target_angle_raw,
                           MANIP_HOME_ANGLE_RAW,
                           MANIP_MAX_ANGLE_RAW);

    p_out_cmd->target_foc_angle = s_manip_ctx.manual_target_angle_raw;

    /* ---------------- Gripper ---------------- */
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
        /* STOP: preserve previous command */
    }

    (void)ModActuator_SetGripper(&s_actuator_ctx, s_manip_ctx.manual_gripper_percent);

    /* ---------------- Roll servo ---------------- */
    s_manip_ctx.manual_roll_degree = Manip_MapAuxToRollDegree(p_rc_intent->aux_knob_val);
    s_manip_ctx.manual_roll_degree = Manip_ClampU8(s_manip_ctx.manual_roll_degree, 0U, 180U);
    (void)ModActuator_SetRoll(&s_actuator_ctx, s_manip_ctx.manual_roll_degree);
}

/**
 * @brief  Command the actuator subsystem to a safe folded-home posture.
 * @param  p_out_cmd Pointer to motion downlink command.
 * @param  gripper_percent Target gripper closure percentage.
 * @param  roll_degree Target roll degree.
 */
static void Manip_CommandHomePose(DataHub_Cmd_t *p_out_cmd,
                                  uint8_t gripper_percent,
                                  uint8_t roll_degree)
{
    if (p_out_cmd == NULL) {
        return;
    }

    p_out_cmd->target_foc_angle = MANIP_HOME_ANGLE_RAW;
    (void)ModActuator_SetGripper(&s_actuator_ctx, gripper_percent);
    (void)ModActuator_SetRoll(&s_actuator_ctx, roll_degree);
}

/* ============================================================================
 * 5. Public API
 * ========================================================================== */

void TaskManipulator_Init(void)
{
    s_task_tick_counter = 0U;

    s_manip_ctx.current_state    = MANIP_STATE_IDLE;
    s_manip_ctx.state_enter_tick = 0U;

    Manip_ResetAutoFilters();
    Manip_ResetManualTargets();

    (void)ModActuator_Init(&s_actuator_ctx);
}

void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const AraVisionData_t *p_vision_data,
                            DataHub_Cmd_t *p_out_cmd)
{
    uint16_t z_error_mm;
    uint16_t roll_error_cdeg;

    if ((p_rc_intent == NULL) || (p_vision_data == NULL) || (p_out_cmd == NULL)) {
        return;
    }

    s_task_tick_counter++;

    /* ---------------- Safe default output ---------------- */
    p_out_cmd->sys_mode         = p_rc_intent->req_mode;
    p_out_cmd->emergency_stop   = false;
    p_out_cmd->target_foc_angle = MANIP_HOME_ANGLE_RAW;

    /* ------------------------------------------------------------------------
     * STAGE 1: Global reset / safety / mode arbitration
     * --------------------------------------------------------------------- */

    /* Dedicated reset pulse: re-enter Motion INIT and reset local manip state. */
    if (p_rc_intent->sys_reset_pulse) {
        Manip_ResetAutoFilters();
        Manip_ResetManualTargets();
        Manip_SetState(MANIP_STATE_IDLE);

        p_out_cmd->sys_mode         = ARA_MODE_INIT;
        p_out_cmd->emergency_stop   = false;
        p_out_cmd->target_foc_angle = MANIP_HOME_ANGLE_RAW;

        (void)ModActuator_SetGripper(&s_actuator_ctx, 0U);
        (void)ModActuator_SetRoll(&s_actuator_ctx, MANIP_ROLL_NEUTRAL_DEG);
        return;
    }

    /* Highest-priority safety gate. */
    if ((p_rc_intent->estop_state == ESTOP_ACTIVE) ||
        (p_vision_data->pc_estop_req) ||
        (!p_rc_intent->is_link_up))
    {
        if (s_manip_ctx.current_state != MANIP_STATE_ERROR_SAFE) {
            Manip_SetState(MANIP_STATE_ERROR_SAFE);
        }
    }
        /* MANUAL takes priority over AUTO FSM. */
    else if (p_rc_intent->req_mode == ARA_MODE_MANUAL) {
        if (s_manip_ctx.current_state != MANIP_STATE_MANUAL) {
            Manip_ResetAutoFilters();
            Manip_SetState(MANIP_STATE_MANUAL);
        }
    }
        /* AUTO mode resumes from IDLE if coming from MANUAL or ERROR. */
    else if (p_rc_intent->req_mode == ARA_MODE_AUTO) {
        if ((s_manip_ctx.current_state == MANIP_STATE_MANUAL) ||
            (s_manip_ctx.current_state == MANIP_STATE_ERROR_SAFE))
        {
            Manip_ResetAutoFilters();
            Manip_SetState(MANIP_STATE_IDLE);
        }
    }
        /* Any other mode falls back to safe IDLE. */
    else {
        if (s_manip_ctx.current_state != MANIP_STATE_IDLE) {
            Manip_ResetAutoFilters();
            Manip_SetState(MANIP_STATE_IDLE);
        }
    }

    /* ------------------------------------------------------------------------
     * STAGE 2: State machine / direct control execution
     * --------------------------------------------------------------------- */
    switch (s_manip_ctx.current_state)
    {
        case MANIP_STATE_IDLE:
        {
            Manip_ResetManualTargets();
            Manip_ResetAutoFilters();

            p_out_cmd->sys_mode = p_rc_intent->req_mode;
            Manip_CommandHomePose(p_out_cmd, 0U, MANIP_ROLL_NEUTRAL_DEG);

            if ((p_rc_intent->req_mode == ARA_MODE_AUTO) && p_vision_data->is_tracking) {
                Manip_SetState(MANIP_STATE_SEEKING);
            }
            break;
        }

        case MANIP_STATE_MANUAL:
        {
            p_out_cmd->sys_mode       = ARA_MODE_MANUAL;
            p_out_cmd->emergency_stop = false;

            Manip_RunManual(p_rc_intent, p_out_cmd);
            break;
        }

        case MANIP_STATE_SEEKING:
        {
            if (!p_vision_data->is_tracking) {
                Manip_SetState(MANIP_STATE_IDLE);
                Manip_CommandHomePose(p_out_cmd, 0U, MANIP_ROLL_NEUTRAL_DEG);
                break;
            }

            s_manip_ctx.filtered_ext_mm =
                    (uint16_t)(((uint32_t)p_vision_data->target_dist_mm * MANIP_LPF_ALPHA_Q8 +
                                (uint32_t)s_manip_ctx.filtered_ext_mm * (256U - MANIP_LPF_ALPHA_Q8)) >> 8);

            s_manip_ctx.filtered_roll =
                    (int16_t)(((int32_t)p_vision_data->target_roll * (int32_t)MANIP_LPF_ALPHA_Q8 +
                               (int32_t)s_manip_ctx.filtered_roll * (int32_t)(256U - MANIP_LPF_ALPHA_Q8)) >> 8);

            p_out_cmd->sys_mode         = ARA_MODE_AUTO;
            p_out_cmd->target_foc_angle = Manip_MapDistanceToAngle(s_manip_ctx.filtered_ext_mm);

            (void)ModActuator_SetGripper(&s_actuator_ctx, 0U);
            (void)ModActuator_SetRoll(&s_actuator_ctx,
                                      Manip_MapAutoRollToServoDegree(s_manip_ctx.filtered_roll));

            z_error_mm    = Manip_Abs16((int16_t)p_vision_data->target_dist_mm -
                                        (int16_t)s_manip_ctx.filtered_ext_mm);
            roll_error_cdeg = Manip_Abs16((int16_t)(p_vision_data->target_roll -
                                                    s_manip_ctx.filtered_roll));

            if (p_vision_data->is_grabbable &&
                (z_error_mm <= MANIP_EXT_DEADBAND_MM) &&
                (roll_error_cdeg <= MANIP_ROLL_DEADBAND_CDEG))
            {
                Manip_SetState(MANIP_STATE_CONVERGING);
            }
            break;
        }

        case MANIP_STATE_CONVERGING:
        {
            if (!p_vision_data->is_tracking) {
                Manip_SetState(MANIP_STATE_IDLE);
                Manip_CommandHomePose(p_out_cmd, 0U, MANIP_ROLL_NEUTRAL_DEG);
                break;
            }

            s_manip_ctx.filtered_ext_mm =
                    (uint16_t)(((uint32_t)p_vision_data->target_dist_mm * MANIP_LPF_ALPHA_Q8 +
                                (uint32_t)s_manip_ctx.filtered_ext_mm * (256U - MANIP_LPF_ALPHA_Q8)) >> 8);

            s_manip_ctx.filtered_roll =
                    (int16_t)(((int32_t)p_vision_data->target_roll * (int32_t)MANIP_LPF_ALPHA_Q8 +
                               (int32_t)s_manip_ctx.filtered_roll * (int32_t)(256U - MANIP_LPF_ALPHA_Q8)) >> 8);

            p_out_cmd->sys_mode         = ARA_MODE_AUTO;
            p_out_cmd->target_foc_angle = Manip_MapDistanceToAngle(s_manip_ctx.filtered_ext_mm);

            (void)ModActuator_SetGripper(&s_actuator_ctx, 0U);
            (void)ModActuator_SetRoll(&s_actuator_ctx,
                                      Manip_MapAutoRollToServoDegree(s_manip_ctx.filtered_roll));

            z_error_mm      = Manip_Abs16((int16_t)p_vision_data->target_dist_mm -
                                          (int16_t)s_manip_ctx.filtered_ext_mm);
            roll_error_cdeg = Manip_Abs16((int16_t)(p_vision_data->target_roll -
                                                    s_manip_ctx.filtered_roll));

            if ((!p_vision_data->is_grabbable) ||
                (z_error_mm > MANIP_EXT_DEADBAND_MM) ||
                (roll_error_cdeg > MANIP_ROLL_DEADBAND_CDEG))
            {
                Manip_SetState(MANIP_STATE_SEEKING);
            } else if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >= MANIP_GRAB_DEBOUNCE_TICKS) {
                Manip_SetState(MANIP_STATE_GRABBING);
            }
            break;
        }

        case MANIP_STATE_GRABBING:
        {
            p_out_cmd->sys_mode         = ARA_MODE_AUTO;
            p_out_cmd->target_foc_angle = Manip_MapDistanceToAngle(s_manip_ctx.filtered_ext_mm);

            (void)ModActuator_SetGripper(&s_actuator_ctx, 100U);
            (void)ModActuator_SetRoll(&s_actuator_ctx,
                                      Manip_MapAutoRollToServoDegree(s_manip_ctx.filtered_roll));

            if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >= MANIP_ACTION_WAIT_TICKS) {
                Manip_SetState(MANIP_STATE_RETRACTING);
            }
            break;
        }

        case MANIP_STATE_RETRACTING:
        {
            p_out_cmd->sys_mode         = ARA_MODE_AUTO;
            p_out_cmd->target_foc_angle = MANIP_HOME_ANGLE_RAW;

            (void)ModActuator_SetGripper(&s_actuator_ctx, 100U);
            (void)ModActuator_SetRoll(&s_actuator_ctx, MANIP_ROLL_NEUTRAL_DEG);

            if ((s_task_tick_counter - s_manip_ctx.state_enter_tick) >= MANIP_ACTION_WAIT_TICKS) {
                Manip_SetState(MANIP_STATE_IDLE);
            }
            break;
        }

        case MANIP_STATE_ERROR_SAFE:
        default:
        {
            p_out_cmd->sys_mode         = ARA_MODE_ERROR;
            p_out_cmd->emergency_stop   = true;
            p_out_cmd->target_foc_angle = MANIP_HOME_ANGLE_RAW;

            (void)ModActuator_SetGripper(&s_actuator_ctx, 0U);
            (void)ModActuator_SetRoll(&s_actuator_ctx, MANIP_ROLL_NEUTRAL_DEG);
            break;
        }
    }
}