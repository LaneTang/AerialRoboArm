/**
 * @file task_motion.c
 * @brief Single-axis motion execution service for the BLDC folding-arm axis (L4).
 * @note  Strictly C99. No RTOS dependencies.
 *        This revision intentionally removes the old demo/business auto-flow
 *        and exposes a command-driven execution kernel based on ext_raw.
 */

#include "task_motion.h"
#include "bsp_pwm.h"
#include <stddef.h>

/* ============================================================================
 * 1. Internal Configuration Constants
 * ========================================================================== */

/**
 * @brief Number of motor pole pairs for the selected 2808 BLDC motor.
 */
#define MOTION_MOTOR_POLE_PAIRS              (7U)

/**
 * @brief Number of 1kHz ticks used to prime the encoder DMA pipeline at boot.
 */
#define MOTION_SENSOR_PRIME_TICKS            (20U)

/**
 * @brief Fixed D-axis alignment voltage used during FOC electrical alignment.
 * @note  Q15 format. ~15% full-scale voltage.
 */
#define MOTION_ALIGN_VOLTAGE_D_Q15           (4915)

/**
 * @brief Time to hold the fixed alignment vector.
 */
#define MOTION_ALIGN_DURATION_TICKS          (500U)

/**
 * @brief Full-scale ext_raw error used to normalize PID input to Q15.
 */
#define MOTION_PID_ERR_FULL_SCALE_EXT_RAW    (1024)

/**
 * @brief Closed-loop q-axis voltage limit.
 * @note  Q15 format. ~25% full-scale voltage.
 */
#define MOTION_CLOSED_LOOP_UQ_LIMIT_Q15      (8192)

/**
 * @brief Position-loop proportional gain in Q8.8 format.
 * @note  384 = 1.5
 */
#define MOTION_PID_POS_KP_Q8_8               (384)

/**
 * @brief Position-loop integral gain in Q8.8 format.
 */
#define MOTION_PID_POS_KI_Q8_8               (0)

/**
 * @brief Position-loop derivative gain in Q8.8 format.
 */
#define MOTION_PID_POS_KD_Q8_8               (0)

/**
 * @brief Maximum integral accumulator magnitude for the position PID.
 */
#define MOTION_PID_INT_MAX                   (131072L)

/**
 * @brief ext_raw error threshold considered "target reached".
 */
#define MOTION_TARGET_REACHED_THD_EXT_RAW    (12)

/* ============================================================================
 * 2. Static Context Allocation
 * ========================================================================== */

/**
 * @brief Singleton runtime context for the motion service.
 */
static TaskMotion_Context_t s_motion_ctx;

/* ============================================================================
 * 3. Private Helper Functions
 * ========================================================================== */

/**
 * @brief  Compute absolute value of a signed 32-bit integer.
 * @param  value Input value.
 * @return Absolute value.
 */
static int32_t Motion_Abs32(int32_t value)
{
    return (value >= 0) ? value : (-value);
}

/**
 * @brief  Wrap an arbitrary signed angle into the AS5600 raw domain [0, 4095].
 * @param  angle Input angle in extended integer domain.
 * @return Wrapped raw-angle value.
 */
static uint16_t Motion_WrapRawAngle(int32_t angle)
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
 * @brief  Clamp ext_raw into the valid calibrated stroke range.
 * @param  ext_raw Input extension coordinate.
 * @return Clamped extension coordinate.
 */
static int32_t Motion_ClampExtRaw(int32_t ext_raw)
{
    if (ext_raw < MOTION_EXT_MIN_RAW) {
        return MOTION_EXT_MIN_RAW;
    }

    if (ext_raw > MOTION_EXT_MAX_RAW) {
        return MOTION_EXT_MAX_RAW;
    }

    return ext_raw;
}

/**
 * @brief  Calculate rotational velocity with wrap-around handling.
 * @param  curr Current AS5600 raw angle.
 * @param  prev Previous AS5600 raw angle.
 * @return Delta counts per millisecond.
 */
static int16_t Motion_CalcVelocity(uint16_t curr, uint16_t prev)
{
    int32_t diff = (int32_t)curr - (int32_t)prev;

    if (diff > 2048) {
        diff -= 4096;
    } else if (diff < -2048) {
        diff += 4096;
    }

    return (int16_t)diff;
}

/**
 * @brief  Convert ext_raw position error into Q15 with saturation.
 * @param  error_ext_raw Signed ext_raw error.
 * @return Q15-normalized error.
 */
static int16_t Motion_ExtErrorToQ15(int32_t error_ext_raw)
{
    int32_t scaled;

    if (error_ext_raw >= MOTION_PID_ERR_FULL_SCALE_EXT_RAW) {
        return (int16_t)32767;
    }

    if (error_ext_raw <= (-MOTION_PID_ERR_FULL_SCALE_EXT_RAW)) {
        return (int16_t)(-32768);
    }

    scaled = (error_ext_raw * 32767) / MOTION_PID_ERR_FULL_SCALE_EXT_RAW;

    if (scaled > 32767) {
        scaled = 32767;
    } else if (scaled < -32768) {
        scaled = -32768;
    }

    return (int16_t)scaled;
}

/**
 * @brief  Enter a new public execution state.
 * @param  new_state Target execution state.
 */
static void Motion_SetExecState(MotionExecState_t new_state)
{
    if (s_motion_ctx.exec_state != new_state) {
        s_motion_ctx.exec_state = new_state;
        s_motion_ctx.state_ticks = 0U;
    }
}

/**
 * @brief  Force the power stage into a safe relaxed state.
 */
static void Motion_StopPowerStage(void)
{
    BSP_PWM_StopAll();
    DrvBldc_Enable(&s_motion_ctx.motor_driver, false);
}

/**
 * @brief  Apply the latest FOC duty outputs to the power stage.
 * @return ARA_OK on success, otherwise an error code from the BLDC driver.
 */
static AraStatus_t Motion_ApplyFocOutputs(void)
{
    return DrvBldc_SetDuties(&s_motion_ctx.motor_driver,
                             s_motion_ctx.foc_algo.duty_a,
                             s_motion_ctx.foc_algo.duty_b,
                             s_motion_ctx.foc_algo.duty_c);
}

/**
 * @brief  Run the position loop from ext_raw error to FOC q-axis voltage.
 * @param  pos_error_ext_raw Signed ext_raw error.
 * @return Q15 torque / q-axis voltage command.
 */
static int16_t Motion_RunPositionController(int32_t pos_error_ext_raw)
{
    int16_t error_q15;
    int16_t uq_q15;

    error_q15 = Motion_ExtErrorToQ15(pos_error_ext_raw);
    uq_q15 = AlgPid_Compute(&s_motion_ctx.pos_pid, error_q15, 0);

    return uq_q15;
}

/**
 * @brief  Update asynchronous encoder read pipeline health.
 * @param  trigger_status Return value from DrvAS5600_TriggerUpdate().
 */
static void Motion_UpdateSensorHealth(AraStatus_t trigger_status)
{
    if ((trigger_status == ARA_OK) || (trigger_status == ARA_BUSY)) {
        if ((s_motion_ctx.hw_status == ARA_OK) || (s_motion_ctx.hw_status == ARA_BUSY)) {
            s_motion_ctx.hw_status = ARA_OK;
        }
    } else {
        s_motion_ctx.hw_status = trigger_status;
        Motion_SetExecState(MOTION_EXEC_ERROR);
    }
}

/**
 * @brief  Execute one closed-loop step toward a target ext_raw.
 * @param  target_ext_raw Target extension coordinate.
 * @param  current_raw Current AS5600 raw angle.
 * @param  current_ext_raw Current extension coordinate.
 * @param  p_target_reached Output reach flag.
 */
static void Motion_RunClosedLoopToExt(int32_t target_ext_raw,
                                      uint16_t current_raw,
                                      int32_t current_ext_raw,
                                      bool *p_target_reached)
{
    AraStatus_t apply_status;
    int32_t pos_error_ext_raw;
    int16_t torque_uq;

    target_ext_raw = Motion_ClampExtRaw(target_ext_raw);
    pos_error_ext_raw = target_ext_raw - current_ext_raw;
    torque_uq = Motion_RunPositionController(pos_error_ext_raw);

    DrvBldc_Enable(&s_motion_ctx.motor_driver, true);
    AlgFoc_Run(&s_motion_ctx.foc_algo, current_raw, torque_uq, 0);
    apply_status = Motion_ApplyFocOutputs();

    if (apply_status != ARA_OK) {
        s_motion_ctx.hw_status = apply_status;
        Motion_SetExecState(MOTION_EXEC_ERROR);
        if (p_target_reached != NULL) {
            *p_target_reached = false;
        }
        return;
    }

    s_motion_ctx.hw_status = ARA_OK;

    if (p_target_reached != NULL) {
        *p_target_reached = (Motion_Abs32(pos_error_ext_raw) <= MOTION_TARGET_REACHED_THD_EXT_RAW);
    }
}

/* ============================================================================
 * 4. Public Coordinate Mapping API
 * ========================================================================== */

int32_t TaskMotion_MapRawToExt(uint16_t raw_angle)
{
    int32_t raw_unwrapped;

    if (raw_angle < MOTION_WRAP_THRESHOLD_RAW) {
        raw_unwrapped = (int32_t)raw_angle + 4096;
    } else {
        raw_unwrapped = (int32_t)raw_angle;
    }

    return raw_unwrapped - (int32_t)MOTION_HOME_RAW_CALIB;
}

uint16_t TaskMotion_MapExtToRaw(int32_t ext_raw)
{
    int32_t raw_unwrapped;

    ext_raw = Motion_ClampExtRaw(ext_raw);
    raw_unwrapped = (int32_t)MOTION_HOME_RAW_CALIB + ext_raw;

    return Motion_WrapRawAngle(raw_unwrapped);
}

/* ============================================================================
 * 5. Public API Implementation
 * ========================================================================== */

void TaskMotion_Init(void)
{
    AraStatus_t init_status;

    s_motion_ctx.exec_state = MOTION_EXEC_UNINIT;
    s_motion_ctx.active_cmd = MOTION_CMD_IDLE;
    s_motion_ctx.uptime_ticks = 0U;
    s_motion_ctx.state_ticks = 0U;

    s_motion_ctx.is_foc_aligned = false;
    s_motion_ctx.foc_zero_offset = 0U;
    s_motion_ctx.home_raw = MOTION_HOME_RAW_CALIB;
    s_motion_ctx.wrap_threshold_raw = MOTION_WRAP_THRESHOLD_RAW;
    s_motion_ctx.hold_target_ext_raw = 0;
    s_motion_ctx.track_target_ext_raw = 0;

    s_motion_ctx.hw_status = ARA_OK;
    s_motion_ctx.prev_angle_raw = 0U;

    init_status = DrvAS5600_Init(&s_motion_ctx.enc_driver, BSP_I2C_MOTION, NULL);
    if (init_status != ARA_OK) {
        s_motion_ctx.hw_status = init_status;
        s_motion_ctx.exec_state = MOTION_EXEC_ERROR;
    } else {
        s_motion_ctx.exec_state = MOTION_EXEC_IDLE;
    }

    DrvBldc_Init(&s_motion_ctx.motor_driver, BSP_GPIO_MOTOR_EN);

    AlgFoc_Init(&s_motion_ctx.foc_algo, MOTION_MOTOR_POLE_PAIRS, BSP_PWM_MAX_DUTY);

    AlgPid_Init(&s_motion_ctx.pos_pid);
    AlgPid_SetGains(&s_motion_ctx.pos_pid,
                    MOTION_PID_POS_KP_Q8_8,
                    MOTION_PID_POS_KI_Q8_8,
                    MOTION_PID_POS_KD_Q8_8,
                    MOTION_CLOSED_LOOP_UQ_LIMIT_Q15,
                    MOTION_PID_INT_MAX);

    if (s_motion_ctx.exec_state != MOTION_EXEC_ERROR) {
        init_status = DrvAS5600_TriggerUpdate(&s_motion_ctx.enc_driver);
        if ((init_status != ARA_OK) && (init_status != ARA_BUSY)) {
            s_motion_ctx.hw_status = init_status;
            s_motion_ctx.exec_state = MOTION_EXEC_ERROR;
        }
    }
}

void TaskMotion_Update(const MotionCmd_t *p_cmd, MotionState_t *p_state)
{
    uint16_t current_raw;
    int16_t current_vel;
    int32_t current_ext_raw;
    bool target_reached = false;
    int32_t active_target_ext_raw = 0;
    AraStatus_t trigger_status;

    if ((p_cmd == NULL) || (p_state == NULL)) {
        return;
    }

    s_motion_ctx.uptime_ticks++;
    s_motion_ctx.state_ticks++;

    current_raw = DrvAS5600_GetRawAngle(&s_motion_ctx.enc_driver);
    current_vel = Motion_CalcVelocity(current_raw, s_motion_ctx.prev_angle_raw);
    current_ext_raw = TaskMotion_MapRawToExt(current_raw);
    s_motion_ctx.prev_angle_raw = current_raw;

    trigger_status = DrvAS5600_TriggerUpdate(&s_motion_ctx.enc_driver);
    Motion_UpdateSensorHealth(trigger_status);

    if (s_motion_ctx.uptime_ticks < MOTION_SENSOR_PRIME_TICKS) {
        Motion_StopPowerStage();
        Motion_SetExecState(MOTION_EXEC_IDLE);
    } else if (p_cmd->cmd == MOTION_CMD_ESTOP) {
        Motion_StopPowerStage();
        s_motion_ctx.hw_status = ARA_ERROR;
        Motion_SetExecState(MOTION_EXEC_ERROR);
    } else if (s_motion_ctx.exec_state == MOTION_EXEC_ERROR) {
        Motion_StopPowerStage();

        if (p_cmd->cmd == MOTION_CMD_IDLE) {
            s_motion_ctx.hw_status = ARA_OK;
            Motion_SetExecState(MOTION_EXEC_IDLE);
        }
    } else {
        switch (p_cmd->cmd)
        {
            case MOTION_CMD_IDLE:
            {
                Motion_StopPowerStage();
                Motion_SetExecState(MOTION_EXEC_IDLE);
                s_motion_ctx.hw_status = ARA_OK;
                break;
            }

            case MOTION_CMD_ALIGN:
            {
                AraStatus_t apply_status;

                if (s_motion_ctx.exec_state != MOTION_EXEC_ALIGNING) {
                    s_motion_ctx.is_foc_aligned = false;
                    s_motion_ctx.foc_zero_offset = 0U;
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetExecState(MOTION_EXEC_ALIGNING);
                }

                DrvBldc_Enable(&s_motion_ctx.motor_driver, true);

                AlgFoc_Run(&s_motion_ctx.foc_algo, 0U, 0, MOTION_ALIGN_VOLTAGE_D_Q15);
                apply_status = Motion_ApplyFocOutputs();

                if (apply_status != ARA_OK) {
                    s_motion_ctx.hw_status = apply_status;
                    Motion_SetExecState(MOTION_EXEC_ERROR);
                    break;
                }

                s_motion_ctx.hw_status = ARA_OK;

                if (s_motion_ctx.state_ticks >= MOTION_ALIGN_DURATION_TICKS) {
                    s_motion_ctx.foc_zero_offset = current_raw;
                    AlgFoc_SetZeroOffset(&s_motion_ctx.foc_algo, s_motion_ctx.foc_zero_offset);
                    s_motion_ctx.is_foc_aligned = true;
                    s_motion_ctx.hold_target_ext_raw = current_ext_raw;
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetExecState(MOTION_EXEC_HOLDING);
                    target_reached = true;
                    active_target_ext_raw = s_motion_ctx.hold_target_ext_raw;
                }
                break;
            }

            case MOTION_CMD_HOLD:
            {
                if (!s_motion_ctx.is_foc_aligned) {
                    Motion_StopPowerStage();
                    Motion_SetExecState(MOTION_EXEC_IDLE);
                    s_motion_ctx.hw_status = ARA_OK;
                    break;
                }

                if ((s_motion_ctx.active_cmd != MOTION_CMD_HOLD) ||
                    (s_motion_ctx.exec_state != MOTION_EXEC_HOLDING))
                {
                    s_motion_ctx.hold_target_ext_raw = current_ext_raw;
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetExecState(MOTION_EXEC_HOLDING);
                }

                active_target_ext_raw = s_motion_ctx.hold_target_ext_raw;
                Motion_RunClosedLoopToExt(active_target_ext_raw,
                                          current_raw,
                                          current_ext_raw,
                                          &target_reached);
                break;
            }

            case MOTION_CMD_GOTO_HOME:
            {
                if (!s_motion_ctx.is_foc_aligned) {
                    Motion_StopPowerStage();
                    Motion_SetExecState(MOTION_EXEC_IDLE);
                    s_motion_ctx.hw_status = ARA_OK;
                    break;
                }

                if ((s_motion_ctx.active_cmd != MOTION_CMD_GOTO_HOME) ||
                    (s_motion_ctx.exec_state != MOTION_EXEC_TRACKING))
                {
                    s_motion_ctx.track_target_ext_raw = 0;
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetExecState(MOTION_EXEC_TRACKING);
                }

                active_target_ext_raw = s_motion_ctx.track_target_ext_raw;
                Motion_RunClosedLoopToExt(active_target_ext_raw,
                                          current_raw,
                                          current_ext_raw,
                                          &target_reached);
                break;
            }

            case MOTION_CMD_GOTO_EXT:
            {
                if (!s_motion_ctx.is_foc_aligned) {
                    Motion_StopPowerStage();
                    Motion_SetExecState(MOTION_EXEC_IDLE);
                    s_motion_ctx.hw_status = ARA_OK;
                    break;
                }

                if ((s_motion_ctx.active_cmd != MOTION_CMD_GOTO_EXT) ||
                    (s_motion_ctx.track_target_ext_raw != Motion_ClampExtRaw(p_cmd->target_ext_raw)) ||
                    (s_motion_ctx.exec_state != MOTION_EXEC_TRACKING))
                {
                    s_motion_ctx.track_target_ext_raw = Motion_ClampExtRaw(p_cmd->target_ext_raw);
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetExecState(MOTION_EXEC_TRACKING);
                }

                active_target_ext_raw = s_motion_ctx.track_target_ext_raw;
                Motion_RunClosedLoopToExt(active_target_ext_raw,
                                          current_raw,
                                          current_ext_raw,
                                          &target_reached);
                break;
            }

            case MOTION_CMD_ESTOP:
            default:
            {
                Motion_StopPowerStage();
                s_motion_ctx.hw_status = ARA_ERROR;
                Motion_SetExecState(MOTION_EXEC_ERROR);
                break;
            }
        }
    }

    s_motion_ctx.active_cmd = p_cmd->cmd;

    p_state->exec_state = s_motion_ctx.exec_state;
    p_state->is_aligned = s_motion_ctx.is_foc_aligned;
    p_state->home_raw = s_motion_ctx.home_raw;
    p_state->raw_angle = current_raw;
    p_state->ext_raw = current_ext_raw;
    p_state->velocity_raw_per_ms = current_vel;
    p_state->target_ext_raw = active_target_ext_raw;
    p_state->target_reached = target_reached;
    p_state->status = s_motion_ctx.hw_status;
}