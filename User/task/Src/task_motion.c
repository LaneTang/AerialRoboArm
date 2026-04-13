/**
 * @file task_motion.c
 * @brief Motion control runnable for the BLDC folding-arm axis (L4).
 * @note  Implements the 1000Hz non-blocking FOC control pipeline.
 *         The module separates electrical FOC alignment from mechanical homing.
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
#define MOTION_BOOT_PRIME_TICKS              (20U)

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
 * @brief Constant torque command used to seek the mechanical home hard stop.
 * @note  Q15 format. Sign depends on the retract direction of the actual rig.
 *        If the axis moves in the wrong direction during homing, invert this value.
 */
#define MOTION_HOME_SEEK_UQ_Q15              (-3277)

/**
 * @brief Velocity threshold used to detect the homing plateau near the hard stop.
 * @note  Unit: AS5600 raw counts per millisecond.
 */
#define MOTION_HOME_PLATEAU_VEL_THD          (1)

/**
 * @brief Required hold time for homing plateau detection.
 */
#define MOTION_HOME_PLATEAU_HOLD_TICKS       (300U)

/**
 * @brief Absolute timeout for the entire homing state.
 */
#define MOTION_HOME_TIMEOUT_TICKS            (3000U)

/**
 * @brief Backoff distance applied after the hard stop is detected.
 * @note  Unit: AS5600 raw counts.
 */
#define MOTION_HOME_BACKOFF_COUNTS           (64U)

/**
 * @brief Backoff direction in raw-angle domain.
 * @note  Set to +1 or -1 according to the real rig. Current default assumes
 *        that a positive raw-angle increment moves the axis away from the home stop.
 */
#define MOTION_HOME_BACKOFF_DIR              (1)

/**
 * @brief Settling delay before entering normal closed loop after initialization.
 */
#define MOTION_READY_SETTLE_TICKS            (20U)

/**
 * @brief Raw-count error that maps to full-scale PID input (Q15 saturation).
 * @note  Tune this according to the usable mechanical stroke and desired stiffness.
 */
#define MOTION_PID_ERR_FULL_SCALE_COUNTS     (1024)

/**
 * @brief Position threshold considered "close enough" during homing backoff.
 * @note  Unit: AS5600 raw counts.
 */
#define MOTION_POS_NEAR_COUNTS               (8)

/**
 * @brief Maximum allowed closed-loop torque command.
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
 * @note  Conservative default kept at zero for bring-up stability.
 */
#define MOTION_PID_POS_KI_Q8_8               (0)

/**
 * @brief Position-loop derivative gain in Q8.8 format.
 * @note  Conservative default kept at zero for first bring-up.
 */
#define MOTION_PID_POS_KD_Q8_8               (0)

/**
 * @brief Maximum integral accumulator magnitude for the position PID.
 */
#define MOTION_PID_INT_MAX                   (131072L)

/* ============================================================================
 * 2. Static Context Allocation
 * ========================================================================== */

/**
 * @brief Singleton runtime context for the motion task.
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
 * @brief  Calculate the shortest signed difference between two raw angles.
 * @param  target Target raw angle.
 * @param  actual Actual raw angle.
 * @return Signed shortest-path error in raw counts.
 */
static int32_t Motion_CalcShortestPathError(uint16_t target, uint16_t actual)
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
 * @brief  Convert a raw-count position error into Q15 with saturation.
 * @param  error_counts Signed position error in AS5600 raw counts.
 * @return Q15-normalized error.
 * @note   The conversion is intentionally clipped by
 *         MOTION_PID_ERR_FULL_SCALE_COUNTS.
 */
static int16_t Motion_CountsToQ15(int32_t error_counts)
{
    int32_t scaled;

    if (error_counts >= MOTION_PID_ERR_FULL_SCALE_COUNTS) {
        return (int16_t)32767;
    }

    if (error_counts <= -MOTION_PID_ERR_FULL_SCALE_COUNTS) {
        return (int16_t)(-32768);
    }

    scaled = (error_counts * 32767) / MOTION_PID_ERR_FULL_SCALE_COUNTS;

    if (scaled > 32767) {
        scaled = 32767;
    } else if (scaled < -32768) {
        scaled = -32768;
    }

    return (int16_t)scaled;
}

/**
 * @brief  Reset calibration flags and references.
 * @note   This does not de-initialize the drivers; it only resets motion-layer
 *         runtime calibration state.
 */
static void Motion_ResetCalibration(void)
{
    s_motion_ctx.is_foc_aligned        = false;
    s_motion_ctx.is_homed              = false;
    s_motion_ctx.foc_zero_offset       = 0U;
    s_motion_ctx.home_angle_raw        = 0U;
    s_motion_ctx.home_backoff_target_raw = 0U;
    s_motion_ctx.homing_phase          = MOTION_HOME_PHASE_SEEK_LIMIT;
    s_motion_ctx.plateau_ticks         = 0U;
    AlgPid_Reset(&s_motion_ctx.pos_pid);
}

/**
 * @brief  Enter a new top-level motion state.
 * @param  new_state Target state.
 */
static void Motion_SetState(MotionState_t new_state)
{
    s_motion_ctx.state       = new_state;
    s_motion_ctx.state_ticks = 0U;

    if (new_state != MOTION_STATE_HOMING) {
        s_motion_ctx.plateau_ticks = 0U;
    }
}

/**
 * @brief  Force the power stage into a safe relaxed state.
 * @note   PWM is stopped first, then the driver enable pin is deasserted.
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
 * @brief  Run the position loop from raw-angle error to FOC q-axis voltage.
 * @param  pos_error_counts Signed position error in raw counts.
 * @return Q15 torque / q-axis voltage command.
 */
static int16_t Motion_RunPositionController(int32_t pos_error_counts)
{
    int16_t error_q15;
    int16_t uq_q15;

    error_q15 = Motion_CountsToQ15(pos_error_counts);
    uq_q15    = AlgPid_Compute(&s_motion_ctx.pos_pid, error_q15, 0);

    return uq_q15;
}

/**
 * @brief  Update the asynchronous encoder read pipeline health.
 * @param  trigger_status Return value from DrvAS5600_TriggerUpdate().
 * @note   ARA_BUSY is treated as non-fatal for the current polling-based DMA
 *         pipeline. Fatal I2C errors push the motion task into ERROR state.
 */
static void Motion_UpdateSensorHealth(AraStatus_t trigger_status)
{
    if ((trigger_status == ARA_OK) || (trigger_status == ARA_BUSY)) {
        if ((s_motion_ctx.hw_status == ARA_OK) || (s_motion_ctx.hw_status == ARA_BUSY)) {
            s_motion_ctx.hw_status = ARA_OK;
        }
    } else {
        s_motion_ctx.hw_status = trigger_status;
        Motion_SetState(MOTION_STATE_ERROR);
    }
}

/* ============================================================================
 * 4. Public API Implementation
 * ========================================================================== */

void TaskMotion_Init(void)
{
    AraStatus_t init_status;

    /* ---------------- Reset runtime context ---------------- */
    s_motion_ctx.state                  = MOTION_STATE_BOOT;
    s_motion_ctx.homing_phase           = MOTION_HOME_PHASE_SEEK_LIMIT;
    s_motion_ctx.uptime_ticks           = 0U;
    s_motion_ctx.state_ticks            = 0U;
    s_motion_ctx.plateau_ticks          = 0U;
    s_motion_ctx.is_foc_aligned         = false;
    s_motion_ctx.is_homed               = false;
    s_motion_ctx.foc_zero_offset        = 0U;
    s_motion_ctx.home_angle_raw         = 0U;
    s_motion_ctx.home_backoff_target_raw = 0U;
    s_motion_ctx.hw_status              = ARA_OK;
    s_motion_ctx.prev_angle_raw         = 0U;

    /* ---------------- Initialize L2 drivers ---------------- */
    init_status = DrvAS5600_Init(&s_motion_ctx.enc_driver, BSP_I2C_MOTION, NULL);
    if (init_status != ARA_OK) {
        s_motion_ctx.hw_status = init_status;
        s_motion_ctx.state     = MOTION_STATE_ERROR;
    }

    DrvBldc_Init(&s_motion_ctx.motor_driver, BSP_GPIO_MOTOR_EN);

    /* ---------------- Initialize L3 algorithms ---------------- */
    AlgFoc_Init(&s_motion_ctx.foc_algo, MOTION_MOTOR_POLE_PAIRS, BSP_PWM_MAX_DUTY);

    AlgPid_Init(&s_motion_ctx.pos_pid);
    AlgPid_SetGains(&s_motion_ctx.pos_pid,
                    MOTION_PID_POS_KP_Q8_8,
                    MOTION_PID_POS_KI_Q8_8,
                    MOTION_PID_POS_KD_Q8_8,
                    MOTION_CLOSED_LOOP_UQ_LIMIT_Q15,
                    MOTION_PID_INT_MAX);

    /* Prime the first asynchronous read request. */
    if (s_motion_ctx.state != MOTION_STATE_ERROR) {
        init_status = DrvAS5600_TriggerUpdate(&s_motion_ctx.enc_driver);
        if ((init_status != ARA_OK) && (init_status != ARA_BUSY)) {
            s_motion_ctx.hw_status = init_status;
            s_motion_ctx.state     = MOTION_STATE_ERROR;
        }
    }
}

void TaskMotion_Update(const DataHub_Cmd_t *p_cmd, DataHub_State_t *p_state)
{
    uint16_t current_angle;
    int16_t  current_vel;
    AraStatus_t trigger_status;

    if ((p_cmd == NULL) || (p_state == NULL)) {
        return;
    }

    /* ---------------- Tick bookkeeping ---------------- */
    s_motion_ctx.uptime_ticks++;
    s_motion_ctx.state_ticks++;

    /* ---------------- Absolute safety preemption ---------------- */
    if (p_cmd->emergency_stop || (p_cmd->sys_mode == ARA_MODE_ERROR)) {
        s_motion_ctx.hw_status = ARA_ERROR;
        Motion_SetState(MOTION_STATE_ERROR);
    }

    /* ---------------- Sensor pipeline ---------------- */
    current_angle = DrvAS5600_GetRawAngle(&s_motion_ctx.enc_driver);
    current_vel   = Motion_CalcVelocity(current_angle, s_motion_ctx.prev_angle_raw);
    s_motion_ctx.prev_angle_raw = current_angle;

    trigger_status = DrvAS5600_TriggerUpdate(&s_motion_ctx.enc_driver);
    Motion_UpdateSensorHealth(trigger_status);

    /* ---------------- Core state machine ---------------- */
    switch (s_motion_ctx.state)
    {
        case MOTION_STATE_BOOT:
        {
            Motion_StopPowerStage();

            if (p_cmd->sys_mode == ARA_MODE_INIT) {
                Motion_ResetCalibration();
                break;
            }

            if ((p_cmd->sys_mode == ARA_MODE_MANUAL) ||
                (p_cmd->sys_mode == ARA_MODE_AUTO))
            {
                if (s_motion_ctx.state_ticks >= MOTION_BOOT_PRIME_TICKS) {
                    DrvBldc_Enable(&s_motion_ctx.motor_driver, true);
                    Motion_SetState(MOTION_STATE_FOC_ALIGN);
                }
            }
            break;
        }

        case MOTION_STATE_FOC_ALIGN:
        {
            AraStatus_t apply_status;

            if (p_cmd->sys_mode == ARA_MODE_INIT) {
                Motion_StopPowerStage();
                Motion_ResetCalibration();
                Motion_SetState(MOTION_STATE_BOOT);
                break;
            }

            DrvBldc_Enable(&s_motion_ctx.motor_driver, true);

            /* Fixed stator field for electrical zero alignment. */
            AlgFoc_Run(&s_motion_ctx.foc_algo, 0U, 0, MOTION_ALIGN_VOLTAGE_D_Q15);
            apply_status = Motion_ApplyFocOutputs();

            if (apply_status != ARA_OK) {
                s_motion_ctx.hw_status = apply_status;
                Motion_SetState(MOTION_STATE_ERROR);
                break;
            }

            if (s_motion_ctx.state_ticks >= MOTION_ALIGN_DURATION_TICKS) {
                s_motion_ctx.foc_zero_offset = current_angle;
                AlgFoc_SetZeroOffset(&s_motion_ctx.foc_algo, s_motion_ctx.foc_zero_offset);
                s_motion_ctx.is_foc_aligned = true;
                s_motion_ctx.homing_phase   = MOTION_HOME_PHASE_SEEK_LIMIT;
                s_motion_ctx.plateau_ticks  = 0U;
                AlgPid_Reset(&s_motion_ctx.pos_pid);
                Motion_SetState(MOTION_STATE_HOMING);
            }
            break;
        }

        case MOTION_STATE_HOMING:
        {
            AraStatus_t apply_status;

            if (p_cmd->sys_mode == ARA_MODE_INIT) {
                Motion_StopPowerStage();
                Motion_ResetCalibration();
                Motion_SetState(MOTION_STATE_BOOT);
                break;
            }

            if (!s_motion_ctx.is_foc_aligned) {
                s_motion_ctx.hw_status = ARA_ERROR;
                Motion_SetState(MOTION_STATE_ERROR);
                break;
            }

            if (s_motion_ctx.state_ticks >= MOTION_HOME_TIMEOUT_TICKS) {
                s_motion_ctx.hw_status = ARA_TIMEOUT;
                Motion_SetState(MOTION_STATE_ERROR);
                break;
            }

            DrvBldc_Enable(&s_motion_ctx.motor_driver, true);

            if (s_motion_ctx.homing_phase == MOTION_HOME_PHASE_SEEK_LIMIT) {
                /* Low-torque seek toward the mechanical hard stop. */
                AlgFoc_Run(&s_motion_ctx.foc_algo, current_angle, MOTION_HOME_SEEK_UQ_Q15, 0);
                apply_status = Motion_ApplyFocOutputs();

                if (apply_status != ARA_OK) {
                    s_motion_ctx.hw_status = apply_status;
                    Motion_SetState(MOTION_STATE_ERROR);
                    break;
                }

                if (Motion_Abs32((int32_t)current_vel) <= MOTION_HOME_PLATEAU_VEL_THD) {
                    s_motion_ctx.plateau_ticks++;
                } else {
                    s_motion_ctx.plateau_ticks = 0U;
                }

                if (s_motion_ctx.plateau_ticks >= MOTION_HOME_PLATEAU_HOLD_TICKS) {
                    s_motion_ctx.home_angle_raw = current_angle;
                    s_motion_ctx.is_homed       = true;
                    s_motion_ctx.home_backoff_target_raw =
                            Motion_WrapRawAngle((int32_t)s_motion_ctx.home_angle_raw +
                                                (MOTION_HOME_BACKOFF_DIR * (int32_t)MOTION_HOME_BACKOFF_COUNTS));
                    s_motion_ctx.homing_phase  = MOTION_HOME_PHASE_BACKOFF;
                    s_motion_ctx.plateau_ticks = 0U;
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                }
            } else {
                int32_t backoff_error;
                int16_t backoff_uq;

                backoff_error = Motion_CalcShortestPathError(s_motion_ctx.home_backoff_target_raw,
                                                             current_angle);
                backoff_uq    = Motion_RunPositionController(backoff_error);

                AlgFoc_Run(&s_motion_ctx.foc_algo, current_angle, backoff_uq, 0);
                apply_status = Motion_ApplyFocOutputs();

                if (apply_status != ARA_OK) {
                    s_motion_ctx.hw_status = apply_status;
                    Motion_SetState(MOTION_STATE_ERROR);
                    break;
                }

                if (Motion_Abs32(backoff_error) <= MOTION_POS_NEAR_COUNTS) {
                    Motion_StopPowerStage();
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetState(MOTION_STATE_READY);
                }
            }
            break;
        }

        case MOTION_STATE_READY:
        {
            Motion_StopPowerStage();

            if (p_cmd->sys_mode == ARA_MODE_INIT) {
                Motion_ResetCalibration();
                Motion_SetState(MOTION_STATE_BOOT);
                break;
            }

            if ((p_cmd->sys_mode == ARA_MODE_MANUAL) ||
                (p_cmd->sys_mode == ARA_MODE_AUTO))
            {
                if (s_motion_ctx.is_foc_aligned &&
                    s_motion_ctx.is_homed &&
                    (s_motion_ctx.state_ticks >= MOTION_READY_SETTLE_TICKS))
                {
                    DrvBldc_Enable(&s_motion_ctx.motor_driver, true);
                    AlgPid_Reset(&s_motion_ctx.pos_pid);
                    Motion_SetState(MOTION_STATE_CLOSED_LOOP);
                }
            }
            break;
        }

        case MOTION_STATE_CLOSED_LOOP:
        {
            AraStatus_t apply_status;
            int32_t     pos_error_counts;
            int16_t     torque_uq;

            if (p_cmd->sys_mode == ARA_MODE_INIT) {
                Motion_StopPowerStage();
                Motion_ResetCalibration();
                Motion_SetState(MOTION_STATE_BOOT);
                break;
            }

            if (p_cmd->sys_mode == ARA_MODE_IDLE) {
                Motion_StopPowerStage();
                Motion_SetState(MOTION_STATE_READY);
                break;
            }

            if ((!s_motion_ctx.is_foc_aligned) || (!s_motion_ctx.is_homed)) {
                s_motion_ctx.hw_status = ARA_ERROR;
                Motion_SetState(MOTION_STATE_ERROR);
                break;
            }

            DrvBldc_Enable(&s_motion_ctx.motor_driver, true);

            /* Compatibility mode:
             * target_foc_angle is still interpreted as a raw AS5600 target angle.
             * The captured home reference is already available for the next-step
             * migration to mechanism-coordinate control.
             */
            pos_error_counts = Motion_CalcShortestPathError(p_cmd->target_foc_angle,
                                                            current_angle);
            torque_uq = Motion_RunPositionController(pos_error_counts);

            AlgFoc_Run(&s_motion_ctx.foc_algo, current_angle, torque_uq, 0);
            apply_status = Motion_ApplyFocOutputs();

            if (apply_status != ARA_OK) {
                s_motion_ctx.hw_status = apply_status;
                Motion_SetState(MOTION_STATE_ERROR);
            }
            break;
        }

        case MOTION_STATE_ERROR:
        default:
        {
            Motion_StopPowerStage();
            AlgPid_Reset(&s_motion_ctx.pos_pid);

            if ((!p_cmd->emergency_stop) && (p_cmd->sys_mode == ARA_MODE_INIT)) {
                s_motion_ctx.hw_status = ARA_OK;
                Motion_ResetCalibration();
                Motion_SetState(MOTION_STATE_BOOT);
            }
            break;
        }
    }

    /* ---------------- Uplink feedback to the low-frequency thread ---------------- */
    p_state->current_foc_angle = current_angle;
    p_state->current_velocity  = current_vel;
    p_state->foc_status        = s_motion_ctx.hw_status;
}