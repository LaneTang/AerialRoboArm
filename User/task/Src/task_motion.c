/**
 * @file task_motion.c
 * @brief Motion Control Logic (L4) Implementation
 * @note  Implements the 1000Hz FOC closed-loop control runnable.
 * STRICTLY NON-BLOCKING. Zero RTOS dependencies.
 * @author ARA Project Coder
 */

#include "task_motion.h"
#include "bsp_gpio.h"    /* For Motor Enable/Disable Definitions */
#include "bsp_pwm.h"     /* For BSP_PWM_MAX_DUTY */
#include <stddef.h>

/* =========================================================
 * 1. Internal Macros & Constants
 * ========================================================= */

#define MOTOR_POLE_PAIRS        (7)     /* 2804/2808 Motor pole pairs */
#define ALIGN_VOLTAGE_Q15       (4915)  /* ~15% of full-scale voltage for alignment */
#define ALIGN_DURATION_TICKS    (2000)  /* 2000 ticks @ 1000Hz = 2.0 Seconds */

/* Default PID Gains for Position Loop (Format: Q15 Fixed Point) */
#define PID_POS_KP              (150)
#define PID_POS_KI              (5)
#define PID_POS_KD              (20)

/* =========================================================
 * 2. Static Context Allocation
 * ========================================================= */

/* Singleton context for the motion task (Avoids dynamic allocation) */
static TaskMotion_Context_t s_motion_ctx;

/* =========================================================
 * 3. Private Helper Functions
 * ========================================================= */

/**
 * @brief Calculate rotational velocity with wrap-around handling
 * @param curr Current AS5600 raw angle (0-4095)
 * @param prev Previous AS5600 raw angle (0-4095)
 * @return Delta counts per millisecond
 */
static int16_t CalculateVelocity(uint16_t curr, uint16_t prev)
{
    int32_t diff = (int32_t)curr - (int32_t)prev;

    /* Handle 12-bit circular wrap-around (Midpoint is 2048) */
    if (diff > 2048) {
        diff -= 4096;
    } else if (diff < -2048) {
        diff += 4096;
    }

    return (int16_t)diff;
}

/**
 * @brief Calculate the shortest path position error
 * @param target Target raw angle
 * @param actual Current raw angle
 * @return Shortest path error (-2048 to +2047)
 */
static int32_t CalculateShortestPathError(uint16_t target, uint16_t actual)
{
    int32_t error = (int32_t)target - (int32_t)actual;

    if (error > 2048) {
        error -= 4096;
    } else if (error < -2048) {
        error += 4096;
    }

    return error;
}

/* =========================================================
 * 4. API Implementation
 * ========================================================= */

void TaskMotion_Init(void)
{
    /* 1. Reset Context */
    s_motion_ctx.state = MOTION_STATE_IDLE;
    s_motion_ctx.tick_count = 0;
    s_motion_ctx.prev_angle_raw = 0;

    /* 2. Initialize L2 Hardware Drivers */
    /* Note: Passing NULL for ISR callback because we rely on asynchronous DMA polling now */
    DrvAS5600_Init(&s_motion_ctx.enc_driver, BSP_I2C_MOTION, NULL);
    DrvBldc_Init(&s_motion_ctx.motor_driver, BSP_GPIO_MOTOR_EN);

    /* 3. Initialize L3 FOC Algorithm */
    AlgFoc_Init(&s_motion_ctx.foc_algo, MOTOR_POLE_PAIRS, BSP_PWM_MAX_DUTY);

    /* 4. Initialize L3 Position PID Controller */
    AlgPid_Init(&s_motion_ctx.pos_pid);
    AlgPid_SetGains(&s_motion_ctx.pos_pid,
                    PID_POS_KP, PID_POS_KI, PID_POS_KD,
                    BSP_PWM_MAX_DUTY, BSP_PWM_MAX_DUTY);
}

void TaskMotion_Update(const DataHub_Cmd_t *p_cmd, DataHub_State_t *p_state)
{
    if (p_cmd == NULL || p_state == NULL) {
        return; /* Invalid pointer protection */
    }

    s_motion_ctx.tick_count++;

    /* ====================================================
     * STAGE 1: Absolute Safety Preemption
     * ==================================================== */
    if (p_cmd->emergency_stop || p_cmd->sys_mode == ARA_MODE_ERROR) {
        s_motion_ctx.state = MOTION_STATE_ERROR;
    }

    /* ====================================================
     * STAGE 2: Sensor Pipeline (Non-Blocking)
     * ==================================================== */
    /* A. Fetch the angle that DMA retrieved during the LAST 1ms tick */
    uint16_t current_angle = DrvAS5600_GetRawAngle(&s_motion_ctx.enc_driver);

    /* B. Calculate Velocity */
    int16_t current_vel = CalculateVelocity(current_angle, s_motion_ctx.prev_angle_raw);
    s_motion_ctx.prev_angle_raw = current_angle;

    /* C. Trigger the NEXT DMA read over I2C (Fire and Forget) */
    AraStatus_t bus_status = DrvAS5600_TriggerUpdate(&s_motion_ctx.enc_driver);

    /* ====================================================
     * STAGE 3: Core State Machine (Control Law Calculation)
     * ==================================================== */
    switch (s_motion_ctx.state)
    {
        case MOTION_STATE_IDLE:
            /* Relax Actuators */
            DrvBldc_Enable(&s_motion_ctx.motor_driver, false);

            /* Start Alignment if system moves out of INIT/IDLE */
            if (p_cmd->sys_mode == ARA_MODE_AUTO || p_cmd->sys_mode == ARA_MODE_MANUAL) {
                if (s_motion_ctx.tick_count > 500) { // 500ms stabilization delay
                    DrvBldc_Enable(&s_motion_ctx.motor_driver, true);
                    s_motion_ctx.state = MOTION_STATE_ALIGNMENT;
                    s_motion_ctx.tick_count = 0; // Reset counter for alignment timer
                }
            }
            break;

        case MOTION_STATE_ALIGNMENT:
            /* Force U_d voltage vector to lock the rotor to electrical zero */
            AlgFoc_Run(&s_motion_ctx.foc_algo, 0, 0, ALIGN_VOLTAGE_Q15);
            DrvBldc_SetDuties(&s_motion_ctx.motor_driver,
                              s_motion_ctx.foc_algo.duty_a,
                              s_motion_ctx.foc_algo.duty_b,
                              s_motion_ctx.foc_algo.duty_c);

            /* Wait for the mechanical rotor to settle */
            if (s_motion_ctx.tick_count >= ALIGN_DURATION_TICKS) {
                /* Calibrate zero offset based on settled physical angle */
                AlgFoc_SetZeroOffset(&s_motion_ctx.foc_algo, current_angle);
                AlgPid_Reset(&s_motion_ctx.pos_pid);
                s_motion_ctx.state = MOTION_STATE_CLOSED_LOOP;
            }
            break;

        case MOTION_STATE_CLOSED_LOOP:
            /* Return to IDLE if requested */
            if (p_cmd->sys_mode == ARA_MODE_IDLE) {
                s_motion_ctx.state = MOTION_STATE_IDLE;
                break;
            }

            /* 1. Calculate Error (Shortest path handling) */
            int32_t pos_error = CalculateShortestPathError(p_cmd->target_foc_angle, current_angle);

            /* 2. Run Position PID (Target = pos_error, Actual = 0)
             * This tricks the standard PID into executing correctly without wrapping issues internally */
            int16_t torque_uq = AlgPid_Compute(&s_motion_ctx.pos_pid, pos_error, 0);

            /* 3. Execute FOC Math */
            AlgFoc_Run(&s_motion_ctx.foc_algo, current_angle, torque_uq, 0);

            /* 4. Update PWM Hardware */
            DrvBldc_SetDuties(&s_motion_ctx.motor_driver,
                              s_motion_ctx.foc_algo.duty_a,
                              s_motion_ctx.foc_algo.duty_b,
                              s_motion_ctx.foc_algo.duty_c);
            break;

        case MOTION_STATE_ERROR:
        default:
            /* CRITICAL: Hardware cutoff */
            DrvBldc_Enable(&s_motion_ctx.motor_driver, false);
            break;
    }

    /* ====================================================
     * STAGE 4: Feedback to Low-Freq Logic Thread
     * ==================================================== */
    p_state->current_foc_angle = current_angle;
    p_state->current_velocity  = current_vel;
    p_state->foc_status        = bus_status; // Report I2C DMA health
}