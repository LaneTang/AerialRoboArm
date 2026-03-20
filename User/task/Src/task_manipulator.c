/**
 * @file task_manipulator.c
 * @brief Arm Manipulator Brain & State Machine (L4) Implementation
 * @note  Strictly C99. No FPU. Zero RTOS dependencies.
 * @author ARA Project Coder
 */

#include "task_manipulator.h"
#include "mod_actuator.h" /* For Gripper and Roll servo commands */
#include <stddef.h>

/* =========================================================
 * 1. Business Logic Macros & Parameters
 * ========================================================= */

#define EXT_DEADBAND_MM         (5)     // Z-Axis error deadband (+/- 5mm)
#define ROLL_DEADBAND_Q15       (600)   // Roll error deadband (Approx 2 degrees)

/* Time parameters scaled to 50Hz (20ms per tick) */
#define GRAB_DEBOUNCE_TICKS     (10)    // 10 ticks * 20ms = 200ms stable required
#define GRAB_WAIT_TICKS         (25)    // 25 ticks * 20ms = 500ms gripper close time

/* LPF Alpha (0-256) -> Smaller = smoother but slower tracking */
#define LPF_ALPHA               (50)

/* =========================================================
 * 2. Static Context & Dependencies
 * ========================================================= */
static TaskManipulator_Context_t s_manip_ctx;
static uint32_t s_task_tick_counter = 0; // Internal 50Hz tick reference

/* [FIXED] 实例化末端执行器上下文，由 Manipulator 任务全权拥有和管理 */
static ModActuator_Context_t s_actuator_ctx;

/* =========================================================
 * 3. Z-Axis Inverse Kinematics (LUT + Linear Interpolation)
 * ========================================================= */
typedef struct {
    uint16_t dist_mm;
    uint16_t angle_raw;
} LutPoint_t;

/**
 * @brief Offline Calibration Table (Z-Distance to AS5600 Angle)
 * @note  MUST be strictly monotonic ascending.
 */
static const LutPoint_t Z_TO_ANGLE_LUT[] = {
        {120, 850},   // Folded Home
        {150, 1200},
        {180, 1650},
        {210, 2100},
        {240, 2600},
        {270, 3150},
        {300, 3700}   // Max Extension
};
#define LUT_SIZE (sizeof(Z_TO_ANGLE_LUT) / sizeof(LutPoint_t))

static uint16_t Map_DistanceToAngle(uint16_t target_mm)
{
    /* Boundary Clamping */
    if (target_mm <= Z_TO_ANGLE_LUT[0].dist_mm) return Z_TO_ANGLE_LUT[0].angle_raw;
    if (target_mm >= Z_TO_ANGLE_LUT[LUT_SIZE - 1].dist_mm) return Z_TO_ANGLE_LUT[LUT_SIZE - 1].angle_raw;

    /* Integer Linear Interpolation */
    for (uint8_t i = 0; i < LUT_SIZE - 1; i++) {
        if (target_mm >= Z_TO_ANGLE_LUT[i].dist_mm && target_mm <= Z_TO_ANGLE_LUT[i+1].dist_mm) {
            int32_t x0 = Z_TO_ANGLE_LUT[i].dist_mm;
            int32_t x1 = Z_TO_ANGLE_LUT[i+1].dist_mm;
            int32_t y0 = Z_TO_ANGLE_LUT[i].angle_raw;
            int32_t y1 = Z_TO_ANGLE_LUT[i+1].angle_raw;

            int32_t interpolated = y0 + ((y1 - y0) * (target_mm - x0)) / (x1 - x0);
            return (uint16_t)interpolated;
        }
    }
    return Z_TO_ANGLE_LUT[0].angle_raw; // Fallback
}

static inline uint16_t IntAbs_U16(int16_t val) {
    return (val < 0) ? -val : val;
}

/* =========================================================
 * 4. Task Initialization & API
 * ========================================================= */

void TaskManipulator_Init(void)
{
    /* 初始化内部状态 */
    s_manip_ctx.current_state = MANIP_STATE_IDLE;
    s_manip_ctx.converge_start_tick = 0;
    s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
    s_manip_ctx.filtered_roll = 0;
    s_task_tick_counter = 0;

    /* [FIXED] 初始化底层的末端执行器驱动硬件 */
    ModActuator_Init(&s_actuator_ctx);
}

void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const AraVisionData_t *p_vision_data,
                            DataHub_Cmd_t *p_out_cmd)
{
    /* Parameter Guard */
    if (p_rc_intent == NULL || p_vision_data == NULL || p_out_cmd == NULL) {
        return;
    }

    s_task_tick_counter++;

    /* ----------------------------------------------------
     * STAGE 1: Global Safety & Mode Overrides
     * ---------------------------------------------------- */
    /* Check E-Stop from RC or PC Vision */
    if (p_rc_intent->estop_state == ESTOP_ACTIVE || p_vision_data->pc_estop_req || !p_rc_intent->is_link_up) {
        s_manip_ctx.current_state = MANIP_STATE_ERROR_SAFE;
    }
        /* If RC requests manual or idle, abort autonomous state machine */
    else if (p_rc_intent->req_mode != ARA_MODE_AUTO && s_manip_ctx.current_state != MANIP_STATE_IDLE) {
        s_manip_ctx.current_state = MANIP_STATE_IDLE;
    }

    /* Set default passthrough outputs */
    p_out_cmd->emergency_stop = false;
    p_out_cmd->sys_mode = p_rc_intent->req_mode; // Reflect RC intent down to FOC

    /* ----------------------------------------------------
     * STAGE 2: Autonomous State Machine (50Hz Tick)
     * ---------------------------------------------------- */
    switch (s_manip_ctx.current_state)
    {
        case MANIP_STATE_IDLE:
            /* Open Gripper, Command Home Position */
            ModActuator_SetGripper(&s_actuator_ctx, 0); // 0% Open
            s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
            p_out_cmd->target_foc_angle = Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm);

            /* Transition -> SEEKING */
            if (p_rc_intent->req_mode == ARA_MODE_AUTO && p_vision_data->is_tracking && p_vision_data->is_grabbable) {
                s_manip_ctx.current_state = MANIP_STATE_SEEKING;
            }
            break;

        case MANIP_STATE_SEEKING:
            /* Check if target lost */
            if (!p_vision_data->is_tracking) {
                s_manip_ctx.current_state = MANIP_STATE_IDLE;
                break;
            }

            /* Execute Integer LPF */
            s_manip_ctx.filtered_ext_mm = (p_vision_data->target_dist_mm * LPF_ALPHA +
                                           s_manip_ctx.filtered_ext_mm * (256 - LPF_ALPHA)) >> 8;
            s_manip_ctx.filtered_roll   = (p_vision_data->target_roll * LPF_ALPHA +
                                           s_manip_ctx.filtered_roll * (256 - LPF_ALPHA)) >> 8;

            /* Output computed targets */
            p_out_cmd->target_foc_angle = Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm);
            ModActuator_SetRoll(&s_actuator_ctx, (uint8_t)(s_manip_ctx.filtered_roll / 100));

            /* Deadband Check: Has the LPF output converged with the raw vision target? */
            uint16_t z_error = IntAbs_U16((int16_t)p_vision_data->target_dist_mm - (int16_t)s_manip_ctx.filtered_ext_mm);
            if (z_error <= EXT_DEADBAND_MM) {
                s_manip_ctx.converge_start_tick = s_task_tick_counter;
                s_manip_ctx.current_state = MANIP_STATE_CONVERGING;
            }
            break;

        case MANIP_STATE_CONVERGING:
            /* Keep LPF updating */
            s_manip_ctx.filtered_ext_mm = (p_vision_data->target_dist_mm * LPF_ALPHA +
                                           s_manip_ctx.filtered_ext_mm * (256 - LPF_ALPHA)) >> 8;
            p_out_cmd->target_foc_angle = Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm);

            z_error = IntAbs_U16((int16_t)p_vision_data->target_dist_mm - (int16_t)s_manip_ctx.filtered_ext_mm);

            /* Break condition: target moved away or is no longer grabbable */
            if (z_error > EXT_DEADBAND_MM || !p_vision_data->is_grabbable) {
                s_manip_ctx.current_state = MANIP_STATE_SEEKING;
            }
                /* Success condition: remained stable for required ticks */
            else if ((s_task_tick_counter - s_manip_ctx.converge_start_tick) >= GRAB_DEBOUNCE_TICKS) {
                s_manip_ctx.converge_start_tick = s_task_tick_counter; // Reuse timer for grabbing
                s_manip_ctx.current_state = MANIP_STATE_GRABBING;
            }
            break;

        case MANIP_STATE_GRABBING:
            /* Blind execution: Ignore vision updates, lock position */
            p_out_cmd->target_foc_angle = Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm);
            ModActuator_SetGripper(&s_actuator_ctx, 100); // 100% Closed

            if ((s_task_tick_counter - s_manip_ctx.converge_start_tick) >= GRAB_WAIT_TICKS) {
                s_manip_ctx.current_state = MANIP_STATE_RETRACTING;
                s_manip_ctx.converge_start_tick = s_task_tick_counter;
            }
            break;

        case MANIP_STATE_RETRACTING:
            /* Keep gripper closed, pull back to home */
            s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
            p_out_cmd->target_foc_angle = Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm);
            ModActuator_SetGripper(&s_actuator_ctx, 100);

            /* Wait sufficient time for retract, then go to IDLE to drop or hold */
            if ((s_task_tick_counter - s_manip_ctx.converge_start_tick) >= GRAB_WAIT_TICKS) {
                s_manip_ctx.current_state = MANIP_STATE_IDLE;
            }
            break;

        case MANIP_STATE_ERROR_SAFE:
            p_out_cmd->emergency_stop = true;
            p_out_cmd->target_foc_angle = Z_TO_ANGLE_LUT[0].angle_raw; // Attempt software retract
            ModActuator_SetGripper(&s_actuator_ctx, 0); // Drop payload
            /* Can only escape ERROR state if RC resets the mode (handled in Stage 1) */
            break;

        default:
            s_manip_ctx.current_state = MANIP_STATE_IDLE;
            break;
    }
}