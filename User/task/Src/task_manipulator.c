/**
 * @file task_manipulator.c
 * @brief Arm Manipulator Brain & State Machine Task (L4)
 * @note  Strictly C99. No FPU. FreeRTOS dependent.
 */

#include "task_manipulator.h"
#include "datahub.h"
#include "datahub_vision_ext.h"
#include "mod_actuator.h"
#include "alg_voltage_foc.h" // Assuming L3 FOC target API is accessible or mediated
#include "FreeRTOS.h"
#include "task.h"

/* =========================================================
 * 1. Business Logic Macros & Parameters
 * ========================================================= */
#define TASK_PERIOD_MS          (20)    // 50Hz Execution Rate

/* Deadband & Debounce */
#define EXT_DEADBAND_MM         (5)     // Z-Axis error deadband (+/- 5mm)
#define ROLL_DEADBAND_Q15       (600)   // Roll error deadband (Approx 2 degrees in Q15)
#define GRAB_DEBOUNCE_MS        (200)   // Must stay in deadband for 200ms

/* LPF Alpha (0-256) -> Smaller = smoother but slower tracking */
#define LPF_ALPHA               (50)

/* Safety Watchdog */
#define VISION_TIMEOUT_MS       (500)   // Comm loss threshold

/* =========================================================
 * 2. Static Context & External L3 Handles
 * ========================================================= */
static TaskManipulator_Context_t s_manip_ctx;

/* Assuming actuator module is initialized and accessible here */
extern ModActuator_Context_t g_actuator_ctx;
/* Assuming a function to set the FOC motor target angle exists in L3 */
extern void Motion_SetTargetAngle(uint16_t target_raw_angle);
extern uint16_t Motion_GetCurrentAngle(void);

/* =========================================================
 * 3. Z-Axis Inverse Kinematics (LUT + Linear Interpolation)
 * ========================================================= */
typedef struct {
    uint16_t dist_mm;
    uint16_t angle_raw;
} LutPoint_t;

/**
 * @brief Offline Calibration Table (Z-Distance to AS5600 Angle)
 * @note  MUST be strictly monotonic (ascending by dist_mm).
 * Example data: Modify based on your physical calibration!
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

/**
 * @brief Linear interpolation to find Target Angle from Target MM
 */
static uint16_t Map_DistanceToAngle(uint16_t target_mm)
{
    /* Boundary Clamping */
    if (target_mm <= Z_TO_ANGLE_LUT[0].dist_mm) return Z_TO_ANGLE_LUT[0].angle_raw;
    if (target_mm >= Z_TO_ANGLE_LUT[LUT_SIZE - 1].dist_mm) return Z_TO_ANGLE_LUT[LUT_SIZE - 1].angle_raw;

    /* Find the bounding interval */
    for (uint8_t i = 0; i < LUT_SIZE - 1; i++) {
        if (target_mm >= Z_TO_ANGLE_LUT[i].dist_mm && target_mm <= Z_TO_ANGLE_LUT[i+1].dist_mm) {

            int32_t x0 = Z_TO_ANGLE_LUT[i].dist_mm;
            int32_t x1 = Z_TO_ANGLE_LUT[i+1].dist_mm;
            int32_t y0 = Z_TO_ANGLE_LUT[i].angle_raw;
            int32_t y1 = Z_TO_ANGLE_LUT[i+1].angle_raw;

            /* Integer Linear Interpolation: Y = Y0 + (Y1 - Y0) * (X - X0) / (X1 - X0) */
            int32_t interpolated = y0 + ((y1 - y0) * (target_mm - x0)) / (x1 - x0);
            return (uint16_t)interpolated;
        }
    }
    return Z_TO_ANGLE_LUT[0].angle_raw; // Fallback
}

/**
 * @brief Helper: Fast absolute value for integers
 */
static inline uint16_t IntAbs_U16(int16_t val) {
    return (val < 0) ? -val : val;
}

/* =========================================================
 * 4. Task Initialization & Entry
 * ========================================================= */

void TaskManipulator_Init(void)
{
    s_manip_ctx.current_state = MANIP_STATE_IDLE;
    s_manip_ctx.converge_start_tick = 0;

    /* Initialize LPF with safe home values */
    s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
    s_manip_ctx.filtered_roll = 0; // Assuming 0 is neutral
}

void TaskManipulator_Entry(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    AraVisionData_t vision_data;

    for (;;)
    {
        /* 1. Global Mode Check & Watchdog */
        AraSysMode_t current_mode = DataHub_GetSysMode();
        uint32_t last_vision_tick = DataHub_GetLastVisionTick();
        uint32_t current_tick = xTaskGetTickCount();

        /* Condition A: System is not in Auto Mode */
        if (current_mode != ARA_MODE_AUTO) {
            s_manip_ctx.current_state = MANIP_STATE_IDLE;
            s_manip_ctx.converge_start_tick = 0;
            goto TASK_DELAY; // Yield execution
        }

        /* Condition B: Vision Link Lost (Watchdog Timeout) */
        if ((current_tick - last_vision_tick) > pdMS_TO_TICKS(VISION_TIMEOUT_MS)) {
            s_manip_ctx.current_state = MANIP_STATE_ERROR_SAFE;
        }

        /* 2. Read Latest Perception Data */
        DataHub_ReadVisionData(&vision_data);

        /* 3. Core State Machine */
        switch (s_manip_ctx.current_state)
        {
            case MANIP_STATE_IDLE:
                /* Open Gripper, Move to Home */
                ModActuator_SetGripper(&g_actuator_ctx, 0);
                s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
                Motion_SetTargetAngle(Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm));

                if (vision_data.is_tracking && vision_data.is_grabbable) {
                    s_manip_ctx.current_state = MANIP_STATE_SEEKING;
                }
                break;

            case MANIP_STATE_SEEKING:
                /* Lost track during seeking? Abort. */
                if (!vision_data.is_tracking || vision_data.pc_estop_req) {
                    s_manip_ctx.current_state = MANIP_STATE_IDLE;
                    break;
                }

                /* Execute Integer LPF to smooth the 20Hz vision target */
                s_manip_ctx.filtered_ext_mm = (vision_data.target_dist_mm * LPF_ALPHA +
                                               s_manip_ctx.filtered_ext_mm * (256 - LPF_ALPHA)) >> 8;

                s_manip_ctx.filtered_roll = (vision_data.target_roll * LPF_ALPHA +
                                             s_manip_ctx.filtered_roll * (256 - LPF_ALPHA)) >> 8;

                /* Output mapped targets to L3 */
                Motion_SetTargetAngle(Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm));
                ModActuator_SetRoll(&g_actuator_ctx, (uint8_t)(s_manip_ctx.filtered_roll / 100)); // Assuming scaling

                /* Deadband Check (Simulated Roll Check, Focus on Z-Axis) */
                uint16_t current_angle = Motion_GetCurrentAngle();
                uint16_t target_angle  = Map_DistanceToAngle(vision_data.target_dist_mm);

                // Roughly map deadband MM to deadband Angle (Simple approximation for condition)
                uint16_t angle_deadband = (Z_TO_ANGLE_LUT[LUT_SIZE-1].angle_raw - Z_TO_ANGLE_LUT[0].angle_raw) /
                                          (Z_TO_ANGLE_LUT[LUT_SIZE-1].dist_mm - Z_TO_ANGLE_LUT[0].dist_mm) * EXT_DEADBAND_MM;

                if (IntAbs_U16((int16_t)current_angle - (int16_t)target_angle) <= angle_deadband) {
                    s_manip_ctx.converge_start_tick = current_tick;
                    s_manip_ctx.current_state = MANIP_STATE_CONVERGING;
                }
                break;

            case MANIP_STATE_CONVERGING:
                /* Keep LPF updating to follow micro-movements */
                s_manip_ctx.filtered_ext_mm = (vision_data.target_dist_mm * LPF_ALPHA +
                                               s_manip_ctx.filtered_ext_mm * (256 - LPF_ALPHA)) >> 8;
                Motion_SetTargetAngle(Map_DistanceToAngle(s_manip_ctx.filtered_ext_mm));

                /* Re-check error. If target moves away, break back to seeking */
                current_angle = Motion_GetCurrentAngle();
                target_angle  = Map_DistanceToAngle(vision_data.target_dist_mm);
                angle_deadband = (Z_TO_ANGLE_LUT[LUT_SIZE-1].angle_raw - Z_TO_ANGLE_LUT[0].angle_raw) /
                                 (Z_TO_ANGLE_LUT[LUT_SIZE-1].dist_mm - Z_TO_ANGLE_LUT[0].dist_mm) * EXT_DEADBAND_MM;

                if (IntAbs_U16((int16_t)current_angle - (int16_t)target_angle) > angle_deadband || !vision_data.is_grabbable) {
                    s_manip_ctx.current_state = MANIP_STATE_SEEKING;
                }
                    /* If stable inside deadband for X ms, GRAB! */
                else if ((current_tick - s_manip_ctx.converge_start_tick) >= pdMS_TO_TICKS(GRAB_DEBOUNCE_MS)) {
                    s_manip_ctx.current_state = MANIP_STATE_GRABBING;
                    s_manip_ctx.converge_start_tick = current_tick; // Re-use as grab timer
                }
                break;

            case MANIP_STATE_GRABBING:
                /* Blind execution: Ignore vision updates here */
                ModActuator_SetGripper(&g_actuator_ctx, 100); // 100% Closed

                /* Wait 500ms for physical servo to close */
                if ((current_tick - s_manip_ctx.converge_start_tick) >= pdMS_TO_TICKS(500)) {
                    s_manip_ctx.current_state = MANIP_STATE_RETRACTING;
                }
                break;

            case MANIP_STATE_RETRACTING:
                /* Pull arm back to home while keeping gripper closed */
                s_manip_ctx.filtered_ext_mm = Z_TO_ANGLE_LUT[0].dist_mm;
                Motion_SetTargetAngle(Z_TO_ANGLE_LUT[0].angle_raw);

                /* Optional: Check if reached home, then IDLE or keep holding */
                if (IntAbs_U16((int16_t)Motion_GetCurrentAngle() - (int16_t)Z_TO_ANGLE_LUT[0].angle_raw) < 50) {
                    // Logic dictates whether to drop it or wait. Let's return to IDLE for next cycle.
                    s_manip_ctx.current_state = MANIP_STATE_IDLE;
                }
                break;

            case MANIP_STATE_ERROR_SAFE:
                /* Triggered by E-STOP or Link Loss */
                Motion_SetTargetAngle(Z_TO_ANGLE_LUT[0].angle_raw); // Force Retract
                ModActuator_SetGripper(&g_actuator_ctx, 0);         // Drop payload

                /* Can only exit via Mode change (System Reset by L5) */
                break;

            default:
                s_manip_ctx.current_state = MANIP_STATE_IDLE;
                break;
        }

        TASK_DELAY:
        /* Absolute precision delay to maintain exact 50Hz task frequency */
        vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(TASK_PERIOD_MS));
    }
}