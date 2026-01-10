/**
 * @file task_motion.h
 * @brief Motion Control Task & State Machine (L4)
 * @note Manages the 1.5kHz FOC Loop
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_voltage_foc.h"
#include "alg_pid.h"

/* --- State Machine --- */
typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_ALIGNMENT, // Force vector to find zero
    MOTION_STATE_CLOSED_LOOP, // Normal running
    MOTION_STATE_ERROR      // Fault state
} MotionState_t;

/* --- Task Context --- */
typedef struct {
    // State
    MotionState_t state;
    uint32_t      tick_count;

    // Components (Aggregation)
    DrvAS5600_Context_t   enc_driver;
    DrvBldc_Context_t     motor_driver;
    AlgFoc_Context_t      foc_algo;
    AlgPid_Context_t      vel_pid;
    AlgPid_Context_t      pos_pid;

    // Control Targets
    int16_t target_velocity; // Q15
    int16_t target_position; // Q15 (if needed)
} TaskMotion_Context_t;

/* --- API --- */

/**
 * @brief Initialize all L2/L3 components and FreeRTOS resources
 */
void TaskMotion_Init(void);

/**
 * @brief The Main Task Entry Point (FreeRTOS Task Function)
 */
void TaskMotion_Entry(void *argument);

/**
 * @brief Set Target Velocity (Thread-safe interface for other tasks)
 * @param velocity_q15 Target velocity
 */
void TaskMotion_SetTargetVelocity(int16_t velocity_q15);

/**
 * @brief Emergency Stop - Instantly disables PWM and sets Error State
 */
void TaskMotion_EmergencyStop(void);

#endif // TASK_MOTION_H