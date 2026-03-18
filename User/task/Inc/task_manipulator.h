/**
 * @file task_manipulator.h
 * @brief Arm Manipulator Brain & State Machine Task (L4)
 * @note  Reads DataHub, applies LPF (Low Pass Filter), manages the FSM,
 * and commands Task_Motion.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"

/* =========================================================
 * 1. State Machine Definitions
 * ========================================================= */

/**
 * @brief Core State Machine for Autonomous Grabbing
 */
typedef enum {
    MANIP_STATE_IDLE = 0,       // Folded, waiting for tracking signal
    MANIP_STATE_SEEKING,        // Target found, aligning Roll & adjusting Ext
    MANIP_STATE_CONVERGING,     // Position error within deadband, waiting for timer debounce
    MANIP_STATE_GRABBING,       // Closing gripper (ignoring new vision data)
    MANIP_STATE_RETRACTING,     // Pulling arm back to fold position
    MANIP_STATE_ERROR_SAFE      // E-Stop or Comm loss. Force retract/disable.
} ManipulatorState_t;

/* =========================================================
 * 2. Task Context Structure
 * ========================================================= */

/**
 * @brief Manipulator Task Context
 */
typedef struct {
    ManipulatorState_t current_state;

    /* Timers & Debouncing */
    uint32_t converge_start_tick; // Tick when position first reached deadband

    /* LPF (Low Pass Filter) State for Target Smoothing */
    int16_t  filtered_roll;       // Smoothed target roll
    uint16_t filtered_ext_mm;     // Smoothed target extension

} TaskManipulator_Context_t;

/* =========================================================
 * 3. Task API
 * ========================================================= */

/**
 * @brief Initialize the Manipulator Task Context
 * @note  Must be called BEFORE FreeRTOS scheduler starts.
 */
void TaskManipulator_Init(void);

/**
 * @brief The Main Manipulator Task Entry Point (FreeRTOS Task)
 * @param argument Task parameters (usually NULL)
 * @note  Typical execution period: 20ms (50Hz).
 * Blocking structure: vTaskDelayUntil().
 */
void TaskManipulator_Entry(void *argument);

#endif /* TASK_MANIPULATOR_H */