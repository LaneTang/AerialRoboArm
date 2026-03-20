/**
 * @file task_manipulator.h
 * @brief Arm Manipulator Brain & State Machine (L4) - Downgraded to Runnable
 * @note  Executes the IK (Inverse Kinematics) and Seeking->Grabbing FSM.
 * Runs at 50Hz, called by Thread_LowFreq_Logic.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"
#include "datahub.h"          /* For RcControlData_t and DataHub_Cmd_t */
#include "mod_vsp_parser.h"   /* For AraVisionData_t */

/* --- State Machine --- */
typedef enum {
    MANIP_STATE_IDLE = 0,       // Folded, waiting for tracking signal
    MANIP_STATE_SEEKING,        // Target found, applying LPF to track
    MANIP_STATE_CONVERGING,     // Position error within deadband, debouncing
    MANIP_STATE_GRABBING,       // Actuating gripper (ignoring vision)
    MANIP_STATE_RETRACTING,     // Pulling arm back to fold position
    MANIP_STATE_ERROR_SAFE      // E-Stop. Force retract/disable.
} ManipulatorState_t;

/* --- Logic Context --- */
typedef struct {
    ManipulatorState_t current_state;
    uint32_t           converge_start_tick;

    /* LPF (Low Pass Filter) State for 20Hz -> 50Hz Target Smoothing */
    int16_t            filtered_roll;
    uint16_t           filtered_ext_mm;
} TaskManipulator_Context_t;

/* --- API --- */

/**
 * @brief Initialize the Manipulator State Machine context.
 */
void TaskManipulator_Init(void);

/**
 * @brief The 50Hz Execution Step.
 * @note  Called exclusively by Thread_LowFreq_Logic (L5).
 * Evaluates inputs and calculates the final target motor angles.
 * * @param p_rc_intent    [IN]  Parsed RC commands (Failsafe/Manual overrides)
 * @param p_vision_data  [IN]  Latest vision target (Distance/Roll, uses `is_grabbable`)
 * @param p_out_cmd      [OUT] The generated command to be sent to HighFreq Thread
 */
void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const AraVisionData_t *p_vision_data,
                            DataHub_Cmd_t *p_out_cmd);

#endif /* TASK_MANIPULATOR_H */