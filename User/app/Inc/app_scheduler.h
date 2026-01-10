/**
 * @file app_scheduler.h
 * @brief Top-Level Application State Machine
 * @note  Manages System Lifecycle (Init -> Calib -> Run -> Error)
 */

#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include "ara_def.h"

/* --- System States --- */
typedef enum {
    SYS_STATE_INIT = 0,      // Hardware Initialization
    SYS_STATE_CALIBRATION,   // Aligning Motor, finding Zero Offset
    SYS_STATE_IDLE,          // System Ready, PWM Off, Waiting for commands
    SYS_STATE_RUNNING,       // Active Control (Velocity/Position Mode)
    SYS_STATE_ERROR          // Fault Condition (Requires Reset)
} SysState_t;

/* --- API --- */

/**
 * @brief Initialize the Scheduler
 * @note  Creates all System Tasks (Motion, Comm, etc.)
 */
void App_Scheduler_Init(void);

/**
 * @brief Scheduler Task Entry Point (Low Priority)
 * @note  Monitors system health, handles state transitions, blinks LEDs.
 */
void App_Scheduler_Entry(void *argument);

/**
 * @brief Request System State Transition
 * @param new_state Requested State
 * @return ARA_OK if transition allowed, ARA_ERROR if forbidden
 */
AraStatus_t App_Scheduler_SetState(SysState_t new_state);

/**
 * @brief Get Current System State
 */
SysState_t App_Scheduler_GetState(void);

#endif // APP_SCHEDULER_H