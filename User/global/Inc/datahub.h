/**
 * @file datahub.h
 * @brief Global Data Exchange Center (L4)
 * @note  Acts as the "Blackboard" for the Multi-Agent System.
 * All cross-task communication MUST go through this hub via thread-safe APIs.
 */

#ifndef DATAHUB_H
#define DATAHUB_H

#include "ara_def.h"

/* =========================================================
 * 1. Global System Enums
 * ========================================================= */

/**
 * @brief System Operation Modes (Controlled by L5 Scheduler)
 */
typedef enum {
    ARA_MODE_INIT = 0,      // System booting and calibrating
    ARA_MODE_IDLE,          // Safe state, actuators disabled, waiting for command
    ARA_MODE_MANUAL,        // RC Direct Control (Operator)
    ARA_MODE_AUTO,          // Autonomous Control (State Machine / Sensors)
    ARA_MODE_ERROR          // Fault state (E-Stop, Link Loss, Overcurrent)
} AraSysMode_t;


/* =========================================================
 * 2. RC Semantic Enums (Mapped from L3)
 * ========================================================= */

/**
 * @brief Arm Action Commands (From SE Channel)
 */
typedef enum {
    ARM_CMD_HOLD = 0,       // Stop movement / Maintain current position
    ARM_CMD_EXTEND,         // Continuous extension (Long Press)
    ARM_CMD_RETRACT         // Pulse retraction command (Short Press)
} AraArmCmd_t;

/**
 * @brief Gripper Action Commands (From SC Channel)
 */
typedef enum {
    GRIPPER_CMD_STOP = 0,   // Abort action / Hold
    GRIPPER_CMD_OPEN,       // Open gripper
    GRIPPER_CMD_CLOSE       // Close gripper
} AraGripperCmd_t;

/**
 * @brief Emergency Stop State (From SD Channel)
 */
typedef enum {
    ESTOP_RELEASED = 0,     // Safe to operate
    ESTOP_ACTIVE            // CRITICAL: Stop all actuators
} AraEStopState_t;


/* =========================================================
 * 3. Data Structures
 * ========================================================= */

/**
 * @brief RC Extracted Intent Data (Written by Task_RC, Read by Scheduler/Motion)
 */
typedef struct {
    bool            is_link_up;      // true = Connected, false = Link Loss (Triggers ERROR)

    AraSysMode_t    req_mode;        // Requested mode from SA (Manual vs Auto)
    AraEStopState_t estop_state;     // E-Stop status from SD

    AraArmCmd_t     arm_cmd;         // Main Arm command from SE
    AraGripperCmd_t gripper_cmd;     // Gripper command from SC
    bool            sys_reset_pulse; // System Reset pulse from SB (true for 1 cycle only)

    uint16_t        aux_knob_val;    // SF Knob mapping: 0 to 1000 (Permille, NO FLOAT)
} RcControlData_t;


/* =========================================================
 * 4. Thread-Safe API
 * @note Implementations in .c MUST use FreeRTOS Mutex or
 * taskENTER_CRITICAL() / taskEXIT_CRITICAL() to prevent data tearing.
 * ========================================================= */

/**
 * @brief Initialize DataHub (Create Mutexes/Queues)
 * @note  Must be called before OS Scheduler starts.
 */
void DataHub_Init(void);

/**
 * @brief Read the current System Operation Mode
 * @return AraSysMode_t current mode
 */
AraSysMode_t DataHub_GetSysMode(void);

/**
 * @brief Force set the System Operation Mode (Used by L5 Scheduler only)
 * @param mode Target mode
 * @return ARA_OK on success
 */
AraStatus_t DataHub_SetSysMode(AraSysMode_t mode);

/**
 * @brief Write the latest parsed RC Semantic Data
 * @param p_rc_data Pointer to the source data
 * @return ARA_OK or ARA_BUSY (if Mutex timeout)
 */
AraStatus_t DataHub_WriteRcData(const RcControlData_t *p_rc_data);

/**
 * @brief Read the latest RC Semantic Data
 * @param p_rc_data Pointer to the destination buffer
 * @return ARA_OK or ARA_BUSY (if Mutex timeout)
 */
AraStatus_t DataHub_ReadRcData(RcControlData_t *p_rc_data);

#endif /* DATAHUB_H */