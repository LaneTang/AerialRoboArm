/**
 * @file datahub.h
 * @brief Global Lock-Free Data Exchange Center & Type Definitions (L4)
 * @note  Acts as the global blackboard for type definitions and the ONLY
 * lock-free bridge between the 1000Hz Control Thread and 50Hz Logic Thread.
 */

#ifndef DATAHUB_H
#define DATAHUB_H

#include "ara_def.h"

/* =========================================================
 * 1. Global System Enums
 * ========================================================= */

/**
 * @brief System Operation Modes
 */
typedef enum {
    ARA_MODE_INIT = 0,      // System booting and calibrating FOC zero offset
    ARA_MODE_IDLE,          // Safe state, actuators relaxed, waiting for command
    ARA_MODE_MANUAL,        // RC Direct Control (Operator)
    ARA_MODE_AUTO,          // Autonomous Control (Vision/State Machine)
    ARA_MODE_ERROR          // Fault state (E-Stop, Link Loss, Sensor I2C Error)
} AraSysMode_t;

/* =========================================================
 * 2. RC Semantic Enums & Intent Structure (Restored for L3)
 * ========================================================= */

typedef enum {
    ARM_CMD_HOLD = 0,       // Stop movement / Maintain current position
    ARM_CMD_EXTEND,         // Continuous extension
    ARM_CMD_RETRACT         // Pulse retraction command
} AraArmCmd_t;

typedef enum {
    GRIPPER_CMD_STOP = 0,   // Abort action / Hold
    GRIPPER_CMD_OPEN,       // Open gripper
    GRIPPER_CMD_CLOSE       // Close gripper
} AraGripperCmd_t;

typedef enum {
    ESTOP_RELEASED = 0,     // Safe to operate
    ESTOP_ACTIVE            // CRITICAL: Stop all actuators
} AraEStopState_t;

/**
 * @brief RC Extracted Intent Data
 * @note  Used as a local passing structure in the 50Hz Logic Thread.
 */
typedef struct {
    bool            is_link_up;      // true = Connected, false = Link Loss
    AraSysMode_t    req_mode;        // Requested mode from RC
    AraEStopState_t estop_state;     // E-Stop status
    AraArmCmd_t     arm_cmd;         // Main Arm command
    AraGripperCmd_t gripper_cmd;     // Gripper command
    bool            sys_reset_pulse; // System Reset pulse
    uint16_t        aux_knob_val;    // Aux Knob mapping: 0 to 1000
} RcControlData_t;


/* =========================================================
 * 3. Cross-Thread Data Structures (The Physical Hub)
 * ========================================================= */

/**
 * @brief Downlink Command Struct (50Hz Logic -> 1000Hz Motion)
 */
typedef struct {
    AraSysMode_t    sys_mode;           // Master mode overriding all behaviors
    bool            emergency_stop;     // Critical flag: true = instantly disable BLDC
    uint16_t        target_foc_angle;   // Target absolute AS5600 angle (0-4095)
} DataHub_Cmd_t;

/**
 * @brief Uplink State Struct (1000Hz Motion -> 50Hz Logic)
 */
/**
 * @brief Uplink State Struct (1000Hz Motion -> 50Hz Logic)
 */
typedef struct {
    uint16_t        current_foc_angle;  /**< Real-time AS5600 angle (0-4095). */
    int16_t         current_velocity;   /**< Real-time calculated velocity (counts/ms). */
    AraStatus_t     foc_status;         /**< Hardware / motion health state. */
    bool            motion_ready;       /**< true when Motion finished calibration and is ready for control. */
} DataHub_State_t;


/* =========================================================
 * 4. Thread-Safe API (Lock-Free)
 * @note Protected by taskENTER_CRITICAL(). Execution < 5us.
 * ========================================================= */

void DataHub_Init(void);

void DataHub_WriteCmd(const DataHub_Cmd_t *p_cmd);
void DataHub_ReadCmd(DataHub_Cmd_t *p_cmd);

void DataHub_WriteState(const DataHub_State_t *p_state);
void DataHub_ReadState(DataHub_State_t *p_state);

#endif /* DATAHUB_H */