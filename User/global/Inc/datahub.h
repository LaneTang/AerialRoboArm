/**
 * @file datahub.h
 * @brief Global Lock-Free Data Exchange Center (L4)
 * @note  Acts as the ONLY bridge between the 1000Hz High-Freq Control Thread
 * and the 50Hz Low-Freq Logic Thread.
 * Strictly NO Mutexes. Uses Cortex-M Critical Sections for microsecond-level fast copy.
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
    ARA_MODE_INIT = 0,      // System booting and calibrating FOC zero offset
    ARA_MODE_IDLE,          // Safe state, actuators relaxed, waiting for command
    ARA_MODE_MANUAL,        // RC Direct Control (Operator)
    ARA_MODE_AUTO,          // Autonomous Control (Vision/State Machine)
    ARA_MODE_ERROR          // Fault state (E-Stop, Link Loss, Sensor I2C Error)
} AraSysMode_t;

/* =========================================================
 * 2. Cross-Thread Data Structures
 * ========================================================= */

/**
 * @brief Downlink Command Struct (50Hz Logic -> 1000Hz Motion)
 * @note  Produced by Thread_LowFreq_Logic, Consumed by Thread_HighFreq_Ctrl.
 */
typedef struct {
    AraSysMode_t    sys_mode;           // Master mode overriding all behaviors
    bool            emergency_stop;     // Critical flag: true = instantly disable BLDC PWM
    uint16_t        target_foc_angle;   // Target absolute AS5600 angle for Z-axis arm (0-4095)

    /* Note: Servo Roll & Gripper commands are NOT here because PWM Servos
     * are updated directly within the 50Hz thread. Only FOC needs 1000Hz data. */
} DataHub_Cmd_t;

/**
 * @brief Uplink State Struct (1000Hz Motion -> 50Hz Logic)
 * @note  Produced by Thread_HighFreq_Ctrl, Consumed by Thread_LowFreq_Logic.
 */
typedef struct {
    uint16_t        current_foc_angle;  // Real-time AS5600 angle (0-4095)
    int16_t         current_velocity;   // Real-time calculated velocity (counts/ms)
    AraStatus_t     foc_status;         // Hardware health (e.g., ARA_ERR_NACK if I2C fails)
} DataHub_State_t;

/* =========================================================
 * 3. Thread-Safe API (Lock-Free)
 * @note All APIs are NON-BLOCKING. Guaranteed execution < 5us.
 * ========================================================= */

/**
 * @brief Initialize DataHub Memory
 * @note  Must be called before the OS Scheduler starts. Zeroes out all structs.
 */
void DataHub_Init(void);

/**
 * @brief Write Downlink Command to DataHub
 * @note  Called by Thread_LowFreq_Logic. Protected by taskENTER_CRITICAL().
 * @param p_cmd Pointer to the computed command struct to flush into Hub.
 */
void DataHub_WriteCmd(const DataHub_Cmd_t *p_cmd);

/**
 * @brief Read Downlink Command from DataHub
 * @note  Called by Thread_HighFreq_Ctrl. Protected by taskENTER_CRITICAL().
 * @param p_cmd Pointer to destination buffer.
 */
void DataHub_ReadCmd(DataHub_Cmd_t *p_cmd);

/**
 * @brief Write Uplink State to DataHub
 * @note  Called by Thread_HighFreq_Ctrl. Protected by taskENTER_CRITICAL().
 * @param p_state Pointer to the fresh state struct to flush into Hub.
 */
void DataHub_WriteState(const DataHub_State_t *p_state);

/**
 * @brief Read Uplink State from DataHub
 * @note  Called by Thread_LowFreq_Logic (e.g., for error-convergence check).
 * @param p_state Pointer to destination buffer.
 */
void DataHub_ReadState(DataHub_State_t *p_state);

#endif /* DATAHUB_H */