/**
 * @file task_motion.h
 * @brief Motion Control Logic (L4) - Downgraded to Runnable
 * @note  Executes the 1000Hz FOC closed-loop control.
 * STRICTLY NON-BLOCKING. All OS dependencies (FreeRTOS) have been removed.
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "datahub.h"
#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_voltage_foc.h"
#include "alg_pid.h"

/* --- State Machine --- */
typedef enum {
    MOTION_STATE_IDLE = 0,
    MOTION_STATE_ALIGNMENT,   // Force vector to find zero offset
    MOTION_STATE_CLOSED_LOOP, // Normal FOC running
    MOTION_STATE_ERROR        // Fault state (Requires reset)
} MotionState_t;

/* --- Logic Context --- */
typedef struct {
    MotionState_t         state;
    uint32_t              tick_count;

    // Components (L2 & L3 Aggregation)
    DrvAS5600_Context_t   enc_driver;
    DrvBldc_Context_t     motor_driver;
    AlgFoc_Context_t      foc_algo;
    AlgPid_Context_t      pos_pid;

    // Internal States
    uint16_t              prev_angle_raw;
} TaskMotion_Context_t;

/* --- API --- */

/**
 * @brief Initialize all L2/L3 components for the Motion pipeline.
 * @note  Must be called before L5 threads start.
 */
void TaskMotion_Init(void);

/**
 * @brief The 1000Hz Execution Step.
 * @note  Called exclusively by Thread_HighFreq_Ctrl (L5).
 * Reads the LAST DMA sensor data, computes FOC, updates PWM, and triggers the NEXT DMA read.
 * Exec time MUST BE < 100us.
 * * @param p_cmd   [IN]  Latest downlink command from DataHub (Contains Target Angle & E-Stop)
 * @param p_state [OUT] Real-time hardware state to be pushed back to DataHub
 */
void TaskMotion_Update(const DataHub_Cmd_t *p_cmd, DataHub_State_t *p_state);

#endif /* TASK_MOTION_H */