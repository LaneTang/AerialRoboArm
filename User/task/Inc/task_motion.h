/**
 * @file task_motion.h
 * @brief Motion control runnable for the BLDC folding-arm axis (L4).
 * @note  Executes the 1000Hz non-blocking FOC control pipeline.
 *         All RTOS dependencies are intentionally removed from this layer.
 *         External API is kept backward-compatible with the current L5/L4
 *         integration: DataHub_Cmd_t::target_foc_angle is still interpreted
 *         as a raw AS5600 target angle.
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "datahub.h"
#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_voltage_foc.h"
#include "alg_pid.h"

/**
 * @brief Motion task top-level state machine.
 */
typedef enum {
    MOTION_STATE_BOOT = 0,      /**< Sensor priming / safe relaxed boot state. */
    MOTION_STATE_FOC_ALIGN,     /**< Rotor electrical alignment for FOC zero calibration. */
    MOTION_STATE_HOMING,        /**< Mechanical homing toward the hard stop and backoff. */
    MOTION_STATE_READY,         /**< Initialization finished; waiting to enter closed loop. */
    MOTION_STATE_CLOSED_LOOP,   /**< Normal closed-loop FOC position control. */
    MOTION_STATE_ERROR          /**< Fatal fault state; power stage is cut off. */
} MotionState_t;

/**
 * @brief Internal sub-phase for the mechanical homing procedure.
 */
typedef enum {
    MOTION_HOME_PHASE_SEEK_LIMIT = 0,   /**< Drive slowly toward the mechanical limit. */
    MOTION_HOME_PHASE_BACKOFF          /**< Back off a small distance after touching the limit. */
} MotionHomingPhase_t;

/**
 * @brief Motion runnable singleton context.
 * @note  The struct is exposed for clarity and documentation consistency,
 *        but the module still owns a single static instance internally.
 */
typedef struct {
    /* ---------------- Runtime State ---------------- */
    MotionState_t        state;                    /**< Current top-level motion state. */
    MotionHomingPhase_t  homing_phase;            /**< Current homing sub-phase. */
    uint32_t             uptime_ticks;            /**< Total 1kHz update ticks since init. */
    uint32_t             state_ticks;             /**< Time spent in current state (1 tick = 1ms). */
    uint32_t             plateau_ticks;           /**< Consecutive ticks satisfying homing plateau condition. */

    /* ---------------- Calibration State ---------------- */
    bool                 is_foc_aligned;          /**< true once foc_zero_offset is captured. */
    bool                 is_homed;                /**< true once home_angle_raw is captured. */
    uint16_t             foc_zero_offset;         /**< Electrical zero reference captured after FOC alignment. */
    uint16_t             home_angle_raw;          /**< Mechanical home reference captured at the hard stop. */
    uint16_t             home_backoff_target_raw; /**< Raw-angle target used during the homing backoff phase. */

    /* ---------------- Observability / Health ---------------- */
    AraStatus_t          hw_status;               /**< Latest hardware/control status exposed to L5. */
    uint16_t             prev_angle_raw;          /**< Previous AS5600 raw angle sample for velocity estimation. */

    /* ---------------- Aggregated Components ---------------- */
    DrvAS5600_Context_t  enc_driver;              /**< L2 encoder driver instance. */
    DrvBldc_Context_t    motor_driver;            /**< L2 BLDC power-stage driver instance. */
    AlgFoc_Context_t     foc_algo;                /**< L3 voltage-FOC algorithm context. */
    AlgPid_Context_t     pos_pid;                 /**< L3 position-loop PID context. */
} TaskMotion_Context_t;

/**
 * @brief  Initialize all L2/L3 components used by the motion pipeline.
 * @note   Must be called before the L5 scheduler starts.
 *         This function does not start motion immediately; actual state
 *         progression happens inside TaskMotion_Update().
 */
void TaskMotion_Init(void);

/**
 * @brief  Execute one 1000Hz motion-control step.
 * @param  p_cmd   Pointer to the latest downlink command from DataHub.
 * @param  p_state Pointer to the uplink state object written back to DataHub.
 * @note   The function is strictly non-blocking and intended to be called only
 *         by the L5 high-frequency control thread.
 * @note   For current branch compatibility, p_cmd->target_foc_angle is still
 *         interpreted as a raw AS5600 target angle. The internally captured
 *         mechanical home reference is reserved for the next-step migration to
 *         a home-referenced mechanism coordinate.
 */
void TaskMotion_Update(const DataHub_Cmd_t *p_cmd, DataHub_State_t *p_state);

#endif /* TASK_MOTION_H */