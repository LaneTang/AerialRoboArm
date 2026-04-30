/**
 * @file task_motion.h
 * @brief Single-axis motion execution service for the BLDC folding-arm axis (L4).
 * @note  This revision removes demo/business auto-flow from Motion.
 *        Motion now behaves as a command-driven execution kernel:
 *        - FOC electrical alignment
 *        - Hold current position
 *        - Go to home position
 *        - Go to target ext_raw position
 *        - Idle / relax
 *        - Emergency stop
 *
 *        The external coordinate is no longer the wrapped AS5600 raw angle.
 *        Instead, Motion exposes a monotonic extension coordinate:
 *
 *            ext_raw = unwrapped_raw - MOTION_HOME_RAW_CALIB
 *
 *        where:
 *        - MOTION_HOME_RAW_CALIB is the folded-home raw value
 *        - raw values below MOTION_WRAP_THRESHOLD_RAW are interpreted as
 *          post-wrap forward-extension values and are unwrapped by +4096
 */

#ifndef TASK_MOTION_H
#define TASK_MOTION_H

#include "ara_def.h"
#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_voltage_foc.h"
#include "alg_pid.h"

/* ============================================================================
 * 1. Calibrated Mechanism Coordinate Definition
 * ========================================================================== */

/**
 * @brief AS5600 raw value at the fully folded home position.
 */
#define MOTION_HOME_RAW_CALIB            (2200U)

/**
 * @brief Raw threshold used to unwrap post-wrap forward-extension values.
 * @note  When raw < threshold, raw is interpreted as raw + 4096.
 */
#define MOTION_WRAP_THRESHOLD_RAW        (800U)

/**
 * @brief Maximum valid extension coordinate in ext_raw domain.
 * @note  Computed from the calibrated home raw and wrap threshold.
 */
#define MOTION_EXT_MAX_RAW               ((int32_t)((4096U + MOTION_WRAP_THRESHOLD_RAW) - MOTION_HOME_RAW_CALIB))

/**
 * @brief Minimum valid extension coordinate in ext_raw domain.
 */
#define MOTION_EXT_MIN_RAW               (0)

/* ============================================================================
 * 2. Command / State Types
 * ========================================================================== */

/**
 * @brief Motion command type issued by the upper layer.
 */
typedef enum {
    MOTION_CMD_IDLE = 0,     /**< Relax the power stage and stay idle. */
    MOTION_CMD_ALIGN,        /**< Execute FOC electrical alignment only. */
    MOTION_CMD_HOLD,         /**< Hold the current position captured on entry. */
    MOTION_CMD_GOTO_HOME,    /**< Closed-loop track ext_raw = 0. */
    MOTION_CMD_GOTO_EXT,     /**< Closed-loop track cmd.target_ext_raw. */
    MOTION_CMD_ESTOP         /**< Immediate emergency stop / latch error. */
} MotionCmdType_t;

/**
 * @brief Public motion execution state.
 */
typedef enum {
    MOTION_EXEC_UNINIT = 0,  /**< Components not initialized yet. */
    MOTION_EXEC_IDLE,        /**< Idle / relaxed state. */
    MOTION_EXEC_ALIGNING,    /**< FOC electrical alignment is in progress. */
    MOTION_EXEC_HOLDING,     /**< Closed-loop holding a captured target. */
    MOTION_EXEC_TRACKING,    /**< Closed-loop tracking an explicit ext_raw target. */
    MOTION_EXEC_ERROR        /**< Fault / E-stop state. */
} MotionExecState_t;

/**
 * @brief Command payload for one 1kHz motion update step.
 */
typedef struct {
    MotionCmdType_t cmd;      /**< Requested motion command type. */
    int32_t         target_ext_raw; /**< Target extension coordinate for MOTION_CMD_GOTO_EXT. */
} MotionCmd_t;

/**
 * @brief Uplink motion state returned by the execution kernel.
 */
typedef struct {
    MotionExecState_t exec_state;          /**< Current execution state. */
    bool              is_aligned;          /**< true after successful FOC electrical alignment. */
    uint16_t          home_raw;            /**< Calibrated folded-home raw value. */
    uint16_t          raw_angle;           /**< Latest AS5600 raw angle. */
    int32_t           ext_raw;             /**< Latest monotonic extension coordinate. */
    int16_t           velocity_raw_per_ms; /**< Raw encoder velocity in counts/ms. */
    int32_t           target_ext_raw;      /**< Active closed-loop target in ext_raw domain. */
    bool              target_reached;      /**< true when tracking error is within threshold. */
    AraStatus_t       status;              /**< Latest hardware/control status. */
} MotionState_t;

/**
 * @brief Motion runnable singleton context.
 * @note  The module still owns a single static instance internally.
 */
typedef struct {
    /* ---------------- Runtime State ---------------- */
    MotionExecState_t  exec_state;            /**< Current public execution state. */
    MotionCmdType_t    active_cmd;            /**< Last accepted command type. */
    uint32_t           uptime_ticks;          /**< Total 1kHz update ticks since init. */
    uint32_t           state_ticks;           /**< Time spent in current state (1 tick = 1ms). */
    int8_t  motor_direction; // 从配置中读取的极性

    /* ---------------- Calibration / Control State ---------------- */
    bool               is_foc_aligned;        /**< true once foc_zero_offset is captured. */
    uint16_t           foc_zero_offset;       /**< Electrical zero reference captured after alignment. */
    uint16_t           home_raw;              /**< Folded-home raw reference. */
    uint16_t           wrap_threshold_raw;    /**< Raw unwrap threshold. */
    int32_t            hold_target_ext_raw;   /**< Captured hold target. */
    int32_t            track_target_ext_raw;  /**< Explicit tracking target. */

    /* ---------------- Observability / Health ---------------- */
    AraStatus_t        hw_status;             /**< Latest hardware/control status. */
    uint16_t           prev_angle_raw;        /**< Previous raw angle sample for velocity estimation. */

    /* ---------------- Aggregated Components ---------------- */
    DrvAS5600_Context_t enc_driver;           /**< L2 encoder driver instance. */
    DrvBldc_Context_t   motor_driver;         /**< L2 BLDC power-stage driver instance. */
    AlgFoc_Context_t    foc_algo;             /**< L3 voltage-FOC algorithm context. */
    AlgPid_Context_t    pos_pid;              /**< L3 position-loop PID context. */
} TaskMotion_Context_t;

/* ============================================================================
 * 3. Public API
 * ========================================================================== */

/**
 * @brief  Initialize all L2/L3 components used by Motion.
 * @note   Must be called before the 1kHz scheduler starts.
 */
void TaskMotion_Init(void);

/**
 * @brief  Execute one 1kHz motion-control step.
 * @param  p_cmd   Pointer to the latest upper-layer motion command.
 * @param  p_state Pointer to the state structure written back to the caller.
 * @note   The function is strictly non-blocking.
 */
void TaskMotion_Update(const MotionCmd_t *p_cmd, MotionState_t *p_state);

/**
 * @brief  Convert AS5600 raw angle into monotonic extension coordinate.
 * @param  raw_angle AS5600 raw angle in [0, 4095].
 * @return ext_raw coordinate referenced to MOTION_HOME_RAW_CALIB.
 */
int32_t TaskMotion_MapRawToExt(uint16_t raw_angle);

/**
 * @brief  Convert monotonic extension coordinate into AS5600 raw angle.
 * @param  ext_raw Extension coordinate.
 * @return Wrapped AS5600 raw angle in [0, 4095].
 */
uint16_t TaskMotion_MapExtToRaw(int32_t ext_raw);

#endif /* TASK_MOTION_H */