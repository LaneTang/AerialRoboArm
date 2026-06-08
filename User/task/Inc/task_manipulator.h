/**
 * @file task_manipulator.h
 * @brief Manipulator FSM runnable (L4).
 *
 * The arbiter has already collapsed RC + Vision + servo state into one
 * unified intent. The manipulator FSM is now a thin behaviour layer that:
 *   - Translates ArbiterOutput into MotionCmd for task_motion.
 *   - Issues end-effector (gripper / roll) commands via mod_actuator.
 *   - Tracks demo state transitions (idle / manual / auto-pick / auto-done).
 *
 * No DataHub access. All inputs explicit. Pure-C state machine.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"
#include "datahub.h"
#include "mod_actuator.h"
#include "task_arbiter.h"
#include "task_motion.h"

/* ============================================================================
 * State enum
 * ============================================================================= */

typedef enum {
    MANIP_STATE_BOOT = 0,        /**< Waiting for first servo-online tick. */
    MANIP_STATE_IDLE,            /**< Safe idle, torque off. */
    MANIP_STATE_MANUAL,          /**< RC direct control. */
    MANIP_STATE_AUTO_GO,         /**< AUTO: traversing to vision target. */
    MANIP_STATE_AUTO_HOLD,       /**< AUTO: target reached, holding. */
    MANIP_STATE_ERROR_SAFE       /**< Latched fault / E-Stop. */
} ManipulatorState_t;

typedef struct {
    ManipulatorState_t current_state;
    uint32_t           state_enter_ms;
    float              last_target_angle_deg;
    ModActuator_Context_t actuator;
} TaskManipulator_Context_t;

/* ============================================================================
 * API
 * ============================================================================= */

void TaskManipulator_Init(void);

/**
 * @brief Run one 50 Hz manipulator step.
 * @param arb       Arbiter decision for this tick.
 * @param mot_state Latest motion feedback (read-only).
 * @param tick_ms   Current tick.
 * @param out_cmd   Output motion command for task_motion.
 */
void TaskManipulator_Update(const ArbiterOutput_t *arb,
                            const MotionState_t   *mot_state,
                            uint32_t               tick_ms,
                            MotionCmd_t           *out_cmd);

/**
 * @brief Read the current FSM state (for telemetry / DataHub publishing).
 */
ManipulatorState_t TaskManipulator_GetState(void);

#endif /* TASK_MANIPULATOR_H */
