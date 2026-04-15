/**
 * @file task_manipulator.h
 * @brief Demo-oriented manipulator brain and state machine runnable (L4).
 * @note  Executes the 50Hz decision logic for the folding-arm BLDC axis and
 *        end-effector servos. This revision is aligned to the current demo goal:
 *        - MANUAL mode directly controls all actuators from RC semantics.
 *        - AUTO mode is a deterministic fixed demo flow.
 *        - Vision input is removed from this task.
 *        - The BLDC target remains expressed as AS5600 raw angle for compatibility
 *          with the current Motion / DataHub contract.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"
#include "datahub.h"

/**
 * @brief Manipulator top-level demo state machine.
 */
typedef enum {
    MANIP_STATE_STARTUP_PARK = 0,  /**< Startup park-hold after motion calibration. */
    MANIP_STATE_IDLE,              /**< Safe demo idle / park-hold state. */
    MANIP_STATE_MANUAL,            /**< Direct operator control from RC. */
    MANIP_STATE_AUTO_MOVE_PICK,    /**< AUTO: move joint to fixed pick angle. */
    MANIP_STATE_AUTO_HOLD_VERIFY,  /**< AUTO: verify joint stays within tolerance. */
    MANIP_STATE_AUTO_GRAB_CLOSE,   /**< AUTO: close gripper after hold verification. */
    MANIP_STATE_AUTO_RETRACT,      /**< AUTO: retract joint to fixed demo angle. */
    MANIP_STATE_AUTO_DONE,         /**< AUTO: final hold state. */
    MANIP_STATE_ERROR_SAFE         /**< Emergency-safe latched state. */
} ManipulatorState_t;

/**
 * @brief Manipulator runnable context.
 */
typedef struct {
    ManipulatorState_t current_state;        /**< Current top-level state. */
    uint32_t           state_enter_tick;     /**< 50Hz tick when current state was entered. */

    uint16_t           manual_target_degree;   /**< Manual-mode joint target in mechanical degree. */
    uint8_t            manual_gripper_percent; /**< Manual-mode gripper target, 0~100%. */
    uint8_t            manual_roll_degree;     /**< Manual-mode roll target, 0~180 degree. */

    bool               auto_start_armed;       /**< AUTO may start only after seeing a non-AUTO state first. */
} TaskManipulator_Context_t;

/**
 * @brief  Initialize the manipulator runnable context and owned actuator module.
 * @note   Must be called before the L5 scheduler starts.
 */
void TaskManipulator_Init(void);

/**
 * @brief  Execute one 50Hz manipulator decision step.
 * @param  p_rc_intent   Pointer to the latest RC semantic intent.
 * @param  p_motion_state Pointer to the latest motion feedback from DataHub.
 * @param  p_out_cmd     Pointer to the downlink command written toward Motion.
 * @note   The function is strictly non-blocking.
 * @note   This revision no longer depends on any vision data.
 */
void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const DataHub_State_t *p_motion_state,
                            DataHub_Cmd_t *p_out_cmd);

#endif /* TASK_MANIPULATOR_H */