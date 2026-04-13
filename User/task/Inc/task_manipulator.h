/**
 * @file task_manipulator.h
 * @brief Arm manipulator brain and state machine runnable (L4).
 * @note  Executes the 50Hz decision logic for the folding-arm BLDC axis and
 *        end-effector servos. This layer contains no RTOS dependencies and is
 *        called exclusively by the L5 low-frequency logic thread.
 */

#ifndef TASK_MANIPULATOR_H
#define TASK_MANIPULATOR_H

#include "ara_def.h"
#include "datahub.h"
#include "mod_vsp_parser.h"

/**
 * @brief Manipulator top-level state machine.
 */
typedef enum {
    MANIP_STATE_IDLE = 0,       /**< Safe idle / home-hold state. */
    MANIP_STATE_MANUAL,         /**< Direct operator control from RC. */
    MANIP_STATE_SEEKING,        /**< Auto tracking with low-pass filtering. */
    MANIP_STATE_CONVERGING,     /**< Auto convergence debounce before grab. */
    MANIP_STATE_GRABBING,       /**< Blind gripper closing while position is held. */
    MANIP_STATE_RETRACTING,     /**< Pull back to home after grab. */
    MANIP_STATE_ERROR_SAFE      /**< Emergency-safe state. */
} ManipulatorState_t;

/**
 * @brief Manipulator runnable context.
 */
typedef struct {
    ManipulatorState_t current_state;        /**< Current top-level manipulator state. */
    uint32_t           state_enter_tick;     /**< 50Hz tick when the current state was entered. */

    int16_t            filtered_roll;        /**< Filtered vision roll target, unit: centi-degree. */
    uint16_t           filtered_ext_mm;      /**< Filtered vision extension target, unit: mm. */

    uint16_t           manual_target_angle_raw; /**< Manual-mode BLDC target in AS5600 raw angle. */
    uint8_t            manual_gripper_percent;  /**< Manual-mode gripper target, 0~100%. */
    uint8_t            manual_roll_degree;      /**< Manual-mode roll target, 0~180 degree. */
} TaskManipulator_Context_t;

/**
 * @brief  Initialize the manipulator runnable context and owned actuator module.
 * @note   Must be called before the L5 scheduler starts.
 */
void TaskManipulator_Init(void);

/**
 * @brief  Execute one 50Hz manipulator decision step.
 * @param  p_rc_intent   Pointer to the latest RC semantic intent.
 * @param  p_vision_data Pointer to the latest validated vision semantic data.
 * @param  p_out_cmd     Pointer to the downlink command written toward Motion.
 * @note   The function is strictly non-blocking.
 * @note   The BLDC axis target is still expressed as a raw AS5600 target angle
 *         for compatibility with the current Motion / DataHub contract.
 */
void TaskManipulator_Update(const RcControlData_t *p_rc_intent,
                            const AraVisionData_t *p_vision_data,
                            DataHub_Cmd_t *p_out_cmd);

#endif /* TASK_MANIPULATOR_H */