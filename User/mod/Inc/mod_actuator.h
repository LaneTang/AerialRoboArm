/**
 * @file mod_actuator.h
 * @brief L3 Functional Module: End-Effector (Gripper & Roll Attitude)
 * @note  Abstracts hardware specifics. Exposes semantic business operations.
 * No FreeRTOS dependencies. Suitable for direct bare-metal unit testing.
 */

#ifndef MOD_ACTUATOR_H
#define MOD_ACTUATOR_H

#include "ara_def.h"
#include "drv_servo.h"

/* --- Error Codes & Macros --- */
#define MOD_ACTUATOR_OK             (0)
#define MOD_ACTUATOR_ERR_NULL       (-1)
#define MOD_ACTUATOR_WARN_JAM       (1)     /* Propagated DRV_SERVO_LIMIT_REACHED */

/* --- Context Definition (OOP Pattern) --- */
/**
 * @brief Actuator Module Context.
 * Aggregates Gripper and Roll servo objects and stores calibration data.
 */
typedef struct {
    // Sub-components (L2 Drivers)
    DrvServo_Context_t servo_gripper;
    DrvServo_Context_t servo_roll;

    // Calibration Data (Mechanical Constraints)
    uint8_t gripper_open_angle;   /**< Calibrated angle for 0% closed */
    uint8_t gripper_close_angle;  /**< Calibrated angle for 100% closed */
} ModActuator_Context_t;

/* --- API --- */

/**
 * @brief  Initialize the actuator subsystem.
 * @param  ctx Pointer to the actuator context instance.
 * @retval MOD_ACTUATOR_OK on success.
 * @note   Non-blocking. Binds BSP_SERVO_1 to Gripper and BSP_SERVO_2 to Roll.
 * Loads default calibration values and sets safe physical limits.
 */
int8_t ModActuator_Init(ModActuator_Context_t *ctx);

/**
 * @brief  Control the gripper opening/closing percentage.
 * @param  ctx     Pointer to the actuator context.
 * @param  percent Target closure percentage (0 = fully open, 100 = fully closed).
 * @retval MOD_ACTUATOR_OK on success. MOD_ACTUATOR_WARN_JAM if limit triggered.
 * @note   Non-blocking. Uses integer linear interpolation based on calibration data.
 */
int8_t ModActuator_SetGripper(ModActuator_Context_t *ctx, uint8_t percent);

/**
 * @brief  Control the roll attitude of the end-effector.
 * @param  ctx    Pointer to the actuator context.
 * @param  degree Target roll angle (0-180 degrees).
 * @retval MOD_ACTUATOR_OK on success. MOD_ACTUATOR_WARN_JAM if limit triggered.
 * @note   Non-blocking. Passes the request to L2 with safety checks.
 */
int8_t ModActuator_SetRoll(ModActuator_Context_t *ctx, uint8_t degree);

#endif // MOD_ACTUATOR_H