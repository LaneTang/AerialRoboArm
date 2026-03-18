/**
 * @file drv_servo.h
 * @brief L2 Hardware Driver: Generic PWM Digital Servo
 * @note  Strictly NO FPU. Pure integer math for angle-to-pulse mapping.
 * Does NOT contain any RTOS dependencies. Non-blocking executions.
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include "ara_def.h"
#include "bsp_pwm.h"

/* --- Error Codes & Macros --- */
/** * @brief Custom status codes for Servo Driver.
 * Assumes ARA_OK (0) is defined in ara_def.h
 */
#define DRV_SERVO_OK                (0)
#define DRV_SERVO_ERR_PARAM         (-1)    /* Null pointer or invalid init param */
#define DRV_SERVO_LIMIT_REACHED     (1)     /* Warning: Angle truncated to safe limits */

/* --- Context Definition (OOP Pattern) --- */
/**
 * @brief Servo Driver Context.
 * Manages physical bounds to prevent hardware damage (stall torque).
 */
typedef struct {
    BspServo_Dev_t servo_id;      /**< Bound hardware PWM channel (BSP_SERVO_1 or 2) */
    uint8_t        min_angle;     /**< Physical lower bound (Degree: 0-180) */
    uint8_t        max_angle;     /**< Physical upper bound (Degree: 0-180) */
    uint8_t        current_angle; /**< State cache of current commanded angle */
} DrvServo_Context_t;

/* --- API --- */

/**
 * @brief  Initialize the servo driver context.
 * @param  ctx        Pointer to the allocated context instance.
 * @param  id         Hardware PWM channel enum.
 * @param  min_limit  Minimum safe physical angle (e.g., 10 to prevent jam).
 * @param  max_limit  Maximum safe physical angle (e.g., 170 to prevent jam).
 * @retval DRV_SERVO_OK on success, DRV_SERVO_ERR_PARAM on invalid limits (min > max).
 * @note   Non-blocking. Does NOT output PWM pulse until SetAngle is called.
 */
int8_t DrvServo_Init(DrvServo_Context_t *ctx, BspServo_Dev_t id, uint8_t min_limit, uint8_t max_limit);

/**
 * @brief  Set servo target angle with software limit protection.
 * @param  ctx           Pointer to the driver context.
 * @param  target_angle  Desired mechanical angle (0-180).
 * @retval DRV_SERVO_OK if within limits.
 * DRV_SERVO_LIMIT_REACHED if truncated to min_angle/max_angle.
 * @note   Non-blocking. Highly optimized.
 * Internal Math: Pulse = 500 + ((Angle * 100) / 9)
 */
int8_t DrvServo_SetAngle(DrvServo_Context_t *ctx, uint8_t target_angle);

#endif // DRV_SERVO_H