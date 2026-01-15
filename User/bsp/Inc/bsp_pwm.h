/**
 * @file bsp_pwm.h
 * @brief PWM Driver for FOC (3-Phase) and Servos
 * @note  OPTIMIZED: Uses uint16_t instead of float for F103 performance.
 */

#ifndef BSP_PWM_H
#define BSP_PWM_H

#include "ara_def.h"

/* --- Configuration Constants --- */
/* FOC PWM Period (ARR). Example: 72MHz / 3600 = 20kHz */
#define BSP_PWM_MAX_DUTY    (1440)

/* --- Device Definition --- */
typedef enum {
    BSP_SERVO_1 = 0,    // Gripper / Aux Axis
    BSP_SERVO_2,
    BSP_SERVO_NUM
} BspServo_Dev_t;

/* --- API --- */

void BSP_PWM_Init(void);

/**
 * @brief  Set 3-Phase Duty Cycle for FOC (Integer Optimized)
 * @param  u  Duty for Phase A (0 to BSP_PWM_MAX_DUTY)
 * @param  v  Duty for Phase B (0 to BSP_PWM_MAX_DUTY)
 * @param  w  Duty for Phase C (0 to BSP_PWM_MAX_DUTY)
 * @note   Directly updates CCR registers. Very fast.
 */
void BSP_PWM_Set3PhaseDuty_U16(uint16_t u, uint16_t v, uint16_t w);

/**
 * @brief  Set Servo Pulse Width
 * @param  servo  Target Servo
 * @param  us     Pulse width in microseconds (500-2500)
 */
void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us);

/**
 * @brief  Emergency Stop - Forces all PWM outputs to safe state (Low/Hi-Z)
 */
void BSP_PWM_StopAll(void);

#endif // BSP_PWM_H