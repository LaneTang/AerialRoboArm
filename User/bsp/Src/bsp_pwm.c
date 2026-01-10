/**
 * @file bsp_pwm.c
 * @brief PWM Driver Implementation (HAL + Direct Register Access)
 * @author ARA Project Coder
 */

#include "bsp_pwm.h"
#include "stm32f1xx_hal.h"

/* --- Hardware Resources --- */
// External handles defined in main.c (CubeMX generated)
extern TIM_HandleTypeDef htim1; // FOC Motor (3-Phase)
extern TIM_HandleTypeDef htim2; // Servos (Auxiliary)

/* --- Configuration Maps --- */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t channel;
} ServoConfig_t;

// Map logical servo IDs to physical timer channels
static const ServoConfig_t servo_map[BSP_SERVO_NUM] = {
        [BSP_SERVO_1] = { &htim2, TIM_CHANNEL_1 }, // Example: PA0
        [BSP_SERVO_2] = { &htim2, TIM_CHANNEL_2 }  // Example: PA1
};

/* --- API Implementation --- */

void BSP_PWM_Init(void)
{
    /* 1. Start FOC PWM Channels (TIM1) */
    // Note: Assuming GPIOs and Timebase are configured in HAL_TIM_PWM_MspInit
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* Ensure Main Output is Enabled (BDTR register for TIM1) */
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 2. Start Servo PWM Channels (TIM2) */
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BSP_PWM_Set3PhaseDuty_U16(uint16_t u, uint16_t v, uint16_t w)
{
    /* Saturation Guard */
    if (u > BSP_PWM_MAX_DUTY) u = BSP_PWM_MAX_DUTY;
    if (v > BSP_PWM_MAX_DUTY) v = BSP_PWM_MAX_DUTY;
    if (w > BSP_PWM_MAX_DUTY) w = BSP_PWM_MAX_DUTY;

    /* * DIRECT REGISTER ACCESS (Optimized)
     * Bypassing HAL_TIM_PWM_Start/Stop overhead for control loop.
     * CCRx registers are shadowed, update happens at next Update Event.
     */
    htim1.Instance->CCR1 = u;
    htim1.Instance->CCR2 = v;
    htim1.Instance->CCR3 = w;
}

void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us)
{
    if (servo >= BSP_SERVO_NUM) return;

    /* Safety Clamping (Standard Servo Range 500-2500us) */
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;

    /* * Update Servo Duty
     * Assumption: Timer Prescaler is set so that 1 tick = 1 us.
     * If TIM2 clock is 72MHz, PSC should be 71.
     */
    __HAL_TIM_SET_COMPARE(servo_map[servo].htim, servo_map[servo].channel, us);
}

void BSP_PWM_StopAll(void)
{
    /* * Emergency Stop Logic
     */

    // 1. FOC Motor: Disable Main Output (Hi-Z state for Bridge)
    // This is the safest hardware-level stop for TIM1.
    __HAL_TIM_MOE_DISABLE(&htim1);

    // Also zero out the duties to be safe upon re-enable
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    // 2. Servos: Set to 0 (No pulse) or specific safe position?
    // Setting CCR to 0 usually stops the pulse generation effectively.
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}