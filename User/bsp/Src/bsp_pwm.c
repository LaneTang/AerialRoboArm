/**
 * @file bsp_pwm.c
 * @brief PWM Driver (TIM2 servo path only after demo_v7 BLDC retirement).
 *
 * The legacy 3-phase TIM1 path was removed when the BLDC stack was
 * retired in demo_v7 phase 1. Only the TIM2-driven hobby-servo PWM
 * channels remain (used for gripper / roll on the manipulator).
 */

#include "bsp_pwm.h"
#include "stm32f1xx_hal.h"

/* --- Hardware Resources --- */
extern TIM_HandleTypeDef htim2; // Servos (Auxiliary)

/* --- Configuration Maps --- */
typedef struct {
    TIM_HandleTypeDef *htim;
    uint32_t           channel;
} ServoConfig_t;

static const ServoConfig_t servo_map[BSP_SERVO_NUM] = {
        [BSP_SERVO_1] = { &htim2, TIM_CHANNEL_1 },
        [BSP_SERVO_2] = { &htim2, TIM_CHANNEL_2 }
};

/* --- API Implementation --- */

void BSP_PWM_Init(void)
{
    /* Pre-load both channels to ~90 deg neutral (1520us pulse for PTK 7462W)
     * BEFORE starting PWM. Without this, the compare registers default to 0,
     * giving a 0us pulse during boot / ERROR phases (when manipulator
     * short-circuits and never writes a valid command). PTK-class digital
     * servos interpret 0us as garbage and spin uncontrollably. */
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 1520);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 1520);

    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us)
{
    if (servo >= BSP_SERVO_NUM) return;

    /* PTK 7462W range: 500-2500 us at a 333 Hz PWM frame. */
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;

    /* Assumes TIM2 prescaler maps 1 tick = 1 us. */
    __HAL_TIM_SET_COMPARE(servo_map[servo].htim, servo_map[servo].channel, us);
}

void BSP_PWM_StopAll(void)
{
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}
