/**
 * @file bsp_pwm.c
 * @brief PWM Driver Implementation (HAL + Direct Register Access)
 * @author ARA Project Coder
 */

#include "bsp_pwm.h"
#include "stm32f1xx_hal.h"

/* --- Configuration --- */
/* * User Parameter: TIM1 ARR = 1440 - 1
 * In Center Aligned Mode, Period = 2 * ARR * T_clk
 * Duty Cycle = CCRx / (ARR + 1)
 * Max Duty Value = 1440
 */
#ifndef BSP_PWM_MAX_DUTY
#define BSP_PWM_MAX_DUTY 1440
#endif

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
    /* [Step 0] Safe State: Zero out duties BEFORE enabling output */
    // 防止上电瞬间电机因为寄存器残留值而猛冲
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    /* [Step 1] Start FOC PWM Channels (TIM1) */
    // Note: Assuming GPIOs are configured as Alternate Function Push-Pull in MSP
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    /* [Step 2] Enable Main Output (MOE Bit) - CRITICAL for TIM1 */
    // 只有置位 MOE，高级定时器的 OCx 引脚才会真正输出波形，否则是高阻/低电平
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* [Step 3] Start Servo PWM Channels (TIM2) */
    // TIM2 是通用定时器，不需要 MOE
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
}

void BSP_PWM_Set3PhaseDuty_U16(uint16_t u, uint16_t v, uint16_t w)
{
    /* Saturation Guard */
    // 限制最大占空比，防止超过 ARR 导致波形反转或由 100% 变 0%
    if (u > BSP_PWM_MAX_DUTY) u = BSP_PWM_MAX_DUTY;
    if (v > BSP_PWM_MAX_DUTY) v = BSP_PWM_MAX_DUTY;
    if (w > BSP_PWM_MAX_DUTY) w = BSP_PWM_MAX_DUTY;

    /* * DIRECT REGISTER ACCESS (Optimized)
     * Bypassing HAL overhead.
     * In Center-Aligned mode, the shadow register update happens at UEV (Update Event).
     */
    htim1.Instance->CCR1 = u;
    htim1.Instance->CCR2 = v;
    htim1.Instance->CCR3 = w;

    /* * 注意：如果之前调用过 BSP_PWM_StopAll()，这里不会自动恢复输出。
     * 必须调用 BSP_PWM_Init() 或手动 ENABLE MOE 才能恢复。
     * 这是为了 FOC 算法的安全设计：出错停止后，必须显式复位。
     */
}

void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us)
{
    if (servo >= BSP_SERVO_NUM) return;

    /* Safety Clamping (Standard Servo Range 500-2500us) */
    if (us < 500)  us = 500;
    if (us > 2500) us = 2500;

    /* * Update Servo Duty
     * Assumption: Timer Prescaler is set so that 1 tick = 1 us.
     * If TIM2 clock is 72MHz, PSC should be 71 (72-1).
     */
    __HAL_TIM_SET_COMPARE(servo_map[servo].htim, servo_map[servo].channel, us);
}

void BSP_PWM_StopAll(void)
{
    /* * Emergency Stop Logic */

    // 1. FOC Motor: Disable Main Output (Hi-Z state for Bridge)
    // 立即切断驱动芯片输入，这是最底层的硬件安全锁
    __HAL_TIM_MOE_DISABLE(&htim1);

    // 2. Zero out the duties
    htim1.Instance->CCR1 = 0;
    htim1.Instance->CCR2 = 0;
    htim1.Instance->CCR3 = 0;

    // 3. Servos: Set to 0 (No pulse)
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, 0);
}