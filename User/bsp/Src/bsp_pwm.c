/**
  ******************************************************************************
  * @file    bsp_pwm.c
  * @brief   BSP PWM 实现
  ******************************************************************************
  */

#include "bsp_pwm.h"

/* --- 外部句柄引用 --- */
// 假设 TIM1 用于 BLDC (高级定时器, 需要互补输出支持请开启 CHxN)
extern TIM_HandleTypeDef htim1;
// 假设 TIM2 用于 舵机 (通用定时器)
extern TIM_HandleTypeDef htim2;

/* --- 内部变量 --- */
// 保存 TIM1 的重装载值 (ARR)，用于将 0.0-1.0 映射为 0-ARR
static uint16_t bldc_tim_period;

/* ============================================================ */
/* 初始化                                                       */
/* ============================================================ */
void BSP_PWM_Init(void) {
    /* 1. BLDC 初始化 (TIM1) */
    // 获取 CubeMX 配置的 ARR 值 (AutoReload Register)
    // 假设配置为中心对齐模式 (Center-Aligned)，ARR 决定了 PWM 频率
    bldc_tim_period = __HAL_TIM_GET_AUTORELOAD(&htim1);

    // 启动 3路 PWM 输出
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);

    // 如果使用高级定时器且配置了死区/互补输出 (CHxN)，需要执行主输出使能
    // 对于 SimpleFOCMini 这种半桥驱动板，通常只需要主通道，但加上这句更安全
    __HAL_TIM_MOE_ENABLE(&htim1);

    /* 2. 舵机 初始化 (TIM2) */
    // 启动 2路 舵机 PWM
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // Servo 1
    HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2); // Servo 2
}

/* ============================================================ */
/* FOC 接口实现                                                  */
/* ============================================================ */
void BSP_PWM_Set3PhaseDuty(float dc_a, float dc_b, float dc_c) {
    uint16_t ccr_a, ccr_b, ccr_c;

    /* 1. 饱和限制 (Clamp) - 安全防护 */
    if (dc_a > 1.0f) dc_a = 1.0f; else if (dc_a < 0.0f) dc_a = 0.0f;
    if (dc_b > 1.0f) dc_b = 1.0f; else if (dc_b < 0.0f) dc_b = 0.0f;
    if (dc_c > 1.0f) dc_c = 1.0f; else if (dc_c < 0.0f) dc_c = 0.0f;

    /* 2. 计算寄存器值 (CCR) */
    // SimpleFOC 算法输出的是 0.0~1.0 的占空比
    // 映射到定时器计数值: CCR = Duty * ARR
    ccr_a = (uint16_t)(dc_a * (float)bldc_tim_period);
    ccr_b = (uint16_t)(dc_b * (float)bldc_tim_period);
    ccr_c = (uint16_t)(dc_c * (float)bldc_tim_period);

    /* 3. 写入寄存器 (使用宏操作，效率最高) */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr_a);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, ccr_b);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, ccr_c);
}

/* ============================================================ */
/* 舵机接口实现                                                  */
/* ============================================================ */
void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us) {
    /* 安全限幅 (标准舵机范围 500-2500us) */
    if (us > 2500) us = 2500;
    if (us < 500)  us = 500;

    /* 写入比较值 */
    // 前提: TIM2 在 CubeMX 中 Prescaler 需配置为 1MHz 计数频率 (1us/tick)
    if (servo == BSP_SERVO_1) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_1, us);
    }
    else if (servo == BSP_SERVO_2) {
        __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_2, us);
    }
}

/* ============================================================ */
/* 安全停机                                                     */
/* ============================================================ */
void BSP_PWM_StopAll(void) {
    // 1. BLDC 归零 (或直接 Stop)
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);

    // 2. 舵机保持当前位置或归中?
    // 安全起见，通常不主动切断舵机信号，防止掉电松脱，
    // 但如果有 Enable 引脚，应在 GPIO BSP 中控制 Disable。
}