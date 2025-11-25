/**
  ******************************************************************************
  * @file    bsp_pwm.h
  * @brief   BSP PWM 驱动 (FOC 3路PWM + 2路舵机PWM)
  * @author  ARA Project Team
  ******************************************************************************
  */

#ifndef __BSP_PWM_H
#define __BSP_PWM_H

#include "stm32f1xx_hal.h" // 根据实际芯片型号调整

/* --- 设备定义 --- */

/** * @brief 舵机通道枚举
 * 对应 TIM2 的不同 Channel
 */
typedef enum {
    BSP_SERVO_1 = 0,    // 舵机1 (例如: 夹爪) - TIM2 CH1
    BSP_SERVO_2,        // 舵机2 (例如: 辅助关节) - TIM2 CH2
    BSP_SERVO_NUM
} BspServo_Dev_t;

/* --- 初始化 --- */
void BSP_PWM_Init(void);

/* --- BLDC FOC 控制接口 --- */
/**
 * @brief  设置 BLDC 3相占空比 (FOC 专用)
 * @param  dc_a: A相占空比 (0.0f ~ 1.0f)
 * @param  dc_b: B相占空比 (0.0f ~ 1.0f)
 * @param  dc_c: C相占空比 (0.0f ~ 1.0f)
 * @note   对应 TIM1 CH1/CH2/CH3
 */
void BSP_PWM_Set3PhaseDuty(float dc_a, float dc_b, float dc_c);

/* --- 舵机控制接口 --- */
/**
 * @brief  设置舵机脉宽
 * @param  servo: 舵机编号 (BSP_SERVO_1 / BSP_SERVO_2)
 * @param  us:    脉宽值，单位微秒 (通常范围 500 - 2500)
 */
void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us);

/* --- 安全停机 --- */
/**
 * @brief  停止所有 PWM 输出 (紧急停机用)
 */
void BSP_PWM_StopAll(void);

#endif // __BSP_PWM_H