//
// Created by User on 2025/10/17.
//

#ifndef FPV_CTRL_DEMO_MotorEncoder_H
#define FPV_CTRL_DEMO_MotorEncoder_H

#include "stm32f1xx_hal.h"
#include "main.h"
#include "Motor.h"
#include <stdint.h>
#include <string.h>
#include <stdio.h>



/* --------------------------------------------------------------
 *  编码器参数宏定义
 * -------------------------------------------------------------- */


/* --------------------------------------------------------------
 *  对外结构体（中断安全，volatile 防止编译器优化）
 * -------------------------------------------------------------- */
typedef struct
{
    volatile uint32_t   position;    // 累计脉冲数
    volatile int        direction;     // -1、0、+1
    volatile float      speed_rpm;    // 减速后转速（rpm）
    float    angle_deg;             // 关节角度（机械映射后）
} MotorEncoder_t;

/* --------------------------------------------------------------
 *  调零参数定义
 * -------------------------------------------------------------- */
#define HOMING_LOW_PWM_DUTY (2000   ) // 10% 占空比，低速调零

/* --------------------------------------------------------------
 * Homing 状态定义
 * -------------------------------------------------------------- */
typedef enum {
    HOMING_IDLE = 0,
    HOMING_RUNNING,
    HOMING_SUCCESS,
    HOMING_TIMEOUT,
    HOMING_FAILED
} HomingState_t;

// Homing Start API
HomingState_t MotorEncoder_Homing_Start(float homing_speed_rpm,
                                        uint32_t timeout_ms);

// Homing State Check API
HomingState_t MotorEncoder_CheckHomingState(uint32_t timeout_ms);

// EXTI 外部中断回调中调用的函数
void MotorEncoder_HOMED_EXTI_Callback(void);

/* --------------------------------------------------------------
 *  公共 API
 * -------------------------------------------------------------- */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_timer);   // 例如 265.2f

int     MotorEncoder_GetDir(void);
float   MotorEncoder_GetSpeed(void);
float   MotorEncoder_GetAngle(void);
uint32_t MotorEncoder_GetPos(void);


/* TIM2 编码器测速 中断回调函数 */
void MotorEncoder_Sensor_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif //FPV_CTRL_DEMO_MotorEncoder_H
