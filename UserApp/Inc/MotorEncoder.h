//
// Created by User on 2025/10/17.
//

#ifndef FPV_CTRL_DEMO_MotorEncoder_H
#define FPV_CTRL_DEMO_MotorEncoder_H

#include "stm32f1xx_hal.h"
#include "main.h"

#include "stm32f1xx_hal.h"
#include <stdint.h>

/* --------------------------------------------------------------
 *  对外结构体（中断安全，volatile 防止编译器优化）
 * -------------------------------------------------------------- */
typedef struct
{
    volatile uint32_t  position;    // 累计脉冲数
    volatile int16_t  direction;    // -1、0、+1
    volatile float    speed_rpm;    // 减速后转速（rpm）
    float    angle_deg;             // 关节角度（机械映射后）
} MotorEncoder_t;

/* --------------------------------------------------------------
 *  公共 API
 * -------------------------------------------------------------- */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_timer,
                       float lines_per_rev);   // 例如 265.2f

int16_t MotorEncoder_GetDir(void);
float   MotorEncoder_GetSpeed(void);
uint32_t MotorEncoder_GetPos(void);

/* 供 HAL 中断回调使用（CubeMX 生成的函数） */
void MotorEncoder_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif //FPV_CTRL_DEMO_MotorEncoder_H
