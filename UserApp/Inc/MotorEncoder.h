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
 *  编码器参数宏定义
 * -------------------------------------------------------------- */
#define ENCODER_LINE_COUNT 11
#define REDUCTION_RATIO 20.4
#define PULSE_PER_REVOLUTION (ENCODER_LINE_COUNT*REDUCTION_RATIO)

/* --------------------------------------------------------------
 *  对外结构体（中断安全，volatile 防止编译器优化）
 * -------------------------------------------------------------- */
typedef struct
{
    volatile uint32_t  position;    // 累计脉冲数
    volatile int8_t  direction;    // -1、0、+1
    volatile float    speed_rpm;    // 减速后转速（rpm）
    float    angle_deg;             // 关节角度（机械映射后）
} MotorEncoder_t;

/* --------------------------------------------------------------
 *  新增：调零函数
 * -------------------------------------------------------------- */
typedef enum {
    HOMING_IDLE,
    HOMING_RUNNING,
    HOMING_SUCCESS,
    HOMING_TIMEOUT,
    HOMING_FAILED
} HomingState_t;

HomingState_t MotorEncoder_Homing_Start(
        GPIO_TypeDef *hall_port, uint16_t hall_pin,
        float homing_speed_rpm,      // 建议 10~30 RPM
        uint32_t timeout_ms          // 超时保护
);

HomingState_t MotorEncoder_GetHomingState(void);

/* --------------------------------------------------------------
 *  公共 API
 * -------------------------------------------------------------- */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_timer,
                       float lines_per_rev);   // 例如 265.2f

int16_t MotorEncoder_GetDir(void);
float   MotorEncoder_GetSpeed(void);
float   MotorEncoder_GetAngle(void);
uint32_t MotorEncoder_GetPos(void);


/* 供 HAL 中断回调使用（CubeMX 生成的函数） */
void MotorEncoder_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);

#endif //FPV_CTRL_DEMO_MotorEncoder_H
