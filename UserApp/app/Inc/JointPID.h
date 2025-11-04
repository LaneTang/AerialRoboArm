//
// Created by User on 2025/10/31.
//

#ifndef FPV_CTRL_DEMO_JOINTPID_H
#define FPV_CTRL_DEMO_JOINTPID_H

#include "Motor.h"
#include "main.h"
#include "MotorEncoder.h"

typedef struct {
    float Kp, Ki, Kd;
    float err_last, err_sum, err_diff;
    float out_max, out_min;
    float integral_limit;
}PID_Controller_t;

typedef struct {
    float Kp, Ki, Kd;
    float out_max, out_min;
    float integral_limit;
}PID_Config_t;

/* 初始化函数 */
void JointPID_Init(TIM_HandleTypeDef *hpid_timer, PID_Controller_t *pid, float Kp, float Ki, float Kd,
                   float out_min_val, float out_max_val, float integral_lim);


void JointPID_Config_Init(TIM_HandleTypeDef *hpid_timer, PID_Controller_t *pid, PID_Config_t *pid_cfg);


/* 事件回调函数 - TIM2注册 */
void PID_Control_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim, PID_Controller_t *pid, float target);

#endif //FPV_CTRL_DEMO_JOINTPID_H
