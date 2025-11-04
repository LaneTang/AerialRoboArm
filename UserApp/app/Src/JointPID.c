//
// Created by User on 2025/10/31.
//

#include "JointPID.h"


static TIM_HandleTypeDef *hpid = NULL;   // TIM4（编码器）

/* --------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------- */
void JointPID_Init(TIM_HandleTypeDef *hpid_timer, PID_Controller_t *pid,
                   float Kp, float Ki, float Kd,
                   float out_min_val, float out_max_val, float integral_lim)
{
    hpid = hpid_timer;

    /* 启动 PID 计算定时器 */
    HAL_TIM_Base_Init(hpid);

    /* PID参数 初始化 */
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->out_min = out_min_val;
    pid->out_max = out_max_val;

    pid->integral_limit = integral_lim;

    pid->err_sum = 0.0f;
    pid->err_last = 0.0f;
    pid->err_diff = 0.0f;

}

void JointPID_Config_Init(TIM_HandleTypeDef *hpid_timer,
                          PID_Controller_t *pid, PID_Config_t *pid_cfg)
{
    hpid = hpid_timer;

    /* 启动 PID 计算定时器 */
    HAL_TIM_Base_Init(hpid);

    /* PID参数结构体 初始化 */
    pid->Kp = pid_cfg->Kp;
    pid->Ki = pid_cfg->Ki;
    pid->Kd = pid_cfg->Kd;

    pid->out_min = pid_cfg->out_min;
    pid->out_max = pid_cfg->out_max;

    pid->integral_limit = pid_cfg->integral_limit;

    pid->err_sum = 0.0f;
    pid->err_last = 0.0f;
    pid->err_diff = 0.0f;

}


/* --------------------------------------------------------------
 *  PID 计算器
 *  计算单位（Deg）
 *  输出值带方向，需要甄别
 * -------------------------------------------------------------- */
float PID_Calc(PID_Controller_t *pid, float target, float feedback)
{
    float err = target - feedback;

    /* 积分环节 */
    pid->err_sum += err;
    if (pid->err_sum >= pid->integral_limit) pid->err_sum = pid->integral_limit;
    if (pid->err_sum <= -pid->integral_limit) pid->err_sum = -pid->integral_limit;


    /* 微分环节 */
    pid->err_diff = err - pid->err_last;
    pid->err_last = err;

    /* 输出 */
    float output = (pid->Kp * err) + (pid->Ki * pid->err_sum) + (pid->Kd * pid->err_diff);

    /* 输出限幅 */
    if (output > pid->out_max) output = pid->out_max;
    if (output < pid->out_min) output = pid->out_min;

    /* 抗积分饱和：输出饱和时回退积分 */
    if (output == pid->out_max || output == pid->out_min) {
        pid->err_sum -= err * 0.1f;  // 轻微回退
    }

    return output;
}

/* --------------------------------------------------------------
 *  PID Control 中断回调函数
 *  TIM2
 * -------------------------------------------------------------- */
void PID_Control_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim,
                                           PID_Controller_t *pid,
                                           float target)
{
    if(htim != hpid) return;

    /* 1. 读取电机角度 */
    float cur_angle = MotorEncoder_GetAngle();

    /* 2. 计算PID */
    float pid_out_pwm = PID_Calc(pid, target, cur_angle);

    /* 3. 输出合法化 */
    int8_t dir = 1;
    if (pid_out_pwm < 0) {
        dir = -1;
        pid_out_pwm = -pid_out_pwm;
    }
    uint32_t pwm_new = (uint32_t )pid_out_pwm;

    /* 4. 写入PWM */
    Motor_SetDirection(dir);
    Motor_SetDuty(pwm_new);

}






