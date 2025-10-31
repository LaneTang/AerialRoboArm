//
// Created by User on 2025/10/31.
//

#include "JointPID.h"
#include "main.h"

static TIM_HandleTypeDef *hpid = NULL;   // TIM4（编码器）

/* --------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------- */
void JointPID_Init(TIM_HandleTypeDef *hpid_timer,
                   PID_Controller_t *pidController,
                   float Kp, float Ki, float Kd,
                   float out_min_val, float out_max_val)
{
    hpid = hpid_timer;

    /* 启动 PID 计算定时器 */
    HAL_TIM_Base_Init(hpid);

    /* 参数赋值 */
    pidController->Kp = Kp;
    pidController->Ki = Ki;
    pidController->Kd = Kd;

    pidController->out_min = out_min_val;
    pidController->out_max = out_max_val;

    /* 状态清零 */
    pidController->err_last  = 0.0f;
    pidController->err_sum   = 0.0f;
    pidController->err_diff  = 0.0f;

}

/**
 * PID计算器
 * @TIM2 Update 调用
 * @param pid
 * @param target, feedback 输入 参考值, 反馈值 - 单位为 Degree
 * @return output 输出 PWM占空比 (0 ~ 19999) -
 */
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








