//
// Created by User on 2025/10/31.
//

#include "JointPID.h"
#include "main.h"

PID_Controller_t joint_pid = {
    .Kp = 1,
    .Ki = 0,
    .Kd = 0,
    .out_min = 1000,
    .out_max = 18000,
    .integral_limit = 1000
};

static TIM_HandleTypeDef *hpid = NULL;   // TIM4（编码器）

/* --------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------- */
void JointPID_Init(TIM_HandleTypeDef *hpid_timer, PID_Controller_t *pid,
                   float Kp, float Ki, float Kd,
                   float out_min_val, float out_max_val)
{
    hpid = hpid_timer;

    /* 启动 PID 计算定时器 */
    HAL_TIM_Base_Init(hpid);

    /* PID参数结构体 初始化 */
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->out_min = out_min_val;
    pid->out_max = out_max_val;



}



/* --------------------------------------------------------------
 *  PID 计算器
 *  计算单位（Deg）
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







