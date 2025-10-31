//
// Created by User on 2025/10/31.
//

#ifndef FPV_CTRL_DEMO_JOINTPID_H
#define FPV_CTRL_DEMO_JOINTPID_H

typedef struct {
    /* 参数 */
    float Kp;               // 比例增益
    float Ki;               // 积分增益
    float Kd;               // 微分增益

    /* 限幅 */
    float out_min;          // 输出下限（PWM计数）
    float out_max;          // 输出上限
    float integral_limit;   // 积分项限幅（防饱和）

    /* 功能 */
//    float deadband;         // 死区（°），防止微小误差抖动
//    float anti_windup_gain; // 抗积分饱和回退系数（0.0 ~ 1.0）

    /* 内部状态（必须初始化为0） */
    float err_last;         // 上次误差
    float err_sum;          // 积分累计
    float err_diff;         // 微分项

} PID_Controller_t;

typedef struct {
    float kp, ki, kd;
    float out_min, out_max;
    float deadband;
} PID_Config_t;

#endif //FPV_CTRL_DEMO_JOINTPID_H
