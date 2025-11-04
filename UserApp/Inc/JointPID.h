//
// Created by User on 2025/10/31.
//

#ifndef FPV_CTRL_DEMO_JOINTPID_H
#define FPV_CTRL_DEMO_JOINTPID_H

typedef struct {
    float Kp, Ki, Kd;
    float err_last, err_sum, err_diff;
    float out_max, out_min;
    float integral_limit;
}PID_Controller_t;

#endif //FPV_CTRL_DEMO_JOINTPID_H
