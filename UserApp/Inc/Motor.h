    //
// Created by User on 2025/10/17.
//

#ifndef FPV_CTRL_DEMO_MOTOR_H
#define FPV_CTRL_DEMO_MOTOR_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define ENCODER_LINE 13
#define GEAR_RATIO 20.409
#define ENCODER_PPR (ENCODER_LINE * 4)
#define GEAR_PPR (ENCODER_LINE * GEAR_RATIO)

void Motor_SetDirection(int8_t dir);
void Motor_SetDuty(uint32_t pwm_val);

#endif //FPV_CTRL_DEMO_MOTOR_H
