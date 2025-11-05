    //
// Created by User on 2025/10/17.
//

#ifndef FPV_CTRL_DEMO_MOTOR_H
#define FPV_CTRL_DEMO_MOTOR_H

#include "stm32f1xx_hal.h"
#include "main.h"

#define ENCODER_LINE_COUNT 11
#define REDUCTION_RATIO 20.409
#define PULSE_PER_REVOLUTION ( ENCODER_LINE_COUNT * REDUCTION_RATIO )

void Motor_SetDirection(int8_t dir);
void Motor_SetDuty(uint32_t pwm_val);
void Motor_Stop(void);

#endif //FPV_CTRL_DEMO_MOTOR_H
