#ifndef __MOTOR_H
#define __MOTOR_H
#include "../../../UserApp/bsp/Inc/Motor.h"
#define PWM_Period      1000

extern uint8_t  Left_Motor_PWM;
extern uint8_t  Right_Motor_PWM;

void Motor_PWM_Init(void);
void L_Wheel_MoveForward(void);
void R_Wheel_MoveForward(void);
void L_Wheel_SetSpeed(float pwm_val);
void R_Wheel_SetSpeed(float pwm_val);
void Wheel_Halt(void);

#endif
