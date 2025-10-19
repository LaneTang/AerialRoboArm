#include "ti_msp_dl_config.h"
#include "Motor.h"


/* 电机控制 */
/** Motor Direction Control
    IN1 IN2  Function
     0   0     Stop
     0   1   Forward
     1   0   Backward
*/

uint8_t  Left_Motor_PWM;
uint8_t  Right_Motor_PWM;


// M1
void M1_Wheel_Direction(uint8_t IN1, uint8_t IN2) {
    if (!IN1 && !IN2) {     // 0   0     Stop
        DL_GPIO_clearPins(GPIO_Motor_PIN_1IN1_PORT, GPIO_Motor_PIN_1IN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PIN_1IN2_PORT, GPIO_Motor_PIN_1IN2_PIN);
    }
    else if (!IN1 && IN2) { // 0   1   Forward
        DL_GPIO_clearPins(GPIO_Motor_PIN_1IN1_PORT, GPIO_Motor_PIN_1IN1_PIN);
        DL_GPIO_setPins(GPIO_Motor_PIN_1IN2_PORT, GPIO_Motor_PIN_1IN2_PIN);
    }
    else if (IN1 && !IN2) { // 1   0   Backward
        DL_GPIO_setPins(GPIO_Motor_PIN_1IN1_PORT, GPIO_Motor_PIN_1IN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PIN_1IN2_PORT, GPIO_Motor_PIN_1IN2_PIN);
    }
}

// M2 - 电机出问题，现在是IN1/IN2 = 1,0 -> Forward
void M2_Wheel_Direction(uint8_t IN1, uint8_t IN2) {
    if (!IN1 && !IN2) {     // 0   0     Stop
        DL_GPIO_clearPins(GPIO_Motor_PIN_2IN1_PORT, GPIO_Motor_PIN_2IN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PIN_2IN2_PORT, GPIO_Motor_PIN_2IN2_PIN);
    }
    else if (!IN1 && IN2) { // 0   1   Forward
        DL_GPIO_clearPins(GPIO_Motor_PIN_2IN1_PORT, GPIO_Motor_PIN_2IN1_PIN);
        DL_GPIO_setPins(GPIO_Motor_PIN_2IN2_PORT, GPIO_Motor_PIN_2IN2_PIN);
    }
    else if (IN1 && !IN2) { // 1   0   Backward
        DL_GPIO_setPins(GPIO_Motor_PIN_2IN1_PORT, GPIO_Motor_PIN_2IN1_PIN);
        DL_GPIO_clearPins(GPIO_Motor_PIN_2IN2_PORT, GPIO_Motor_PIN_2IN2_PIN);
    }
}

/**PWM
    * pwm_val取值（Period）： 0~1000
*/ 
void PWM_0_SetPWM(float pwm_val){
    if (pwm_val >= 1000) pwm_val = 1000;
    else if (pwm_val <= 0) pwm_val = 0;

    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, pwm_val, DL_TIMER_CC_0_INDEX);
}

void PWM_1_SetPWM(float pwm_val){
    if (pwm_val >= 1000) pwm_val = 1000;
    else if (pwm_val <= 0) pwm_val = 0;

    DL_TimerG_setCaptureCompareValue(PWM_Motor_INST, pwm_val, DL_TIMER_CC_1_INDEX);
}

/*External Function*/
void Motor_PWM_Init(void) {
    DL_TimerG_startCounter(PWM_Motor_INST); // PWM Init
}

void L_Wheel_MoveForward(void) {
    M1_Wheel_Direction(0, 1);
}

void R_Wheel_MoveForward(void) {
    M2_Wheel_Direction(1, 0);
}

void L_Wheel_SetSpeed(float pwm_val) {
    Left_Motor_PWM = pwm_val;
    PWM_1_SetPWM(Left_Motor_PWM);
}

void R_Wheel_SetSpeed(float pwm_val) {
    Right_Motor_PWM = pwm_val;
    PWM_0_SetPWM(Right_Motor_PWM);
}

void Wheel_Halt(void) {
    L_Wheel_SetSpeed(0);
    R_Wheel_SetSpeed(0);
}


