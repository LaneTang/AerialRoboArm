//
// Created by User on 2025/10/17.
//

#include "Motor.h"
#include "tim.h"

// 全局变量用于速度计算
int32_t last_cnt = 0;
uint32_t last_time = 0;


/**
 * MotorState           Stop    Forward Backward
 * @param IN1(PA11)     0       0       1
 * @param IN2(PA10)     0       1       0
 */
void Motor_SetDirection(int8_t dir) {
    if (dir == 1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_SET);
    } else if (dir == -1) {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOA, GPIO_PIN_10, GPIO_PIN_RESET);
    }


}

void Motor_SetDuty(uint32_t pwm_val) { // 0 ~ 19999
    if (pwm_val >= 19999) pwm_val = 19999;
    else if (pwm_val <= 0) pwm_val = 0;

    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_2, pwm_val);

}