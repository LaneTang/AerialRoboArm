/*
 * Copyright (c) 2020, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
#include <stdio.h>
#include "ti/devices/msp/m0p/mspm0g350x.h"
#include "ti_msp_dl_config.h"
#include "Serial.h"
#include "Track.h"
#include "Motor.h"
#include "oled.h"
#include "math.h"


#define PI              3.14159
#define PWM_Period      1000
#define Count_Period    500
#define Count_Freq      (1000/Count_Period)

uint8_t z_reset[3]={0xFF,0xAA,0x52};		// 0xFF,0xAA,0x52


uint8_t             M1_Count = 0;
uint8_t             M2_Count = 0;

// float               L_Speed;
// float               R_Speed;

// Angle params
uint8_t status;
float pitch=0,roll=0,yaw=0;

uint8_t isBlack;

/* PID params */ 
// 速度环参数
double Speed_Left=0;
double Speed_Right=0;
double PID_OutputL;
double PID_OutputR;
double L_Speed_Desire;
double R_Speed_Desire;

double err_L_last, err_L;
double P_Left, I_Left, D_Left;
double KpL = 0.1;
double KiL = 0;
double KdL = 0;

double err_R_last, err_R;
double P_Right, I_Right, D_Right;
double KpR = 0.1;
double KiR = 0;
double KdR = 0;

double dErrL=0;
double dErrR=0;

float Ts = 0.2; //  PID更新间隔

// 角度环参数
int toward = 0;
double P_Ang, I_Ang, D_Ang;
double Kp = 1.5;
double Ki = 0.4;
double Kd = 0;
float desireAng, currentAng;
float PID_Ang_output;
float errAng, errAngLast, errAngDiff;
uint16_t left_pwm, right_pwm;

float basePWM = 200;
uint8_t prompt_time_ms = 100;

/** 小车状态机状态变量
****************
    * 小车执行的问题：
    * Q1 ~ 4 - 1 ~ 4
    * Qdebug - 5
****************
    * 小车的运行状态:
    * 待机 - 0
    * 空地上自动跑 - 1
    * 在黑线上面循迹 - 2
    * 调试 - 3
    * 上电启动 - 114
****************
    * 小车运行朝向（设ABCD4个角为1234）：
    * AtoB - 12
    * onBC - 23
    * CtoD - 34
    * onAD - 14
    * AtoC - 13
    * BtoD - 24
    * initial - 514
****************
*/
typedef enum {
    Q1 = 1,
    Q2,
    Q3,
    Q4,
    Qdebug
}Question;

typedef enum {
    standby = 0,
    followLine = 1,
    followPath = 2,
    debug = 3,
    justGo = 8,
    powerUp = 114
}working_condition;

typedef enum {
    AtoB = 12,
    onBC = 23,
    CtoD = 34,
    onAD = 14,
    AtoC = 13,
    BtoD = 24,
    initial = 514
}car_direction;

// 状态变量
working_condition carStatus;    // 小车工作状态
Question question;              // 正在做的问题
car_direction direction;        // 小车的朝向
uint8_t Q4_loop;                // 第四问的圈数

/* 函数封装 */
void sys_init(void) {
    SYSCFG_DL_init();

    /* UART0 - 串口调试 */
	NVIC_ClearPendingIRQ(UART_0_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_0_INST_INT_IRQN); 

    /* UART1 - 角度接收 */
    NVIC_ClearPendingIRQ(UART_JY60_INST_INT_IRQN);
	NVIC_EnableIRQ(UART_JY60_INST_INT_IRQN);
    
    /*OLED*/
    // OLED_Init();
    // OLED_ShowString(6, 6, (uint8_t*)"tomorrow KFC!", 4);

    /* EXTI INT - Encoder Counting & Button Event*/
    // NVIC_EnableIRQ(GPIOB_INT_IRQn);


    /* TIMER - Calc PID */
    DL_TimerA_startCounter(TIMER_computePID_INST);
    NVIC_EnableIRQ(TIMER_computePID_INST_INT_IRQN);

    /* TIMER - Send Debug Info*/
    DL_TimerA_startCounter(TIMER_Debug_INST);
    NVIC_EnableIRQ(TIMER_Debug_INST_INT_IRQN);

    /* Motor */
    DL_TimerG_startCounter(PWM_Motor_INST);     // PWM Init

    // Wheel init
    L_Wheel_MoveForward();
    R_Wheel_MoveForward();
    L_Wheel_SetSpeed(0);                        // Set Motor Speed 
    R_Wheel_SetSpeed(0);
}

void follow_straight_line(float desAng) {
    desireAng = desAng;
    // 作用于轮子 
    L_Wheel_SetSpeed(basePWM - PID_Ang_output);
    R_Wheel_SetSpeed(basePWM + PID_Ang_output);
}

// 归零PID参数
void reset_PID(void) {
    desireAng = 0;
    currentAng = 0;
    PID_Ang_output = 0;
    errAng = 0; 
    errAngLast = 0;
    errAngDiff = 0;
    P_Ang = 0;
    I_Ang = 0;
    D_Ang = 0;
    UART_transmitString(UART0, "-------- RESET PID --------\r\n");
}

// yaw angle reset to 0
void reset_yaw(void) {
    UART_transmitArray(UART_JY60_INST, z_reset, 3);
    UART_transmitString(UART0, "-------- RESET YAW --------\r\n");
}

// Buzzer and LED work for customed ms 
void prompt_toggle() { 
    DL_GPIO_togglePins(GPIOA, GPIO_Flag_PROMPT_Buzzer_PIN | GPIO_Flag_PROMPT_LED_PIN);
}

// 按键确认状态
void state_confirm(void) {
    // when button pressed, GPIO input level change and break while
    // !!!need to modify to fix pins logic!!!

    // Key value aquisition
    // Pull-Up resistor - high level if not pressed
    uint32_t isPressed = DL_GPIO_readPins(GPIO_Questions_PORT, GPIO_Questions_PIN_Q1_PIN | GPIO_Questions_PIN_Q2_PIN 
        | GPIO_Questions_PIN_Q3_PIN | GPIO_Questions_PIN_Q4_PIN);
    

    // if detect low level --> button pressed
    if (isPressed) delay_ms(20);
    
    while (1) {  
        if ((isPressed & GPIO_Questions_PIN_Q1_PIN) ==
            GPIO_Questions_PIN_Q1_PIN) {
                carStatus = followLine;
                question = Q1;
                direction = AtoB;
                break;
        }

        else if ((isPressed & GPIO_Questions_PIN_Q2_PIN) ==
            GPIO_Questions_PIN_Q2_PIN) {
                carStatus = followLine;
                question = Q2;
                direction = AtoB;
                delay_ms(20);   // --> button release
                break;
        }

        else if ((isPressed & GPIO_Questions_PIN_Q3_PIN) ==
            GPIO_Questions_PIN_Q3_PIN) {
                carStatus = followLine;
                question = Q3;
                direction = AtoC;
                delay_ms(20);   // --> button release
                break;
        }

        else if ((isPressed & GPIO_Questions_PIN_Q4_PIN) ==
            GPIO_Questions_PIN_Q4_PIN) {
                carStatus = followLine;
                question = Q4;
                direction = AtoC;
                Q4_loop = 0;
                delay_ms(20);   // --> button release
                break;
        }
    }

    printf("-------- BUTTON PRESS! %d SELECTED! --------", question);
}

uint8_t isBlack = 0b00000000;
uint8_t isBlack_old;

// 通过对状态反复检测是否已经进入黑线了，消除抖动
uint8_t detect_blackline() {
    if (meet_black()) {
        isBlack = (isBlack | 0b00000001) << 1;
    }
    isBlack = isBlack << 1;
}

int main(void) {
    UART_transmitString(UART0, "-------- SYS INIT START --------\r\n");

    sys_init();

    UART_transmitString(UART0, "-------- SYS INIT DONE --------\r\n");

    // init and reset params
    carStatus = powerUp;
    direction = initial;
    question = Qdebug;
    reset_PID();
    reset_yaw();

    UART_transmitString(UART0, "-------- State init done --------\r\n");

    /* 1. 确定小车的工作状态 */ 
    // printf("\r\n\r\n START STATE CONFIRM! \r\n\r\n");
    // state_confirm();
    // printf("\r\n\r\n DONE STATE CONFIRM!  \r\n\r\n");

    carStatus = followLine;
    direction = AtoB;
    question = Q2;
    
    /* 2. 开始运行 */ 
    while (1) {
        
        if (!DL_GPIO_readPins(GPIO_Flag_PIN_ResetYaw_PORT, GPIO_Flag_PIN_ResetYaw_PIN)) {
            reset_PID();
            reset_yaw();
        }

        /* sensor info update */
        // speed measurement - M1_Count 是在50ms周期内测出的编码器读到的角度
        // L_Speed = M1_Count * Count_Freq * PI * 48 / 330;
        // R_Speed = M2_Count * Count_Freq * PI * 48 / 330;
        
        // 灰度检测
        if (meet_black()) {
            isBlack = (isBlack | 0b00000001) << 1;
        }
        isBlack = meet_black();

        switch (question) {
            case Q1: {      //Q1
                
                if (carStatus == followLine) {              // 直行
                    follow_straight_line(0);
                    if (isBlack & isBlack_old) {
                        carStatus = standby;
                    }
                } else if (carStatus == standby) {       // 待机
                    Wheel_Halt();
                    prompt_toggle();
                    delay_ms(prompt_time_ms);
                    prompt_toggle();
                }

                break;
            }
            case Q2: {  //  A - B | B - C | C - D | D - A
                // A-B-C-D
                if (carStatus == followLine && direction == AtoB) {
                    follow_straight_line(0);

                    if (isBlack & isBlack_old ) {
                        // prompt_toggle();

                        carStatus = followPath;
                        direction = onBC;
                        // 走弧线了，现在要冻结PID
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);
                        // 复位PID值
                        reset_PID();
                        
                        // delay_ms(prompt_time_ms);
                        // prompt_toggle();
                    }
                } else if (carStatus == followPath && direction == onBC) {
                    follow_blackline_multiChannel(isBlack);

                    if (!(isBlack || isBlack_old) ) {  // 出BC
                        // prompt_toggle();

                        carStatus = followLine;
                        direction = CtoD;
                        NVIC_EnableIRQ(UART_JY60_INST_INT_IRQN);    // PID enable

                        // delay_ms(prompt_time_ms);
                        // prompt_toggle();
                    }
                } else if (carStatus == followLine && direction == CtoD) {
                    follow_straight_line(180);

                    if (isBlack & isBlack_old ) {
                        carStatus = followPath;
                        direction = onAD;
                        // 复位PID值
                        reset_PID();
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);

                        // prompt_toggle();
                        // delay_ms(prompt_time_ms);
                        // prompt_toggle();
                    }
                } else if (carStatus == followPath && direction == onAD) {

                    if (!(isBlack || isBlack_old) ) {
                        // prompt_toggle();

                        carStatus = standby;

                        // delay_ms(prompt_time_ms);
                        // prompt_toggle();
                    }
                } else if (carStatus == standby) {

                    Wheel_Halt();
                }

                break;
            }    
            case Q3: {   

                // A-C-B-D
                if (carStatus == followLine && direction == AtoC) {  // A - C

                    if (isBlack && isBlack_old) {
                        prompt_toggle();
                        carStatus = followPath;
                        direction = onBC;
                        // 走弧线了，现在要冻结PID
                        // 复位PID值
                        reset_PID();
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                    else follow_straight_line(305);     // A - C 沿50°
                } else if (carStatus == followPath && direction == onBC) {  // C - B

                    if (!(isBlack || isBlack_old)) {  // 出BC
                        prompt_toggle();

                        carStatus = followLine;
                        direction = BtoD;
                        NVIC_EnableIRQ(UART_JY60_INST_INT_IRQN);    // PID enable

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                    else follow_blackline_multiChannel(isBlack);
                } else if (carStatus == followLine && direction == BtoD) {    // B - D

                    if (isBlack && isBlack_old) { 
                        prompt_toggle();  

                        carStatus = followPath;
                        direction = onAD;
                        // 复位PID值
                        reset_PID();
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                    else follow_straight_line(235);    // B - D 沿-50°
                } else if (carStatus == followPath && direction == onAD) {   // D - A

                    if (!(isBlack || isBlack_old)) {    // 出AD
                        prompt_toggle();

                        carStatus = standby;

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                } else if (carStatus == standby) {
                    Wheel_Halt();
                }
                
                break;
            }
            case Q4: {
                // A-C-B-D
                if (carStatus == followLine && direction == AtoC) {  // A - C

                    if (isBlack && isBlack_old) {
                        prompt_toggle();

                        carStatus = followPath;
                        direction = onBC;
                        // 走弧线了，现在要冻结PID
                        // 复位PID值
                        reset_PID();
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    } 
                    else follow_straight_line(305);     // A - C 沿50°
                } else if (carStatus == followPath && direction == onBC) {  // C - B

                    if (!(isBlack || isBlack_old)) {  // 出BC
                        prompt_toggle();

                        carStatus = followLine;
                        direction = BtoD;
                        NVIC_EnableIRQ(UART_JY60_INST_INT_IRQN);    // PID enable

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    } 
                    else follow_blackline_multiChannel(isBlack);
                } else if (carStatus == followLine && direction == BtoD) {    // B - D
                
                    if (isBlack && isBlack_old) { 
                        prompt_toggle();  

                        carStatus = followPath;
                        direction = onAD;
                        // 复位PID值
                        reset_PID();
                        // 失能TIM, 关闭PID功能
                        NVIC_DisableIRQ(UART_JY60_INST_INT_IRQN);

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                    else follow_straight_line(235);    // B - D 沿-50°
                } else if (carStatus == followPath && direction == onAD) {   // D - A

                    if (!(isBlack || isBlack_old)) {    // 出AD
                        prompt_toggle();

                        // for reach end of a loop, count 1 time and break after 4 loops 
                        if (Q4_loop++ >=4) {   
                            carStatus = standby;
                        }else {
                            carStatus = followLine;
                            direction = AtoC;
                        }

                        delay_ms(prompt_time_ms);
                        prompt_toggle();
                    }
                } else if (carStatus == standby) {
                    Wheel_Halt();
                }
                break;
            }
            case Qdebug: {
                // A - B - C
                // 直行时遇到黑线
                // if (carStatus == followLine && isBlack && direction == AtoB) {
                //     carStatus = followPath;
                //     direction = onBC;
                // }  
                // else if (carStatus == followPath && !isBlack && direction == onBC) {
                //     carStatus = standby;              // 离开圆弧
                //     direction = CtoD;
                // }

                // follow_blackline_new(isBlack);
                // if (carStatus == followPath) {
                //     follow_blackline(isBlack);
                //     if (!(isBlack || isBlack_old)) {    // 如果 double check 都不在线上 - 认为出弧线了
                //         carStatus = standby;
                //     }
                // }
                // else if (carStatus == standby) {
                //     //  Wheel_Halt();
                // }
                
                /* follow fixed angle to move */
                follow_straight_line(0);
                // follow_straight_line(305);
                

                /* follow black line to move */
                // follow_blackline_multiChannel(isBlack);

                break;
            }
            default:    break;
        }

        /* Rx data package Processing*/

        if (RxFlag_DEBUG) {
            RxFunc();
        }
        if (RxFlag_JY60) {
            // Ang Update
            roll  = gRxAngDataBuf[1]<<8 | gRxAngDataBuf[0];
			pitch = gRxAngDataBuf[3]<<8 | gRxAngDataBuf[2];
			yaw   = gRxAngDataBuf[5]<<8 | gRxAngDataBuf[4];
            
            // ang range = 0 ~ 360
            roll  = roll  / 32768 * 180;
            pitch = pitch / 32768 * 180;
            yaw   = yaw   / 32768 * 180;
        }
        
        // printf("blackChannel=%d, ", isBlack);
        for (int i = 0; i < 7; i++) printf("%d",tubes[i]);
        printf("\r\n");

        // Update Gray Info
        isBlack_old = isBlack;
        delay_ms(50);
    }
}


/* 控制函数 */
void computeAngPID(void) {
    // 角度环PID，可能的角度取值有: 0 ~ 360，0 表示直行
    currentAng = yaw;
    errAng = desireAng - currentAng;
    
    // A to B 时
    // 确保errAng在0到360之间
    // if (direction == AtoB || direction == AtoC) {

    // }
    if (errAng > 180) {
        errAng -= 360;
    } else if (errAng < -180) {
        errAng += 360;
    }

    // 更新角度偏差的方向
    if (errAng < 0) { // 反向偏差, 说明左偏了
        toward = -1;
    } else { // 正向偏差, 说明右偏了
        toward = 1;
    }

    // 比例项
    P_Ang = Kp * errAng;

    // 积分项，积分限值
    if (I_Ang < 800 && I_Ang > -800) {
        I_Ang += Ki * (errAng * Ts);
    }

    // 误差微分
    errAngDiff = (errAng - errAngLast) / Ts;

    // 微分项
    D_Ang = Kd * errAngDiff;

    // 输出
    PID_Ang_output = P_Ang + I_Ang + D_Ang;
    
    if (PID_Ang_output <= -300) {
        PID_Ang_output = -300;
    } else if (PID_Ang_output >= 300) {
        PID_Ang_output = 300;
    }

    // 保存上一次的误差
    errAngLast = errAng;
}

void computeWheelPID() {
    // 左轮的PID
    left_pwm = Left_Motor_PWM;
    err_L = L_Speed_Desire - left_pwm;           // 误差
    P_Left = KpL * err_L;                       // 比例项
    if(PID_OutputL < 1000 && PID_OutputL > 0)   // 积分限值
        I_Left += KiL*(err_L * Ts);             // 积分项
    dErrL = (err_L - err_L_last) / Ts;          // 误差微分
    D_Left = KdL * dErrL;                       // 微分项
    // 输出
    PID_OutputL = P_Left + I_Left + D_Left;
    if      (PID_OutputL<=-300)        PID_OutputL = -300;
    else if (PID_OutputL>=300)      PID_OutputL = 300;
    // 右轮的PID
    right_pwm = Right_Motor_PWM;
    err_R = R_Speed_Desire - Right_Motor_PWM;           // 误差
    P_Right = KpR * err_R;                      // 比例项
    if(PID_OutputR < 1000 && PID_OutputR > 0)   // 积分限值
        I_Right += KiR *(err_R * Ts);           // 积分项
    dErrR = (err_R - err_R_last) / Ts;          // 误差微分
    D_Right = KdR * dErrR;                      // 微分项
    // 输出
    PID_OutputR = P_Right + I_Right + D_Right;
    if      (PID_OutputR<=-300)        PID_OutputR = -300;
    else if (PID_OutputR>=300)      PID_OutputR = 300;
    // 更新误差
    err_L_last = err_L;
    err_R_last = err_R;
}

/* 中断 */
// Compute PID
void TIMER_computePID_INST_IRQHandler(void) {
    switch (DL_TimerA_getPendingInterrupt(TIMER_computePID_INST)) {
        case DL_TIMER_IIDX_ZERO:

            // computeWheelPID();
            computeAngPID();        // 角度环

            break;
        default:
            break;
    }
}

// serial print debug info
void TIMER_Debug_INST_IRQHandler(void) {
    switch (DL_TimerA_getPendingInterrupt(TIMER_Debug_INST)) {
        case DL_TIMER_IIDX_ZERO:
            /* Debug Info */
            printf("\r\n-------- PID Update! --------\r\n");
            printf("--- Question = %d\r\n", question);
            printf("--- carStatus: %d --- carDirection: %d --\r\n", carStatus, direction);
            printf("blackChannel=%d, ", isBlack);
            for (int i = 0; i < 7; i++) printf("%d",tubes[i]);
            printf("\r\n");
            printf("Encoder Number: %d, %d \r\n", M1_Count, M2_Count);
            printf("desire Angle = %.3f, current Angle = %.3f\r\n", desireAng, currentAng);
            // printf("pitch = %.3f, roll = %.3f, yaw = %.3f\r\n", roll, pitch, yaw);
            printf("error = %.3f\r\n",errAng );
            printf("P = %.3f, I = %.3f, D = %.3f\r\n", P_Ang , I_Ang , D_Ang);
            printf("left pwm = %d, right pwm = %d  ", Left_Motor_PWM, Right_Motor_PWM);
            printf("PID_Output =  %.3f\r\n", PID_Ang_output);

            // UART_transmitFloat(L_Speed, 4, 2);
            // UART_transmitString("\r\n");
            // UART_transmitFloat(R_Speed, 4, 2);
            // UART_transmitString("\r\n");
            // M2_Count = 0;
            // M1_Count = 0;

            break;
        default:
            break;
    }
}


// Question Select Button Interrupt
void GROUP1_IRQHandler(void) {
    /* Repeat with GPIOB Port */
    uint32_t isPressed = DL_GPIO_getEnabledInterruptStatus(GPIO_Questions_PORT, GPIO_Questions_PIN_Q1_PIN | GPIO_Questions_PIN_Q2_PIN 
        | GPIO_Questions_PIN_Q3_PIN | GPIO_Questions_PIN_Q4_PIN);
    
    if ((isPressed & GPIO_Questions_PIN_Q1_PIN) ==
        GPIO_Questions_PIN_Q1_PIN) {
            carStatus = followLine;
            question = Q1;
            direction = AtoB;
            DL_GPIO_clearInterruptStatus(GPIO_Questions_PORT, GPIO_Questions_PIN_Q1_PIN);
    }

    else if ((isPressed & GPIO_Questions_PIN_Q2_PIN) ==
        GPIO_Questions_PIN_Q2_PIN) {
            carStatus = followLine;
            question = Q2;
            direction = AtoB;
            DL_GPIO_clearInterruptStatus(GPIO_Questions_PORT, GPIO_Questions_PIN_Q2_PIN);
    }

    else if ((isPressed & GPIO_Questions_PIN_Q3_PIN) ==
        GPIO_Questions_PIN_Q3_PIN) {
            carStatus = followLine;
            question = Q3;
            direction = AtoC;
            DL_GPIO_clearInterruptStatus(GPIO_Questions_PORT, GPIO_Questions_PIN_Q3_PIN);
    }

    else if ((isPressed & GPIO_Questions_PIN_Q4_PIN) ==
        GPIO_Questions_PIN_Q4_PIN) {
            carStatus = followLine;
            question = Q4;
            direction = AtoC;
            Q4_loop = 0;
            DL_GPIO_clearInterruptStatus(GPIO_Questions_PORT, GPIO_Questions_PIN_Q4_PIN);
    }

    printf("\r\n\r\n DONE STATE CONFIRM!  \r\n\r\n");
}

// Encoder EXTI
void GROUP2_IRQHandler(void) {
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GPIO_Encoder_PIN_1A_PIN | GPIO_Encoder_PIN_1B_PIN);
    if ((gpioA & GPIO_Encoder_PIN_1A_PIN) == GPIO_Encoder_PIN_1A_PIN) {
        DL_GPIO_togglePins(GPIO_Flag_USER_LED_1_PORT, GPIO_Flag_USER_LED_1_PIN);

        M1_Count++;

        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_Encoder_PIN_1A_PIN);
    }

    if ((gpioA & GPIO_Encoder_PIN_1B_PIN) == GPIO_Encoder_PIN_1B_PIN) {
        DL_GPIO_togglePins(GPIO_Flag_USER_LED_1_PORT, GPIO_Flag_USER_LED_1_PIN);

        M2_Count++;

        DL_GPIO_clearInterruptStatus(GPIOA, GPIO_Encoder_PIN_1B_PIN);
    }

    
}


