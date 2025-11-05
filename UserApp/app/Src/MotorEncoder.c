//
// Created by User on 2025/10/17.
//

#include "MotorEncoder.h"

/* --------------------------------------------------------------
 *  内部静态变量
 * -------------------------------------------------------------- */
static TIM_HandleTypeDef *henc = NULL;   // TIM4（编码器）
static TIM_HandleTypeDef *htim_ini = NULL;   // TIM2（100ms 定时）

static uint16_t volatile cnt_last = 0;   // 测速用
static float    lines_per_rev = ((float)PULSE_PER_REVOLUTION * 4);   // 默认四倍频，可被 Init 覆盖

static MotorEncoder_t enc = {0};

/* --------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------- */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_timer)
{
    henc = htim_encoder;
    htim_ini = htim_timer;
//
//    if (lines_per_rev_user > 0.0f)
//        lines_per_rev = lines_per_rev_user;

    /* 启动编码器接口 */
    HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL);

    /* 启动 100ms 定时中断 */
    HAL_TIM_Base_Start_IT(htim_ini);

    /* 读取初始计数值 */
    cnt_last = __HAL_TIM_GET_COUNTER(henc);
    enc.position = 0;
    enc.direction = 0;
    enc.speed_rpm = 0.0f;
}

/* --------------------------------------------------------------
 *  对外 API
 * -------------------------------------------------------------- */
int MotorEncoder_GetDir(void)   { return enc.direction; }
float   MotorEncoder_GetSpeed(void) { return enc.speed_rpm; }
float   MotorEncoder_GetAngle(void) { return enc.angle_deg; }
uint32_t MotorEncoder_GetPos(void)   { return enc.position; }

/* --------------------------------------------------------------
 *  编码器测速 中断回调函数
 *  TIM2
 *  CW  - 增加计数
 *  CCW - 减少计数
 * -------------------------------------------------------------- */
void MotorEncoder_Sensor_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != htim_ini) return;   // 防止误入

    uint16_t cnt_now = __HAL_TIM_GET_COUNTER(henc);

    /* ---------- 1) 获取位置差值 并 更新累计位置 ---------- */
    int32_t delta_pos = (int16_t)cnt_now - (int16_t)cnt_last;

    enc.position += delta_pos;

    /* ---------- 2) 转换角度 并 更新累计角度 ---------- */
    float delta_angle = (float)delta_pos * 360.0f / lines_per_rev;

    enc.angle_deg += delta_angle;

    /* ---------- 3) 方向 ---------- */
    if (delta_pos > 0)      enc.direction =  1;
    else if (delta_pos < 0) enc.direction = -1;
    else                    enc.direction =  0;

    /* ---------- 4) 转速 (rpm) ---------- */
    /* Δcnt 为 100ms 内脉冲数，×10 → 每秒脉冲数 */
    float pulses_per_sec = (float)delta_pos * 10.0f;
    enc.speed_rpm = pulses_per_sec / lines_per_rev;   // 减速后 rpm

    /* ---------- 5) 保存本次计数 ---------- */
    cnt_last = cnt_now;
}

/* --------------------------------------------------------------
 *  内部静态变量（调零用）
 * -------------------------------------------------------------- */
static uint32_t     homing_start_tick = 0;
static float        homing_target_rpm = 0.0f;
static uint32_t     timeout = 10000;
static HomingState_t homing_state = HOMING_IDLE;

HomingState_t MotorEncoder_Homing_Start(float homing_speed_rpm, uint32_t timeout_ms)
{
    if (homing_state != HOMING_IDLE && homing_state != HOMING_SUCCESS) {
        return homing_state;  // 正在调零中或未重置
    }

    // 启动调零状态
    homing_state = HOMING_RUNNING;
    homing_start_tick = HAL_GetTick();

    /* 转速映射 */
//    homing_target_rpm = -homing_speed_rpm;  // 反转（负值）

    /* 启动反转 */
    Motor_SetDirection(-1);
    // 但是现在没有速度对pwm的映射，怎么办呢，直接乱设置一个速度好了
    Motor_SetDuty(HOMING_LOW_PWM_DUTY); // 使用固定的低速 PWM

    return HOMING_RUNNING;

}

HomingState_t MotorEncoder_CheckHomingState(uint32_t timeout_ms)
{
    if (homing_state != HOMING_RUNNING) {
        return homing_state;
    }

    /* 超时保护 */
    if (HAL_GetTick() - homing_start_tick > timeout_ms) {
        Motor_Stop();
        homing_state = HOMING_TIMEOUT;
        return HOMING_TIMEOUT;
    }

    return homing_state;
}

void MotorEncoder_HOMED_EXTI_Callback(void)
{
    // 只有在 HOMING_RUNNING 状态下触发才有效
    if (homing_state != HOMING_RUNNING) {
        return;
    }

    /* 1. 立即停止电机 */
    Motor_Stop(); // 假设 Motor_Stop() 清零所有 PWM 占空比

    /* 2. 重置编码器计数 */
    __HAL_TIM_SET_COUNTER(henc, 0); // 定时器计数值清零
    cnt_last = 0;                  // 测速用的上次计数值清零

    /* 3. 重置结构体累计值 */
    enc.position = 0;
    enc.angle_deg = 0.0f;
    enc.direction = 0;
    enc.speed_rpm = 0.0f;

    /* 4. 更新状态 */
    printf("调零结束\r\n");
    homing_state = HOMING_SUCCESS;

    /* 外部中断停用 */
    HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
}
