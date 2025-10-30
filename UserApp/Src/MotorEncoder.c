//
// Created by User on 2025/10/17.
//

#include "MotorEncoder.h"
#include <string.h>

/* --------------------------------------------------------------
 *  内部静态变量
 * -------------------------------------------------------------- */
static TIM_HandleTypeDef *henc = NULL;   // TIM4（编码器）
static TIM_HandleTypeDef *htim_ini = NULL;   // TIM2（100ms 定时）

static uint16_t cnt_last = 0;
static float    lines_per_rev = (265.2f * 4);   // 默认四倍频，可被 Init 覆盖

static MotorEncoder_t enc = {0};

/* --------------------------------------------------------------
 *  初始化
 * -------------------------------------------------------------- */
void MotorEncoder_Init(TIM_HandleTypeDef *htim_encoder,
                       TIM_HandleTypeDef *htim_timer,
                       float lines_per_rev_user)
{
    henc = htim_encoder;
    htim_ini = htim_timer;

    if (lines_per_rev_user > 0.0f)
        lines_per_rev = lines_per_rev_user;

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
int16_t MotorEncoder_GetDir(void)   { return enc.direction; }
float   MotorEncoder_GetSpeed(void) { return enc.speed_rpm; }
uint32_t MotorEncoder_GetPos(void)   { return enc.position; }

/* --------------------------------------------------------------
 *  TIM2 Update 中断回调（CubeMX 自动调用）
 *  CW  - 增加计数
 *  CCW - 减少计数
 * -------------------------------------------------------------- */
void MotorEncoder_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim != htim_ini) return;   // 防止误入

    uint16_t cnt_now = __HAL_TIM_GET_COUNTER(henc);

    /* ---------- 1) 获取位置差值 ---------- */
    int32_t delta_pos = (cnt_now - cnt_last);

    /* ---------- 2) 更新累计位置 ---------- */
    enc.position += delta_pos;

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




