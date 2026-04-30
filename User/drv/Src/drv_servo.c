/**
 * @file drv_servo.c
 * @brief L2 Hardware Driver: Generic PWM Digital Servo Implementation
 * @author ARA Project Coder
 */

#include "drv_servo.h"
#include <stddef.h> // For NULL

int8_t DrvServo_Init(DrvServo_Context_t *ctx, BspServo_Dev_t id, uint8_t min_limit, uint8_t max_limit)
{
    if (ctx == NULL) {
        return DRV_SERVO_ERR_PARAM;
    }

    // 参数校验：如果最小值大于最大值，则为非法参数
    if (min_limit > max_limit) {
        return DRV_SERVO_ERR_PARAM;
    }

    ctx->servo_id = id;
    ctx->min_angle = min_limit;
    ctx->max_angle = max_limit;

    // 初始化时将当前角度缓存设为安全中点，但不立刻输出PWM（防止上电乱动）
    ctx->current_angle = (min_limit + max_limit) / 2;

    return DRV_SERVO_OK;
}

int8_t DrvServo_SetAngle(DrvServo_Context_t *ctx, uint8_t target_angle)
{
    int8_t status = DRV_SERVO_OK;

    if (ctx == NULL) {
        return DRV_SERVO_ERR_PARAM;
    }

    // 1. 物理限位保护 (防堵转)
    if (target_angle < ctx->min_angle) {
        target_angle = ctx->min_angle;
        status = DRV_SERVO_LIMIT_REACHED;
    } else if (target_angle > ctx->max_angle) {
        target_angle = ctx->max_angle;
        status = DRV_SERVO_LIMIT_REACHED;
    }

    // 2. 纯整型算力优化：映射公式 Pulse = 500 + (Angle * 2000 / 180)
    // 化简为：Pulse = 500 + ((Angle * 100) / 9)
    // 精度分析：Target_angle最大180。180*100 = 18000。
    // 18000 远小于 uint16_t 的最大值 65535，绝对不会溢出。
    uint16_t pulse_us = 500 + (((uint16_t)target_angle * 100) / 9);

    // 3. 调用 L1 BSP 接口直接更新寄存器
    BSP_PWM_SetServoPulse(ctx->servo_id, pulse_us);

    // 4. 更新状态缓存
    ctx->current_angle = target_angle;

    return status;
}