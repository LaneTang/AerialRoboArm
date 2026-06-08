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

    // 2. Piecewise linear mapping for PTK 7462W (mid = 1520us):
    //    0°→500us, 90°→1520us, 180°→2500us
    //    Lower half: Pulse = 500 + (angle * 1020 / 90) = 500 + (angle * 34 / 3)
    //    Upper half: Pulse = 1520 + ((angle-90) * 980 / 90) = 1520 + ((angle-90) * 98 / 9)
    uint16_t pulse_us;
    if (target_angle <= 90U) {
        pulse_us = 500U + ((uint16_t)target_angle * 34U / 3U);
    } else {
        pulse_us = 1520U + ((uint16_t)(target_angle - 90U) * 98U / 9U);
    }

    // 3. 调用 L1 BSP 接口直接更新寄存器
    BSP_PWM_SetServoPulse(ctx->servo_id, pulse_us);

    // 4. 更新状态缓存
    ctx->current_angle = target_angle;

    return status;
}