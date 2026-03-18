/**
 * @file mod_actuator.c
 * @brief L3 Functional Module: End-Effector Implementation
 * @author ARA Project Coder
 */

#include "mod_actuator.h"
#include <stddef.h> // For NULL
#include <stdint.h>

int8_t ModActuator_Init(ModActuator_Context_t *ctx)
{
    if (ctx == NULL) {
        return MOD_ACTUATOR_ERR_NULL;
    }

    int8_t err;

    // 1. 初始化夹爪舵机 (默认绑定 SERVO_1, 物理防堵转限位设为 0~180 全量程)
    err = DrvServo_Init(&ctx->servo_gripper, BSP_SERVO_1, 0, 180);
    if (err != DRV_SERVO_OK) return err;

    // 2. 初始化姿态舵机 (默认绑定 SERVO_2, 物理防堵转限位设为 0~180 全量程)
    err = DrvServo_Init(&ctx->servo_roll, BSP_SERVO_2, 0, 180);
    if (err != DRV_SERVO_OK) return err;

    // 3. 加载夹爪标定数据
    // (需根据机械结构实际安装的齿轮位宽微调，此处预设安全默认值)
    ctx->gripper_open_angle = 10;   // 张开时舵机对应的物理角度
    ctx->gripper_close_angle = 160; // 闭合时舵机对应的物理角度

    return MOD_ACTUATOR_OK;
}

int8_t ModActuator_SetGripper(ModActuator_Context_t *ctx, uint8_t percent)
{
    if (ctx == NULL) {
        return MOD_ACTUATOR_ERR_NULL;
    }

    // 容错处理：限制百分比在 0-100 之间
    if (percent > 100) {
        percent = 100;
    }

    // 1. 线性插值计算物理角度 (防溢出设计，使用带符号 32 位整型做中间过渡)
    int32_t range = (int32_t)ctx->gripper_close_angle - (int32_t)ctx->gripper_open_angle;
    int32_t offset = (range * (int32_t)percent) / 100;
    uint8_t target_angle = (uint8_t)((int32_t)ctx->gripper_open_angle + offset);

    // 2. 下发给 L2 层驱动
    int8_t ret = DrvServo_SetAngle(&ctx->servo_gripper, target_angle);

    // 3. 错误码映射转换
    if (ret == DRV_SERVO_LIMIT_REACHED) {
        return MOD_ACTUATOR_WARN_JAM;
    } else if (ret != DRV_SERVO_OK) {
        return MOD_ACTUATOR_ERR_NULL;
    }

    return MOD_ACTUATOR_OK;
}

int8_t ModActuator_SetRoll(ModActuator_Context_t *ctx, uint8_t degree)
{
    if (ctx == NULL) {
        return MOD_ACTUATOR_ERR_NULL;
    }

    // 直接透传给 L2 姿态舵机，L2 层会进行防堵转的 min/max 检查
    int8_t ret = DrvServo_SetAngle(&ctx->servo_roll, degree);

    if (ret == DRV_SERVO_LIMIT_REACHED) {
        return MOD_ACTUATOR_WARN_JAM;
    } else if (ret != DRV_SERVO_OK) {
        return MOD_ACTUATOR_ERR_NULL;
    }

    return MOD_ACTUATOR_OK;
}