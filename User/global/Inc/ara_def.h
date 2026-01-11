/**
 * @file ara_def.h
 * @brief Global definitions for ARA Robot Arm Project
 * @note  Universal Status Codes & Constants for L1-L4 layers.
 */

#ifndef ARA_DEF_H
#define ARA_DEF_H

#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>

/* --- 1. Universal Status Codes (全局状态码) --- */
/* 统一用于 BSP底层、Driver驱动层、Algo算法层、Task业务层 */
typedef enum {
    ARA_OK              = 0,

    /* Generic Errors */
    ARA_ERROR           = -1,   // 未分类的一般错误
    ARA_BUSY            = -2,   // 资源被占用 (Mutex/DMA)
    ARA_TIMEOUT         = -3,   // 等待超时
    ARA_ERR_PARAM       = -4,   // 输入参数非法

    /* Hardware/BSP Level */
    ARA_ERR_DMA         = -10,  // DMA 传输失败
    ARA_ERR_IO          = -11,  // 物理层读写失败 (HAL Error)

    /* Driver/Sensor Level */
    ARA_ERR_NACK        = -20,  // I2C 设备无应答
    ARA_ERR_CRC         = -21,  // 数据校验失败
    ARA_ERR_DISCONNECTED= -22,  // 传感器断连

    /* Algorithm Level */
    ARA_ERR_MATH        = -30   // 计算溢出/除零 (Fixed-Point Overflow)

} AraStatus_t;

/* --- 2. Math Constants (Fixed Point Q15) --- */
/* 1.0f represented in Q15 */
#define Q15_ONE         (32768)
/* PI represented in Q15 */
#define Q15_PI          (102944)

/* --- 3. Common Callback Type --- */
typedef void (*AraCallback_t)(void);

#endif // ARA_DEF_H