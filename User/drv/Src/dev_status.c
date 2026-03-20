/**
 * @file dev_status.c
 * @brief L2 Device Layer: System Status Indicator Implementation
 */

#include "dev_status.h"
#include "bsp_gpio.h" // Access to L1 BSP logic

/* --- Implementation --- */

void DEV_Status_LED_ToggleLed(void)
{
    // 1. 获取当前硬件管脚电平状态 (L1 Read)
    bool current_state = BSP_GPIO_Read(BSP_GPIO_LED_STATUS);

    // 2. 将反转后的状态写入硬件管脚 (L1 Write)
    BSP_GPIO_Write(BSP_GPIO_LED_STATUS, !current_state);
}