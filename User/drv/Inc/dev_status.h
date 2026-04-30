/**
 * @file dev_status.h
 * @brief L2 Device Layer: System Status Indicator
 * @note  Provides semantic control over system status representations (e.g., LEDs).
 */

#ifndef DEV_STATUS_H
#define DEV_STATUS_H

#include "ara_def.h"

/* --- API --- */

/**
 * @brief Toggle the system status LED.
 * @note  Reads current hardware state via L1 BSP and applies inverted state.
 */
void DEV_Status_LED_ToggleLed(void);

#endif // DEV_STATUS_H