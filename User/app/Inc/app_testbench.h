/**
 * @file app_testbench.h
 * @brief Dual-thread on-board demo testbench container for ARA project.
 * @note  This module preserves the production-style physical architecture:
 *        - 1ms high-frequency control thread
 *        - 20ms low-frequency logic / test orchestration thread
 *        It is intended for demo bring-up and staged validation.
 */

#ifndef APP_TESTBENCH_H
#define APP_TESTBENCH_H

#include "ara_def.h"

/**
 * @brief High-frequency physical thread period in milliseconds.
 */
#define TESTBENCH_HIGH_FREQ_PERIOD_MS     (1U)

/**
 * @brief High-frequency thread stack size in FreeRTOS words.
 */
#define TESTBENCH_HIGH_FREQ_STACK         (256U)

/**
 * @brief Low-frequency physical thread period in milliseconds.
 */
#define TESTBENCH_LOW_FREQ_PERIOD_MS      (20U)

/**
 * @brief Low-frequency thread stack size in FreeRTOS words.
 */
#define TESTBENCH_LOW_FREQ_STACK          (384U)

/**
 * @brief  Initialize and mount the dual-thread demo testbench.
 * @note   Call this instead of App_Threads_Init() when entering staged testing.
 */
void App_Testbench_Init(void);

#endif /* APP_TESTBENCH_H */