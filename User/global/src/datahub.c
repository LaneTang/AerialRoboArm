/**
 * @file datahub.c
 * @brief Global Lock-Free Data Exchange Center Implementation (L4)
 * @note  Strictly NO FreeRTOS Mutexes. Uses CPU-level interrupt masking
 * (Critical Sections) to guarantee atomicity and prevent data tearing.
 * Execution time is guaranteed to be under 1 microsecond (72MHz).
 */

#include "datahub.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stddef.h>

/* =========================================================
 * 1. Private Static Memory Allocations (.bss)
 * ========================================================= */

/**
 * @brief The actual physical memory blocks for cross-thread exchange.
 * @note Statically allocated. Total RAM footprint: ~12 bytes.
 */
static DataHub_Cmd_t   s_cmd_hub;
static DataHub_State_t s_state_hub;

/* =========================================================
 * 2. API Implementations
 * ========================================================= */

/**
 * @brief Initialize the DataHub with safe default states.
 * @note  Must be called before FreeRTOS scheduler starts.
 */
void DataHub_Init(void)
{
    /* Protect initialization in case it's called post-scheduler start */
    taskENTER_CRITICAL();

    /* --- Command Hub Safe Defaults --- */
    s_cmd_hub.sys_mode          = ARA_MODE_INIT;
    s_cmd_hub.emergency_stop    = true;             // CRITICAL: Safe fallback default
    s_cmd_hub.target_foc_angle  = 0;

    /* --- State Hub Safe Defaults --- */
    s_state_hub.current_foc_angle = 0;
    s_state_hub.current_velocity  = 0;
    s_state_hub.foc_status        = ARA_OK;         // Assume healthy until reported otherwise

    taskEXIT_CRITICAL();
}

/**
 * @brief Write a new command set to the DataHub (Producer: 50Hz Logic Thread).
 * @param p_cmd Pointer to the populated command structure.
 */
void DataHub_WriteCmd(const DataHub_Cmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    /* Cortex-M3 instruction: CPSID I (Disable global interrupts) */
    taskENTER_CRITICAL();

    /* Direct struct assignment is translated by GCC into a highly optimized
     * LDM/STM (Load/Store Multiple) block copy or consecutive LDR/STR instructions.
     * Takes ~4 clock cycles for this small struct. */
    s_cmd_hub = *p_cmd;

    /* Cortex-M3 instruction: CPSIE I (Enable global interrupts) */
    taskEXIT_CRITICAL();
}

/**
 * @brief Read the latest command set from the DataHub (Consumer: 1000Hz Motion Thread).
 * @param p_cmd Pointer to the destination command structure.
 */
void DataHub_ReadCmd(DataHub_Cmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *p_cmd = s_cmd_hub;
    taskEXIT_CRITICAL();
}

/**
 * @brief Write current hardware state to the DataHub (Producer: 1000Hz Motion Thread).
 * @param p_state Pointer to the populated state structure.
 */
void DataHub_WriteState(const DataHub_State_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_state_hub = *p_state;
    taskEXIT_CRITICAL();
}

/**
 * @brief Read the latest hardware state from the DataHub (Consumer: 50Hz Logic Thread).
 * @param p_state Pointer to the destination state structure.
 */
void DataHub_ReadState(DataHub_State_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *p_state = s_state_hub;
    taskEXIT_CRITICAL();
}