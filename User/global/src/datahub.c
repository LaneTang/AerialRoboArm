/**
 * @file mod_datahub.c
 * @brief Global Lock-Free Data Exchange Center Implementation (L4)
 * @note  Strictly avoids FreeRTOS Mutexes to guarantee 1000Hz FOC timing.
 * Utilizes Cortex-M3 taskENTER_CRITICAL() for atomic memcpy.
 */

#include "datahub.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h> // Required for fast memcpy
#include <stddef.h> // Required for NULL

/* =========================================================
 * 1. Static Memory Allocation (The "Blackboard")
 * ========================================================= */

/* * These static instances reside in the BSS/Data segment.
 * They are the single source of truth for inter-thread data.
 */
static DataHub_Cmd_t   s_cmd_hub;
static DataHub_State_t s_state_hub;

/* =========================================================
 * 2. Interface Implementation
 * ========================================================= */

/**
 * @brief Initialize DataHub Memory
 */
void DataHub_Init(void)
{
    // Zero out the entire memory block first
    memset(&s_cmd_hub, 0, sizeof(DataHub_Cmd_t));
    memset(&s_state_hub, 0, sizeof(DataHub_State_t));

    // Force safe default states (Critical for hardware safety on boot)
    s_cmd_hub.sys_mode = ARA_MODE_INIT;
    s_cmd_hub.emergency_stop = true;      // Default to E-STOP until explicitly cleared
    s_cmd_hub.target_foc_angle = 0;

    // Initialize state to zero/safe values
    s_state_hub.current_foc_angle = 0;
    s_state_hub.current_velocity = 0;
    // Assuming 0 equates to ARA_OK or equivalent success state in AraStatus_t
    s_state_hub.foc_status = 0;
}

/**
 * @brief Write Downlink Command to DataHub
 */
void DataHub_WriteCmd(const DataHub_Cmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return; // Guard against null pointer dereference
    }

    // Mask all interrupts (up to configMAX_SYSCALL_INTERRUPT_PRIORITY)
    taskENTER_CRITICAL();
    {
        // Cortex-M3 optimized memcpy.
        // Size is approx 8 bytes, takes < 10 CPU cycles (<< 1us).
        memcpy(&s_cmd_hub, p_cmd, sizeof(DataHub_Cmd_t));
    }
    // Unmask interrupts
    taskEXIT_CRITICAL();
}

/**
 * @brief Read Downlink Command from DataHub
 */
void DataHub_ReadCmd(DataHub_Cmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    {
        memcpy(p_cmd, &s_cmd_hub, sizeof(DataHub_Cmd_t));
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief Write Uplink State to DataHub
 */
void DataHub_WriteState(const DataHub_State_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    {
        memcpy(&s_state_hub, p_state, sizeof(DataHub_State_t));
    }
    taskEXIT_CRITICAL();
}

/**
 * @brief Read Uplink State from DataHub
 */
void DataHub_ReadState(DataHub_State_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    {
        memcpy(p_state, &s_state_hub, sizeof(DataHub_State_t));
    }
    taskEXIT_CRITICAL();
}