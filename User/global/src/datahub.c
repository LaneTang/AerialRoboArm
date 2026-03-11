/**
 * @file datahub.c
 * @brief Global Data Exchange Center (L4) Implementation
 * @note  Implements thread-safe access to system state and cross-task data.
 */

#include "datahub.h"
#include <string.h>

/* [FreeRTOS Includes] */
#include "FreeRTOS.h"
#include "semphr.h"
#include "task.h"

/* =========================================================
 * Internal Storage (The "Blackboard")
 * ========================================================= */

/* 1. Global System Mode */
static AraSysMode_t    global_sys_mode;

/* 2. RC Semantic Data Cache */
static RcControlData_t global_rc_data;

/* =========================================================
 * IPC Resources (Inter-Process Communication)
 * ========================================================= */

/* Mutex to protect the RC Control Data struct */
static SemaphoreHandle_t rc_data_mutex = NULL;

/* * Note on `global_sys_mode`:
 * Since AraSysMode_t is a standard enum (32-bit integer on ARM Cortex-M3),
 * its assignment is natively atomic. However, to be perfectly strict and
 * prevent any compiler reordering issues, we will use FreeRTOS Critical
 * Sections (taskENTER_CRITICAL) which are ultra-fast for simple variable access.
 */


/* =========================================================
 * API Implementation
 * ========================================================= */

void DataHub_Init(void)
{
    /* 1. Initialize System Mode to Safe Boot State */
    global_sys_mode = ARA_MODE_INIT;

    /* 2. Initialize RC Data to Safe Defaults (Failsafe) */
    memset(&global_rc_data, 0, sizeof(RcControlData_t));
    global_rc_data.is_link_up  = false;
    global_rc_data.estop_state = ESTOP_ACTIVE;  // System locked by default
    global_rc_data.arm_cmd     = ARM_CMD_HOLD;
    global_rc_data.gripper_cmd = GRIPPER_CMD_STOP;

    /* 3. Create Mutex for complex structs */
    rc_data_mutex = xSemaphoreCreateMutex();

    /* Ensure mutex creation was successful */
    configASSERT(rc_data_mutex != NULL);
}

AraSysMode_t DataHub_GetSysMode(void)
{
    AraSysMode_t mode;

    /* Fast critical section for single variable read */
    taskENTER_CRITICAL();
    mode = global_sys_mode;
    taskEXIT_CRITICAL();

    return mode;
}

AraStatus_t DataHub_SetSysMode(AraSysMode_t mode)
{
    /* Fast critical section for single variable write */
    taskENTER_CRITICAL();
    global_sys_mode = mode;
    taskEXIT_CRITICAL();

    return ARA_OK;
}

AraStatus_t DataHub_WriteRcData(const RcControlData_t *p_rc_data)
{
    if (p_rc_data == NULL || rc_data_mutex == NULL) {
        return ARA_ERR_PARAM;
    }

    /* Wait up to 10ms to acquire the Mutex */
    if (xSemaphoreTake(rc_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* Safe Memory Copy */
        memcpy(&global_rc_data, p_rc_data, sizeof(RcControlData_t));

        /* Release the Mutex */
        xSemaphoreGive(rc_data_mutex);
        return ARA_OK;
    }

    /* Mutex timeout (another task held it too long) */
    return ARA_BUSY;
}

AraStatus_t DataHub_ReadRcData(RcControlData_t *p_rc_data)
{
    if (p_rc_data == NULL || rc_data_mutex == NULL) {
        return ARA_ERR_PARAM;
    }

    /* Wait up to 10ms to acquire the Mutex */
    if (xSemaphoreTake(rc_data_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        /* Safe Memory Copy */
        memcpy(p_rc_data, &global_rc_data, sizeof(RcControlData_t));

        /* Release the Mutex */
        xSemaphoreGive(rc_data_mutex);
        return ARA_OK;
    }

    /* Mutex timeout */
    return ARA_BUSY;
}