/**
 * @file task_rc.c
 * @brief RC Data Collection and Semantic Dispatch Task (L4) Implementation
 * @note  FreeRTOS Task. Integrates DRV_ELRS (L2) and MOD_RC_SEMANTIC (L3).
 */

#include "task_rc.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* =========================================================
 * Internal Context Instance
 * ========================================================= */
/* Static allocation to avoid dynamic memory (malloc) after initialization */
static TaskRc_Context_t rc_ctx;

/* =========================================================
 * Internal Helper Functions
 * ========================================================= */

/**
 * @brief Helper to generate a safe default channel array for initialization
 */
static void generate_default_channels(uint16_t *channels)
{
    for (int i = 0; i < DRV_ELRS_MAX_CHANNELS; i++) {
        channels[i] = MOD_RC_VAL_MID; // 992 (Neutral position)
    }
    /* E-Stop Channel (SD): Default to UP (Low value) -> ACTIVE for safety */
    channels[MOD_RC_IDX_SD] = MOD_RC_SW_LOW - 100;
}

/**
 * @brief Helper to apply Failsafe values when link is lost
 */
static void apply_failsafe_intent(RcControlData_t *intent)
{
    intent->is_link_up      = false;
    intent->req_mode        = ARA_MODE_IDLE;      // Force system out of Auto/Manual
    intent->estop_state     = ESTOP_ACTIVE;       // CRITICAL: Lock all actuators
    intent->arm_cmd         = ARM_CMD_HOLD;       // Stop arm movement
    intent->gripper_cmd     = GRIPPER_CMD_STOP;   // Stop gripper
    intent->sys_reset_pulse = false;
    intent->aux_knob_val    = 0;
}


/* =========================================================
 * API Implementation
 * ========================================================= */

void TaskRc_Init(void)
{
    /* Clear context memory */
    memset(&rc_ctx, 0, sizeof(TaskRc_Context_t));

    /* 1. Initialize L2 Driver (Bind to ELRS UART) */
    AraStatus_t l2_status = DrvElrs_Init(&rc_ctx.elrs_driver, BSP_UART_ELRS);

    /* 2. Initialize L3 Semantic Logic */
    uint16_t default_chs[DRV_ELRS_MAX_CHANNELS];
    generate_default_channels(default_chs);
    AraStatus_t l3_status = ModRcSemantic_Init(&rc_ctx.semantic_logic, default_chs);

    /* Mark as initialized only if both components are OK */
    if (l2_status == ARA_OK && l3_status == ARA_OK) {
        rc_ctx.is_initialized = true;
    }
}

void TaskRc_Entry(void *argument)
{
    /* Prevent running if not properly initialized */
    if (!rc_ctx.is_initialized) {
        vTaskDelete(NULL);
    }

    /* FreeRTOS Absolute Delay Setup */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TASK_RC_PERIOD_MS);

    /* Task Main Loop */
    for (;;)
    {
        /* 1. Wait for the next cycle (e.g., 20ms = 50Hz) */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        /* 2. Get current system time in ms */
        /* Assuming configTICK_RATE_HZ == 1000, TickCount is exactly ms */
        uint32_t current_tick_ms = (uint32_t)xTaskGetTickCount();

        /* 3. Update L2 Driver (Reads UART RingBuffer, Checks CRC & Timeout) */
        DrvElrs_Update(&rc_ctx.elrs_driver, current_tick_ms);

        /* 4. Check Link Status & Process Semantics */
        bool link_is_up = DrvElrs_IsLinkUp(&rc_ctx.elrs_driver);

        /* Start with a clean slate for the new intent */
        memset(&rc_ctx.last_intent, 0, sizeof(RcControlData_t));

        if (link_is_up) {
            /* Link is healthy: Translate raw channels to Semantic Intents via L3 */
            ModRcSemantic_Process(&rc_ctx.semantic_logic,
                                  rc_ctx.elrs_driver.channels,
                                  current_tick_ms,
                                  &rc_ctx.last_intent);

            rc_ctx.last_intent.is_link_up = true;
        } else {
            /* Link is lost: Enforce hardware failsafe semantics */
            apply_failsafe_intent(&rc_ctx.last_intent);
        }

        /* 5. Publish Intent to Global DataHub */
        /* L5 Scheduler and TaskMotion will read this in their own cycles */
        DataHub_WriteRcData(&rc_ctx.last_intent);
    }
}