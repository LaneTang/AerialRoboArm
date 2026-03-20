/**
 * @file task_rc.c
 * @brief RC Data Collection & Semantic Logic (L4) Implementation
 * @note  Strictly C99. Zero RTOS dependencies.
 * Extracts high-level intent from raw L2 ELRS data via L3 Semantic module.
 * @author ARA Project Coder
 */

#include "task_rc.h"
#include "bsp_uart.h"  /* For BSP_UART_ELRS */
#include <string.h>    /* For memset */

/* =========================================================
 * 1. Internal Context Allocation
 * ========================================================= */

/* Singleton context for the RC task (Definition comes from task_rc.h) */
static TaskRc_Context_t s_rc_ctx;

/* =========================================================
 * 2. Private Helper Functions
 * ========================================================= */

/**
 * @brief Helper to generate a safe default channel array for initialization
 * @note  Sets all sticks to center (992) and forces E-Stop channel to Active (Low).
 */
static void generate_default_channels(uint16_t *channels)
{
    for (int i = 0; i < DRV_ELRS_MAX_CHANNELS; i++) {
        channels[i] = MOD_RC_VAL_MID; /* 992 */
    }
    /* E-Stop Channel (CH10 / Index 9): Default to UP (Low value) -> ACTIVE for safety */
    channels[9] = MOD_RC_SW_LOW - 100;
}

/**
 * @brief Helper to apply Failsafe values when link is lost or uninitialized
 */
static void apply_failsafe_intent(RcControlData_t *intent)
{
    memset(intent, 0, sizeof(RcControlData_t));

    intent->is_link_up      = false;              // REPLACED old is_failsafe logic
    intent->req_mode        = ARA_MODE_IDLE;      // Force system out of Auto/Manual
    intent->estop_state     = ESTOP_ACTIVE;       // CRITICAL: Lock all actuators
    intent->arm_cmd         = ARM_CMD_HOLD;       // Stop arm movement
    intent->gripper_cmd     = GRIPPER_CMD_STOP;   // Stop gripper
    intent->sys_reset_pulse = false;
    intent->aux_knob_val    = 0;
}

/* =========================================================
 * 3. API Implementation
 * ========================================================= */

void TaskRc_Init(void)
{
    /* Clear context memory */
    memset(&s_rc_ctx, 0, sizeof(TaskRc_Context_t));

    /* 1. Initialize L2 Driver (Bind to ELRS UART) */
    AraStatus_t l2_status = DrvElrs_Init(&s_rc_ctx.elrs_driver, BSP_UART_ELRS);

    /* 2. Initialize L3 Semantic Logic */
    uint16_t default_chs[DRV_ELRS_MAX_CHANNELS];
    generate_default_channels(default_chs);
    AraStatus_t l3_status = ModRcSemantic_Init(&s_rc_ctx.semantic_logic, default_chs);

    /* Mark as initialized only if both components are OK */
    if (l2_status == ARA_OK && l3_status == ARA_OK) {
        s_rc_ctx.is_initialized = true;
    }
}

void TaskRc_Update(uint32_t current_tick_ms, RcControlData_t *p_out_intent)
{
    /* Parameter Guard */
    if (p_out_intent == NULL) {
        return;
    }

    /* Hardware Init Guard */
    if (!s_rc_ctx.is_initialized) {
        apply_failsafe_intent(p_out_intent);
        return;
    }

    /* 1. Update L2 Driver (Reads UART RingBuffer, Checks CRC & Timeout) */
    DrvElrs_Update(&s_rc_ctx.elrs_driver, current_tick_ms);

    /* 2. Check Link Status */
    if (DrvElrs_IsLinkUp(&s_rc_ctx.elrs_driver)) {

        /* Clear the output struct before population */
        memset(p_out_intent, 0, sizeof(RcControlData_t));

        /* Link is healthy: Translate raw channels to Semantic Intents via L3 */
        ModRcSemantic_Process(&s_rc_ctx.semantic_logic,
                              s_rc_ctx.elrs_driver.channels,
                              current_tick_ms,
                              p_out_intent);

        /* Explicitly assert link up state */
        p_out_intent->is_link_up = true;

    } else {
        /* Link is lost: Enforce hardware failsafe semantics */
        apply_failsafe_intent(p_out_intent);
    }
}