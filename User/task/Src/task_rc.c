/**
 * @file task_rc.c
 * @brief RC Data Collection & Semantic Logic Runnable (L4) Implementation
 * @note  Strictly C99. Zero RTOS dependencies.
 *        This runnable orchestrates:
 *        - L2 ELRS protocol driver update
 *        - L3 semantic / debug RC mapping
 *        - Link-loss failsafe output
 */

#include "task_rc.h"
#include "bsp_uart.h"
#include <string.h>

/* =========================================================
 * 1. Internal Context Allocation
 * ========================================================= */

/**
 * @brief Singleton context for RC task.
 */
static TaskRc_Context_t s_rc_ctx;

/* =========================================================
 * 2. Private Helper Functions
 * ========================================================= */

/**
 * @brief  Generate a safe default raw-channel snapshot for initialization.
 * @param  p_channels Output channel array.
 * @note   Sticks default to MID. E-stop channel defaults to ACTIVE.
 */
static void TaskRc_GenerateDefaultChannels(uint16_t *p_channels)
{
    uint8_t i;

    if (p_channels == NULL) {
        return;
    }

    for (i = 0U; i < DRV_ELRS_MAX_CHANNELS; i++) {
        p_channels[i] = MOD_RC_VAL_MID;
    }

    /* CH10 / SD: default to ACTIVE for safety. */
    p_channels[MOD_RC_IDX_SD] = (uint16_t)(MOD_RC_SW_LOW - 100U);
}

/**
 * @brief  Apply safe formal failsafe intent.
 * @param  p_out_intent Output semantic intent.
 */
static void TaskRc_ApplyFailsafeIntent(RcControlData_t *p_out_intent)
{
    if (p_out_intent == NULL) {
        return;
    }

    memset(p_out_intent, 0, sizeof(RcControlData_t));

    p_out_intent->is_link_up      = false;
    p_out_intent->req_mode        = ARA_MODE_IDLE;
    p_out_intent->estop_state     = ESTOP_ACTIVE;
    p_out_intent->arm_cmd         = ARM_CMD_HOLD;
    p_out_intent->gripper_cmd     = GRIPPER_CMD_STOP;
    p_out_intent->sys_reset_pulse = false;
    p_out_intent->aux_knob_val    = 0U;
}

/**
 * @brief  Apply safe debug failsafe output.
 * @param  p_out_debug Output debug RC data.
 */
static void TaskRc_ApplyFailsafeDebug(RcDebugAnalogData_t *p_out_debug)
{
    if (p_out_debug == NULL) {
        return;
    }

    memset(p_out_debug, 0, sizeof(RcDebugAnalogData_t));

    p_out_debug->is_link_up       = false;
    p_out_debug->req_mode         = (uint8_t)ARA_MODE_IDLE;
    p_out_debug->estop_state      = (uint8_t)ESTOP_ACTIVE;
    p_out_debug->ch1_percent      = 0;
    p_out_debug->aux_knob_val     = 0U;
    p_out_debug->sys_reset_pulse  = false;
}

/**
 * @brief  Refresh ELRS parser and fetch a safe channel snapshot.
 * @param  current_tick_ms Current system tick in milliseconds.
 * @param  p_out_channels Output channel array.
 * @return ARA_OK on success, otherwise driver / link error code.
 * @note   CRC errors are treated as non-fatal when link is still up and a valid
 *         previous channel snapshot exists.
 */
static AraStatus_t TaskRc_RefreshAndCopyChannels(uint32_t current_tick_ms,
                                                 uint16_t *p_out_channels)
{
    AraStatus_t drv_status;
    AraStatus_t copy_status;

    if (p_out_channels == NULL) {
        return ARA_ERR_PARAM;
    }

    drv_status = DrvElrs_Update(&s_rc_ctx.elrs_driver, current_tick_ms);

    if (!DrvElrs_IsLinkUp(&s_rc_ctx.elrs_driver)) {
        return ARA_ERR_DISCONNECTED;
    }

    copy_status = DrvElrs_CopyChannels(&s_rc_ctx.elrs_driver,
                                       p_out_channels,
                                       DRV_ELRS_MAX_CHANNELS);
    if (copy_status != ARA_OK) {
        return copy_status;
    }

    /* CRC errors are non-fatal if link remains up and snapshot copy succeeded. */
    if ((drv_status != ARA_OK) && (drv_status != ARA_ERR_CRC)) {
        return drv_status;
    }

    return ARA_OK;
}

/* =========================================================
 * 3. Public API Implementation
 * ========================================================= */

void TaskRc_Init(void)
{
    AraStatus_t l2_status;
    AraStatus_t l3_status;
    uint16_t default_chs[DRV_ELRS_MAX_CHANNELS];

    memset(&s_rc_ctx, 0, sizeof(TaskRc_Context_t));

    l2_status = DrvElrs_Init(&s_rc_ctx.elrs_driver, BSP_UART_ELRS);

    TaskRc_GenerateDefaultChannels(default_chs);
    l3_status = ModRcSemantic_Init(&s_rc_ctx.semantic_logic, default_chs);

    if ((l2_status == ARA_OK) && (l3_status == ARA_OK)) {
        s_rc_ctx.is_initialized = true;
    }
}

void TaskRc_Update(uint32_t current_tick_ms, RcControlData_t *p_out_intent)
{
    AraStatus_t status;
    uint16_t channels[DRV_ELRS_MAX_CHANNELS];

    if (p_out_intent == NULL) {
        return;
    }

    if (!s_rc_ctx.is_initialized) {
        TaskRc_ApplyFailsafeIntent(p_out_intent);
        return;
    }

    status = TaskRc_RefreshAndCopyChannels(current_tick_ms, channels);
    if (status != ARA_OK) {
        TaskRc_ApplyFailsafeIntent(p_out_intent);
        return;
    }

    memset(p_out_intent, 0, sizeof(RcControlData_t));

    status = ModRcSemantic_Process(&s_rc_ctx.semantic_logic,
                                   channels,
                                   current_tick_ms,
                                   p_out_intent);
    if (status != ARA_OK) {
        TaskRc_ApplyFailsafeIntent(p_out_intent);
        return;
    }

    p_out_intent->is_link_up = true;
}

void TaskRc_UpdateDebugAnalog(uint32_t current_tick_ms,
                              RcDebugAnalogData_t *p_out_debug)
{
    AraStatus_t status;
    uint16_t channels[DRV_ELRS_MAX_CHANNELS];

    if (p_out_debug == NULL) {
        return;
    }

    if (!s_rc_ctx.is_initialized) {
        TaskRc_ApplyFailsafeDebug(p_out_debug);
        return;
    }

    status = TaskRc_RefreshAndCopyChannels(current_tick_ms, channels);
    if (status != ARA_OK) {
        TaskRc_ApplyFailsafeDebug(p_out_debug);
        return;
    }

    memset(p_out_debug, 0, sizeof(RcDebugAnalogData_t));

    status = ModRcSemantic_ProcessDebugAnalog(&s_rc_ctx.semantic_logic,
                                              channels,
                                              current_tick_ms,
                                              p_out_debug);
    if (status != ARA_OK) {
        TaskRc_ApplyFailsafeDebug(p_out_debug);
        return;
    }

    p_out_debug->is_link_up = true;
}