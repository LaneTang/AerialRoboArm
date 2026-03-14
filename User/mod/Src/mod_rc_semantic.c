/**
 * @file mod_rc_semantic.c
 * @brief RC Semantic Mapping Module (L3) Implementation
 * @note  Pure logic layer. NO FLOATING POINT MATH. NO HARDWARE CALLS.
 */

#include "mod_rc_semantic.h"

/* =========================================================
 * Internal Helper Functions
 * ========================================================= */

/**
 * @brief Helper to map 2-pos switch to boolean
 * @return true if channel value > MID, false otherwise
 */
static bool map_2pos(uint16_t ch_val)
{
    return (ch_val > MOD_RC_VAL_MID);
}

/**
 * @brief Helper to map 3-pos switch to 0, 1, 2 (Up, Mid, Down)
 */
static uint8_t map_3pos(uint16_t ch_val)
{
    if (ch_val < MOD_RC_SW_LOW) {
        return 0; // UP
    } else if (ch_val > MOD_RC_SW_HIGH) {
        return 2; // DOWN
    }
    return 1;     // MID
}

/**
 * @brief Helper to map analog channel to permille (0-1000) using INTEGER math
 */
static uint16_t map_analog_permille(uint16_t ch_val)
{
    /* Clamp the input to standard bounds */
    if (ch_val < MOD_RC_VAL_MIN) ch_val = MOD_RC_VAL_MIN;
    if (ch_val > MOD_RC_VAL_MAX) ch_val = MOD_RC_VAL_MAX;

    uint32_t range = MOD_RC_VAL_MAX - MOD_RC_VAL_MIN;
    uint32_t value = ch_val - MOD_RC_VAL_MIN;

    /* Calculate permille: (value * 1000) / range
     * Max intermediate value: 1639 * 1000 = 1,639,000 (safely fits in uint32_t)
     */
    return (uint16_t)((value * 1000) / range);
}


/* =========================================================
 * API Implementation
 * ========================================================= */

AraStatus_t ModRcSemantic_Init(ModRcSemantic_Context_t *p_ctx, const uint16_t *initial_chs)
{
    if (p_ctx == NULL || initial_chs == NULL) {
        return ARA_ERR_PARAM;
    }

    /* Prime the state variables to prevent false edge-triggers on boot */
    p_ctx->se_last_state = map_2pos(initial_chs[MOD_RC_IDX_SE]);
    p_ctx->se_raw_state = p_ctx->se_last_state;
    p_ctx->se_debounce_start_ms = 0;
    p_ctx->se_press_start_ms = 0;
    p_ctx->se_long_triggered = false;
    p_ctx->se_pulse_end_ms = 0;
    p_ctx->se_pulse_active = false;

    p_ctx->sb_last_pos = map_3pos(initial_chs[MOD_RC_IDX_SB]);

    return ARA_OK;
}

AraStatus_t ModRcSemantic_Process(ModRcSemantic_Context_t *p_ctx,
                                  const uint16_t *channels,
                                  uint32_t current_tick_ms,
                                  RcControlData_t *out_data)
{
    if (p_ctx == NULL || channels == NULL || out_data == NULL) {
        return ARA_ERR_PARAM;
    }

    /* ---------------------------------------------------------
     * 1. State-based Semantics (SA, SC, SD, SF)
     * --------------------------------------------------------- */

    /* SA: Mode Request (2-pos: Up=Auto, Down=Manual) */
    if (map_2pos(channels[MOD_RC_IDX_SA])) {
        out_data->req_mode = ARA_MODE_MANUAL;
    } else {
        out_data->req_mode = ARA_MODE_AUTO;
    }

    /* SC: Gripper Command (3-pos) */
    uint8_t sc_pos = map_3pos(channels[MOD_RC_IDX_SC]);
    if (sc_pos == 0)      out_data->gripper_cmd = GRIPPER_CMD_OPEN;
    else if (sc_pos == 2) out_data->gripper_cmd = GRIPPER_CMD_CLOSE;
    else                  out_data->gripper_cmd = GRIPPER_CMD_STOP;

    /* SD: E-Stop State (2-pos: Up=Active, Down=Released/Safe) */
    if (map_2pos(channels[MOD_RC_IDX_SD]) == false) {
        out_data->estop_state = ESTOP_ACTIVE;
    } else {
        out_data->estop_state = ESTOP_RELEASED;
    }

    /* SF: Aux Knob (Analog 0-1000 permille) */
    out_data->aux_knob_val = map_analog_permille(channels[MOD_RC_IDX_SF]);


    /* ---------------------------------------------------------
     * 2. Timing-based Semantics (SE - Arm Control)
     * --------------------------------------------------------- */
    bool se_sample = map_2pos(channels[MOD_RC_IDX_SE]);

    /* Default to HOLD every cycle unless explicitly overridden below */
    out_data->arm_cmd = ARM_CMD_HOLD;

    /* Non-blocking debounce: mimic double-check-after-20ms without HAL_Delay */
    if (se_sample != p_ctx->se_raw_state) {
        p_ctx->se_raw_state = se_sample;
        p_ctx->se_debounce_start_ms = current_tick_ms;
    }

    bool se_now = p_ctx->se_last_state;
    if ((se_now != p_ctx->se_raw_state) &&
        ((current_tick_ms - p_ctx->se_debounce_start_ms) >= MOD_RC_SE_DEBOUNCE_MS)) {
        bool se_prev = p_ctx->se_last_state;
        se_now = p_ctx->se_raw_state;
        p_ctx->se_last_state = se_now;

        /* Rising Edge: confirmed press */
        if (!se_prev && se_now) {
            p_ctx->se_press_start_ms = current_tick_ms;
            p_ctx->se_long_triggered = false;
        }
        /* Falling Edge: confirmed release */
        else if (se_prev && !se_now) {
            uint32_t press_duration = current_tick_ms - p_ctx->se_press_start_ms;

            if (!p_ctx->se_long_triggered && press_duration >= MOD_RC_SE_SHORT_PRESS_MIN_MS) {
                /* Trigger pulse and set hold timer */
                p_ctx->se_pulse_active = true;
                p_ctx->se_pulse_end_ms = current_tick_ms + MOD_RC_SE_PULSE_HOLD_MS;
            }
        }
    }

    /* Check if pulse should be output (either just triggered or still holding) */
    if (p_ctx->se_pulse_active) {
        if (current_tick_ms < p_ctx->se_pulse_end_ms) {
            out_data->arm_cmd = ARM_CMD_RETRACT;  // Hold pulse for multiple cycles
        } else {
            p_ctx->se_pulse_active = false;  // Pulse expired
        }
    }

    /* Button is currently being held after debounce */
    if (p_ctx->se_last_state) {
        uint32_t press_duration = current_tick_ms - p_ctx->se_press_start_ms;

        if (press_duration >= MOD_RC_SE_LONG_PRESS_MS) {
            out_data->arm_cmd = ARM_CMD_EXTEND;   // Continuous output while held
            p_ctx->se_long_triggered = true;
            p_ctx->se_pulse_active = false;  // Cancel any pending pulse
        }
    }


    /* ---------------------------------------------------------
     * 3. Edge-based Semantics (SB - System Reset)
     * --------------------------------------------------------- */
    uint8_t sb_now = map_3pos(channels[MOD_RC_IDX_SB]);
    out_data->sys_reset_pulse = false;

    /* Detect edge transitioning TO the Down position (2) */
    if (p_ctx->sb_last_pos != 2 && sb_now == 2) {
        out_data->sys_reset_pulse = true;  // Pulse output for exactly 1 cycle
    }
    p_ctx->sb_last_pos = sb_now;


    /* Note: Link state must be handled by L4 (Task_RC), as L3 doesn't know about the hardware */

    return ARA_OK;
}
