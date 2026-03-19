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
    if (ch_val < MOD_RC_VAL_MIN) ch_val = MOD_RC_VAL_MIN;
    if (ch_val > MOD_RC_VAL_MAX) ch_val = MOD_RC_VAL_MAX;

    {
        uint32_t range = MOD_RC_VAL_MAX - MOD_RC_VAL_MIN;
        uint32_t value = ch_val - MOD_RC_VAL_MIN;
        return (uint16_t)((value * 1000U) / range);
    }
}

/**
 * @brief Map CH1 raw value to signed percent (-100 ~ +100)
 */
static int16_t map_ch1_percent(int16_t ch_val)
{
    int32_t delta;

    if (ch_val < MOD_RC_VAL_MIN) ch_val = MOD_RC_VAL_MIN;
    if (ch_val > MOD_RC_VAL_MAX) ch_val = MOD_RC_VAL_MAX;

    delta = (int32_t)ch_val - (int32_t)MOD_RC_VAL_MID;

    if (delta >= 0) {
        int32_t pos_span = (int32_t)MOD_RC_VAL_MAX - (int32_t)MOD_RC_VAL_MID;
        return (int16_t)((delta * 100) / pos_span);
    }

    {
        int32_t neg_span = (int32_t)MOD_RC_VAL_MID - (int32_t)MOD_RC_VAL_MIN;
        return (int16_t)((delta * 100) / neg_span);
    }
}

/* =========================================================
 * API Implementation
 * ========================================================= */

AraStatus_t ModRcSemantic_Init(ModRcSemantic_Context_t *p_ctx, const uint16_t *initial_chs)
{
    (void)initial_chs;

    if (p_ctx == NULL || initial_chs == NULL) {
        return ARA_ERR_PARAM;
    }

    p_ctx->arm_last_cmd = ARM_CMD_HOLD;

    p_ctx->sb_last_pos = map_3pos(initial_chs[MOD_RC_IDX_SB]);
    p_ctx->sb_last_fire_ms = 0;
    p_ctx->sb_pulse_end_ms = 0;
    p_ctx->sb_pulse_active = false;

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
    {
        uint8_t sc_pos = map_3pos(channels[MOD_RC_IDX_SC]);
        if (sc_pos == 0)      out_data->gripper_cmd = GRIPPER_CMD_OPEN;
        else if (sc_pos == 2) out_data->gripper_cmd = GRIPPER_CMD_CLOSE;
        else                  out_data->gripper_cmd = GRIPPER_CMD_STOP;
    }

    /* SD: E-Stop State (2-pos: Up=Active, Down=Released/Safe) */
    if (map_2pos(channels[MOD_RC_IDX_SD]) == false) {
        out_data->estop_state = ESTOP_ACTIVE;
    } else {
        out_data->estop_state = ESTOP_RELEASED;
    }

    /* SF: Aux Knob (Analog 0-1000 permille) */
    out_data->aux_knob_val = map_analog_permille(channels[MOD_RC_IDX_SF]);


    /* ---------------------------------------------------------
     * 2. CH1-based Arm Semantics with Hysteresis
     * Gate: MANUAL + ESTOP_RELEASED (LinkUp handled in L4 failsafe)
     * --------------------------------------------------------- */
    out_data->arm_cmd = ARM_CMD_HOLD;

    if ((out_data->req_mode == ARA_MODE_MANUAL) && (out_data->estop_state == ESTOP_RELEASED)) {
        int16_t ch1_pct = map_ch1_percent((int16_t)channels[MOD_RC_IDX_CH1]);
        int16_t abs_pct = (ch1_pct >= 0) ? ch1_pct : (int16_t)(-ch1_pct);

        switch (p_ctx->arm_last_cmd) {
            case ARM_CMD_EXTEND:
                if (abs_pct < MOD_RC_CH1_EXIT_HOLD_ABS_PCT) {
                    p_ctx->arm_last_cmd = ARM_CMD_HOLD;
                }
                break;

            case ARM_CMD_RETRACT:
                if (abs_pct < MOD_RC_CH1_EXIT_HOLD_ABS_PCT) {
                    p_ctx->arm_last_cmd = ARM_CMD_HOLD;
                }
                break;

            case ARM_CMD_HOLD:
            default:
                if (ch1_pct > MOD_RC_CH1_ENTER_EXTEND_PCT) {
                    p_ctx->arm_last_cmd = ARM_CMD_EXTEND;
                } else if (ch1_pct < MOD_RC_CH1_ENTER_RETRACT_PCT) {
                    p_ctx->arm_last_cmd = ARM_CMD_RETRACT;
                }
                break;
        }

        out_data->arm_cmd = p_ctx->arm_last_cmd;
    } else {
        p_ctx->arm_last_cmd = ARM_CMD_HOLD;
    }


    /* ---------------------------------------------------------
     * 3. Level-based Semantics (SB - Down level + cooldown)
     * --------------------------------------------------------- */
    {
        uint8_t sb_now = map_3pos(channels[MOD_RC_IDX_SB]);
        out_data->sys_reset_pulse = false;

        /* While SB stays Down, allow retrigger at most once per cooldown window */
        if (sb_now == 2U) {
            if ((uint32_t)(current_tick_ms - p_ctx->sb_last_fire_ms) >= MOD_RC_SB_COOLDOWN_MS) {
                p_ctx->sb_pulse_active = true;
                p_ctx->sb_pulse_end_ms = current_tick_ms + MOD_RC_SB_PULSE_HOLD_MS;
                p_ctx->sb_last_fire_ms = current_tick_ms;
            }
        }

        if (p_ctx->sb_pulse_active) {
            if (current_tick_ms < p_ctx->sb_pulse_end_ms) {
                out_data->sys_reset_pulse = true;
            } else {
                p_ctx->sb_pulse_active = false;
            }
        }

        p_ctx->sb_last_pos = sb_now;
    }

    return ARA_OK;
}


