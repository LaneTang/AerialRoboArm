/**
 * @file mod_rc_semantic.c
 * @brief RC Semantic Mapping Module (L3) Implementation
 * @note  Pure logic layer. No floating-point math. No hardware calls.
 */

#include "mod_rc_semantic.h"

/* =========================================================
 * Internal Helper Functions
 * ========================================================= */

/**
 * @brief  Map a 2-position switch to boolean.
 * @param  ch_val Raw channel value.
 * @return true if above MID, otherwise false.
 */
static bool map_2pos(uint16_t ch_val)
{
    return (ch_val > MOD_RC_VAL_MID);
}

/**
 * @brief  Map a 3-position switch to {0,1,2}.
 * @param  ch_val Raw channel value.
 * @return 0 for UP, 1 for MID, 2 for DOWN.
 */
static uint8_t map_3pos(uint16_t ch_val)
{
    if (ch_val < MOD_RC_SW_LOW) {
        return 0U;
    }

    if (ch_val > MOD_RC_SW_HIGH) {
        return 2U;
    }

    return 1U;
}

/**
 * @brief  Map an analog channel to permille [0, 1000].
 * @param  ch_val Raw channel value.
 * @return Permille value.
 */
static uint16_t map_analog_permille(uint16_t ch_val)
{
    uint32_t range;
    uint32_t value;

    if (ch_val < MOD_RC_VAL_MIN) {
        ch_val = MOD_RC_VAL_MIN;
    }

    if (ch_val > MOD_RC_VAL_MAX) {
        ch_val = MOD_RC_VAL_MAX;
    }

    range = (uint32_t)MOD_RC_VAL_MAX - (uint32_t)MOD_RC_VAL_MIN;
    value = (uint32_t)ch_val - (uint32_t)MOD_RC_VAL_MIN;

    return (uint16_t)((value * 1000U) / range);
}

/**
 * @brief  Map CH1 raw value to signed percent [-100, +100].
 * @param  ch_val Raw channel value.
 * @return Signed percentage.
 */
static int16_t map_ch1_percent(int16_t ch_val)
{
    int32_t delta;

    if (ch_val < (int16_t)MOD_RC_VAL_MIN) {
        ch_val = (int16_t)MOD_RC_VAL_MIN;
    }

    if (ch_val > (int16_t)MOD_RC_VAL_MAX) {
        ch_val = (int16_t)MOD_RC_VAL_MAX;
    }

    delta = (int32_t)ch_val - (int32_t)MOD_RC_VAL_MID;

    if (delta >= 0) {
        int32_t pos_span = (int32_t)MOD_RC_VAL_MAX - (int32_t)MOD_RC_VAL_MID;
        return (int16_t)((delta * 100L) / pos_span);
    }

    {
        int32_t neg_span = (int32_t)MOD_RC_VAL_MID - (int32_t)MOD_RC_VAL_MIN;
        return (int16_t)((delta * 100L) / neg_span);
    }
}

/**
 * @brief  Update SB edge-triggered single-pulse state.
 * @param  p_ctx Pointer to module context.
 * @param  sb_now Current decoded SB position.
 * @param  current_tick_ms Current system tick.
 * @return true while reset pulse is active, otherwise false.
 */
static bool update_sb_reset_pulse(ModRcSemantic_Context_t *p_ctx,
                                  uint8_t sb_now,
                                  uint32_t current_tick_ms)
{
    bool pulse_out = false;

    if ((p_ctx->sb_last_pos != 2U) && (sb_now == 2U)) {
        p_ctx->sb_pulse_active = true;
        p_ctx->sb_pulse_start_ms = current_tick_ms;
    }

    if (p_ctx->sb_pulse_active) {
        if ((uint32_t)(current_tick_ms - p_ctx->sb_pulse_start_ms) < MOD_RC_SB_PULSE_HOLD_MS) {
            pulse_out = true;
        } else {
            p_ctx->sb_pulse_active = false;
        }
    }

    p_ctx->sb_last_pos = sb_now;
    return pulse_out;
}

/**
 * @brief  Map an analog channel to physical degree [0, 180].
 */
static uint8_t map_analog_degree(uint16_t ch_val)
{
    uint32_t range;
    uint32_t value;

    if (ch_val < MOD_RC_VAL_MIN) ch_val = MOD_RC_VAL_MIN;
    if (ch_val > MOD_RC_VAL_MAX) ch_val = MOD_RC_VAL_MAX;

    range = (uint32_t)MOD_RC_VAL_MAX - (uint32_t)MOD_RC_VAL_MIN;
    value = (uint32_t)ch_val - (uint32_t)MOD_RC_VAL_MIN;

    return (uint8_t)((value * 180U) / range);
}

/* =========================================================
 * API Implementation
 * ========================================================= */

AraStatus_t ModRcSemantic_Init(ModRcSemantic_Context_t *p_ctx,
                               const uint16_t *initial_chs)
{
    if ((p_ctx == NULL) || (initial_chs == NULL)) {
        return ARA_ERR_PARAM;
    }

    p_ctx->arm_last_cmd = ARM_CMD_HOLD;
    p_ctx->sb_last_pos = map_3pos(initial_chs[MOD_RC_IDX_SB]);
    p_ctx->sb_pulse_start_ms = 0U;
    p_ctx->sb_pulse_active = false;

    return ARA_OK;
}

AraStatus_t ModRcSemantic_Process(ModRcSemantic_Context_t *p_ctx,
                                  const uint16_t *channels,
                                  uint32_t current_tick_ms,
                                  RcControlData_t *out_data)
{
    int16_t ch1_pct;
    int16_t abs_pct;
    uint8_t sc_pos;
    uint8_t sb_now;

    if ((p_ctx == NULL) || (channels == NULL) || (out_data == NULL)) {
        return ARA_ERR_PARAM;
    }

    /* ---------------- State-based semantics ---------------- */

    /* SA: Up = AUTO, Down = MANUAL */
    if (map_2pos(channels[MOD_RC_IDX_SA])) {
        out_data->req_mode = ARA_MODE_MANUAL;
    } else {
        out_data->req_mode = ARA_MODE_AUTO;
    }

    /* SC: Gripper */
    sc_pos = map_3pos(channels[MOD_RC_IDX_SC]);
    if (sc_pos == 0U) {
        out_data->gripper_cmd = GRIPPER_CMD_OPEN;
    } else if (sc_pos == 2U) {
        out_data->gripper_cmd = GRIPPER_CMD_CLOSE;
    } else {
        out_data->gripper_cmd = GRIPPER_CMD_STOP;
    }

    /* SD: Up = ACTIVE, Down = RELEASED */
    if (!map_2pos(channels[MOD_RC_IDX_SD])) {
        out_data->estop_state = ESTOP_ACTIVE;
    } else {
        out_data->estop_state = ESTOP_RELEASED;
    }

    /* SF: Aux knob */
    out_data->aux_knob_val = map_analog_permille(channels[MOD_RC_IDX_SF]);

    /* ---------------- CH1 arm semantics with hysteresis ---------------- */
    out_data->arm_cmd = ARM_CMD_HOLD;

    if ((out_data->req_mode == ARA_MODE_MANUAL) &&
        (out_data->estop_state == ESTOP_RELEASED))
    {
        ch1_pct = map_ch1_percent((int16_t)channels[MOD_RC_IDX_CH1]);
        abs_pct = (ch1_pct >= 0) ? ch1_pct : (int16_t)(-ch1_pct);

        switch (p_ctx->arm_last_cmd) {
            case ARM_CMD_EXTEND:
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

    /* ---------------- SB edge-triggered reset pulse ---------------- */
    sb_now = map_3pos(channels[MOD_RC_IDX_SB]);
    out_data->sys_reset_pulse = update_sb_reset_pulse(p_ctx, sb_now, current_tick_ms);

    return ARA_OK;
}

AraStatus_t ModRcSemantic_ProcessDebugAnalog(ModRcSemantic_Context_t *p_ctx,
                                             const uint16_t *channels,
                                             uint32_t current_tick_ms,
                                             RcDebugAnalogData_t *out_data)
{
    uint8_t sb_now;

    if ((p_ctx == NULL) || (channels == NULL) || (out_data == NULL)) {
        return ARA_ERR_PARAM;
    }

    /* SA: Up = AUTO, Down = MANUAL */
    if (map_2pos(channels[MOD_RC_IDX_SA])) {
        out_data->req_mode = (uint8_t)ARA_MODE_MANUAL;
    } else {
        out_data->req_mode = (uint8_t)ARA_MODE_AUTO;
    }

    /* SD: Up = ACTIVE, Down = RELEASED */
    if (!map_2pos(channels[MOD_RC_IDX_SD])) {
        out_data->estop_state = (uint8_t)ESTOP_ACTIVE;
    } else {
        out_data->estop_state = (uint8_t)ESTOP_RELEASED;
    }

    out_data->ch1_percent = map_ch1_percent((int16_t)channels[MOD_RC_IDX_CH1]);
    out_data->aux_knob_val = map_analog_permille(channels[MOD_RC_IDX_SF]);

    sb_now = map_3pos(channels[MOD_RC_IDX_SB]);
    out_data->sys_reset_pulse = update_sb_reset_pulse(p_ctx, sb_now, current_tick_ms);

    out_data->roll_angle = map_analog_degree(channels[MOD_RC_IDX_CH3]);
    out_data->gripper_angle = map_analog_degree(channels[MOD_RC_IDX_SF]); // CH8 就是 SF

    return ARA_OK;
}