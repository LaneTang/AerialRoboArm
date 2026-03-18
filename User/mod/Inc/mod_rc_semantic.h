/**
 * @file mod_rc_semantic.h
 * @brief RC Semantic Mapping Module (L3)
 * @note  Pure logic layer. Converts raw CRSF channels (0-2047) into high-level
 * system intents (Auto/Manual, E-Stop, Arm logic).
 * STRICTLY NO FLOATING POINT MATH. NO HARDWARE CALLS.
 */

#ifndef MOD_RC_SEMANTIC_H
#define MOD_RC_SEMANTIC_H

#include "ara_def.h"
#include "datahub.h"    // Requires RcControlData_t and semantic enums

/* =========================================================
 * 1. Logic & Timing Constants
 * ========================================================= */

/* Legacy SE constants kept for compatibility (SE arm logic is no longer used) */
#define MOD_RC_SE_DEBOUNCE_MS        (30U)
#define MOD_RC_SE_SHORT_PRESS_MIN_MS (30U)
#define MOD_RC_SE_LONG_PRESS_MS      (800U)
#define MOD_RC_SE_PULSE_HOLD_MS      (100U)

/* SB: Down-level trigger with cooldown */
#define MOD_RC_SB_PULSE_HOLD_MS      (150U)  // Pulse hold duration for observability/reliability
#define MOD_RC_SB_COOLDOWN_MS        (200U)  // At most one trigger every 200ms while Down

/* CH1 stick thresholds for arm control (percent scale: -100 ~ +100) */
#define MOD_RC_CH1_ENTER_EXTEND_PCT  (55)    // > +55 => EXTEND
#define MOD_RC_CH1_ENTER_RETRACT_PCT (-55)   // < -55 => RETRACT
#define MOD_RC_CH1_EXIT_HOLD_ABS_PCT (35)    // |x| < 35 => HOLD (hysteresis)

/* Channel Mapping Indices (CRSF array is 0-indexed) */
#define MOD_RC_IDX_CH1 0  // CH1: Stick X axis (Arm analog intent)
#define MOD_RC_IDX_SA  4  // CH5: Mode Request (2-pos)
#define MOD_RC_IDX_SE  5  // CH6: Legacy button input (unused in arm mapping)
#define MOD_RC_IDX_SC  6  // CH7: Gripper (3-pos)
#define MOD_RC_IDX_SF  7  // CH8: Aux Knob (Analog)
#define MOD_RC_IDX_SB  8  // CH9: System Reset (3-pos)
#define MOD_RC_IDX_SD  9  // CH10: E-Stop (2-pos)

/* CRSF Signal Thresholds */
#define MOD_RC_VAL_MIN 172
#define MOD_RC_VAL_MAX 1811
#define MOD_RC_VAL_MID 992
#define MOD_RC_SW_LOW  600
#define MOD_RC_SW_HIGH 1400

/* =========================================================
 * 2. Module Context Structure
 * ========================================================= */
typedef struct {
    /* --- Arm control state (CH1 hysteresis latch) --- */
    AraArmCmd_t arm_last_cmd;

    /* --- SB (System Reset) Level + Cooldown State --- */
    uint8_t  sb_last_pos;       // Reserved for compatibility/debug visibility
    uint32_t sb_last_fire_ms;   // Last trigger tick for cooldown limiting
    uint32_t sb_pulse_end_ms;   // Tick when SB pulse output should end
    bool     sb_pulse_active;   // Flag indicating SB pulse is being held
} ModRcSemantic_Context_t;

/* =========================================================
 * 3. Module API
 * ========================================================= */
AraStatus_t ModRcSemantic_Init(ModRcSemantic_Context_t *p_ctx, const uint16_t *initial_chs);

AraStatus_t ModRcSemantic_Process(ModRcSemantic_Context_t *p_ctx,
                                  const uint16_t *channels,
                                  uint32_t current_tick_ms,
                                  RcControlData_t *out_data);

#endif /* MOD_RC_SEMANTIC_H */