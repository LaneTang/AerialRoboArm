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

/* Time thresholds for SE Button (Arm Control) */
#define MOD_RC_SE_DEBOUNCE_MS        (30U)   // Stable-state debounce window (increased for reliability)
#define MOD_RC_SE_SHORT_PRESS_MIN_MS (30U)   // Minimum valid short-press width (reduced for better sensitivity)
#define MOD_RC_SE_LONG_PRESS_MS      (800U)  // Long press trigger threshold
#define MOD_RC_SE_PULSE_HOLD_MS      (100U)  // How long to hold the pulse output (for better detection)

/* Channel Mapping Indices (CRSF array is 0-indexed) */
#define MOD_RC_IDX_SA  4  // CH5: Mode Request (2-pos)
#define MOD_RC_IDX_SE  5  // CH6: Arm Control (Momentary Button)
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

/**
 * @brief RC Semantic Module Context.
 * Stores historical state for edge detection and timing logic.
 */
typedef struct {
    /* --- SE (Arm Control) Timing State --- */
    bool     se_last_state;         // Previous debounced state (true = pressed)
    bool     se_raw_state;          // Latest raw sampled state
    uint32_t se_debounce_start_ms;  // Tick when raw state last changed
    uint32_t se_press_start_ms;     // Tick when button was first pressed
    bool     se_long_triggered;     // Flag to prevent repeated long-press triggers
    uint32_t se_pulse_end_ms;       // Tick when pulse output should end
    bool     se_pulse_active;       // Flag indicating pulse is being held

    /* --- SB (System Reset) Edge State --- */
    uint8_t  sb_last_pos;           // Previous physical position (0=Up, 1=Mid, 2=Down)

} ModRcSemantic_Context_t;


/* =========================================================
 * 3. Module API
 * ========================================================= */

/**
 * @brief Initialize the Semantic Module Context
 * @param p_ctx       Pointer to module context
 * @param initial_chs Pointer to the initial 16-channel array from L2.
 * Used to prime the 'last_state' variables and prevent
 * false edge-triggers on power-up.
 * @return ARA_OK on success, ARA_ERR_PARAM if null pointers.
 */
AraStatus_t ModRcSemantic_Init(ModRcSemantic_Context_t *p_ctx, const uint16_t *initial_chs);

/**
 * @brief Process raw channels into Semantic Intent Data
 * @note  This function is stateless to the hardware but stateful to time.
 * @param p_ctx            Pointer to module context
 * @param channels         Pointer to array of 16 raw channel values (0-2047)
 * @param current_tick_ms  Current system tick (used for button timing)
 * @param out_data         Pointer to the output semantic structure (RcControlData_t)
 * @return ARA_OK on successful parsing. ARA_ERR_PARAM on null pointers.
 */
AraStatus_t ModRcSemantic_Process(ModRcSemantic_Context_t *p_ctx,
                                  const uint16_t *channels,
                                  uint32_t current_tick_ms,
                                  RcControlData_t *out_data);

#endif /* MOD_RC_SEMANTIC_H */
