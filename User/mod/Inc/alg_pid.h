/**
 * @file alg_pid.h
 * @brief Fixed-Point PID Controller
 * @warning CRITICAL MATH FORMAT:
 * - Input/Output Signals: Q15 (int16_t) -> Range [-1.0, 0.999]
 * - Gains (Kp/Ki/Kd):     Q8.8 (int16_t) -> Range [-128.0, 127.99]
 * Example: Gain of 1.5 should be passed as (1.5 * 256) = 384
 */

#ifndef ALG_PID_H
#define ALG_PID_H

#include "ara_def.h"

/* --- Context Definition --- */
typedef struct {
    /* --- Gains (Q8.8 Format) --- */
    int16_t Kp;
    int16_t Ki;
    int16_t Kd;

    /* --- Limits --- */
    int16_t out_max;        // Max output magnitude (Q15)
    int32_t int_max;        // Max integral accumulator value (Prevent int32 overflow)

    /* --- State --- */
    int32_t integral_sum;   // Internal accumulator
    int16_t prev_error;     // Previous error for D-term

} AlgPid_Context_t;

/* --- API --- */

/**
 * @brief Initialize PID Context
 * @param ctx Pointer to context
 * @note  Must call SetGains after Init
 */
void AlgPid_Init(AlgPid_Context_t *ctx);

/**
 * @brief Set PID Gains
 * @param kp Proportional Gain (Q8.8 format: 256 = 1.0)
 * @param ki Integral Gain     (Q8.8 format)
 * @param kd Derivative Gain   (Q8.8 format)
 * @param max_out Maximum output limit (Q15 format)
 * @param max_int Maximum integral limit (Absolute value)
 */
void AlgPid_SetGains(AlgPid_Context_t *ctx,
                     int16_t kp,
                     int16_t ki,
                     int16_t kd,
                     int16_t max_out,
                     int32_t max_int);

/**
 * @brief Reset PID internal state (Integrator & Previous Error)
 * @note  Call this when engaging the controller from IDLE state
 */
void AlgPid_Reset(AlgPid_Context_t *ctx);

/**
 * @brief Compute PID Output
 * @param target   Target value (Q15)
 * @param measured Current measured value (Q15)
 * @return Control Output (Q15). Saturated to +/- out_max.
 */
int16_t AlgPid_Compute(AlgPid_Context_t *ctx, int16_t target, int16_t measured);

#endif // ALG_PID_H