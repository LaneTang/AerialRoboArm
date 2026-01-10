/**
 * @file alg_pid.c
 * @brief Fixed-Point PID Controller Implementation (Q15 / Q8.8 Mixed Format)
 * @author ARA Project Coder
 */

#include "alg_pid.h"
#include <string.h> // For memset

/* --- Macros --- */
#define GAIN_SCALE_SHIFT    (8)     // Shift for Q8.8 gains to align with Q15
#define Q15_MIN             (-32768)
#define Q15_MAX             (32767)

/* --- Helper: Clamp int32 --- */
static inline int32_t clamp_i32(int32_t val, int32_t max_abs) {
    if (val > max_abs) return max_abs;
    if (val < -max_abs) return -max_abs;
    return val;
}

/* --- Helper: Clamp int16 --- */
static inline int16_t clamp_i16(int32_t val, int16_t max_abs) {
    if (val > max_abs) return max_abs;
    if (val < -max_abs) return -max_abs;
    return val;
}

/* --- API Implementation --- */

void AlgPid_Init(AlgPid_Context_t *ctx)
{
    if (ctx == NULL) return;

    // Clear entire context
    memset(ctx, 0, sizeof(AlgPid_Context_t));
}

void AlgPid_SetGains(AlgPid_Context_t *ctx,
                     int16_t kp,
                     int16_t ki,
                     int16_t kd,
                     int16_t max_out,
                     int32_t max_int)
{
    if (ctx == NULL) return;

    ctx->Kp = kp;
    ctx->Ki = ki;
    ctx->Kd = kd;

    // Store limits (Absolutes)
    ctx->out_max = (max_out < 0) ? (int16_t)-max_out : max_out;
    ctx->int_max = (max_int < 0) ? -max_int : max_int;

    // Optional: Reset state when gains change excessively?
    // Usually kept separate. We leave state untouched here.
}

void AlgPid_Reset(AlgPid_Context_t *ctx)
{
    if (ctx == NULL) return;

    ctx->integral_sum = 0;
    ctx->prev_error = 0;
}

int16_t AlgPid_Compute(AlgPid_Context_t *ctx, int16_t target, int16_t measured)
{
    if (ctx == NULL) return 0;

    /* 1. Calculate Error (Q15) */
    // Input range Q15 [-1.0, 1.0].
    // Subtraction might slightly exceed int16 temporarily, use int32.
    int32_t error = (int32_t)target - (int32_t)measured;

    /* 2. Proportional Term */
    // Logic: (Error_Q15 * Kp_Q8.8) = Result_Q23.8
    // Target: Q15.
    // Operation: Result >> 8
    int32_t p_term = (error * (int32_t)ctx->Kp) >> GAIN_SCALE_SHIFT;

    /* 3. Integral Term */
    // Logic: Accumulate (Error * Ki) directly into Q23 accumulator.
    // This provides higher resolution for the integrator.
    int32_t i_step = (error * (int32_t)ctx->Ki); // Q23

    ctx->integral_sum += i_step;

    // Anti-Windup: Clamp Accumulator
    ctx->integral_sum = clamp_i32(ctx->integral_sum, ctx->int_max);

    // Convert Accumulator (Q23) to Output (Q15)
    int32_t i_term = ctx->integral_sum >> GAIN_SCALE_SHIFT;

    /* 4. Derivative Term */
    // Logic: (dError_Q15 * Kd_Q8.8) >> 8
    int32_t d_error = error - (int32_t)ctx->prev_error;
    int32_t d_term = (d_error * (int32_t)ctx->Kd) >> GAIN_SCALE_SHIFT;

    // Save state
    // Clamp prev_error to int16 range just in case, though error shouldn't exceed it normally
    if (error > 32767) ctx->prev_error = 32767;
    else if (error < -32768) ctx->prev_error = -32768;
    else ctx->prev_error = (int16_t)error;

    /* 5. Summation & Saturation */
    int32_t out = p_term + i_term + d_term;

    // Final Clamp to Output Limits (Q15)
    return clamp_i16(out, ctx->out_max);
}