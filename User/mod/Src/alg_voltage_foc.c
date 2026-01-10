/**
 * @file alg_voltage_foc.c
 * @brief Voltage-based FOC Algorithm Implementation (Fixed-Point)
 * @author ARA Project Coder
 */

#include "alg_voltage_foc.h"

/* --- Constants --- */
#define FOC_Q15_ONE         (32768)
#define FOC_SQRT3_BY_2      (28377) // sqrt(3)/2 in Q15
#define FOC_LUT_SIZE        (256)
#define FOC_LUT_MASK        (0xFF)

/* --- Sinusoid Look-Up Table (Q15) --- */
/* Generated for 0 to 2*PI, Amplitude 32767. Size 256. */
static const int16_t sin_lut[FOC_LUT_SIZE] = {
        0,   804,  1608,  2410,  3212,  4011,  4808,  5602,  6393,  7179,  7962,  8739,  9512, 10278, 11039, 11793,
        12539, 13279, 14010, 14732, 15446, 16151, 16846, 17530, 18204, 18868, 19519, 20159, 20787, 21403, 22005, 22594,
        23170, 23731, 24279, 24811, 25329, 25832, 26319, 26790, 27245, 27683, 28105, 28510, 28898, 29268, 29621, 29956,
        30273, 30571, 30852, 31113, 31356, 31580, 31785, 31971, 32137, 32285, 32412, 32521, 32609, 32678, 32728, 32757,
        32767, 32757, 32728, 32678, 32609, 32521, 32412, 32285, 32137, 31971, 31785, 31580, 31356, 31113, 30852, 30571,
        30273, 29956, 29621, 29268, 28898, 28510, 28105, 27683, 27245, 26790, 26319, 25832, 25329, 24811, 24279, 23731,
        23170, 22594, 22005, 21403, 20787, 20159, 19519, 18868, 18204, 17530, 16846, 16151, 15446, 14732, 14010, 13279,
        12539, 11793, 11039, 10278,  9512,  8739,  7962,  7179,  6393,  5602,  4808,  4011,  3212,  2410,  1608,   804,
        0,  -804, -1608, -2410, -3212, -4011, -4808, -5602, -6393, -7179, -7962, -8739, -9512, -10278, -11039, -11793,
        -12539, -13279, -14010, -14732, -15446, -16151, -16846, -17530, -18204, -18868, -19519, -20159, -20787, -21403, -22005, -22594,
        -23170, -23731, -24279, -24811, -25329, -25832, -26319, -26790, -27245, -27683, -28105, -28510, -28898, -29268, -29621, -29956,
        -30273, -30571, -30852, -31113, -31356, -31580, -31785, -31971, -32137, -32285, -32412, -32521, -32609, -32678, -32728, -32757,
        -32767, -32757, -32728, -32678, -32609, -32521, -32412, -32285, -32137, -31971, -31785, -31580, -31356, -31113, -30852, -30571,
        -30273, -29956, -29621, -29268, -28898, -28510, -28105, -27683, -27245, -26790, -26319, -25832, -25329, -24811, -24279, -23731,
        -23170, -22594, -22005, -21403, -20787, -20159, -19519, -18868, -18204, -17530, -16846, -16151, -15446, -14732, -14010, -13279,
        -12539, -11793, -11039, -10278,  9512,  8739,  7962,  7179,  6393,  5602,  4808,  4011,  3212,  2410,  1608,   804
};

/* --- Helper Macros --- */
#define CLAMP_U16(x, limit) ((x) > (limit) ? (limit) : (x))

void AlgFoc_Init(AlgFoc_Context_t *ctx, uint8_t pole_pairs, uint16_t pwm_period)
{
    if (ctx == NULL) return;

    ctx->pole_pairs = pole_pairs;
    ctx->pwm_period = pwm_period;
    ctx->zero_offset = 0;

    // Reset runtime state
    ctx->electric_angle = 0;
    ctx->duty_a = 0;
    ctx->duty_b = 0;
    ctx->duty_c = 0;
}

void AlgFoc_SetZeroOffset(AlgFoc_Context_t *ctx, uint16_t raw_offset)
{
    if (ctx == NULL) return;

    // Ensure 12-bit range
    ctx->zero_offset = raw_offset & 0x0FFF;
}

void AlgFoc_Run(AlgFoc_Context_t *ctx, uint16_t raw_angle_12bit, int16_t u_q, int16_t u_d)
{
    if (ctx == NULL) return;

    /* --- 1. Electrical Angle Calculation --- */
    // Raw (12-bit) -> Corrected Mechanical
    int16_t mech_angle = (int16_t)(raw_angle_12bit & 0x0FFF) - (int16_t)ctx->zero_offset;

    // Normalize to [0, 4095] (Handle negative wrap)
    // 0x0FFF is 4095.
    // If mech_angle is -100, (-100 & 0xFFF) = 3996. Correct.
    mech_angle &= 0x0FFF;

    // Mech -> Electrical
    // Formula: (Mech * Pairs) % OneRotation
    // Note: Use uint32 for calculation to avoid overflow before modulo
    // Map 0-4095 (Mech) to 0-255 (LUT Index)
    // Factor: 256 / 4096 = 1/16.
    // Index = (Mech * Poles) / 16

    uint32_t angle_calc = (uint32_t)mech_angle * ctx->pole_pairs;
    // Map to 8-bit LUT index (0-255)
    // Original resolution 4096. We need modulo 256 relative to full circle.
    // The previous logic 'angle_calc' is effectively in units of (1/4096) * Poles.
    // We want units of (1/256). So we shift right by 4 (divide by 16).
    uint8_t angle_idx = (uint32_t)(angle_calc >> 4) & FOC_LUT_MASK;

    ctx->electric_angle = angle_idx; // Store for debug

    /* --- 2. Sine / Cosine Lookup --- */
    // sin(theta)
    int16_t s = sin_lut[angle_idx];
    // cos(theta) = sin(theta + 90deg). 90deg is 64 steps in 256-entry table.
    int16_t c = sin_lut[(angle_idx + 64) & FOC_LUT_MASK];

    /* --- 3. Inverse Park Transform (D/Q -> Alpha/Beta) --- */
    // U_alpha = U_d * cos - U_q * sin
    // U_beta  = U_d * sin + U_q * cos
    // Inputs are Q15. LUT is Q15. Result is Q30 -> Shift 15 -> Q15.

    int32_t u_alpha_32 = ((int32_t)u_d * c) - ((int32_t)u_q * s);
    int32_t u_beta_32  = ((int32_t)u_d * s) + ((int32_t)u_q * c);

    int16_t u_alpha = (int16_t)(u_alpha_32 >> 15);
    int16_t u_beta  = (int16_t)(u_beta_32  >> 15);

    /* --- 4. Inverse Clarke Transform (Alpha/Beta -> A/B/C) --- */
    // Ua = U_alpha
    // Ub = (-U_alpha + sqrt(3)*U_beta) / 2
    // Uc = (-U_alpha - sqrt(3)*U_beta) / 2

    int32_t sqrt3_beta = ((int32_t)FOC_SQRT3_BY_2 * u_beta); // Q30
    // We need Q15 result for the term (sqrt3 * beta).
    // sqrt3_beta is Q30. Shift 15 -> Q15.
    int16_t term_beta = (int16_t)(sqrt3_beta >> 15);

    int16_t u_a = u_alpha;
    int16_t u_b = (-u_alpha / 2) + term_beta;
    int16_t u_c = (-u_alpha / 2) - term_beta;

    /* --- 5. SVPWM (Midpoint Clamping / Center Aligned) --- */
    // Find min and max
    int16_t min_v = u_a;
    int16_t max_v = u_a;

    if (u_b < min_v) min_v = u_b;
    if (u_b > max_v) max_v = u_b;
    if (u_c < min_v) min_v = u_c;
    if (u_c > max_v) max_v = u_c;

    // Calculate common mode offset
    int16_t mid = (min_v + max_v) / 2;

    // Apply offset (Result is still Q15 signed)
    u_a -= mid;
    u_b -= mid;
    u_c -= mid;

    /* --- 6. PWM Generation (Q15 Signed -> 0..ARR) --- */
    // Range Q15 signed: -32768 to 32767
    // Target: 0 to ctx->pwm_period
    // Center (0V) should be pwm_period / 2.

    // Formula: duty = (pwm_period / 2) + (u_x * pwm_period / 65536)
    // Optimized: duty = (pwm_period >> 1) + ((u_x * pwm_period) >> 16)
    // Note: u_x * pwm_period fits in int32 if pwm_period < 65535.

    int32_t half_period = ctx->pwm_period >> 1;

    int32_t duty_a_calc = half_period + (((int32_t)u_a * ctx->pwm_period) >> 16);
    int32_t duty_b_calc = half_period + (((int32_t)u_b * ctx->pwm_period) >> 16);
    int32_t duty_c_calc = half_period + (((int32_t)u_c * ctx->pwm_period) >> 16);

    /* --- 7. Output & Saturation --- */
    // Clamp to 0..pwm_period safe range
    if (duty_a_calc < 0) duty_a_calc = 0;
    if (duty_b_calc < 0) duty_b_calc = 0;
    if (duty_c_calc < 0) duty_c_calc = 0;

    ctx->duty_a = CLAMP_U16((uint16_t)duty_a_calc, ctx->pwm_period);
    ctx->duty_b = CLAMP_U16((uint16_t)duty_b_calc, ctx->pwm_period);
    ctx->duty_c = CLAMP_U16((uint16_t)duty_c_calc, ctx->pwm_period);
}