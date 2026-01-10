/**
 * @file alg_voltage_foc.h
 * @brief Voltage-based FOC Core Algorithm (Inverse Park + SVPWM)
 * @note  STRICTLY Fixed-Point. NO Float.
 * Math Model: Electrical Angle = (Mech * Poles) - ZeroOffset
 */

#ifndef ALG_VOLTAGE_FOC_H
#define ALG_VOLTAGE_FOC_H

#include "ara_def.h"

/* --- Context Definition --- */
typedef struct {
    /* --- Configuration (Set via Init) --- */
    uint8_t  pole_pairs;      // Motor pole pairs
    uint16_t pwm_period;      // Max PWM Duty Count (ARR value)

    /* --- Calibration --- */
    uint16_t zero_offset;     // Mechanical zero position (0-4095)

    /* --- Runtime State --- */
    int16_t  electric_angle;  // Calculated electrical angle (Q15 or Raw mapped)

    /* --- Outputs (0 to pwm_period) --- */
    uint16_t duty_a;
    uint16_t duty_b;
    uint16_t duty_c;

} AlgFoc_Context_t;

/* --- API --- */

/**
 * @brief Initialize FOC Context
 * @param ctx Pointer to context
 * @param pole_pairs Motor pole pairs
 * @param pwm_period The PWM Auto-Reload Register (ARR) value (e.g., 3600)
 */
void AlgFoc_Init(AlgFoc_Context_t *ctx, uint8_t pole_pairs, uint16_t pwm_period);

/**
 * @brief Set the Mechanical Zero Offset
 * @param raw_offset Sensor raw value (0-4095) at zero electrical angle
 */
void AlgFoc_SetZeroOffset(AlgFoc_Context_t *ctx, uint16_t raw_offset);

/**
 * @brief Run FOC Calculation (Inverse Park -> SVPWM)
 * @param raw_angle_12bit Current Sensor Raw Value (0-4095)
 * @param u_q Target Voltage Q-Axis (Torque). Q15 Format (-32768 to 32767)
 * @param u_d Target Voltage D-Axis (Flux). Normally 0. Q15 Format.
 * @note  Updates duty_a/_b/_c in context structure.
 */
void AlgFoc_Run(AlgFoc_Context_t *ctx, uint16_t raw_angle_12bit, int16_t u_q, int16_t u_d);

#endif // ALG_VOLTAGE_FOC_H