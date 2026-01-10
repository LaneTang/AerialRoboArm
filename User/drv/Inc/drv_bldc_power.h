/**
 * @file drv_bldc_power.h
 * @brief BLDC Power Stage Driver (L2)
 * @note  Manages 3-Phase PWM and Enable Pin. Integer Math Only.
 */

#ifndef DRV_BLDC_POWER_H
#define DRV_BLDC_POWER_H

#include "ara_def.h"
#include "bsp_pwm.h"
#include "bsp_gpio.h"

/* --- Context --- */
typedef struct {
    BspGpio_Pin_t en_pin;       // Enable Pin (e.g., BSP_GPIO_MOTOR_EN)
    bool          is_enabled;   // Logic State
    uint16_t      max_duty;     // Copy of BSP_PWM_MAX_DUTY for clamping
} DrvBldc_Context_t;

/* --- API --- */

/**
 * @brief Initialize Power Stage
 * @param p_ctx  Driver Context
 * @param en_pin GPIO Pin ID for Enable signal
 */
void DrvBldc_Init(DrvBldc_Context_t *p_ctx, BspGpio_Pin_t en_pin);

/**
 * @brief Enable/Disable Motor Driver
 * @note  Controls the Hardware EN pin directly
 */
void DrvBldc_Enable(DrvBldc_Context_t *p_ctx, bool state);

/**
 * @brief Apply 3-Phase Duty Cycles (Integer)
 * * @param u, v, w: Duty Cycle values (0 to BSP_PWM_MAX_DUTY)
 * @return ARA_OK if successful
 * @note   Returns ARA_ERR_PARAM and triggers Emergency Stop if values > max_duty.
 */
AraStatus_t DrvBldc_SetDuties(DrvBldc_Context_t *p_ctx, uint16_t u, uint16_t v, uint16_t w);

#endif // DRV_BLDC_POWER_H