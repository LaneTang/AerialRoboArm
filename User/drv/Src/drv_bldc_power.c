/**
 * @file drv_bldc_power.c
 * @brief BLDC Power Stage Driver Implementation (Integer & GPIO Aware)
 * @author ARA Project Coder
 */

#include "drv_bldc_power.h"

/* --- API Implementation --- */

void DrvBldc_Init(DrvBldc_Context_t *p_ctx, BspGpio_Pin_t en_pin)
{
    if (p_ctx == NULL) {
        return;
    }

    // 1. Setup Context
    p_ctx->en_pin = en_pin;
    p_ctx->max_duty = BSP_PWM_MAX_DUTY; // Import constant from BSP
    p_ctx->is_enabled = false;

    // 2. Force Hardware into Safe State immediately
    // Disable the Driver Chip (Pin 6 on SimpleFOC Mini)
    BSP_GPIO_Write(p_ctx->en_pin, false);

    // Stop PWM generation (Hi-Z or Low)
    BSP_PWM_StopAll();
}

void DrvBldc_Enable(DrvBldc_Context_t *p_ctx, bool state)
{
    if (p_ctx == NULL) {
        return;
    }

    p_ctx->is_enabled = state;

    if (state) {
        // Enable Sequence:
        // 1. Ensure PWMs are zero first to prevent "jump"
        BSP_PWM_Set3PhaseDuty_U16(0, 0, 0);
        // 2. Enable Driver Chip
        BSP_GPIO_Write(p_ctx->en_pin, true);
    } else {
        // Disable Sequence (Safety First):
        // 1. Disable Driver Chip (Hardware Cutoff)
        BSP_GPIO_Write(p_ctx->en_pin, false);
        // 2. Stop PWM Timer outputs
        BSP_PWM_StopAll();
    }
}

AraStatus_t DrvBldc_SetDuties(DrvBldc_Context_t *p_ctx, uint16_t u, uint16_t v, uint16_t w)
{
    if (p_ctx == NULL) {
        return ARA_ERR_PARAM;
    }

    // 1. Safety Gate: Do not write to registers if logically disabled
    if (!p_ctx->is_enabled) {
        // Return Generic Error to signal L3 that command was ignored
        return ARA_ERROR;
    }

    // 2. Sanity Check / Over-modulation Protection
    // If L3 calculates values exceeding hardware limits, something is wrong.
    if ((u > p_ctx->max_duty) || (v > p_ctx->max_duty) || (w > p_ctx->max_duty)) {

        // CRITICAL FAULT: L3 Algorithm is unstable or outputting garbage.
        // Action: Emergency Stop immediately.
        DrvBldc_Enable(p_ctx, false);

        return ARA_ERR_PARAM;
    }

    // 3. Apply to Hardware (Direct Register Access via BSP)
    BSP_PWM_Set3PhaseDuty_U16(u, v, w);

    return ARA_OK;
}