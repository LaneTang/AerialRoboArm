/**
 * @file task_mock_vision.c
 * @brief Vision Mock Gateway (L4) Implementation
 * @note  Strictly C99. Zero RTOS dependencies.
 * Generates deterministic ARA-VSP test vectors based on time sequences.
 * @author ARA Project Coder
 */

#include "task_mock_vision.h"
#include <string.h>

/* =========================================================
 * 1. Internal Context Allocation
 * ========================================================= */

static TaskMockVision_Context_t s_mock_ctx;

/* =========================================================
 * 2. API Implementation
 * ========================================================= */

void TaskMockVision_Init(void)
{
    memset(&s_mock_ctx, 0, sizeof(TaskMockVision_Context_t));
    s_mock_ctx.current_scenario = MOCK_SCENARIO_IDLE;
}

AraStatus_t TaskMockVision_SetScenario(MockScenario_t new_scenario)
{
    /* In a bare-metal/Runnables context without OS mutexes, we rely on the caller
     * (e.g., a CLI interrupt or debug watch) to not cause tearing.
     * 32-bit enum assignment is atomic on Cortex-M3. */
    s_mock_ctx.current_scenario = new_scenario;
    s_mock_ctx.scenario_start_tick = 0; // Will be lazily initialized on next update
    s_mock_ctx.step_counter = 0;

    return ARA_OK;
}

void TaskMockVision_Update(uint32_t current_tick_ms, AraVisionData_t *p_out_vision)
{
    if (p_out_vision == NULL) {
        return;
    }

    /* Initialize start tick on the first run of a new scenario */
    if (s_mock_ctx.scenario_start_tick == 0) {
        s_mock_ctx.scenario_start_tick = current_tick_ms;
    }

    uint32_t elapsed_ms = current_tick_ms - s_mock_ctx.scenario_start_tick;

    /* 1. Default Safe Frame Output */
    memset(p_out_vision, 0, sizeof(AraVisionData_t));
    p_out_vision->seq_id = (uint8_t)(s_mock_ctx.step_counter++ % 256);
    p_out_vision->is_tracking = false;
    p_out_vision->is_grabbable = false;
    p_out_vision->pc_estop_req = false;
    p_out_vision->target_dist_mm = 0;
    p_out_vision->target_roll = 0;

    /* 2. Execute Time-Based Scenario Script */
    switch (s_mock_ctx.current_scenario)
    {
        case MOCK_SCENARIO_IDLE:
            /* Outputs the default empty/safe frame */
            break;

        case MOCK_SCENARIO_HAPPY_PATH:
            p_out_vision->is_tracking = true;

            if (elapsed_ms < 1000) {
                /* Stage 1 (0-1s): Target found far away */
                p_out_vision->target_dist_mm = 280;
                p_out_vision->target_roll = 1500; // 15.00 degrees
            }
            else if (elapsed_ms < 3000) {
                /* Stage 2 (1s-3s): Smooth linear approach */
                /* Moving from 280mm -> 200mm, and 15deg -> 0deg over 2000ms */
                uint32_t phase_time = elapsed_ms - 1000;
                p_out_vision->target_dist_mm = 280 - (80 * phase_time) / 2000;
                p_out_vision->target_roll = 1500 - (1500 * phase_time) / 2000;

                /* Trigger grabbable zone when close enough */
                if (p_out_vision->target_dist_mm <= 210) {
                    p_out_vision->is_grabbable = true;
                }
            }
            else {
                /* Stage 3 (>3s): Steady in grabbable zone, waiting for grab */
                p_out_vision->target_dist_mm = 200;
                p_out_vision->target_roll = 0;
                p_out_vision->is_grabbable = true;
            }
            break;

        case MOCK_SCENARIO_TARGET_LOST:
            if (elapsed_ms < 1500) {
                p_out_vision->is_tracking = true;
                p_out_vision->target_dist_mm = 250;
            } else {
                /* Suddenly drop tracking after 1.5 seconds */
                p_out_vision->is_tracking = false;
            }
            break;

        case MOCK_SCENARIO_WIFI_JITTER:
            /* Simulate terrible network/vision dropouts */
            /* Toggle tracking visibility every 200ms */
            if ((elapsed_ms / 200) % 2 == 0) {
                p_out_vision->is_tracking = true;
                p_out_vision->target_dist_mm = 220;
            } else {
                p_out_vision->is_tracking = false;
            }
            break;

        case MOCK_SCENARIO_PC_ESTOP:
            p_out_vision->pc_estop_req = true;
            break;

        default:
            break;
    }
}