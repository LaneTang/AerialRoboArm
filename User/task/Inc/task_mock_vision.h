/**
 * @file task_mock_vision.h
 * @brief Vision Mock Gateway (L4) - Downgraded to Runnable
 * @note  Injects deterministic test vectors into the system for Desktop Testing.
 */

#ifndef TASK_MOCK_VISION_H
#define TASK_MOCK_VISION_H

#include "ara_def.h"
#include "mod_vsp_parser.h" /* For AraVisionData_t */

/* --- Mock Scenarios --- */
typedef enum {
    MOCK_SCENARIO_IDLE = 0,
    MOCK_SCENARIO_HAPPY_PATH,       // Track -> Align -> Grab -> Retract
    MOCK_SCENARIO_TARGET_LOST,
    MOCK_SCENARIO_WIFI_JITTER,
    MOCK_SCENARIO_PC_ESTOP          // Force Emergency Stop
} MockScenario_t;

/* --- Logic Context --- */
typedef struct {
    MockScenario_t  current_scenario;
    uint32_t        scenario_start_tick;
    uint32_t        step_counter;
} TaskMockVision_Context_t;

/* --- API --- */

/**
 * @brief Initialize Mock Scenario runner.
 */
void TaskMockVision_Init(void);

/**
 * @brief Change the current test scenario (Thread-safe, can be called via CLI).
 */
AraStatus_t TaskMockVision_SetScenario(MockScenario_t new_scenario);

/**
 * @brief The 50Hz Execution Step.
 * @note  Generates fake vision coordinates based on the active scenario script.
 * * @param current_tick_ms [IN]  Current system time
 * @param p_out_vision    [OUT] The generated fake vision frame (passed to Manipulator)
 */
void TaskMockVision_Update(uint32_t current_tick_ms, AraVisionData_t *p_out_vision);

#endif /* TASK_MOCK_VISION_H */