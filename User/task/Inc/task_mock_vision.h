/**
 * @file task_mock_vision.h
 * @brief Vision Mock Gateway Task (L4) for Agile Testing
 * @note  Replaces task_vision.h during development. Injects deterministic
 * test vectors into the DataHub to verify Task_Manipulator logic
 * without needing the actual PC/Camera hardware.
 */

#ifndef TASK_MOCK_VISION_H
#define TASK_MOCK_VISION_H

#include "ara_def.h"
#include "datahub_vision_ext.h" /* To inject AraVisionData_t */

/* =========================================================
 * 1. Mock Test Scenarios
 * ========================================================= */

/**
 * @brief Pre-defined testing scenarios (Test Vectors)
 */
typedef enum {
    MOCK_SCENARIO_IDLE = 0,         // Sends nothing (tests timeout/watchdog)

    MOCK_SCENARIO_HAPPY_PATH,       // Standard successful grab:
    // Track -> Align -> Reach Deadband -> Wait -> Grab -> Retract

    MOCK_SCENARIO_TARGET_LOST,      // Target appears, then disappears midway (tests Fallback logic)

    MOCK_SCENARIO_WIFI_JITTER,      // Simulates network latency (sends frames randomly between 20ms-400ms)

    MOCK_SCENARIO_PC_ESTOP,         // Triggers the PC Emergency Stop flag suddenly

    MOCK_SCENARIO_NUM
} MockScenario_t;

/* =========================================================
 * 2. Task Context Structure
 * ========================================================= */

/**
 * @brief Mock Task Context
 */
typedef struct {
    MockScenario_t  current_scenario;
    uint32_t        scenario_start_tick;
    uint32_t        step_counter;       // Internal state for the current test script

    AraVisionData_t mock_payload;       // The dummy data to be injected
} TaskMockVision_Context_t;

/* =========================================================
 * 3. Module API
 * ========================================================= */

/**
 * @brief Initialize the Mock Vision Task
 * @note  Replaces TaskVision_Init() in main.c when ENABLE_MOCK_VISION is defined.
 */
void TaskMockVision_Init(void);

/**
 * @brief Main Mock Task Entry Point (FreeRTOS Task)
 * @param argument Task parameters
 * @note  Runs at 20Hz (50ms). Generates mock AraVisionData_t based on the
 * current scenario script and pushes it via DataHub_WriteVisionData().
 */
void TaskMockVision_Entry(void *argument);

/**
 * @brief Dynamically switch the active test scenario
 * @note  Thread-safe. Can be triggered via a Debug UART CLI command or
 * by mapping an RC transmitter switch (e.g., CH8 Aux Knob) to this function.
 * @param new_scenario The scenario to execute
 * @return ARA_OK on success
 */
AraStatus_t TaskMockVision_SetScenario(MockScenario_t new_scenario);

#endif /* TASK_MOCK_VISION_H */