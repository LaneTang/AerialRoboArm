/**
 * @file app_threads.c
 * @brief Top-Level Physical Thread Container & Scheduler (L5) Implementation
 * @note  Strictly C99. Assembles L4 Runnables into FreeRTOS physical threads.
 * @author ARA Project Coder
 */

#include "app_threads.h"
#include "datahub.h"

/* L4 Business Logic Runnables */
#include "task_motion.h"
#include "task_manipulator.h"
#include "task_rc.h"
#include "task_mock_vision.h"

/* FreeRTOS Dependencies */
#include "FreeRTOS.h"
#include "task.h"

/* BSP / Hardware for Watchdog/Status LED */
#include "dev_status.h"

/* =========================================================
 * 1. Thread Handles
 * ========================================================= */

static TaskHandle_t s_high_freq_task_handle = NULL;
static TaskHandle_t s_low_freq_task_handle  = NULL;

/* =========================================================
 * 2. Physical Thread Entry Functions (Private)
 * ========================================================= */

/**
 * @brief High-Frequency Control Thread (1000Hz)
 * @note  Priority: Realtime. Pure FOC calculation pipeline. NO BLOCKING.
 */
static void Thread_HighFreq_Ctrl(void *argument)
{
    /* Absolute frequency control */
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(THREAD_HIGH_FREQ_PERIOD_MS);

    /* Local buffers for Lock-Free DataHub exchange */
    DataHub_Cmd_t   in_cmd;
    DataHub_State_t out_state;

    for (;;)
    {
        /* 1. Pull latest command from Low-Freq Brain (Lock-Free) */
        DataHub_ReadCmd(&in_cmd);

        /* 2. Execute Motion Pipeline (Sensor DMA -> PID -> FOC -> PWM) */
        /* Takes < 100 microseconds to complete */
        TaskMotion_Update(&in_cmd, &out_state);

        /* 3. Push current hardware state back to Hub (Lock-Free) */
        DataHub_WriteState(&out_state);

        /* 4. Sleep exactly until the next 1ms tick */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/**
 * @brief Low-Frequency Logic Thread (50Hz)
 * @note  Priority: Normal. The "Brain". Gathers RC/Vision -> Decides -> Commands.
 */
static void Thread_LowFreq_Logic(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(THREAD_LOW_FREQ_PERIOD_MS);

    /* Local stack structures for safe data pipelining */
    RcControlData_t rc_intent;
    AraVisionData_t vision_data;
    DataHub_Cmd_t   out_cmd;

    uint32_t led_tick_counter = 0;

    for (;;)
    {
        uint32_t current_tick_ms = (uint32_t)xTaskGetTickCount();

        /* ====================================================
         * STAGE 1: Perception (Gather Inputs)
         * ==================================================== */

        /* 1A. Fetch RC Semantic Intent */
        TaskRc_Update(current_tick_ms, &rc_intent);

        /* 1B. Fetch Vision Data */
#if ENABLE_MOCK_VISION
        TaskMockVision_Update(current_tick_ms, &vision_data);
#else
        /* Placeholder for real UART VSP logic */
        // TaskVision_Update(current_tick_ms, &vision_data); 
#endif

        /* ====================================================
         * STAGE 2: Decision (State Machine)
         * ==================================================== */

        /* Generate target FOC angle & E-Stop flag based on inputs */
        TaskManipulator_Update(&rc_intent, &vision_data, &out_cmd);

        /* ====================================================
         * STAGE 3: Action & Comm (Push to Hub)
         * ==================================================== */

        /* Send computed commands to the 1000Hz Thread (Lock-Free) */
        DataHub_WriteCmd(&out_cmd);

        /* ====================================================
         * STAGE 4: System Watchdog & Status LED
         * ==================================================== */
        led_tick_counter++;
        if (out_cmd.emergency_stop || out_cmd.sys_mode == ARA_MODE_ERROR) {
            /* Error: Fast blink (Approx 100ms period -> toggle every 2 ticks) */
            if (led_tick_counter % 2 == 0) DEV_Status_LED_ToggleLed();
        } else if (out_cmd.sys_mode == ARA_MODE_AUTO) {
            /* Auto Mode: Solid heartbeat (Approx 400ms period -> toggle every 10 ticks) */
            if (led_tick_counter % 10 == 0) DEV_Status_LED_ToggleLed();
        } else {
            /* Idle/Manual Mode: Slow blink (Approx 2000ms period -> toggle every 50 ticks) */
            if (led_tick_counter % 50 == 0) DEV_Status_LED_ToggleLed();
        }

        /* ====================================================
         * STAGE 5: Strict 50Hz Sleep
         * ==================================================== */
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* =========================================================
 * 3. Public API Implementation
 * ========================================================= */

void App_Threads_Init(void)
{
    /* 1. Initialize Global Data Blackboard First */
    DataHub_Init();

    /* 2. Initialize L4 Business Modules */
    TaskMotion_Init();
    TaskRc_Init();
    TaskManipulator_Init();

#if ENABLE_MOCK_VISION
    TaskMockVision_Init();
    /* Set default test script for desktop testing */
    TaskMockVision_SetScenario(MOCK_SCENARIO_HAPPY_PATH);
#else
    // TaskVision_Init();
#endif

    /* 3. Mount Physical Threads to FreeRTOS Scheduler */
    /* Note: configMAX_PRIORITIES is typically 7 in CubeMX. 
     * Highest numeric value = highest logical priority in FreeRTOS. */

    xTaskCreate(Thread_HighFreq_Ctrl,
                "HighFreqCtrl",
                THREAD_HIGH_FREQ_STACK,
                NULL,
                configMAX_PRIORITIES - 1, /* Realtime Priority */
                &s_high_freq_task_handle);

    xTaskCreate(Thread_LowFreq_Logic,
                "LowFreqLogic",
                THREAD_LOW_FREQ_STACK,
                NULL,
                configMAX_PRIORITIES - 3, /* Normal/Above-Normal Priority */
                &s_low_freq_task_handle);
}