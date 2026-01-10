/**
 * @file app_scheduler.c
 * @brief Top-Level Application Scheduler Implementation
 * @author ARA Project Coder
 */

#include "app_scheduler.h"
#include "task_motion.h"
#include "bsp_gpio.h"
#include "bsp_uart.h" // For logging (if needed)

#include "FreeRTOS.h"
#include "task.h"

/* --- Configuration --- */
#define SCHED_LED_PERIOD_INIT    (500)  // ms
#define SCHED_LED_PERIOD_IDLE    (1000) // ms (Slow heartbeat)
#define SCHED_LED_PERIOD_RUN     (200)  // ms (Fast flash)
#define SCHED_LED_PERIOD_ERR     (50)   // ms (Panic strobe)

/* --- Private Context --- */
static struct {
    volatile SysState_t current_state;
    TaskHandle_t        task_handle;
} app_ctx;

/* --- Private Prototypes --- */
static void SystemErrorHandler(void);

/* --- API Implementation --- */

void App_Scheduler_Init(void)
{
    // 1. Initialize State
    app_ctx.current_state = SYS_STATE_INIT;

    // 2. Initialize Hardware (BSP Level)
    // Note: Assuming BSP_Init groups logic, or we call individuals here
    // GPIO for LED is crucial for status
    // (BSP_GPIO_Init logic usually inside main's MX_GPIO_Init,
    // but we assume mapped correctly in bsp_gpio.c)

    // 3. Initialize L4 Tasks (Business Logic)
    TaskMotion_Init();
    // TaskComm_Init(); // Future: Communication Task

    // 4. Create Scheduler Task (Self)
    // Priority: Low (Supervisory)
//    xTaskCreate(App_Scheduler_Entry, "Scheduler", 128, NULL, 1, &app_ctx.task_handle);
}

void App_Scheduler_Entry(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    uint32_t led_interval = SCHED_LED_PERIOD_INIT;
    uint32_t tick_acc = 0;

    /* Transition to IDLE/CALIB after boot */
    // In a real scenario, we might wait for a "Ready" signal from Motion Task
    App_Scheduler_SetState(SYS_STATE_CALIBRATION);

    for (;;) {
        // Dynamic Loop Period based on State
        switch (app_ctx.current_state) {
            case SYS_STATE_INIT:
                led_interval = SCHED_LED_PERIOD_INIT;
                break;
            case SYS_STATE_CALIBRATION:
                led_interval = SCHED_LED_PERIOD_INIT;
                // Monitor Motion Task?
                break;
            case SYS_STATE_IDLE:
                led_interval = SCHED_LED_PERIOD_IDLE;
                break;
            case SYS_STATE_RUNNING:
                led_interval = SCHED_LED_PERIOD_RUN;
                break;
            case SYS_STATE_ERROR:
                led_interval = SCHED_LED_PERIOD_ERR;
                break;
        }

        // Heartbeat LED Logic
        // Toggle LED every 'led_interval' ticks
        BSP_GPIO_Write(BSP_GPIO_LED_STATUS, (tick_acc % 2 == 0));
        tick_acc++;

        // Basic Health Monitor
        // If we are in Error state, ensure Motion is stopped (Redundant safety)
        if (app_ctx.current_state == SYS_STATE_ERROR) {
            TaskMotion_EmergencyStop();
        }

        // Wait for next cycle
        vTaskDelay(pdMS_TO_TICKS(led_interval));
    }
}

AraStatus_t App_Scheduler_SetState(SysState_t new_state)
{
    // 1. Check Transitions
    SysState_t old_state = app_ctx.current_state;

    // If already in ERROR, only allow Reset (Init)
    if (old_state == SYS_STATE_ERROR && new_state != SYS_STATE_INIT) {
        return ARA_ERROR; // Rejected
    }

    // 2. Execute Entry Actions
    switch (new_state) {
        case SYS_STATE_ERROR:
            SystemErrorHandler();
            break;

        case SYS_STATE_RUNNING:
            // Could enable specific high-performance logs here
            break;

        case SYS_STATE_IDLE:
            // Could set Target Velocity to 0 here
            TaskMotion_SetTargetVelocity(0);
            break;

        default:
            break;
    }

    // 3. Commit State
    app_ctx.current_state = new_state;

    return ARA_OK;
}

SysState_t App_Scheduler_GetState(void)
{
    return app_ctx.current_state;
}

/* --- Private Helper --- */

static void SystemErrorHandler(void)
{
    // 1. Trigger L4 Emergency Stop
    TaskMotion_EmergencyStop();

    // 2. Log error (if UART available)
    // BSP_UART_Printf("CRITICAL SYSTEM ERROR\r\n");
}