///**
// * @file test_console.c
// * @brief Hardware Acceptance Test Suite (Safe Mode)
// * @note  Designed for STM32F103 + SimpleFOC Mini + 2804 Gimbal Motor
// */
//
//#include "test_console.h"
//#include "bsp_uart.h"
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "task.h"
//#include "main.h"
//#include "drv_as5600.h"
//#include "drv_bldc_power.h"
//
///* ==========================================
// * SAFETY CONFIGURATION (DO NOT CHANGE)
// * ========================================== */
//// 2804 电机极对数 (通常 7 或 11，设错只影响转速显示，不影响转动)
//#define POLE_PAIRS       7
//
//// [安全电压阈值]
//// 12V 供电下:
//// 300  ≈ 2.5V (保守启动)
//// 500  ≈ 4.1V (有力，微温，安全上限)
//// 绝对不要超过 600！
//#define TEST_VOLTAGE     300
//
//#define CONSOLE_BUF_SIZE 64
//
///* --- Global Instances --- */
//static DrvAS5600_Context_t as5600_ctx;
//static DrvBldc_Context_t   bldc_ctx;
//static SemaphoreHandle_t   sem_as5600_done = NULL;
//
///* --- Private State --- */
//static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
//static bool    is_drv_init = false;
//
///* --- Optimized Sine LUT (Quarter Wave: 0~90 deg) --- */
//// Flash usage optimization. Maps 0~90 deg to 0~1000.
//static const int16_t sin_quarter_lut[65] = {
//        0,   24,   49,   73,   98,  122,  146,  171,
//        195,  219,  242,  266,  290,  313,  336,  359,
//        382,  405,  427,  449,  471,  492,  513,  534,
//        555,  575,  595,  615,  634,  653,  672,  690,
//        708,  726,  743,  760,  777,  793,  809,  824,
//        839,  853,  867,  881,  894,  907,  919,  931,
//        943,  954,  964,  974,  983,  991,  999, 1006,
//        1013, 1019, 1025, 1030, 1034, 1037, 1040, 1042,
//        1043
//};
//
///* --- Prototypes --- */
//static void AS5600_Callback_ISR(void);
//static int16_t FastSin(uint8_t angle_idx);
//static void Run_System_Diagnostic(void);
//static void Run_OpenLoop_Spin(void);
//static void PrintMenu(void);
//static void ExecuteCommand(char cmd);
//
///* ==========================================
// * Initialization
// * ========================================== */
//
//void TestConsole_Init(void) {
//    BSP_UART_Printf("\r\n=== ARA PLATFORM: HARDWARE ACCEPTANCE TEST ===\r\n");
//    BSP_UART_Printf("Safe Voltage Limit: %d/1440\r\n", TEST_VOLTAGE);
//
//    sem_as5600_done = xSemaphoreCreateBinary();
//
//    // Init Drivers
//    AraStatus_t as_status = DrvAS5600_Init(&as5600_ctx, BSP_I2C_MOTION, AS5600_Callback_ISR);
//    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);
//
//    if (as_status == ARA_OK) {
//        BSP_UART_Printf("[Init] Hardware OK. Ready for test.\r\n");
//        is_drv_init = true;
//    } else {
//        BSP_UART_Printf("[Init] FATAL: AS5600 Not Found (Err: %d)\r\n", as_status);
//        BSP_UART_Printf("Check I2C Wiring or Pull-ups.\r\n");
//    }
//
//    PrintMenu();
//}
//
//void TestConsole_TaskLoop(void) {
//    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
//    if (len > 0) {
//        for (uint16_t i = 0; i < len; i++) {
//            ExecuteCommand((char)rx_buffer[i]);
//        }
//    }
//    vTaskDelay(pdMS_TO_TICKS(20));
//}
//
///* ==========================================
// * Helper Functions
// * ========================================== */
//
//// Efficient Sine Lookup with symmetry
//static int16_t FastSin(uint8_t angle) {
//    uint8_t idx = angle;
//    int16_t val;
//    if (idx <= 64)       val = sin_quarter_lut[idx];
//    else if (idx <= 128) val = sin_quarter_lut[128 - idx];
//    else if (idx <= 192) val = -sin_quarter_lut[idx - 128];
//    else                 val = -sin_quarter_lut[256 - idx];
//
//    // Clamp to +/- 1000
//    if (val > 1000) return 1000;
//    if (val < -1000) return -1000;
//    return val;
//}
//
//// ISR Callback for Async I2C
//static void AS5600_Callback_ISR(void) {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if (sem_as5600_done != NULL) {
//        xSemaphoreGiveFromISR(sem_as5600_done, &xHigherPriorityTaskWoken);
//    }
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
///* ==========================================
// * TEST ROUTINE 1: System Diagnostic
// * ========================================== */
//static void Run_System_Diagnostic(void) {
//    BSP_UART_Printf("\r\n--- Starting Auto-Diagnostic ---\r\n");
//
//    // Step 1: Check AS5600
//    BSP_UART_Printf("[1/3] Checking Sensor... ");
//    DrvAS5600_TriggerUpdate(&as5600_ctx);
//    if (xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(20)) == pdTRUE) {
//        uint16_t raw = DrvAS5600_GetRawAngle(&as5600_ctx);
//        BSP_UART_Printf("PASS (Raw: %d)\r\n", raw);
//    } else {
//        BSP_UART_Printf("FAIL (Timeout)\r\n");
//        return;
//    }
//
//    // Step 2: Check Enable Pin Logic
//    BSP_UART_Printf("[2/3] Checking Driver Enable... ");
//    DrvBldc_Enable(&bldc_ctx, true);
//    HAL_Delay(50);
//    // User needs to verify voltage here usually, but we assume software logic works
//    BSP_UART_Printf("ENABLED (Measure EN pin if unsure)\r\n");
//
//    // Step 3: Short Phase Pulse (Check Connection)
//    BSP_UART_Printf("[3/3] Pulse Test (Listen for 'Tick')... \r\n");
//    // Pulse U phase very briefly
//    DrvBldc_SetDuties(&bldc_ctx, TEST_VOLTAGE, 0, 0);
//    HAL_Delay(100);
//    DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0); // Release
//    BSP_UART_Printf("      Pulse Complete.\r\n");
//
//    DrvBldc_Enable(&bldc_ctx, false);
//    BSP_UART_Printf("--- Diagnostic Done. Ready for Spin. ---\r\n> ");
//}
//
///* ==========================================
// * TEST ROUTINE 2: Open Loop Spin
// * ========================================== */
//static void Run_OpenLoop_Spin(void) {
//    BSP_UART_Printf("\r\n[SPIN] Motor Running at ~2 rad/s.\r\n");
//    BSP_UART_Printf("[INFO] Voltage Limit: %d (Safe)\r\n", TEST_VOLTAGE);
//    BSP_UART_Printf("Press ANY KEY to Stop.\r\n");
//
//    DrvBldc_Enable(&bldc_ctx, true);
//
//    uint32_t tick_now = HAL_GetTick();
//    float target_velocity = 3.0f; // Slow, visible rotation
//    float electrical_angle = 0.0f;
//
//    // Coeff = 256 / 2PI * dt(0.001) ≈ 0.04074
//    float angle_step_per_ms = target_velocity * POLE_PAIRS * 0.04074f;
//
//    for(;;) {
//        // Exit check
//        if (BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, 1) > 0) break;
//
//        // Integration
//        uint32_t new_tick = HAL_GetTick();
//        uint32_t dt_ms = new_tick - tick_now;
//
//        if (dt_ms > 0) {
//            tick_now = new_tick;
//            electrical_angle += angle_step_per_ms * dt_ms;
//            while (electrical_angle >= 256.0f) electrical_angle -= 256.0f;
//            while (electrical_angle < 0.0f)    electrical_angle += 256.0f;
//        }
//
//        // SPWM Generation
//        uint8_t theta = (uint8_t)electrical_angle;
//        int16_t duty_u = 720 + (FastSin(theta) * TEST_VOLTAGE / 1000);
//        int16_t duty_v = 720 + (FastSin((uint8_t)(theta + 85)) * TEST_VOLTAGE / 1000);
//        int16_t duty_w = 720 + (FastSin((uint8_t)(theta + 171)) * TEST_VOLTAGE / 1000);
//
//        DrvBldc_SetDuties(&bldc_ctx, duty_u, duty_v, duty_w);
//
//        // Monitor
//        static uint32_t print_ts = 0;
//        if (HAL_GetTick() - print_ts > 200) {
//            print_ts = HAL_GetTick();
//            // Async Read
//            DrvAS5600_TriggerUpdate(&as5600_ctx);
//            // Non-blocking wait check (if failed, use old value)
//            if(xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(5)) == pdTRUE) {
//                uint16_t raw = DrvAS5600_GetRawAngle(&as5600_ctx);
//                BSP_UART_Printf("Raw: %04d | Duty: %d/%d/%d\r\n", raw, duty_u, duty_v, duty_w);
//            }
//        }
//        HAL_Delay(2);
//    }
//
//    // Safe Stop
//    DrvBldc_Enable(&bldc_ctx, false);
//    BSP_PWM_StopAll();
//    BSP_UART_Printf("\r\n[SPIN] Stopped.\r\n> ");
//}
//
///* ==========================================
// * UI Logic
// * ========================================== */
//
//static void PrintMenu(void) {
//    BSP_UART_Printf("\r\n--- Acceptance Test Menu ---\r\n");
//    BSP_UART_Printf("[d] Run System Diagnostic (Static)\r\n");
//    BSP_UART_Printf("[o] Run Open-Loop Spin (Motion)\r\n");
//    BSP_UART_Printf("----------------------------\r\n> ");
//}
//
//static void ExecuteCommand(char cmd) {
//    if (!is_drv_init) {
//        BSP_UART_Printf("Hardware Init Failed.\r\n> ");
//        return;
//    }
//
//    // 使用刚才测试的电压值 (600)
//    uint16_t lock_volts = TEST_VOLTAGE;
//
//    switch (cmd) {
//        // [原有功能]
//        case 'o':
//        case 'O':
//            Run_OpenLoop_Spin();
//            break;
//
//            // [新增] 静态相位诊断 (Static Phase Diagnostic)
//            // 手动激活某一相，检查是否有磁力锁死
//
//        case '1': // 测试 U 相
//            DrvBldc_Enable(&bldc_ctx, true);
//            DrvBldc_SetDuties(&bldc_ctx, lock_volts, 0, 0);
//            HAL_Delay(10); // 等待电流稳定
//            DrvAS5600_TriggerUpdate(&as5600_ctx); HAL_Delay(2);
//            BSP_UART_Printf("[TEST U] U=%d, V=0, W=0 | Raw: %d\r\n> ",
//                            lock_volts, DrvAS5600_GetRawAngle(&as5600_ctx));
//            break;
//
//        case '2': // 测试 V 相
//            DrvBldc_Enable(&bldc_ctx, true);
//            DrvBldc_SetDuties(&bldc_ctx, 0, lock_volts, 0);
//            HAL_Delay(10);
//            DrvAS5600_TriggerUpdate(&as5600_ctx); HAL_Delay(2);
//            BSP_UART_Printf("[TEST V] U=0, V=%d, W=0 | Raw: %d\r\n> ",
//                            lock_volts, DrvAS5600_GetRawAngle(&as5600_ctx));
//            break;
//
//        case '3': // 测试 W 相
//            DrvBldc_Enable(&bldc_ctx, true);
//            DrvBldc_SetDuties(&bldc_ctx, 0, 0, lock_volts);
//            HAL_Delay(10);
//            DrvAS5600_TriggerUpdate(&as5600_ctx); HAL_Delay(2);
//            BSP_UART_Printf("[TEST W] U=0, V=0, W=%d | Raw: %d\r\n> ",
//                            lock_volts, DrvAS5600_GetRawAngle(&as5600_ctx));
//            break;
//
//        case 's': // 释放
//            DrvBldc_Enable(&bldc_ctx, false);
//            BSP_PWM_StopAll();
//            BSP_UART_Printf("[STOP] Released.\r\n> ");
//            break;
//
//        case 'h':
//        case '?':
//            PrintMenu();
//            break;
//
//        case '\r': case '\n': break;
//        default: BSP_UART_Printf("Unknown: %c\r\n> ", cmd); break;
//    }
//}