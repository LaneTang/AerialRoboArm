///**
// * @file test_console.c
// * @brief System Test Console (Integrated with L2 Drivers)
// */
//
//#include "test_console.h"
//#include "bsp_uart.h"
//#include "FreeRTOS.h"
//#include "semphr.h"
//#include "task.h"
//
///* --- Driver Includes --- */
//#include "drv_as5600.h"
//#include "drv_bldc_power.h"
//
///* --- Configuration --- */
//#define CONSOLE_BUF_SIZE 64
//
///* --- Global Driver Instances (L2) --- */
//// 这些通常定义在 L3/L4 层，但在调试阶段我们把它们放在这里
//static DrvAS5600_Context_t as5600_ctx;
//static DrvBldc_Context_t   bldc_ctx;
//
///* --- Synchronization Objects --- */
//static SemaphoreHandle_t   sem_as5600_done = NULL;
//
///* --- Private State --- */
//static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
//static bool    is_drv_init = false;
//
///* --- Forward Declarations --- */
//static void PrintMenu(void);
//static void ExecuteCommand_DrvTest(char cmd);
//static void AS5600_Callback_ISR(void); // 回调函数
//
///* --- Init --- */
//void TestConsole_Init(void) {
//    BSP_UART_Printf("\r\n=== ARA PLATFORM: DRIVER TEST MODE ===\r\n");
//
//    // 1. 初始化信号量
//    sem_as5600_done = xSemaphoreCreateBinary();
//
//    // 2. 初始化 AS5600 驱动 (L2)
//    // 注意：我们将 AS5600_Callback_ISR 注册给驱动，DMA 完成后会自动调用它
//    AraStatus_t as_status = DrvAS5600_Init(&as5600_ctx, BSP_I2C_MOTION, AS5600_Callback_ISR);
//
//    // 3. 初始化 BLDC 驱动 (L2)
//    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);
//
//    if (as_status == ARA_OK) {
//        BSP_UART_Printf("[Init] Drivers Initialized OK.\r\n");
//        is_drv_init = true;
//    } else {
//        BSP_UART_Printf("[Init] AS5600 Driver Init FAILED: %d\r\n", as_status);
//    }
//
//    PrintMenu();
//}
//
///* --- Main Task Loop --- */
//void TestConsole_TaskLoop(void) {
//    // 读取 UART 输入
//    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
//
//    if (len > 0) {
//        for (uint16_t i = 0; i < len; i++) {
//            ExecuteCommand_DrvTest((char)rx_buffer[i]);
//        }
//    }
//
//    vTaskDelay(pdMS_TO_TICKS(20));
//}
//
///* --- Callback Implementation --- */
///**
// * @brief AS5600 DMA 完成回调
// * @note  这个函数在 ISR 上下文中被调用，必须快进快出
// */
//static void AS5600_Callback_ISR(void) {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if (sem_as5600_done != NULL) {
//        xSemaphoreGiveFromISR(sem_as5600_done, &xHigherPriorityTaskWoken);
//    }
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
///* --- Command Logic --- */
//static void PrintMenu(void) {
//    BSP_UART_Printf("\r\n--- L2 Driver Test Menu ---\r\n");
//    BSP_UART_Printf("[a] AS5600: Trigger DMA Read (Async)\r\n");
//    BSP_UART_Printf("[m] BLDC: Enable Motor\r\n");
//    BSP_UART_Printf("[n] BLDC: Disable Motor\r\n");
//    BSP_UART_Printf("[v] BLDC: Set Duty (U=300, V=600, W=900)\r\n");
//    BSP_UART_Printf("[s] BLDC: STOP (All Zero)\r\n");
//    BSP_UART_Printf("---------------------------\r\n> ");
//}
//
//static void ExecuteCommand_DrvTest(char cmd) {
//    if (!is_drv_init) {
//        BSP_UART_Printf("Drivers not initialized!\r\n> ");
//        return;
//    }
//
//    switch (cmd) {
//        // --- AS5600 测试 (异步读取) ---
//        case 'a':
//        case 'A': {
//            // 1. 触发读取 (立即返回，不阻塞)
//            AraStatus_t ret = DrvAS5600_TriggerUpdate(&as5600_ctx);
//            if (ret != ARA_OK) {
//                BSP_UART_Printf("[AS5600] Trigger Failed: %d\r\n> ", ret);
//                break;
//            }
//
//            // 2. 等待回调信号量 (模拟 L4 任务的等待行为)
//            // 超时设为 10ms，足够 I2C 传输完成
//            if (xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(10)) == pdTRUE) {
//                // 3. 读取结果
//                uint16_t raw = DrvAS5600_GetRawAngle(&as5600_ctx);
//
//                // 定点数打印角度
//                uint32_t deg_x100 = (raw * 36000) / 4096;
//                BSP_UART_Printf("[AS5600] Raw: %04d | Deg: %d.%02d\r\n> ",
//                                raw, deg_x100 / 100, deg_x100 % 100);
//            } else {
//                BSP_UART_Printf("[AS5600] Timeout! Callback did not fire.\r\n> ");
//            }
//            BSP_UART_Printf("???");
//            break;
//        }
//
//            // --- BLDC 电源管理测试 ---
//        case 'm': // Enable
//            DrvBldc_Enable(&bldc_ctx, true);
//            BSP_UART_Printf("[BLDC] ENABLED.\r\n> ");
//            break;
//
//        case 'n': // Disable
//            DrvBldc_Enable(&bldc_ctx, false);
//            BSP_UART_Printf("[BLDC] DISABLED.\r\n> ");
//            break;
//
//        case 'v': // Set Dummy Duty
//            // 设置一个安全的低占空比进行测试
//            if (DrvBldc_SetDuties(&bldc_ctx, 300, 600, 900) == ARA_OK) {
//                BSP_UART_Printf("[BLDC] Duty Set: 300/600/900\r\n> ");
//            } else {
//                BSP_UART_Printf("[BLDC] Error setting duty (Check Max Duty)\r\n> ");
//            }
//            break;
//
//        case 's': // Stop
//            // 设为 0 但保持 Enable 状态 (Soft Stop)
//            DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
//            BSP_UART_Printf("[BLDC] Output Zeroed.\r\n> ");
//            break;
//
//            // --- 辅助命令 ---
//        case 'h':
//        case '?':
//            PrintMenu();
//            break;
//
//        case '\r':
//        case '\n':
//            break;
//
//        default:
//            BSP_UART_Printf("Unknown: %c\r\n> ", cmd);
//            break;
//    }
//}