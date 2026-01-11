//
// Created by tanga on 2026/1/11.
//

#include "test_console.h"
#include "bsp_gpio.h"

#include "FreeRTOS.h"
#include "task.h"

void TestConsole_TaskLoop()
{
    TickType_t init_time = xTaskGetTickCount();

//    BSP_GPIO_Write(1, 0);

    for (;;) {

        BSP_UART_Printf("run time: %d", xTaskGetTickCount() - init_time);
        vTaskDelay(100);
    }
};
