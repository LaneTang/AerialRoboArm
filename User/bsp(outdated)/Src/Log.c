//
// Created by User on 2025/11/20.
//

#include "Log.h"

#include <stdio.h>
#include <stdarg.h>
#include <string.h>
#include "usart.h"
#include "FreeRTOS.h"
#include "semphr.h" // 引入信号量/互斥量头文件

// 1. 定义一个互斥量句柄
SemaphoreHandle_t xLogMutex = NULL;

// 2. 初始化互斥量（通常在 main 或 freertos 初始化任务中调用）
void Log_Init(void) {
    if (xLogMutex == NULL) {
        xLogMutex = xSemaphoreCreateMutex();
    }
}

// 3. 线程安全的打印函数
void App_Printf(const char *format, ...) {
    char buffer[128]; // 根据需要调整缓冲区大小
    va_list args;

    // 如果互斥量还没初始化，直接返回或由你决定怎么处理
    if (xLogMutex == NULL) return;

    // 获取互斥量，等待时间为最大（也可以设为 10ms 等）
    // 作用：只有拿到锁的任务才能使用串口，其他任务必须排队
    if (xSemaphoreTake(xLogMutex, portMAX_DELAY) == pdTRUE) {

        // 格式化字符串
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);

        // 通过 HAL 库发送 (阻塞模式发送，简单可靠)
        HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), 0xFFFF);

        // 释放互斥量
        xSemaphoreGive(xLogMutex);
    }
}