/**
 * @file app_threads.h
 * @brief Top-Level Physical Thread Container & Scheduler (L5)
 */

#ifndef APP_THREADS_H
#define APP_THREADS_H

#include "ara_def.h"

/* Enable Mock Vision for desktop agile testing (1 = Mock, 0 = Real UART) */
#define ENABLE_MOCK_VISION          (1)

/* Thread Physical Parameters */
#define THREAD_HIGH_FREQ_PERIOD_MS  (1)
#define THREAD_HIGH_FREQ_STACK      (256) /* Words = 1024 Bytes */

#define THREAD_LOW_FREQ_PERIOD_MS   (20)
#define THREAD_LOW_FREQ_STACK       (384) /* Words = 1536 Bytes */

void App_Threads_Init(void);

#endif /* APP_THREADS_H */