/**
 * @file app_housekeeping.h
 * @brief System housekeeping invoked from CubeMX defaultTask (L5).
 *
 * defaultTask keeps its CubeMX-generated shell. Its body calls
 * App_Housekeeping_Step() at 1 Hz to perform non-critical janitor work:
 *   - Conditional IWDG refresh (only if ControlTask heartbeat is fresh).
 *   - Future: CPU usage stats, stack high-watermark monitoring.
 *
 * This module intentionally depends on nothing else in User/app so that the
 * defaultTask shell remains CubeMX-friendly.
 */

#ifndef APP_HOUSEKEEPING_H
#define APP_HOUSEKEEPING_H

#include "ara_def.h"

void App_Housekeeping_Init(void);
void App_Housekeeping_Step(void);

#endif /* APP_HOUSEKEEPING_H */
