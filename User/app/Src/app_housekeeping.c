/**
 * @file app_housekeeping.c
 * @brief defaultTask body for demo_v7 (L5).
 */

#include "app_housekeeping.h"
#include "app_control.h"
#include "ara_prio.h"

#include "stm32f1xx_hal.h"
#include "cmsis_os2.h"

#ifdef HAL_IWDG_MODULE_ENABLED
extern IWDG_HandleTypeDef hiwdg;
#endif

void App_Housekeeping_Init(void)
{
    /* Reserved for future self-test / statistics init. */
}

void App_Housekeeping_Step(void)
{
    const uint32_t now_ms = osKernelGetTickCount();
    const uint32_t last_hb = App_Control_GetHeartbeatMs();

#ifdef HAL_IWDG_MODULE_ENABLED
    /* Refuse to feed IWDG if ControlTask heartbeat is stale. That way a
     * hung ControlTask will trigger an IWDG reset instead of being masked. */
    if ((last_hb != 0U) &&
        ((uint32_t)(now_ms - last_hb) < ARA_HEARTBEAT_TIMEOUT_MS)) {
        HAL_IWDG_Refresh(&hiwdg);
    }
#else
    (void)now_ms;
    (void)last_hb;
#endif

    /* Future: uxTaskGetStackHighWaterMark + CPU busy-ratio reporting. */
}
