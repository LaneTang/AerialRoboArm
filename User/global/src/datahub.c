/**
 * @file datahub.c
 * @brief Single-writer multi-reader DataHub implementation (demo_v7).
 */

#include "datahub.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

static DataHub_t s_hub;

void DataHub_Init(void)
{
    taskENTER_CRITICAL();

    memset(&s_hub, 0, sizeof(s_hub));

    /* Safe defaults: system is booting, nothing is online, E-Stop is armed. */
    s_hub.current_mode         = ARA_MODE_INIT;
    s_hub.estop_active         = true;
    s_hub.led_pattern          = LED_PATTERN_BOOT_FAST_BLINK;
    s_hub.arbiter_mode         = ARA_MODE_INIT;
    s_hub.servo_status         = ARA_ERR_DISCONNECTED;
    s_hub.rc_link_up           = false;
    s_hub.vision_link_up       = false;

    taskEXIT_CRITICAL();
}

void DataHub_Publish(const DataHub_t *snapshot)
{
    if (snapshot == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    s_hub = *snapshot;
    taskEXIT_CRITICAL();
}

void DataHub_Read(DataHub_t *out)
{
    if (out == NULL) {
        return;
    }
    taskENTER_CRITICAL();
    *out = s_hub;
    taskEXIT_CRITICAL();
}

AraLedPattern_t DataHub_GetLedPattern(void)
{
    taskENTER_CRITICAL();
    AraLedPattern_t p = s_hub.led_pattern;
    taskEXIT_CRITICAL();
    return p;
}

AraSysMode_t DataHub_GetCurrentMode(void)
{
    taskENTER_CRITICAL();
    AraSysMode_t m = s_hub.current_mode;
    taskEXIT_CRITICAL();
    return m;
}
