/**
 * @file debug_request.c
 * @brief Single-slot Console -> ControlTask request channel.
 */

#include "debug_request.h"

#include "FreeRTOS.h"
#include "task.h"

/* Single global slot. Initialised to empty. */
static DebugRequest_t s_slot;
static bool           s_pending;

void DebugRequest_Init(void)
{
    taskENTER_CRITICAL();
    s_slot.type           = DBG_REQ_NONE;
    s_slot.arg1           = 0;
    s_slot.arg2           = 0;
    s_slot.posted_tick_ms = 0U;
    s_pending             = false;
    taskEXIT_CRITICAL();
}

void DebugRequest_Post(DebugReqType_t type, int32_t arg1, int32_t arg2)
{
    if ((type == DBG_REQ_NONE) || (type >= DBG_REQ_TYPE_COUNT)) {
        return;
    }

    taskENTER_CRITICAL();
    s_slot.type           = type;
    s_slot.arg1           = arg1;
    s_slot.arg2           = arg2;
    s_slot.posted_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    s_pending             = true;
    taskEXIT_CRITICAL();
}

bool DebugRequest_TakePending(DebugRequest_t *out)
{
    if (out == NULL) {
        return false;
    }

    bool taken = false;
    taskENTER_CRITICAL();
    if (s_pending) {
        *out      = s_slot;
        s_pending = false;
        taken     = true;
    }
    taskEXIT_CRITICAL();
    return taken;
}
