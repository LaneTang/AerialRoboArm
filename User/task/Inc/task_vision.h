/**
 * @file task_vision.h
 * @brief L4 Vision input runnable.
 *
 * Produces VisionIntent_t samples for the Arbiter.
 *
 * demo_v7 default implementation is Mock-only. H13/HC-13 vision input can be
 * enabled at compile time with TASK_VISION_USE_H13 once a UART is available.
 */

#ifndef TASK_VISION_H
#define TASK_VISION_H

#include "ara_def.h"
#include "datahub.h"

#ifndef TASK_VISION_USE_H13
#define TASK_VISION_USE_H13 (0)
#endif

/**
 * @brief Initialise the vision runnable (resets Mock state).
 */
void TaskVision_Init(void);

/**
 * @brief Produce the current vision intent for this tick.
 * @param tick_ms  Current system tick in milliseconds.
 * @param out      Output intent structure.
 */
void TaskVision_Update(uint32_t tick_ms, VisionIntent_t *out);

/**
 * @brief Inject a mock target, called by ControlTask when it consumes
 *        DBG_REQ_MOCK_VISION_TARGET.
 * @param angle_deg   Target joint angle in degrees.
 * @param duration_ms Target validity window (self-expires after this).
 * @param tick_ms     Current tick timestamp.
 */
void TaskVision_MockInjectTarget(int16_t  angle_deg,
                                 uint32_t duration_ms,
                                 uint32_t tick_ms);

/**
 * @brief Clear any pending mock target immediately.
 */
void TaskVision_MockClear(void);

#endif /* TASK_VISION_H */
