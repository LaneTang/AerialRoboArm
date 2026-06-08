/**
 * @file app_control.h
 * @brief L5 Main control task entry (demo_v7).
 *
 * ControlTask ties the full 50 Hz loop together:
 *   1. Consume DebugRequest (mode switch, mock vision trigger, etc).
 *   2. TaskRc_Update       -> RcControlData_t
 *   3. TaskVision_Update   -> VisionIntent_t
 *   4. TaskArbiter_Decide  -> ArbiterOutput_t
 *   5. TaskManipulator_Update -> MotionCmd_t
 *   6. TaskMotion_Update   -> MotionState_t (may do ST3215 IO)
 *   7. Publish DataHub snapshot.
 *   8. Update heartbeat for Housekeeping / IWDG.
 *
 * ControlTask is the SOLE DataHub writer. DebugTask and defaultTask read it.
 *
 * Initialisation is split:
 *   - App_Control_InitDeps(): pre-scheduler. Initialises L4 runnables and
 *     DataHub. No thread creation.
 *   - App_Control_Init(): post-scheduler (from RTOS_THREADS section of
 *     freertos.c). Creates the ControlTask thread.
 */

#ifndef APP_CONTROL_H
#define APP_CONTROL_H

#include "ara_def.h"
#include "ara_prio.h"

/* ============================================================================
 * Lifecycle
 * ============================================================================= */

/**
 * @brief Initialise L4 runnables / driver contexts. Call from main() before
 *        osKernelInitialize().
 */
void App_Control_InitDeps(void);

/**
 * @brief Create the ControlTask thread. Call from USER CODE BEGIN RTOS_THREADS
 *        in freertos.c.
 */
void App_Control_Init(void);

/* ============================================================================
 * Observability
 * ============================================================================= */

/**
 * @brief Return the timestamp of the most recent successful ControlTask tick.
 * @note  Used by App_Housekeeping to gate IWDG refresh.
 */
uint32_t App_Control_GetHeartbeatMs(void);

/**
 * @brief Snapshot the bring-up force-mode state.
 * @param  out_angle_deg  When force is active, receives the locked target.
 *                        Untouched when force is inactive. May be NULL.
 * @return true when force mode is currently active.
 *
 * Used by DebugTask to render force status alongside the periodic snapshot.
 * Reading is racy with the ControlTask writer but correctness only requires
 * eventual consistency for an operator-facing readout.
 */
bool App_Control_GetForceState(int16_t *out_angle_deg);

#endif /* APP_CONTROL_H */
