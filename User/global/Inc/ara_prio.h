/**
 * @file ara_prio.h
 * @brief Unified task priority constants for the ARA project (demo_v7).
 *
 * All RTOS tasks in the project MUST use the priority constants defined here.
 * This file is the single source of truth for the priority lattice.
 *
 * Rationale:
 *   The legacy demo_v6 codebase mixed `tskIDLE_PRIORITY + N` values (tiny
 *   integers) with the CubeMX-generated `osPriorityNormal` enum (=24). That
 *   created an invisible inversion where the CubeMX-default `defaultTask`
 *   actually pre-empted the manually-created control task. demo_v7 removes
 *   that footgun by adopting CMSIS-OS v2 enum values everywhere.
 *
 * Lattice (high to low):
 *   ARA_PRIO_CONTROL       = osPriorityAboveNormal (32)
 *   ARA_PRIO_HOUSEKEEPING  = osPriorityNormal      (24)   <- defaultTask
 *   ARA_PRIO_DEBUG         = osPriorityLow         ( 8)
 *   tskIDLE_PRIORITY       =                       ( 0)
 *
 * The 2-step gap between ControlTask and Housekeeping leaves room for a
 * future "RealTime IO" task (e.g. high-rate ST3215 multi-servo) without
 * having to rebalance everything.
 */

#ifndef ARA_PRIO_H
#define ARA_PRIO_H

#include "cmsis_os2.h"

/* ============================================================================
 * Task priorities
 * ========================================================================== */

/** Main control task: input -> arbiter -> FSM -> servo IO. */
#define ARA_PRIO_CONTROL          (osPriorityAboveNormal)

/** CubeMX defaultTask, retasked as housekeeping (heartbeat watchdog). */
#define ARA_PRIO_HOUSEKEEPING     (osPriorityNormal)

/** Debug / console / telemetry. May be pre-empted by control freely. */
#define ARA_PRIO_DEBUG            (osPriorityLow)

/* ============================================================================
 * Task stack sizes (in CMSIS-OS bytes, not FreeRTOS words)
 * ========================================================================== */

/** ControlTask: arbiter + FSM + ST3215 IO buffers + printf-free hot path. */
#define ARA_STACK_CONTROL_BYTES        (2048U)   /* 512 words */

/** DebugTask: printf format buffers dominate. */
#define ARA_STACK_DEBUG_BYTES          (1024U)   /* 256 words */

/** Housekeeping: minimal, only watchdog refresh + statistics. */
#define ARA_STACK_HOUSEKEEPING_BYTES   ( 512U)   /* 128 words; matches CubeMX defaultTask */

/* ============================================================================
 * Task period constants
 * ========================================================================== */

#define ARA_PERIOD_CONTROL_MS          (20U)    /* 50 Hz */
#define ARA_PERIOD_DEBUG_MS            (100U)   /* 10 Hz */
#define ARA_PERIOD_HOUSEKEEPING_MS     (1000U)  /*  1 Hz */

/* ============================================================================
 * Watchdog contract
 * ========================================================================== */

/**
 * @brief ControlTask heartbeat freshness window. Housekeeping refuses to
 *        feed IWDG when (now - last_heartbeat) exceeds this value.
 * @note  Must be greater than ARA_PERIOD_HOUSEKEEPING_MS plus margin so a
 *        single missed control tick does not trigger reset by itself.
 */
#define ARA_HEARTBEAT_TIMEOUT_MS       (3000U)

#endif /* ARA_PRIO_H */
