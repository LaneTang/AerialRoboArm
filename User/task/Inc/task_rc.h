/**
 * @file task_rc.h
 * @brief RC Data Collection and Semantic Dispatch Task (L4)
 * @note  FreeRTOS Task. Aggregates L2 (DRV_ELRS) and L3 (MOD_RC_SEMANTIC).
 * Periodically polls the receiver, parses semantics, and updates the DataHub.
 */

#ifndef TASK_RC_H
#define TASK_RC_H

#include "ara_def.h"
#include "drv_elrs.h"
#include "mod_rc_semantic.h"
#include "datahub.h"

/* =========================================================
 * 1. Task Configuration
 * ========================================================= */

/* Task execution period in milliseconds (e.g., 20ms = 50Hz) */
#define TASK_RC_PERIOD_MS   (20U)


/* =========================================================
 * 2. Task Context Structure
 * ========================================================= */

/**
 * @brief RC Task Context (Aggregation of L2 and L3 components)
 * @note  Follows the Object-Oriented context pattern.
 */
typedef struct {
    /* --- State --- */
    uint32_t                tick_count;       // Current OS Tick
    bool                    is_initialized;   // Initialization flag

    /* --- Components (L2 & L3 Aggregation) --- */
    DrvElrs_Context_t       elrs_driver;      // L2: Hardware-agnostic ELRS driver
    ModRcSemantic_Context_t semantic_logic;   // L3: Pure logic semantic parser

    /* --- Output Cache --- */
    RcControlData_t         last_intent;      // Cached semantic intent to be pushed to DataHub
} TaskRc_Context_t;


/* =========================================================
 * 3. Task API
 * ========================================================= */

/**
 * @brief Initialize the RC Task components
 * @note  Must be called BEFORE the FreeRTOS scheduler starts.
 * Internally initializes DrvElrs (L2) and ModRcSemantic (L3).
 */
void TaskRc_Init(void);

/**
 * @brief The Main RC Task Entry Point (FreeRTOS Task Function)
 * @param argument Pointer to task parameters (usually NULL)
 * @note  Infinite loop structure:
 * 1. vTaskDelayUntil() for periodic execution.
 * 2. Calls DrvElrs_Update() to process UART RingBuffer.
 * 3. Checks Link Status. If Link Down -> triggers GLOBAL ESTOP via DataHub.
 * 4. Calls ModRcSemantic_Process() to extract high-level intent.
 * 5. Calls DataHub_WriteRcData() to publish the intent for L4/L5 consumers.
 */
void TaskRc_Entry(void *argument);


#endif /* TASK_RC_H */