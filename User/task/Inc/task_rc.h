/**
 * @file task_rc.h
 * @brief RC Data Collection & Semantic Logic (L4) - Downgraded to Runnable
 * @note  Extracts high-level intent from raw L2 ELRS data.
 */

#ifndef TASK_RC_H
#define TASK_RC_H

#include "ara_def.h"
#include "datahub.h"          /* For RcControlData_t */
#include "drv_elrs.h"
#include "mod_rc_semantic.h"

/* --- Logic Context --- */
typedef struct {
    bool                    is_initialized;
    DrvElrs_Context_t       elrs_driver;
    ModRcSemantic_Context_t semantic_logic;
} TaskRc_Context_t;

/* --- API --- */

/**
 * @brief Initialize RC dependencies (UART, Protocol Parsers).
 */
void TaskRc_Init(void);

/**
 * @brief The 50Hz Execution Step.
 * @note  Reads the UART RingBuffer, checks CRC, applies failsafes if link lost.
 * * @param current_tick_ms [IN]  Current system time for timeout evaluation
 * @param p_out_intent    [OUT] The extracted semantic intent (passed to Manipulator)
 */
void TaskRc_Update(uint32_t current_tick_ms, RcControlData_t *p_out_intent);

#endif /* TASK_RC_H */