/**
 * @file task_rc.h
 * @brief RC Data Collection & Semantic Logic Runnable (L4)
 * @note  Extracts high-level intent from raw L2 ELRS data. This task owns
 *        L2 ELRS driver orchestration and L3 semantic mapping.
 */

#ifndef TASK_RC_H
#define TASK_RC_H

#include "ara_def.h"
#include "datahub.h"
#include "drv_elrs.h"
#include "mod_rc_semantic.h"

/* =========================================================
 * 1. Task Context
 * ========================================================= */

/**
 * @brief RC task runtime context.
 */
typedef struct {
    bool                    is_initialized; /**< Combined init state of owned dependencies. */
    DrvElrs_Context_t       elrs_driver;    /**< Owned L2 ELRS driver. */
    ModRcSemantic_Context_t semantic_logic; /**< Owned L3 semantic module. */
} TaskRc_Context_t;

/* =========================================================
 * 2. API
 * ========================================================= */

/**
 * @brief  Initialize RC task dependencies.
 * @note   This binds the ELRS driver to the dedicated UART device and seeds
 *         the semantic module with safe default channels.
 */
void TaskRc_Init(void);

/**
 * @brief  Execute one formal RC semantic update step.
 * @param  current_tick_ms Current system tick in milliseconds.
 * @param  p_out_intent Output semantic intent for manipulator / upper logic.
 * @note   This API is intended for formal MANUAL / AUTO system flows.
 *         When link is down or initialization failed, safe failsafe intent
 *         is emitted.
 */
void TaskRc_Update(uint32_t current_tick_ms, RcControlData_t *p_out_intent);

/**
 * @brief  Execute one debug RC analog update step.
 * @param  current_tick_ms Current system tick in milliseconds.
 * @param  p_out_debug Output debug-oriented analog RC data.
 * @note   This API is intended for TB0 module debugging scenarios where CH1
 *         should be observed as direct analog percentage rather than collapsed
 *         into EXTEND / RETRACT / HOLD semantics.
 */
void TaskRc_UpdateDebugAnalog(uint32_t current_tick_ms,
                              RcDebugAnalogData_t *p_out_debug);

/**
 * @brief  Query whether TaskRc has completed dependency initialization.
 * @return true if initialized successfully, otherwise false.
 */
bool TaskRc_IsInitialized(void);

/**
 * @brief  Copy the latest valid raw ELRS channel snapshot.
 * @param  p_out_channels Output buffer for all raw channels.
 * @param  out_count Number of elements available in @p p_out_channels.
 * @return ARA_OK on success,
 *         ARA_ERR_PARAM on invalid input,
 *         ARA_ERR_DISCONNECTED when link is currently down or TaskRc is not initialized.
 */
AraStatus_t TaskRc_CopyRawChannels(uint16_t *p_out_channels, uint8_t out_count);

#endif /* TASK_RC_H */