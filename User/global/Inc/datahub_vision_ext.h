/**
 * @file datahub_vision_ext.h
 * @brief Vision Extension definitions for DataHub (L4)
 * @note  These items MUST be merged into the main datahub.h / datahub.c.
 */

#ifndef DATAHUB_VISION_EXT_H
#define DATAHUB_VISION_EXT_H

#include "mod_vsp_parser.h" /* Needs AraVisionData_t */

/* Add these to the existing DataHub API section */

/**
 * @brief Write the latest validated vision data to the Hub
 * @note  Thread-safe. Must update an internal 'last_vision_tick' timestamp
 * inside the critical section.
 * @param p_vision_data Pointer to the parsed data
 * @return ARA_OK on success
 */
AraStatus_t DataHub_WriteVisionData(const AraVisionData_t *p_vision_data);

/**
 * @brief Read the latest vision data for decision making
 * @note  Thread-safe. Used by Task_Manipulator.
 * @param p_vision_data Pointer to destination buffer
 * @return ARA_OK on success
 */
AraStatus_t DataHub_ReadVisionData(AraVisionData_t *p_vision_data);

/**
 * @brief Get the FreeRTOS tick of the last successfully received vision frame
 * @note  Thread-safe. Used by L5 Scheduler to detect Wi-Fi/PC disconnections.
 * @return Tick count in milliseconds
 */
uint32_t DataHub_GetLastVisionTick(void);

#endif /* DATAHUB_VISION_EXT_H */