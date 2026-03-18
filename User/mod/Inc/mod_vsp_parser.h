/**
 * @file mod_vsp_parser.h
 * @brief ARA-VSP (Vision Simplex Protocol) Parser Module (L3)
 * @note  Pure state-machine parser. Decodes 9-byte frames from UART RingBuffer.
 * STRICTLY NO FLOATING POINT. Little-Endian format expected.
 */

#ifndef MOD_VSP_PARSER_H
#define MOD_VSP_PARSER_H

#include "ara_def.h"

/* =========================================================
 * 1. Protocol Definitions (ARA-VSP)
 * ========================================================= */
#define VSP_FRAME_HEADER_1      (0xAA)
#define VSP_FRAME_HEADER_2      (0x55)
#define VSP_FRAME_LENGTH        (9U)

/* Perception Flag Bitmasks (Byte 3) */
#define VSP_FLAG_IS_TRACKING    (1U << 0)  // Bit 0: Target found in view
#define VSP_FLAG_IS_GRABBABLE   (1U << 1)  // Bit 1: Target within Z-axis depth threshold
#define VSP_FLAG_ESTOP          (1U << 2)  // Bit 2: PC requested emergency stop

/* =========================================================
 * 2. Data Structures
 * ========================================================= */

/**
 * @brief Parsed High-Level Vision Semantic Data
 * @note  Passed to DataHub after successful CRC & Sequence validation.
 */
typedef struct {
    uint8_t  seq_id;            // Sequence ID for tracking drops/latency
    bool     is_tracking;       // true if target is in frame
    bool     is_grabbable;      // true if target depth is within grasp range
    bool     pc_estop_req;      // true if PC software demands E-Stop

    int16_t  target_roll;       // Target Roll Angle in Q15 or *100 format (e.g., 3050 = 30.50 deg)
    uint16_t target_dist_mm;    // Absolute Z-axis distance in millimeters
} AraVisionData_t;

/**
 * @brief Parser State Machine Enum
 */
typedef enum {
    VSP_STATE_WAIT_H1 = 0,
    VSP_STATE_WAIT_H2,
    VSP_STATE_PAYLOAD
} VspParserState_t;

/**
 * @brief Parser Context (Object-Oriented Handle)
 */
typedef struct {
    VspParserState_t state;
    uint8_t          buffer[VSP_FRAME_LENGTH];
    uint8_t          byte_index;

    uint8_t          last_seq_id;      // Used to detect dropped frames
    uint32_t         valid_frame_cnt;  // Statistics: successfully parsed frames
    uint32_t         crc_error_cnt;    // Statistics: corrupted frames
} ModVspParser_Context_t;

/* =========================================================
 * 3. Module API
 * ========================================================= */

/**
 * @brief Initialize the Parser Context
 * @param p_ctx Pointer to the parser context
 */
void ModVspParser_Init(ModVspParser_Context_t *p_ctx);

/**
 * @brief Feed a single byte into the parsing state machine
 * @note  Non-blocking. Call this in a loop for every byte pulled from UART RingBuffer.
 * @param p_ctx     Pointer to the parser context
 * @param in_byte   The raw byte received
 * @param p_out_data Pointer to output struct. Only populated if ARA_OK is returned.
 * @return ARA_OK if a full valid frame is parsed.
 * ARA_BUSY if still parsing.
 * ARA_ERR_CRC if checksum fails.
 */
AraStatus_t ModVspParser_FeedByte(ModVspParser_Context_t *p_ctx,
                                  uint8_t in_byte,
                                  AraVisionData_t *p_out_data);

#endif /* MOD_VSP_PARSER_H */