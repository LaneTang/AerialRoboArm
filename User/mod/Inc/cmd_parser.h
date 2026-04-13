/**
 * @file    ara_cmd_parser.h
 * @brief   L3 Layer: Protocol Parser & Packer for PC-MCU Communication
 * @note    Stateless & Hardware Agnostic. No direct BSP calls.
 * Float data is assumed to be strictly Little-Endian (IEEE 754).
 */

#ifndef ARA_CMD_PARSER_H
#define ARA_CMD_PARSER_H

#include <stdint.h>
#include <stdbool.h>

/*===========================================================================*/
/* Constant & Macro Definitions                                              */
/*===========================================================================*/
#define ARA_CMD_HEADER_1       0xAA
#define ARA_CMD_HEADER_2       0x55
#define ARA_CMD_MAX_PAYLOAD    128   /**< Maximum payload length (mainly for LOG) */

/*===========================================================================*/
/* Enumerations                                                              */
/*===========================================================================*/

/**
 * @brief Function IDs defined in Protocol V2.1
 */
typedef enum {
    ARA_CMD_ID_PLOT        = 0x01,  /**< MCU -> PC: High-speed waveform */
    ARA_CMD_ID_LOG         = 0x02,  /**< MCU -> PC: Text log */
    ARA_CMD_ID_SET_PARAM   = 0x10,  /**< PC -> MCU: Set PID/Target */
    ARA_CMD_ID_GET_PID     = 0x11,  /**< PC -> MCU: Request current PID */
    ARA_CMD_ID_REPORT_PID  = 0x12,  /**< MCU -> PC: Report current PID */
    ARA_CMD_ID_ACK         = 0xF0   /**< MCU -> PC: Command execution result */
} AraCmd_FuncId_t;

/**
 * @brief Parameter IDs for CMD_SET_PARAM
 */
typedef enum {
    ARA_PARAM_P            = 0x01,
    ARA_PARAM_I            = 0x02,
    ARA_PARAM_D            = 0x03,
    ARA_PARAM_TARGET_ANGLE = 0x04
} AraCmd_ParamId_t;

/**
 * @brief ACK Status Codes
 */
typedef enum {
    ARA_ACK_SUCCESS        = 0x00,
    ARA_ACK_ERR_OUTOFBOUND = 0x01,
    ARA_ACK_ERR_CHECKSUM   = 0x02,
    ARA_ACK_ERR_UNKNOWN_ID = 0x03
} AraCmd_AckStatus_t;

/**
 * @brief Parser internal state machine states
 */
typedef enum {
    PARSE_STATE_HEAD1 = 0,
    PARSE_STATE_HEAD2,
    PARSE_STATE_CMD,
    PARSE_STATE_LEN,
    PARSE_STATE_PAYLOAD,
    PARSE_STATE_CHECKSUM
} AraCmd_ParseState_t;

/*===========================================================================*/
/* Data Structures                                                           */
/*===========================================================================*/

/**
 * @brief Payload structure for plotting (4x float32)
 */
typedef struct {
    float target_angle;
    float actual_angle;
    float velocity;
    float current;
} AraCmd_PlotData_t;

/**
 * @brief Payload structure for PID reporting (3x float32)
 */
typedef struct {
    float p_gain;
    float i_gain;
    float d_gain;
} AraCmd_PidData_t;

/**
 * @brief Application Callbacks (L4 implements these, L3 calls them)
 */
typedef struct {
    /** * @brief Called when a SET_PARAM command is successfully received & verified
     */
    void (*OnSetParam)(AraCmd_ParamId_t param_id, float value);

    /** * @brief Called when a GET_PID request is successfully received & verified
     */
    void (*OnGetPidReq)(void);

    /**
     * @brief Called when a frame fails checksum validation (for logging/metrics)
     */
    void (*OnParseError)(AraCmd_FuncId_t cmd_id);
} AraCmd_Callbacks_t;

/**
 * @brief Parser Context (Simulates OOP object instance)
 */
typedef struct {
    AraCmd_ParseState_t state;
    uint8_t             current_cmd;
    uint8_t             expected_len;
    uint8_t             payload_idx;
    uint8_t             calc_checksum;
    uint8_t             payload_buf[ARA_CMD_MAX_PAYLOAD];

    AraCmd_Callbacks_t  callbacks;  /**< Registered callbacks */
} AraCmd_ParserCtx_t;

/*===========================================================================*/
/* API Functions                                                             */
/*===========================================================================*/

/**
 * @brief  Initialize the parser context and register callbacks.
 * @param  ctx Pointer to the parser context.
 * @param  cbs Pointer to the callback function structure.
 */
void AraCmd_Init(AraCmd_ParserCtx_t *ctx, const AraCmd_Callbacks_t *cbs);

/**
 * @brief  Feed a single byte into the protocol state machine.
 * Typically called by L4 in a loop reading from UART RingBuffer.
 * Non-blocking. O(1) complexity per byte.
 * @param  ctx  Pointer to the parser context.
 * @param  byte The received byte.
 */
void AraCmd_FeedByte(AraCmd_ParserCtx_t *ctx, uint8_t byte);

/*===========================================================================*/
/* Packet Generators (Output) - Returns length of generated frame            */
/*===========================================================================*/

/**
 * @brief  Pack a CMD_PLOT frame into the provided buffer.
 * @param  out_buf Buffer to hold the frame (must be at least 21 bytes).
 * @param  data    Pointer to the plot data.
 * @return Number of bytes packed.
 */
uint16_t AraCmd_PackPlot(uint8_t *out_buf, const AraCmd_PlotData_t *data);

/**
 * @brief  Pack a CMD_LOG frame into the provided buffer.
 * @param  out_buf Buffer to hold the frame.
 * @param  text    Null-terminated string (will be truncated if too long).
 * @return Number of bytes packed.
 */
uint16_t AraCmd_PackLog(uint8_t *out_buf, const char *text);

/**
 * @brief  Pack a CMD_ACK frame into the provided buffer.
 * @param  out_buf Buffer to hold the frame (must be at least 7 bytes).
 * @param  echo_cmd The command ID being acknowledged.
 * @param  status   Execution status.
 * @return Number of bytes packed.
 */
uint16_t AraCmd_PackAck(uint8_t *out_buf, uint8_t echo_cmd, AraCmd_AckStatus_t status);

/**
 * @brief  Pack a CMD_REPORT_PID frame into the provided buffer.
 * @param  out_buf Buffer to hold the frame (must be at least 17 bytes).
 * @param  pid     Pointer to the PID data.
 * @return Number of bytes packed.
 */
uint16_t AraCmd_PackPidReport(uint8_t *out_buf, const AraCmd_PidData_t *pid);

#endif /* ARA_CMD_PARSER_H */