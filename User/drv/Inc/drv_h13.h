/**
 * @file drv_h13.h
 * @brief L2 Driver: H13/HC-13 transparent UART link and vision packet parser.
 *
 * Reference demo:
 *   D:/Study_data/stm32project/H13-demo
 *
 * The demo uses:
 *   - USART1 PA9/PA10, 9600 8N1
 *   - PB0 KEY pin, HIGH = transparent mode, LOW = AT mode
 *   - single-byte interrupt RX
 *
 * In AerialRoboArm all three UARTs are already allocated, so this driver is
 * intentionally hardware-agnostic for now. Future BSP wiring should feed
 * received bytes into DrvH13_PushRxByte() and send AT commands built here.
 */

#ifndef DRV_H13_H
#define DRV_H13_H

#include "ara_def.h"

/* ============================================================================
 * Configuration
 * ========================================================================== */

#define H13_UART_BAUD_DEFAULT           (9600U)
#define H13_RX_BUF_SIZE                 (256U)
#define H13_LINE_BUF_SIZE               (64U)
#define H13_VISION_TTL_MS               (300U)

/* H13 AT mode air-rate command. User requirement: S7 = highest rate. */
typedef enum {
    H13_AIR_RATE_S1 = 1,
    H13_AIR_RATE_S2 = 2,
    H13_AIR_RATE_S3 = 3,
    H13_AIR_RATE_S4 = 4,
    H13_AIR_RATE_S5 = 5,
    H13_AIR_RATE_S6 = 6,
    H13_AIR_RATE_S7 = 7,
} H13AirRate_t;

typedef struct {
    bool     target_present;
    int16_t  target_angle_deg;
    uint16_t target_speed;
    uint8_t  confidence;
    uint32_t timestamp_ms;
} H13VisionSample_t;

/* ============================================================================
 * API
 * ========================================================================== */

/**
 * @brief Reset all H13 parser and RX state.
 * @note  Does not touch GPIO or UART; current project has no free UART.
 */
void DrvH13_Init(void);

/**
 * @brief Build the AT command used to select air rate S1..S7.
 * @param rate  Desired H13 air-rate level. S7 is the project default.
 * @param out   Destination buffer.
 * @param len   Destination capacity.
 * @return Bytes written, or 0 on parameter error.
 */
uint16_t DrvH13_BuildSetAirRateCmd(H13AirRate_t rate, uint8_t *out, uint16_t len);

/**
 * @brief Convenience wrapper for the project requirement: configure S7.
 */
uint16_t DrvH13_BuildSetS7Cmd(uint8_t *out, uint16_t len);

/**
 * @brief Push one received transparent-mode byte into the software RX FIFO.
 * @note  Future UART ISR / DMA drain code should call this.
 */
void DrvH13_PushRxByte(uint8_t byte);

/**
 * @brief Read raw bytes from the H13 RX FIFO.
 */
uint16_t DrvH13_Recv(uint8_t *data, uint16_t len);

/**
 * @brief Flush raw RX bytes and partial line parser state.
 */
void DrvH13_Flush(void);

/**
 * @brief Drain RX bytes, parse PC vision text frames, and return freshest sample.
 *
 * Supported PC-side transparent text frames:
 *   V,<angle_deg>,<speed>,<confidence>\n
 *   VISION,<angle_deg>,<speed>,<confidence>\n
 *
 * Example:
 *   V,35,750,92\n
 *
 * Angle is clamped to [-180,+180], confidence to [0,100]. A sample remains
 * valid for H13_VISION_TTL_MS after its timestamp.
 */
bool DrvH13_PollVision(uint32_t now_ms, H13VisionSample_t *out);

/**
 * @brief Diagnostics: cumulative bytes accepted into the RX FIFO.
 */
uint32_t DrvH13_GetRxBytes(void);

/**
 * @brief Diagnostics: cumulative valid vision frames parsed.
 */
uint32_t DrvH13_GetVisionFrames(void);

#endif /* DRV_H13_H */
