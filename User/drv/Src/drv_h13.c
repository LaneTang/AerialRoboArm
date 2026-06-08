/**
 * @file drv_h13.c
 * @brief L2 Driver: H13/HC-13 transparent UART link and vision parser.
 */

#include "drv_h13.h"

#include <stdlib.h>
#include <string.h>

/* ============================================================================
 * RX FIFO
 * ========================================================================== */

static uint8_t  s_rx_buf[H13_RX_BUF_SIZE];
static volatile uint16_t s_rx_head = 0U;   /* ISR/BSP writes */
static uint16_t s_rx_tail = 0U;            /* task reads */

static char     s_line_buf[H13_LINE_BUF_SIZE];
static uint16_t s_line_len = 0U;

static H13VisionSample_t s_latest;
static uint32_t s_rx_bytes = 0U;
static uint32_t s_vision_frames = 0U;

/* ============================================================================
 * Helpers
 * ========================================================================== */

static int16_t clamp_i16(int32_t v, int16_t lo, int16_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return (int16_t)v;
}

static uint16_t clamp_u16(int32_t v, uint16_t hi)
{
    if (v < 0) return 0U;
    if (v > (int32_t)hi) return hi;
    return (uint16_t)v;
}

static uint8_t clamp_u8(int32_t v, uint8_t hi)
{
    if (v < 0) return 0U;
    if (v > (int32_t)hi) return hi;
    return (uint8_t)v;
}

static bool parse_i32_token(char **cursor, int32_t *out)
{
    if ((cursor == NULL) || (*cursor == NULL) || (out == NULL)) {
        return false;
    }

    char *end = NULL;
    long value = strtol(*cursor, &end, 10);
    if (end == *cursor) {
        return false;
    }

    *out = (int32_t)value;
    if (*end == ',') {
        end++;
    }
    *cursor = end;
    return true;
}

static bool parse_vision_line(char *line, uint32_t now_ms)
{
    if (line == NULL) {
        return false;
    }

    char *cursor = line;
    if (strncmp(cursor, "VISION,", 7U) == 0) {
        cursor += 7U;
    } else if (strncmp(cursor, "V,", 2U) == 0) {
        cursor += 2U;
    } else {
        return false;
    }

    int32_t angle = 0;
    int32_t speed = 0;
    int32_t conf = 100;

    if (!parse_i32_token(&cursor, &angle)) {
        return false;
    }
    if (!parse_i32_token(&cursor, &speed)) {
        speed = 0;
    }
    if (!parse_i32_token(&cursor, &conf)) {
        conf = 100;
    }

    s_latest.target_present    = true;
    s_latest.target_angle_deg  = clamp_i16(angle, -180, 180);
    s_latest.target_speed      = clamp_u16(speed, 65535U);
    s_latest.confidence        = clamp_u8(conf, 100U);
    s_latest.timestamp_ms      = now_ms;
    s_vision_frames++;
    return true;
}

static void consume_parser_byte(uint8_t byte, uint32_t now_ms)
{
    if (byte == '\r') {
        return;
    }

    if (byte == '\n') {
        s_line_buf[s_line_len] = '\0';
        if (s_line_len > 0U) {
            (void)parse_vision_line(s_line_buf, now_ms);
        }
        s_line_len = 0U;
        return;
    }

    if (s_line_len < (H13_LINE_BUF_SIZE - 1U)) {
        s_line_buf[s_line_len++] = (char)byte;
    } else {
        /* Drop an overlong line and wait for the next newline. */
        s_line_len = 0U;
    }
}

/* ============================================================================
 * Public API
 * ========================================================================== */

void DrvH13_Init(void)
{
    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_line_len = 0U;
    memset(&s_latest, 0, sizeof(s_latest));
    s_rx_bytes = 0U;
    s_vision_frames = 0U;
}

uint16_t DrvH13_BuildSetAirRateCmd(H13AirRate_t rate, uint8_t *out, uint16_t len)
{
    if ((out == NULL) || (len < 7U) ||
        (rate < H13_AIR_RATE_S1) || (rate > H13_AIR_RATE_S7)) {
        return 0U;
    }

    out[0] = 'A';
    out[1] = 'T';
    out[2] = '+';
    out[3] = 'S';
    out[4] = (uint8_t)('0' + (uint8_t)rate);
    out[5] = '\r';
    out[6] = '\n';
    return 7U;
}

uint16_t DrvH13_BuildSetS7Cmd(uint8_t *out, uint16_t len)
{
    return DrvH13_BuildSetAirRateCmd(H13_AIR_RATE_S7, out, len);
}

void DrvH13_PushRxByte(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % H13_RX_BUF_SIZE);
    if (next != s_rx_tail) {
        s_rx_buf[s_rx_head] = byte;
        s_rx_head = next;
        s_rx_bytes++;
    }
}

uint16_t DrvH13_Recv(uint8_t *data, uint16_t len)
{
    if ((data == NULL) || (len == 0U)) {
        return 0U;
    }

    uint16_t count = 0U;
    while ((count < len) && (s_rx_tail != s_rx_head)) {
        data[count++] = s_rx_buf[s_rx_tail];
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % H13_RX_BUF_SIZE);
    }
    return count;
}

void DrvH13_Flush(void)
{
    s_rx_tail = s_rx_head;
    s_line_len = 0U;
}

bool DrvH13_PollVision(uint32_t now_ms, H13VisionSample_t *out)
{
    uint8_t byte = 0U;
    while (DrvH13_Recv(&byte, 1U) == 1U) {
        consume_parser_byte(byte, now_ms);
    }

    bool fresh = s_latest.target_present &&
                 ((uint32_t)(now_ms - s_latest.timestamp_ms) <= H13_VISION_TTL_MS);

    if (fresh) {
        if (out != NULL) {
            *out = s_latest;
        }
        return true;
    }

    if (out != NULL) {
        memset(out, 0, sizeof(*out));
    }
    return false;
}

uint32_t DrvH13_GetRxBytes(void)
{
    return s_rx_bytes;
}

uint32_t DrvH13_GetVisionFrames(void)
{
    return s_vision_frames;
}
