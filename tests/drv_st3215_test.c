/**
 * @file drv_st3215_test.c
 * @brief PC-side byte-level conformance test for drv_st3215.
 *
 * Compile (Windows MinGW / msys):
 *   gcc -std=c99 -Wall -Wextra -I../User/global/Inc -I../User/drv/Inc \
 *       drv_st3215_test.c ../User/drv/Src/drv_st3215.c -o drv_st3215_test.exe
 *
 * Compile (Linux):
 *   gcc -std=c99 -Wall -Wextra -I../User/global/Inc -I../User/drv/Inc \
 *       drv_st3215_test.c ../User/drv/Src/drv_st3215.c -o drv_st3215_test
 *
 * Run:
 *   ./drv_st3215_test
 *
 * Exits 0 on full pass, non-zero on any failure with a hex diff dump.
 */

#include "drv_st3215.h"

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

/* =============================================================================
 * Test framework (minimal)
 * ============================================================================= */

static int g_pass = 0;
static int g_fail = 0;

static void hex_dump(const char *label, const uint8_t *buf, uint8_t len)
{
    printf("    %-10s [", label);
    for (uint8_t i = 0; i < len; i++) {
        printf("%02X%s", buf[i], (i + 1 < len) ? " " : "");
    }
    printf("]\n");
}

#define ASSERT_EQ_INT(actual, expected, label)                                 \
    do {                                                                       \
        if ((long)(actual) != (long)(expected)) {                              \
            printf("  [FAIL] %s: expected=%ld actual=%ld\n",                   \
                   (label), (long)(expected), (long)(actual));                 \
            g_fail++;                                                          \
            return;                                                            \
        }                                                                      \
    } while (0)

#define ASSERT_BYTES_EQ(actual, expected, len)                                 \
    do {                                                                       \
        if (memcmp((actual), (expected), (len)) != 0) {                        \
            printf("  [FAIL] byte mismatch:\n");                               \
            hex_dump("expected", (expected), (len));                           \
            hex_dump("actual",   (actual),   (len));                           \
            g_fail++;                                                          \
            return;                                                            \
        }                                                                      \
    } while (0)

#define TEST_PASS(name)                                                        \
    do {                                                                       \
        printf("  [PASS] %s\n", (name));                                       \
        g_pass++;                                                              \
    } while (0)

/* =============================================================================
 * Encoding tests
 * ============================================================================= */

static void test_encode_ping_id1(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x01, 0x02, 0x01, 0xFB };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodePing(buf, 1);
    ASSERT_EQ_INT(n, sizeof(expected), "encode_ping_id1 length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_ping_id1");
}

static void test_encode_ping_id42(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x2A, 0x02, 0x01, 0xD2 };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodePing(buf, 42);
    ASSERT_EQ_INT(n, sizeof(expected), "encode_ping_id42 length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_ping_id42");
}

static void test_encode_read_feedback_id1(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x01, 0x04, 0x02, 0x38, 0x0F, 0xB1 };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeReadFeedback(buf, 1);
    ASSERT_EQ_INT(n, sizeof(expected), "encode_read_feedback length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_read_feedback_id1");
}

static void test_encode_write_byte_torque_enable(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x01, 0x04, 0x03, 0x28, 0x01, 0xCE };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWriteByte(buf, 1, ST3215_REG_TORQUE_ENABLE, 1);
    ASSERT_EQ_INT(n, sizeof(expected), "torque_enable length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_write_byte_torque_enable");
}

static void test_encode_write_word_max_angle(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x01, 0x05, 0x03, 0x0B, 0xFF, 0x0F, 0xDD };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWriteWord(buf, 1, ST3215_REG_MAX_ANGLE_LIMIT_L, 4095);
    ASSERT_EQ_INT(n, sizeof(expected), "write_word length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_write_word_max_angle");
}

static void test_encode_write_byte_lock_open(void)
{
    const uint8_t expected[] = { 0xFF, 0xFF, 0x01, 0x04, 0x03, 0x37, 0x00, 0xC0 };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWriteByte(buf, 1, ST3215_REG_LOCK, ST3215_LOCK_OPEN);
    ASSERT_EQ_INT(n, sizeof(expected), "lock_open length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_write_byte_lock_open");
}

static void test_encode_writepos_pos_2048_zero(void)
{
    const uint8_t expected[] = {
        0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29,
        0x00, 0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0xC0
    };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWritePos(buf, 1, 2048, 0, 0);
    ASSERT_EQ_INT(n, sizeof(expected), "writepos_2048 length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_writepos_pos_2048_zero");
}

static void test_encode_writepos_negative(void)
{
    const uint8_t expected[] = {
        0xFF, 0xFF, 0x01, 0x0A, 0x03, 0x29,
        0x32, 0x64, 0x80, 0x00, 0x00, 0xE8, 0x03, 0x51
    };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWritePos(buf, 1, -100, 1000, 50);
    ASSERT_EQ_INT(n, sizeof(expected), "writepos_neg length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_writepos_negative");
}

static void test_encode_writepos_max(void)
{
    const uint8_t expected[] = {
        0xFF, 0xFF, 0x02, 0x0A, 0x03, 0x29,
        0x64, 0xFF, 0x0F, 0x00, 0x00, 0xA0, 0x0F, 0xDC
    };
    uint8_t buf[ST3215_TX_BUF_SIZE];
    uint8_t n = DrvSt3215_EncodeWritePos(buf, 2, 4095, 4000, 100);
    ASSERT_EQ_INT(n, sizeof(expected), "writepos_max length");
    ASSERT_BYTES_EQ(buf, expected, sizeof(expected));
    TEST_PASS("encode_writepos_max");
}

/* =============================================================================
 * Parsing tests
 * ============================================================================= */

static void test_parse_ack_ok(void)
{
    const uint8_t frame[] = { 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFC };
    uint8_t err = 0xAA;
    St3215_ParseResult_t r = DrvSt3215_ParseAck(frame, sizeof(frame), 1, &err);
    ASSERT_EQ_INT(r,   ST3215_PARSE_OK,  "parse_ack_ok result");
    ASSERT_EQ_INT(err, 0,                "parse_ack_ok error_bits");
    TEST_PASS("parse_ack_ok");
}

static void test_parse_ack_overload(void)
{
    const uint8_t frame[] = { 0xFF, 0xFF, 0x01, 0x02, 0x20, 0xDC };
    uint8_t err = 0;
    St3215_ParseResult_t r = DrvSt3215_ParseAck(frame, sizeof(frame), 1, &err);
    ASSERT_EQ_INT(r,   ST3215_PARSE_SERVO_ERROR, "parse_ack_overload result");
    ASSERT_EQ_INT(err, ST3215_ERRBIT_OVERLOAD,   "parse_ack_overload bits");
    TEST_PASS("parse_ack_overload");
}

static void test_parse_ack_bad_checksum(void)
{
    const uint8_t frame[] = { 0xFF, 0xFF, 0x01, 0x02, 0x00, 0xFE }; /* should be FC */
    St3215_ParseResult_t r = DrvSt3215_ParseAck(frame, sizeof(frame), 1, NULL);
    ASSERT_EQ_INT(r, ST3215_PARSE_BAD_CHECKSUM, "parse_ack_bad_checksum");
    TEST_PASS("parse_ack_bad_checksum");
}

static void test_parse_ack_bad_id(void)
{
    const uint8_t frame[] = { 0xFF, 0xFF, 0x02, 0x02, 0x00, 0xFB };
    St3215_ParseResult_t r = DrvSt3215_ParseAck(frame, sizeof(frame), 1, NULL);
    ASSERT_EQ_INT(r, ST3215_PARSE_BAD_ID, "parse_ack_bad_id");
    TEST_PASS("parse_ack_bad_id");
}

static void test_parse_feedback_nominal(void)
{
    const uint8_t frame[] = {
        0xFF, 0xFF, 0x01, 0x11, 0x00,
        0x00, 0x08, 0x64, 0x00, 0xC8, 0x00,    /* pos, spd, load */
        0x4A, 0x23,                             /* volt, temp */
        0x00, 0x00,                             /* reserved 64-65 */
        0x00,                                   /* moving */
        0x00, 0x00,                             /* reserved 67-68 */
        0x32, 0x00,                             /* current */
        0x70                                    /* checksum */
    };
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    uint8_t err = 0xFF;
    St3215_ParseResult_t r = DrvSt3215_ParseFeedback(frame, sizeof(frame),
                                                     1, 12345U, &fb, &err);
    ASSERT_EQ_INT(r,              ST3215_PARSE_OK, "parse_fb_nominal result");
    ASSERT_EQ_INT(err,             0,              "parse_fb_nominal err");
    ASSERT_EQ_INT(fb.position,    2048,            "fb position");
    ASSERT_EQ_INT(fb.speed,       100,             "fb speed");
    ASSERT_EQ_INT(fb.load,        200,             "fb load");
    ASSERT_EQ_INT(fb.voltage_dv,  74,              "fb voltage");
    ASSERT_EQ_INT(fb.temp_c,      35,              "fb temp");
    ASSERT_EQ_INT(fb.moving,      0,               "fb moving");
    ASSERT_EQ_INT(fb.current,     50,              "fb current");
    ASSERT_EQ_INT(fb.timestamp_ms, 12345,          "fb timestamp");
    TEST_PASS("parse_feedback_nominal");
}

static void test_parse_feedback_negative_position(void)
{
    /* Position = -100 -> wire 0x8064 -> L=0x64, H=0x80
     * Nominal sum = 0x18F (low byte 0x8F).
     * Pos bytes change from 0x00 0x08 (sum 0x08) to 0x64 0x80 (sum 0xE4).
     * Delta = +0xDC. New low byte = 0x8F + 0xDC = 0x16B -> 0x6B.
     * checksum = ~0x6B = 0x94 */
    const uint8_t frame[] = {
        0xFF, 0xFF, 0x01, 0x11, 0x00,
        0x64, 0x80, 0x64, 0x00, 0xC8, 0x00,
        0x4A, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00,
        0x94
    };
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    St3215_ParseResult_t r = DrvSt3215_ParseFeedback(frame, sizeof(frame),
                                                     1, 0U, &fb, NULL);
    ASSERT_EQ_INT(r,           ST3215_PARSE_OK, "neg_pos parse result");
    ASSERT_EQ_INT(fb.position, -100,            "neg_pos value");
    TEST_PASS("parse_feedback_negative_position");
}

static void test_parse_feedback_negative_load(void)
{
    /* Load = -300 -> wire 0x052C -> L=0x2C, H=0x05
     * Nominal load bytes 0xC8 0x00 (sum 0xC8). New 0x2C 0x05 (sum 0x31). Delta -0x97.
     * New low byte = 0x8F - 0x97 = -0x08 -> 0xF8.
     * checksum = ~0xF8 = 0x07 */
    const uint8_t frame[] = {
        0xFF, 0xFF, 0x01, 0x11, 0x00,
        0x00, 0x08, 0x64, 0x00, 0x2C, 0x05,
        0x4A, 0x23, 0x00, 0x00, 0x00, 0x00, 0x00, 0x32, 0x00,
        0x07
    };
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    St3215_ParseResult_t r = DrvSt3215_ParseFeedback(frame, sizeof(frame),
                                                     1, 0U, &fb, NULL);
    ASSERT_EQ_INT(r,        ST3215_PARSE_OK, "neg_load parse result");
    ASSERT_EQ_INT(fb.load, -300,             "neg_load value");
    TEST_PASS("parse_feedback_negative_load");
}

/* =============================================================================
 * ClassifyMotion tests
 * ============================================================================= */

static void test_classify_arrived(void)
{
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position = 2010; fb.speed = 2; fb.load = 50; fb.moving = 0;
    St3215_MotionStatus_t s = DrvSt3215_ClassifyMotion(&fb, 2000, NULL);
    ASSERT_EQ_INT(s, ST3215_MOTION_ARRIVED, "classify_arrived");
    TEST_PASS("classify_arrived");
}

static void test_classify_moving(void)
{
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position = 1500; fb.speed = 200; fb.load = 100; fb.moving = 1;
    St3215_MotionStatus_t s = DrvSt3215_ClassifyMotion(&fb, 2000, NULL);
    ASSERT_EQ_INT(s, ST3215_MOTION_MOVING, "classify_moving");
    TEST_PASS("classify_moving");
}

static void test_classify_stalled(void)
{
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position = 1500; fb.speed = 0; fb.load = 100; fb.moving = 0;
    St3215_MotionStatus_t s = DrvSt3215_ClassifyMotion(&fb, 2000, NULL);
    ASSERT_EQ_INT(s, ST3215_MOTION_STALLED, "classify_stalled");
    TEST_PASS("classify_stalled");
}

static void test_classify_overload(void)
{
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position = 2000; fb.speed = 0; fb.load = 900; fb.moving = 1;
    St3215_MotionStatus_t s = DrvSt3215_ClassifyMotion(&fb, 2000, NULL);
    ASSERT_EQ_INT(s, ST3215_MOTION_OVERLOAD, "classify_overload");
    TEST_PASS("classify_overload");
}

/* Holding a payload against gravity: at target, stationary, but high load.
 * Must classify as ARRIVED so the upper FSM does not strip torque. */
static void test_classify_arrived_under_load(void)
{
    St3215_Feedback_t fb;
    memset(&fb, 0, sizeof(fb));
    fb.position = 2010; fb.speed = 2; fb.load = 900; fb.moving = 0;
    St3215_MotionStatus_t s = DrvSt3215_ClassifyMotion(&fb, 2000, NULL);
    ASSERT_EQ_INT(s, ST3215_MOTION_ARRIVED, "classify_arrived_under_load");
    TEST_PASS("classify_arrived_under_load");
}

/* =============================================================================
 * Lifecycle / online tests
 * ============================================================================= */

static void test_lifecycle_online(void)
{
    DrvSt3215_Context_t ctx;
    AraStatus_t s = DrvSt3215_Init(&ctx, 1);
    ASSERT_EQ_INT(s, ARA_OK, "init ok");
    ASSERT_EQ_INT(DrvSt3215_IsOnline(&ctx, 100U), 0, "fresh ctx not online");

    DrvSt3215_NoteIoOk(&ctx, 100U);
    ASSERT_EQ_INT(DrvSt3215_IsOnline(&ctx, 200U), 1, "after-ok online");
    ASSERT_EQ_INT(ctx.fail_count, 0U,                "fail_count reset");

    DrvSt3215_NoteIoFail(&ctx, ST3215_ERRBIT_VOLTAGE);
    ASSERT_EQ_INT(ctx.fail_count, 1U,                              "fail_count++");
    ASSERT_EQ_INT(ctx.last_error_bits, ST3215_ERRBIT_VOLTAGE,      "last_error");

    /* Still online (within OFFLINE_MS) */
    ASSERT_EQ_INT(DrvSt3215_IsOnline(&ctx, 200U), 1, "still online after fail");

    /* Past offline threshold */
    ASSERT_EQ_INT(DrvSt3215_IsOnline(&ctx, 100U + ST3215_OFFLINE_MS + 1U), 0,
                  "offline after timeout");
    TEST_PASS("lifecycle_online");
}

static void test_init_rejects_bad_id(void)
{
    DrvSt3215_Context_t ctx;
    ASSERT_EQ_INT(DrvSt3215_Init(&ctx, 0),    ARA_ERR_PARAM, "reject id=0");
    ASSERT_EQ_INT(DrvSt3215_Init(&ctx, 0xFE), ARA_ERR_PARAM, "reject broadcast");
    ASSERT_EQ_INT(DrvSt3215_Init(NULL,   1),  ARA_ERR_PARAM, "reject null ctx");
    TEST_PASS("init_rejects_bad_id");
}

/* =============================================================================
 * Main
 * ============================================================================= */

int main(void)
{
    printf("=== drv_st3215 byte-level conformance test ===\n");

    printf("\n[Encoding]\n");
    test_encode_ping_id1();
    test_encode_ping_id42();
    test_encode_read_feedback_id1();
    test_encode_write_byte_torque_enable();
    test_encode_write_word_max_angle();
    test_encode_write_byte_lock_open();
    test_encode_writepos_pos_2048_zero();
    test_encode_writepos_negative();
    test_encode_writepos_max();

    printf("\n[Parsing]\n");
    test_parse_ack_ok();
    test_parse_ack_overload();
    test_parse_ack_bad_checksum();
    test_parse_ack_bad_id();
    test_parse_feedback_nominal();
    test_parse_feedback_negative_position();
    test_parse_feedback_negative_load();

    printf("\n[ClassifyMotion]\n");
    test_classify_arrived();
    test_classify_moving();
    test_classify_stalled();
    test_classify_overload();
    test_classify_arrived_under_load();

    printf("\n[Lifecycle]\n");
    test_lifecycle_online();
    test_init_rejects_bad_id();

    printf("\n=== Result: %d passed, %d failed ===\n", g_pass, g_fail);
    return (g_fail == 0) ? 0 : 1;
}
