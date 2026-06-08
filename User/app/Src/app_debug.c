/**
 * @file app_debug.c
 * @brief L5 DebugTask: Console input, DataHub status dump, LED render.
 *
 * Minimal skeleton for demo_v7. Existing richer menu logic in the legacy
 * app_testbench.c remains in the tree (compiled out via CMake EXCLUDE in a
 * later phase) to serve as reference while this file grows.
 */

#include "app_debug.h"
#include "app_control.h"
#include "datahub.h"
#include "debug_request.h"
#include "bsp_uart.h"
#include "dev_status.h"
#include "task_motion.h"   /* TASK_MOTION_ANGLE_MIN_DEG / MAX_DEG */
#include "task_rc.h"       /* TaskRc_CopyRawChannels */
#include "task_arbiter.h"  /* ARB_REASON_* enum for human-readable [DBG] */

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

static osThreadId_t s_debug_handle = NULL;

/* =============================================================================
 * Console input
 * ============================================================================= */

/* Force-mode bring-up state mirrored locally so we can apply ±/preset keys
 * without round-tripping through DataHub. Authoritative copy lives in
 * app_control via DBG_REQ_FORCE_GOTO_ANGLE; this is just an echo for the
 * input UI. */
static int16_t s_console_force_angle = 0;

/* Multi-char line buffer used by the 'g <angle>' command. The buffer
 * is only read/written from DebugTask, so no locking is needed. */
#define CONSOLE_LINE_MAX  16
static char     s_line_buf[CONSOLE_LINE_MAX];
static uint8_t  s_line_len = 0;
static bool     s_line_active = false;   /* true after the user presses 'g' */

static bool s_raw_dump_active = false;

static void console_print_help(void)
{
    BSP_UART_Printf(
        "[DBG] keys:\r\n"
        "  v=mock-vision 120deg/3s   c=clear-vision    f=clear-fault\r\n"
        "  e=force ON (lock 0deg)    k=force OFF\r\n"
        "  1=goto 0  2=goto 90  3=goto 180\r\n"
        "  +=+10deg  -=-10deg\r\n"
        "  g <deg>=force HX8 to arbitrary angle, eg 'g 47<enter>'\r\n"
        "  p <ch> <deg>=lock PTK (ch 0=grip,1=roll; deg 0..180; -1=release)\r\n"
        "  P=release both PTK channels back to RC\r\n"
        "  r=raw channel dump on/off\r\n"
        "  ?=this help\r\n");
}

static void console_send_force(int16_t angle_deg, bool enable)
{
    s_console_force_angle = angle_deg;
    DebugRequest_Post(DBG_REQ_FORCE_GOTO_ANGLE,
                      (int32_t)angle_deg,
                      enable ? 0 : -1);
    if (enable) {
        BSP_UART_Printf("[DBG] force ON, tgt=%d deg\r\n", (int)angle_deg);
    } else {
        BSP_UART_Printf("[DBG] force OFF\r\n");
    }
}

static int parse_signed_int(const char *s, uint8_t len, int32_t *out)
{
    if ((s == NULL) || (len == 0U)) return -1;
    uint8_t  i   = 0U;
    int32_t  acc = 0;
    int32_t  sign = 1;
    if (s[0] == '-') { sign = -1; i = 1U; }
    else if (s[0] == '+') { i = 1U; }
    if (i >= len) return -1;
    for (; i < len; i++) {
        if (s[i] < '0' || s[i] > '9') return -1;
        acc = acc * 10 + (s[i] - '0');
        if (acc > 100000) return -1;   /* overflow guard */
    }
    *out = sign * acc;
    return 0;
}

static void console_finish_line(void)
{
    /* Two recognised line commands:
     *   g <angle>          → HX8 force-goto
     *   p <ch> <deg|-1>    → PTK end-effector force (ch: 0=PA0 grip, 1=PA1 roll;
     *                        deg 0..180 lock, -1 release)
     */
    if (s_line_len < 2U) {
        BSP_UART_Printf("[DBG] bad cmd\r\n");
        s_line_len    = 0U;
        s_line_active = false;
        return;
    }

    if (s_line_buf[0] == 'g') {
        uint8_t i = 1U;
        while ((i < s_line_len) && (s_line_buf[i] == ' ')) i++;
        int32_t deg;
        if ((i >= s_line_len) ||
            (parse_signed_int(&s_line_buf[i], (uint8_t)(s_line_len - i), &deg) != 0)) {
            BSP_UART_Printf("[DBG] bad angle, try 'g 47'\r\n");
        } else {
            if (deg < TASK_MOTION_ANGLE_MIN_DEG) deg = TASK_MOTION_ANGLE_MIN_DEG;
            if (deg > TASK_MOTION_ANGLE_MAX_DEG) deg = TASK_MOTION_ANGLE_MAX_DEG;
            console_send_force((int16_t)deg, true);
        }
    } else if (s_line_buf[0] == 'p') {
        uint8_t i = 1U;
        while ((i < s_line_len) && (s_line_buf[i] == ' ')) i++;
        if (i >= s_line_len || (s_line_buf[i] != '0' && s_line_buf[i] != '1')) {
            BSP_UART_Printf("[DBG] bad ch, try 'p 0 90' (0=grip,1=roll)\r\n");
        } else {
            int32_t ch = s_line_buf[i] - '0';
            i++;
            while ((i < s_line_len) && (s_line_buf[i] == ' ')) i++;
            int32_t deg;
            if ((i >= s_line_len) ||
                (parse_signed_int(&s_line_buf[i], (uint8_t)(s_line_len - i), &deg) != 0)) {
                BSP_UART_Printf("[DBG] bad deg, try 'p 0 90' or 'p 1 -1'\r\n");
            } else {
                DebugRequest_Post(DBG_REQ_FORCE_PTK_ANGLE, ch, deg);
                if (deg < 0) {
                    BSP_UART_Printf("[DBG] PTK ch=%ld released to RC\r\n", (long)ch);
                } else {
                    BSP_UART_Printf("[DBG] PTK ch=%ld locked to %ld deg\r\n",
                                    (long)ch, (long)deg);
                }
            }
        }
    } else {
        BSP_UART_Printf("[DBG] bad cmd, try 'g <angle>' or 'p <ch> <deg>'\r\n");
    }

    s_line_len    = 0U;
    s_line_active = false;
}

static void console_handle_line_char(char c)
{
    if ((c == '\r') || (c == '\n')) {
        console_finish_line();
        return;
    }
    if ((c == 0x08) || (c == 0x7F)) {  /* backspace / DEL */
        if (s_line_len > 0U) {
            s_line_len--;
            BSP_UART_Printf("\b \b");
        }
        return;
    }
    if (s_line_len < (CONSOLE_LINE_MAX - 1U)) {
        s_line_buf[s_line_len++] = c;
        /* Echo so the operator can see what they typed. */
        char echo[2] = { c, 0 };
        BSP_UART_Printf("%s", echo);
    }
}

static void console_poll(uint32_t tick_ms)
{
    uint8_t buf[16];
    uint16_t n = BSP_UART_Read(BSP_UART_DEBUG, buf, sizeof(buf));
    for (uint16_t i = 0; i < n; i++) {
        char c = (char)buf[i];

        if (s_line_active) {
            console_handle_line_char(c);
            continue;
        }

        switch (c) {
        case 'v':
            DebugRequest_Post(DBG_REQ_MOCK_VISION_TARGET, 120, 3000);
            BSP_UART_Printf("[DBG] mock vision target 120 deg / 3s\r\n");
            break;
        case 'c':
            DebugRequest_Post(DBG_REQ_MOCK_VISION_CLEAR, 0, 0);
            BSP_UART_Printf("[DBG] mock vision cleared\r\n");
            break;
        case 'f':
            DebugRequest_Post(DBG_REQ_CLEAR_FAULT, 0, 0);
            BSP_UART_Printf("[DBG] clear fault requested\r\n");
            break;
        case 'e': {
            /* Lock force mode at the servo's current physical position so
             * enabling force does NOT command an unwanted slew back to 0
             * deg. After 'e' the servo holds where it already is, then the
             * operator can step from there with +/-/1/2/3/g. */
            DataHub_t hub;
            DataHub_Read(&hub);
            int32_t raw = hub.servo_position_angle_deg;
            int32_t cur = (raw >= 0) ? (raw + 5) / 10 : (raw - 5) / 10;
            if (cur < TASK_MOTION_ANGLE_MIN_DEG) cur = TASK_MOTION_ANGLE_MIN_DEG;
            if (cur > TASK_MOTION_ANGLE_MAX_DEG) cur = TASK_MOTION_ANGLE_MAX_DEG;
            console_send_force((int16_t)cur, true);
            break;
        }
        case 'k':
            console_send_force(s_console_force_angle, false);
            break;
        case '1':
            console_send_force(0, true);
            break;
        case '2':
            console_send_force(90, true);
            break;
        case '3':
            console_send_force(180, true);
            break;
        case '+': {
            int32_t a = (int32_t)s_console_force_angle + 10;
            if (a > TASK_MOTION_ANGLE_MAX_DEG) a = TASK_MOTION_ANGLE_MAX_DEG;
            console_send_force((int16_t)a, true);
            break;
        }
        case '-': {
            int32_t a = (int32_t)s_console_force_angle - 10;
            if (a < TASK_MOTION_ANGLE_MIN_DEG) a = TASK_MOTION_ANGLE_MIN_DEG;
            console_send_force((int16_t)a, true);
            break;
        }
        case 'g':
            s_line_active = true;
            s_line_len    = 0U;
            s_line_buf[s_line_len++] = 'g';
            BSP_UART_Printf("g");
            break;
        case 'p':
            s_line_active = true;
            s_line_len    = 0U;
            s_line_buf[s_line_len++] = 'p';
            BSP_UART_Printf("p");
            break;
        case 'P':
            DebugRequest_Post(DBG_REQ_FORCE_PTK_ANGLE, 0, -1);
            DebugRequest_Post(DBG_REQ_FORCE_PTK_ANGLE, 1, -1);
            BSP_UART_Printf("[DBG] PTK released (both ch back to RC)\r\n");
            break;
        case 'r':
            s_raw_dump_active = !s_raw_dump_active;
            BSP_UART_Printf("[DBG] raw channel dump %s\r\n",
                            s_raw_dump_active ? "ON (1 Hz)" : "OFF");
            break;
        case '?':
            console_print_help();
            break;
        default:
            break;
        }
    }
    (void)tick_ms;
}

/* =============================================================================
 * LED render from DataHub pattern
 * ============================================================================= */

static void led_render(uint32_t tick_ms)
{
    AraLedPattern_t p = DataHub_GetLedPattern();
    bool on = false;
    switch (p) {
    case LED_PATTERN_OFF:               on = false; break;
    case LED_PATTERN_BOOT_FAST_BLINK:   on = ((tick_ms / 100U) & 1U); break;
    case LED_PATTERN_IDLE_SLOW_BLINK:   on = ((tick_ms / 500U) & 1U); break;
    case LED_PATTERN_MANUAL_HEARTBEAT:  on = ((tick_ms % 1000U) < 80U); break;
    case LED_PATTERN_AUTO_SOLID:        on = true; break;
    case LED_PATTERN_ERROR_SOS:         on = ((tick_ms / 150U) & 1U); break;
    default:                             on = false; break;
    }
    DevStatus_LedSet(on);
}

/* =============================================================================
 * Periodic snapshot print
 * ============================================================================= */

static uint32_t s_snapshot_print_counter = 0U;

static void raw_channel_dump(void)
{
    uint16_t ch[DRV_ELRS_MAX_CHANNELS];
    AraStatus_t st = TaskRc_CopyRawChannels(ch, DRV_ELRS_MAX_CHANNELS);
    if (st != ARA_OK) {
        BSP_UART_Printf("[RC] link down, no data\r\n");
        return;
    }
    BSP_UART_Printf("[RC] CH1=%5u CH4=%5u SA=%5u SC=%5u SF=%5u SB=%5u SD=%5u\r\n",
                    (unsigned)ch[0],   /* CH1 main arm stick */
                    (unsigned)ch[3],   /* CH4 gripper stick */
                    (unsigned)ch[4],   /* CH5  SA mode */
                    (unsigned)ch[6],   /* CH7  SC gripper switch */
                    (unsigned)ch[7],   /* CH8  SF roll wheel */
                    (unsigned)ch[8],   /* CH9  SB reset pulse */
                    (unsigned)ch[9]);  /* CH10 SD e-stop */
}

static const char *mode_to_str(uint8_t m)
{
    switch (m) {
    case ARA_MODE_INIT:   return "INIT  ";
    case ARA_MODE_IDLE:   return "IDLE  ";
    case ARA_MODE_MANUAL: return "MANUAL";
    case ARA_MODE_AUTO:   return "AUTO  ";
    case ARA_MODE_ERROR:  return "ERROR ";
    default:              return "?     ";
    }
}

static const char *reason_to_str(uint8_t r)
{
    switch (r) {
    case ARB_REASON_BOOT:                return "BOOT       ";
    case ARB_REASON_ESTOP_OPERATOR:      return "ESTOP_SD   ";
    case ARB_REASON_ESTOP_RC_LOSS:       return "ESTOP_RCLOS";
    case ARB_REASON_MANUAL_RC:           return "MANUAL_RC  ";
    case ARB_REASON_AUTO_VISION_FRESH:   return "AUTO_FRESH ";
    case ARB_REASON_AUTO_VISION_STALE:   return "AUTO_STALE ";
    case ARB_REASON_IDLE_DEFAULT:        return "IDLE       ";
    case ARB_REASON_SERVO_OFFLINE:       return "NO_SERVO   ";
    case ARB_REASON_FAULT_PENDING_RESET: return "WAIT_RESET ";
    default:                             return "?          ";
    }
}

static void periodic_snapshot(void)
{
    s_snapshot_print_counter++;
    if ((s_snapshot_print_counter % 10U) != 0U) {
        return; /* print once per second */
    }
    DataHub_t s;
    DataHub_Read(&s);
    int16_t force_angle = 0;
    bool    force_on    = App_Control_GetForceState(&force_angle);

    int32_t pos_raw = (int32_t)s.servo_position_angle_deg;
    bool    pos_neg = (pos_raw < 0);
    int32_t pos_abs = pos_neg ? -pos_raw : pos_raw;
    int32_t pos_int = pos_abs / 10;
    int32_t pos_frc = pos_abs % 10;

    int32_t tgt_raw = force_on ? ((int32_t)force_angle * 10) : (int32_t)s.servo_target_angle_deg;
    bool    tgt_neg = (tgt_raw < 0);
    int32_t tgt_abs = tgt_neg ? -tgt_raw : tgt_raw;
    int32_t tgt_int = tgt_abs / 10;
    int32_t tgt_frc = tgt_abs % 10;

    BSP_UART_Printf("[DBG] %s %s rc=%d vis=%d pos=%s%d.%d tgt=%s%d.%d load=%d roll=%3u grip=%3u tx=%lu rx=%lu erx=%lu rec=%lu%s\r\n",
                    mode_to_str((uint8_t)s.current_mode),
                    reason_to_str(s.arbiter_reason_code),
                    (int)s.rc_link_up,
                    (int)s.vision_link_up,
                    pos_neg ? "-" : "", (int)pos_int, (int)pos_frc,
                    tgt_neg ? "-" : "", (int)tgt_int, (int)tgt_frc,
                    (int)s.servo_load,
                    (unsigned)s.end_roll_deg,
                    (unsigned)s.end_gripper_deg,
                    (unsigned long)BSP_UART_Fsus_GetTxBytes(),
                    (unsigned long)BSP_UART_Fsus_GetRxBytes(),
                    (unsigned long)BSP_UART_Elrs_GetRxBytes(),
                    (unsigned long)BSP_UART_RxDma_GetRecoveries(),
                    force_on ? " [FORCE]" : "");

    if (s_raw_dump_active) {
        raw_channel_dump();
    }
}

/* =============================================================================
 * Task body
 * ============================================================================= */

static void DebugTaskEntry(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();

    BSP_UART_Printf("\r\n[DBG] ARA demo_v7 up. press '?' for help\r\n");

    for (;;) {
        const uint32_t now_ms = osKernelGetTickCount();
        console_poll(now_ms);
        periodic_snapshot();
        led_render(now_ms);
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARA_PERIOD_DEBUG_MS));
    }
}

/* =============================================================================
 * Lifecycle
 * ============================================================================= */

void App_Debug_InitDeps(void)
{
    /* Nothing pre-scheduler for now. */
}

void App_Debug_Init(void)
{
    const osThreadAttr_t attr = {
        .name       = "DebugTask",
        .stack_size = ARA_STACK_DEBUG_BYTES,
        .priority   = ARA_PRIO_DEBUG,
    };
    s_debug_handle = osThreadNew(DebugTaskEntry, NULL, &attr);
}
