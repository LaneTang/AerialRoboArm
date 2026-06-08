/**
 * @file app_control.c
 * @brief L5 ControlTask implementation (demo_v7).
 */

#include "app_control.h"
#include "datahub.h"
#include "debug_request.h"

#include "task_rc.h"
#include "task_vision.h"
#include "task_arbiter.h"
#include "task_manipulator.h"
#include "task_motion.h"

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os2.h"

#include <string.h>

/* =============================================================================
 * Internal state
 * ============================================================================= */

typedef enum {
    CTRL_PHASE_BOOT = 0,
    CTRL_PHASE_SERVO_PING,
    CTRL_PHASE_SERVO_CONFIG,
    CTRL_PHASE_SERVO_ENABLE,
    CTRL_PHASE_RUNNING,
    CTRL_PHASE_FAULT
} ControlPhase_t;

static osThreadId_t     s_control_handle    = NULL;
static volatile uint32_t s_heartbeat_ms     = 0U;
static ControlPhase_t    s_phase            = CTRL_PHASE_BOOT;
static uint32_t          s_phase_enter_ms   = 0U;
static uint32_t          s_ping_attempts    = 0U;

/* Latest artefacts (kept file-static so we can publish partial snapshots
 * during bring-up phases). */
static RcControlData_t  s_rc;
static VisionIntent_t   s_vis;
static ArbiterOutput_t  s_arb;
static MotionCmd_t      s_mcmd;
static MotionState_t    s_mstate;

/* Force-mode bring-up state. When s_force_active is true, step_running
 * skips Arbiter/Manipulator and drives TaskMotion_Update directly with
 * s_force_angle_deg. Toggled exclusively via DBG_REQ_FORCE_GOTO_ANGLE. */
static bool    s_force_active     = false;
static int16_t s_force_angle_deg  = 0;

/* PTK end-effector force overrides. -1 = follow arbiter; 0..180 = lock to
 * that angle. Independent from s_force_active so operator can fix the
 * gripper while still flying the main arm via RC. */
static int16_t s_ptk_gripper_force_deg = -1;
static int16_t s_ptk_roll_force_deg    = -1;

/* Two-step reset latch. ControlTask owns this so arbiter can stay a pure
 * function. Set to true when arbiter reports a latching fault (SD active,
 * RC loss); cleared when arbiter signals fault_reset_consumed. */
static bool    s_fault_latched    = false;

/* =============================================================================
 * Helpers
 * ============================================================================= */

static void enter_phase(ControlPhase_t p, uint32_t now_ms)
{
    s_phase          = p;
    s_phase_enter_ms = now_ms;
}

static AraLedPattern_t led_for_phase(ControlPhase_t p, AraLedPattern_t arb_led)
{
    switch (p) {
    case CTRL_PHASE_BOOT:
    case CTRL_PHASE_SERVO_PING:
    case CTRL_PHASE_SERVO_CONFIG:
    case CTRL_PHASE_SERVO_ENABLE:
        return LED_PATTERN_BOOT_FAST_BLINK;
    case CTRL_PHASE_FAULT:
        return LED_PATTERN_ERROR_SOS;
    case CTRL_PHASE_RUNNING:
    default:
        return arb_led;
    }
}

static void publish_hub(uint32_t now_ms, uint32_t loop_count)
{
    DataHub_t snap;
    memset(&snap, 0, sizeof(snap));

    snap.current_mode             = (s_phase == CTRL_PHASE_RUNNING) ? s_arb.mode : ARA_MODE_INIT;
    snap.estop_active             = s_arb.estop || (s_phase != CTRL_PHASE_RUNNING);
    snap.led_pattern              = led_for_phase(s_phase, s_arb.led_pattern);
    snap.arbiter_mode             = s_arb.mode;
    snap.arbiter_reason_code      = (uint8_t)s_arb.reason_code;

    snap.servo_target_angle_deg   = (int16_t)(s_arb.target_angle_deg * 10);
    snap.servo_target_speed       = s_arb.target_speed;
    snap.servo_target_acc         = s_arb.target_acc;
    snap.servo_torque_request     = s_arb.torque_request;
    snap.end_roll_deg             = s_arb.roll_degree;
    snap.end_gripper_deg          = s_arb.gripper_angle;

    if (s_mstate.feedback_valid) {
        snap.servo_position_angle_deg = (int16_t)(s_mstate.feedback.angle_deg * 10.0f);
        snap.servo_load               = s_mstate.feedback.current_ma;
        snap.servo_voltage_dv         = (uint8_t)(s_mstate.feedback.voltage_mv / 100U);
        snap.servo_moving             = s_mstate.is_moving;
    }
    snap.servo_status = s_mstate.servo_online ? ARA_OK : ARA_ERR_DISCONNECTED;

    snap.rc_link_up      = s_rc.is_link_up;
    snap.vision_link_up  = s_vis.target_present;
    snap.rc_last_ok_ms   = now_ms;      /* refined later when link timestamps tracked */
    snap.vision_last_ok_ms = s_vis.last_update_tick_ms;

    snap.last_update_tick_ms = now_ms;
    snap.control_loop_count  = loop_count;

    DataHub_Publish(&snap);
}

static void handle_debug_request(uint32_t now_ms)
{
    DebugRequest_t req;
    if (!DebugRequest_TakePending(&req)) {
        return;
    }
    switch (req.type) {
    case DBG_REQ_MOCK_VISION_TARGET:
        TaskVision_MockInjectTarget((int16_t)req.arg1,
                                    (req.arg2 > 0) ? (uint32_t)req.arg2 : 1000U,
                                    now_ms);
        break;
    case DBG_REQ_MOCK_VISION_CLEAR:
        TaskVision_MockClear();
        break;
    case DBG_REQ_CLEAR_FAULT:
        if (s_phase == CTRL_PHASE_FAULT) {
            s_ping_attempts = 0U;
            enter_phase(CTRL_PHASE_SERVO_PING, now_ms);
        }
        break;
    case DBG_REQ_FORCE_GOTO_ANGLE:
        if (req.arg2 < 0) {
            s_force_active = false;
            /* Reseed incremental accumulator so RC takes over from current position. */
            if (s_mstate.feedback_valid) {
                TaskRc_ReseedIncremental((int16_t)s_mstate.feedback.angle_deg);
            }
        } else {
            int32_t deg = req.arg1;
            if (deg < TASK_MOTION_ANGLE_MIN_DEG) deg = TASK_MOTION_ANGLE_MIN_DEG;
            if (deg > TASK_MOTION_ANGLE_MAX_DEG) deg = TASK_MOTION_ANGLE_MAX_DEG;
            s_force_angle_deg = (int16_t)deg;
            s_force_active    = true;
        }
        break;
    case DBG_REQ_FORCE_PTK_ANGLE: {
        int16_t *slot = (req.arg1 == 0) ? &s_ptk_gripper_force_deg
                                        : &s_ptk_roll_force_deg;
        if (req.arg2 < 0) {
            *slot = -1;
        } else {
            int32_t deg = req.arg2;
            if (deg < 0)   deg = 0;
            if (deg > 180) deg = 180;
            *slot = (int16_t)deg;
        }
        break;
    }
    case DBG_REQ_SWITCH_MODE:
    case DBG_REQ_SET_ESTOP:
    case DBG_REQ_CALIBRATE_ZERO:
    default:
        /* Forwarded into Arbiter input injection in a later iteration. */
        break;
    }
}

/* =============================================================================
 * Bring-up phase steps
 * ============================================================================= */

static void step_boot(uint32_t now_ms)
{
    /* One-tick settle, then try to ping servo. */
    (void)now_ms;
    enter_phase(CTRL_PHASE_SERVO_PING, now_ms);
    s_ping_attempts = 0U;
}

static void step_ping(uint32_t now_ms)
{
#if TASK_MOTION_USE_MOCK
    /* Mock mode: skip right through to RUNNING. */
    enter_phase(CTRL_PHASE_RUNNING, now_ms);
#elif TASK_MOTION_SKIP_PING_FOR_BENCH
    /* Bench mode: HX8 / UC01 not powered, only PTK end-effector under test.
     * Pretend main arm is reachable so manipulator FSM is allowed to drive
     * PA0/PA1. Caller must NOT issue MANUAL/AUTO motion commands. */
    enter_phase(CTRL_PHASE_SERVO_CONFIG, now_ms);
#else
    /* Real mode: probe servo via TaskMotion_Update (ServoMonitor read). */
    MotionCmd_t probe = {
        .torque_on          = false,
        .target_angle_deg   = 0.0f,
        .velocity_deg_per_s = 0.0f,
        .t_acc_ms           = 0U,
        .t_dec_ms           = 0U,
        .power_mw           = 0U,
        .force_update       = true,
    };
    MotionState_t ms;
    TaskMotion_Update(&probe, now_ms, &ms);
    if (ms.servo_online) {
        enter_phase(CTRL_PHASE_SERVO_CONFIG, now_ms);
    } else {
        s_ping_attempts++;
        if (s_ping_attempts > 150U) {   /* 3 s at 50 Hz */
            enter_phase(CTRL_PHASE_FAULT, now_ms);
        }
    }
#endif
}

static void step_config(uint32_t now_ms)
{
    /* Servo configuration is currently a no-op pass-through.
     *
     * step_ping above has already issued one torque-off WritePos +
     * ReadFeedback round-trip and confirmed servo_online; that doubles
     * as a liveness + ID/baud sanity check. Anything beyond that
     * (MODE / MIN / MAX / TORQUE_LIMIT / LOCK validation) would require:
     *   - new READ-byte encode/parse helpers in drv_st3215
     *   - a per-register readback against expected defaults
     *   - a bring-up policy decision: read-only check, or auto-rewrite
     *     EEPROM (which has limited write cycles)
     *
     * For demo_v7 the policy is: physically inspect / set EEPROM via the
     * vendor utility once before first power-up. If a future regression
     * makes this insufficient, escalate this stub to A2 in the bring-up
     * notes (drv_st3215 read helpers + per-register validation).
     */
    enter_phase(CTRL_PHASE_SERVO_ENABLE, now_ms);
}

static void step_enable(uint32_t now_ms)
{
    enter_phase(CTRL_PHASE_RUNNING, now_ms);
}

static void step_fault(uint32_t now_ms)
{
    (void)now_ms;
    /* Idle loop. Exits via DBG_REQ_CLEAR_FAULT. */
}

static void step_running_force(uint32_t now_ms)
{
    s_mcmd.torque_on          = true;
    s_mcmd.target_angle_deg   = (float)s_force_angle_deg;
    s_mcmd.velocity_deg_per_s = TASK_MOTION_DEFAULT_VELOCITY;
    s_mcmd.t_acc_ms           = TASK_MOTION_DEFAULT_T_ACC_MS;
    s_mcmd.t_dec_ms           = TASK_MOTION_DEFAULT_T_DEC_MS;
    s_mcmd.power_mw           = TASK_MOTION_DEFAULT_POWER_MW;
    s_mcmd.force_update       = false;

    TaskMotion_Update(&s_mcmd, now_ms, &s_mstate);

    TaskVision_Update(now_ms, &s_vis);
}

static void step_running_normal(uint32_t now_ms)
{
    TaskVision_Update(now_ms, &s_vis);

    ArbiterInput_t in = {
        .rc              = &s_rc,
        .vision          = &s_vis,
        .servo_online    = s_mstate.servo_online,
        .tick_ms         = now_ms,
        .prev_mode       = s_arb.mode,
        .vision_stale_ms = ARBITER_DEFAULT_VISION_STALE_MS,
        .fault_latched   = s_fault_latched,
    };
    TaskArbiter_Decide(&in, &s_arb);

    /* Two-step reset side effects: when arbiter consumes sys_reset_pulse,
     * reseed CH1 incremental accumulator to current servo position so
     * MANUAL re-entry starts from wherever the arm physically is. */
    if (s_arb.fault_reset_consumed) {
        int16_t cur = s_mstate.feedback_valid
                      ? (int16_t)s_mstate.feedback.angle_deg : 0;
        TaskRc_ReseedIncremental(cur);
    }
    s_fault_latched = s_arb.fault_latched_next;

    /* Console PTK force overrides take priority over arbiter for the
     * named end-effector channels only. HX8 main arm path is unchanged. */
    if (s_ptk_gripper_force_deg >= 0) {
        s_arb.gripper_angle = (uint8_t)s_ptk_gripper_force_deg;
    }
    if (s_ptk_roll_force_deg >= 0) {
        s_arb.roll_degree = (uint8_t)s_ptk_roll_force_deg;
    }

    TaskManipulator_Update(&s_arb, &s_mstate, now_ms, &s_mcmd);

    TaskMotion_Update(&s_mcmd, now_ms, &s_mstate);
}

static void step_running(uint32_t now_ms)
{
    if (s_force_active) {
        step_running_force(now_ms);
    } else {
        step_running_normal(now_ms);
    }
}

/* =============================================================================
 * Task entry
 * ============================================================================= */

static void ControlTaskEntry(void *arg)
{
    (void)arg;
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t   loop_cnt  = 0U;

    for (;;) {
        const uint32_t now_ms = osKernelGetTickCount();

        handle_debug_request(now_ms);

        /* RC parsing must run every tick regardless of phase, so channel
         * sniffing and link-health observation work even when servo
         * bring-up fails (or no servo is attached during RC-only tests). */
        TaskRc_Update(now_ms, &s_rc);

        switch (s_phase) {
        case CTRL_PHASE_BOOT:         step_boot(now_ms);    break;
        case CTRL_PHASE_SERVO_PING:   step_ping(now_ms);    break;
        case CTRL_PHASE_SERVO_CONFIG: step_config(now_ms);  break;
        case CTRL_PHASE_SERVO_ENABLE: step_enable(now_ms);  break;
        case CTRL_PHASE_RUNNING:      step_running(now_ms); break;
        case CTRL_PHASE_FAULT:        step_fault(now_ms);   break;
        default:
            enter_phase(CTRL_PHASE_FAULT, now_ms);
            break;
        }

        publish_hub(now_ms, loop_cnt);
        loop_cnt++;

        s_heartbeat_ms = now_ms;
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(ARA_PERIOD_CONTROL_MS));
    }
}

/* =============================================================================
 * Lifecycle API
 * ============================================================================= */

void App_Control_InitDeps(void)
{
    DebugRequest_Init();
    DataHub_Init();

    TaskRc_Init();
    TaskVision_Init();
    TaskManipulator_Init();
    TaskMotion_Init();

    memset(&s_rc,     0, sizeof(s_rc));
    memset(&s_vis,    0, sizeof(s_vis));
    memset(&s_arb,    0, sizeof(s_arb));
    memset(&s_mcmd,   0, sizeof(s_mcmd));
    memset(&s_mstate, 0, sizeof(s_mstate));

    s_phase          = CTRL_PHASE_BOOT;
    s_phase_enter_ms = 0U;
    s_heartbeat_ms   = 0U;
    s_fault_latched  = false;
}

void App_Control_Init(void)
{
    const osThreadAttr_t attr = {
        .name       = "ControlTask",
        .stack_size = ARA_STACK_CONTROL_BYTES,
        .priority   = ARA_PRIO_CONTROL,
    };
    s_control_handle = osThreadNew(ControlTaskEntry, NULL, &attr);
}

uint32_t App_Control_GetHeartbeatMs(void)
{
    return s_heartbeat_ms;
}

bool App_Control_GetForceState(int16_t *out_angle_deg)
{
    if ((out_angle_deg != NULL) && s_force_active) {
        *out_angle_deg = s_force_angle_deg;
    }
    return s_force_active;
}
