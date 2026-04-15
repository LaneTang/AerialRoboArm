/**
 * @file app_testbench.c
 * @brief Dual-thread on-board demo testbench implementation for ARA project.
 * @note  Strictly C99. Reuses formal L4 runnables and formal DataHub bridge.
 *        This module is intended for demo bring-up under the same dual-thread
 *        architecture as the production application.
 */

#include "app_testbench.h"
#include "datahub.h"
#include "task_motion.h"
#include "task_rc.h"
#include "task_manipulator.h"
#include "bsp_uart.h"
#include "dev_status.h"
#include "mod_actuator.h"
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ============================================================================
 * 1. Testbench Modes / Configuration
 * ========================================================================== */

/**
 * @brief Top-level testbench mode.
 */
typedef enum {
    TESTBENCH_MODE_MENU = 0,     /**< Main menu / quiet state. */
    TESTBENCH_MODE_MOD_DEBUG,    /**< Module debug: CH1 direct analog BLDC control. */
    TESTBENCH_MODE_BLDC_3POS,    /**< BLDC three-target closed-loop demo. */
    TESTBENCH_MODE_RC_MANUAL,    /**< ELRS MANUAL demo. */
    TESTBENCH_MODE_AUTO_FIXED    /**< AUTO fixed-flow demo. */
} TestbenchMode_t;

/**
 * @brief Number of low-frequency ticks used to issue a short Motion INIT pulse.
 * @note  2 ticks * 20ms = 40ms. INIT is used only as a reset pulse, not as a
 *        long blocking initialization window.
 */
#define TESTBENCH_MODE_ENTRY_INIT_PULSE_TICKS (2U)

/**
 * @brief Periodic log interval in low-frequency ticks.
 * @note  5 ticks * 20ms = 100ms.
 */
#define TESTBENCH_LOG_INTERVAL_TICKS          (5U)

/**
 * @brief BLDC three-target demo hold duration at each target.
 * @note  50 ticks * 20ms = 1s.
 */
#define TESTBENCH_BLDC_HOLD_TICKS             (50U)

/**
 * @brief BLDC three-target reach tolerance in degree.
 */
#define TESTBENCH_BLDC_REACH_TOL_DEG          (3U)

/**
 * @brief Module debug CH1 deadband in percent.
 */
#define TESTBENCH_MODDBG_CH1_DEADBAND_PCT     (8)

/**
 * @brief Module debug CH1 max degree step per 20ms tick.
 * @note  ch1_percent / 25 => [-4, +4] deg per tick.
 */
#define TESTBENCH_MODDBG_CH1_DIVISOR          (25)

/**
 * @brief Demo joint mechanical zero angle in AS5600 raw domain.
 * @note  Keep this aligned with task_manipulator.c.
 */
#define TESTBENCH_DEMO_JOINT_ZERO_RAW         (850U)

/**
 * @brief Positive mechanical direction in raw domain.
 * @note  Keep this aligned with task_manipulator.c.
 */
#define TESTBENCH_DEMO_POSITIVE_DIR           (1)

/**
 * @brief Demo park angle in degree.
 */
#define TESTBENCH_DEMO_PARK_DEG               (15U)

/**
 * @brief Demo pick angle in degree.
 */
#define TESTBENCH_DEMO_PICK_DEG               (120U)

/**
 * @brief Demo retract angle in degree.
 */
#define TESTBENCH_DEMO_RETRACT_DEG            (30U)

/**
 * @brief Manual/debug minimum joint angle in degree.
 */
#define TESTBENCH_DEMO_MIN_DEG                (0U)

/**
 * @brief Manual/debug maximum joint angle in degree.
 */
#define TESTBENCH_DEMO_MAX_DEG                (130U)

/* ============================================================================
 * 2. Static Thread Handles / Shared State
 * ========================================================================== */

/**
 * @brief Handle of the high-frequency physical thread.
 */
static TaskHandle_t s_tb_high_freq_task_handle = NULL;

/**
 * @brief Handle of the low-frequency physical thread.
 */
static TaskHandle_t s_tb_low_freq_task_handle = NULL;

/**
 * @brief Current testbench mode.
 */
static TestbenchMode_t s_tb_mode = TESTBENCH_MODE_MENU;

/**
 * @brief Monotonic low-frequency tick counter.
 */
static uint32_t s_tb_low_freq_tick = 0U;

/**
 * @brief Remaining forced-reset ticks after mode entry.
 */
static uint8_t s_tb_mode_reset_ticks = 0U;

/**
 * @brief One-shot snapshot request flag.
 */
static bool s_tb_print_snapshot_req = false;

/**
 * @brief Whether the current mode is waiting for Motion to become ready.
 */
static bool s_tb_wait_motion_ready = false;

/**
 * @brief Module-debug direct joint target in degree.
 */
static uint16_t s_tb_moddbg_target_deg = TESTBENCH_DEMO_PARK_DEG;

/**
 * @brief BLDC three-target demo current target index.
 */
static uint8_t s_tb_bldc_target_idx = 0U;

/**
 * @brief BLDC three-target demo hold counter.
 */
static uint32_t s_tb_bldc_hold_ticks = 0U;

/**
 * @brief Shared actuator instance owned by testbench for non-manipulator modes.
 */
static ModActuator_Context_t s_tb_actuator_ctx;

/**
 * @brief Whether the shared actuator instance has been initialized.
 */
static bool s_tb_actuator_initialized = false;

/**
 * @brief Fixed target list of BLDC three-target demo.
 */
static const uint16_t s_tb_bldc_targets_deg[3] = {
        TESTBENCH_DEMO_PARK_DEG,
        TESTBENCH_DEMO_PICK_DEG,
        TESTBENCH_DEMO_RETRACT_DEG
};

/* ============================================================================
 * 3. Private Helper Functions
 * ========================================================================== */

/**
 * @brief  Compute absolute value of a signed 32-bit integer.
 * @param  value Input value.
 * @return Absolute value.
 */
static int32_t Testbench_Abs32(int32_t value)
{
    return (value >= 0) ? value : (-value);
}

/**
 * @brief  Wrap an arbitrary signed angle into AS5600 raw domain [0, 4095].
 * @param  angle Input angle in extended integer domain.
 * @return Wrapped raw-angle value.
 */
static uint16_t Testbench_WrapRawAngle(int32_t angle)
{
    while (angle < 0) {
        angle += 4096;
    }

    while (angle >= 4096) {
        angle -= 4096;
    }

    return (uint16_t)angle;
}

/**
 * @brief  Convert mechanical degree into raw encoder counts.
 * @param  degree Mechanical angle in degree.
 * @return Equivalent raw-angle offset.
 */
static uint16_t Testbench_DegToRaw(uint16_t degree)
{
    return (uint16_t)(((uint32_t)degree * 4096U) / 360U);
}

/**
 * @brief  Convert a demo mechanical angle into absolute raw-angle target.
 * @param  degree Mechanical angle in degree.
 * @return Absolute AS5600 raw target.
 */
static uint16_t Testbench_DemoAngleToRaw(uint16_t degree)
{
    int32_t target;

    target = (int32_t)TESTBENCH_DEMO_JOINT_ZERO_RAW +
             ((int32_t)TESTBENCH_DEMO_POSITIVE_DIR * (int32_t)Testbench_DegToRaw(degree));

    return Testbench_WrapRawAngle(target);
}

/**
 * @brief  Compute shortest signed raw-angle error.
 * @param  target Target raw angle.
 * @param  actual Actual raw angle.
 * @return Signed shortest-path error in raw counts.
 */
static int32_t Testbench_CalcShortestPathError(uint16_t target, uint16_t actual)
{
    int32_t error = (int32_t)target - (int32_t)actual;

    if (error > 2048) {
        error -= 4096;
    } else if (error < -2048) {
        error += 4096;
    }

    return error;
}

/**
 * @brief  Clamp an unsigned 16-bit value into inclusive range.
 * @param  value Input value.
 * @param  min_value Minimum allowed value.
 * @param  max_value Maximum allowed value.
 * @return Clamped value.
 */
static uint16_t Testbench_ClampU16(uint16_t value, uint16_t min_value, uint16_t max_value)
{
    if (value < min_value) {
        return min_value;
    }

    if (value > max_value) {
        return max_value;
    }

    return value;
}

/**
 * @brief  Map RC aux knob permille to roll degree.
 * @param  aux_permille Aux knob value in [0, 1000].
 * @return Servo degree in [0, 180].
 */
static uint8_t Testbench_MapAuxToRollDegree(uint16_t aux_permille)
{
    if (aux_permille > 1000U) {
        aux_permille = 1000U;
    }

    return (uint8_t)(((uint32_t)aux_permille * 180U) / 1000U);
}

/**
 * @brief  Check whether current joint angle is within demo tolerance.
 * @param  p_motion_state Pointer to motion feedback.
 * @param  target_deg Target mechanical degree.
 * @param  tol_deg Allowed error in degree.
 * @return true if considered reached, otherwise false.
 */
static bool Testbench_IsJointReachedDeg(const DataHub_State_t *p_motion_state,
                                        uint16_t target_deg,
                                        uint16_t tol_deg)
{
    uint16_t target_raw;
    uint16_t tol_raw;
    int32_t error;

    if (p_motion_state == NULL) {
        return false;
    }

    if (p_motion_state->foc_status != ARA_OK) {
        return false;
    }

    target_raw = Testbench_DemoAngleToRaw(target_deg);
    tol_raw = Testbench_DegToRaw(tol_deg);
    error = Testbench_CalcShortestPathError(target_raw, p_motion_state->current_foc_angle);

    return (Testbench_Abs32(error) <= (int32_t)tol_raw);
}

/**
 * @brief  Convert testbench mode to readable string.
 * @param  mode Testbench mode.
 * @return Constant string name.
 */
static const char *Testbench_GetModeName(TestbenchMode_t mode)
{
    switch (mode) {
        case TESTBENCH_MODE_MENU:
            return "MENU";
        case TESTBENCH_MODE_MOD_DEBUG:
            return "TB0_MODULE_DEBUG";
        case TESTBENCH_MODE_BLDC_3POS:
            return "TB1_BLDC_3TARGET";
        case TESTBENCH_MODE_RC_MANUAL:
            return "TB2_ELRS_MANUAL";
        case TESTBENCH_MODE_AUTO_FIXED:
            return "TB3_AUTO_FIXED";
        default:
            return "UNKNOWN";
    }
}

/**
 * @brief  Print the top-level main menu.
 */
static void Testbench_PrintMainMenu(void)
{
    BSP_UART_Printf("\r\n=== ARA DEMO TESTBENCH ===\r\n");
    BSP_UART_Printf("0. Module Debug\r\n");
    BSP_UART_Printf("1. BLDC Three-Target Closed-Loop Demo\r\n");
    BSP_UART_Printf("2. ELRS MANUAL Demo\r\n");
    BSP_UART_Printf("3. AUTO Fixed-Flow Demo\r\n");
    BSP_UART_Printf("--------------------------\r\n");
    BSP_UART_Printf("Select: ");
}

/**
 * @brief  Print mode-entry banner.
 * @param  mode Entered mode.
 */
static void Testbench_PrintModeBanner(TestbenchMode_t mode)
{
    BSP_UART_Printf("\r\n[TB] Enter %s\r\n", Testbench_GetModeName(mode));
    BSP_UART_Printf("[TB] q = return to main menu\r\n");
    BSP_UART_Printf("[TB] p = print one-shot snapshot\r\n");

    switch (mode) {
        case TESTBENCH_MODE_MOD_DEBUG:
            BSP_UART_Printf("[TB0] CH1 direct analog controls BLDC target.\r\n");
            BSP_UART_Printf("[TB0] SF controls roll servo. Gripper remains safe-open.\r\n");
            break;

        case TESTBENCH_MODE_BLDC_3POS:
            BSP_UART_Printf("[TB1] Sequence: +15 -> +120 -> +30 deg\r\n");
            break;

        case TESTBENCH_MODE_RC_MANUAL:
            BSP_UART_Printf("[TB2] Use ELRS MANUAL controls.\r\n");
            BSP_UART_Printf("[TB2] SA/SD/SC/SF/CH1 follow formal RC semantic chain.\r\n");
            break;

        case TESTBENCH_MODE_AUTO_FIXED:
            BSP_UART_Printf("[TB3] Wait in park, then toggle SA to AUTO to start fixed flow.\r\n");
            break;

        case TESTBENCH_MODE_MENU:
        default:
            break;
    }
}

static void Testbench_EnterMode(TestbenchMode_t new_mode)
{
    s_tb_mode = new_mode;
    s_tb_mode_reset_ticks = (new_mode == TESTBENCH_MODE_MENU) ? 0U : TESTBENCH_MODE_ENTRY_INIT_PULSE_TICKS;
    s_tb_wait_motion_ready = (new_mode == TESTBENCH_MODE_MENU) ? false : true;
    s_tb_print_snapshot_req = true;

    if (new_mode == TESTBENCH_MODE_MOD_DEBUG) {
        s_tb_moddbg_target_deg = TESTBENCH_DEMO_PARK_DEG;
    } else if (new_mode == TESTBENCH_MODE_BLDC_3POS) {
        s_tb_bldc_target_idx = 0U;
        s_tb_bldc_hold_ticks = 0U;
    } else if ((new_mode == TESTBENCH_MODE_RC_MANUAL) ||
               (new_mode == TESTBENCH_MODE_AUTO_FIXED))
    {
        TaskManipulator_Init();
    }

    if (new_mode == TESTBENCH_MODE_MENU) {
        Testbench_PrintMainMenu();
    } else {
        Testbench_PrintModeBanner(new_mode);
    }
}

/**
 * @brief  Ensure shared actuator instance is initialized for non-manipulator modes.
 */
static void Testbench_EnsureActuatorInit(void)
{
    if (!s_tb_actuator_initialized) {
        (void)ModActuator_Init(&s_tb_actuator_ctx);
        s_tb_actuator_initialized = true;
    }
}

/**
 * @brief  Apply safe non-manipulator end-effector pose.
 * @param  gripper_percent Gripper percentage in [0, 100].
 * @param  roll_degree Roll degree in [0, 180].
 */
static void Testbench_CommandAuxActuators(uint8_t gripper_percent, uint8_t roll_degree)
{
    Testbench_EnsureActuatorInit();
    (void)ModActuator_SetGripper(&s_tb_actuator_ctx, gripper_percent);
    (void)ModActuator_SetRoll(&s_tb_actuator_ctx, roll_degree);
}

/**
 * @brief  Guard mode entry by issuing a short INIT pulse and then waiting until
 *         Motion reports ready.
 * @param  p_motion_state Pointer to latest motion feedback.
 * @param  p_out_cmd Output command.
 * @return true if the entry guard is still active and the caller must not run
 *         the actual sub-task yet; false if the mode may proceed normally.
 */
static bool Testbench_RunModeEntryGuard(const DataHub_State_t *p_motion_state,
                                        DataHub_Cmd_t *p_out_cmd)
{
    if ((p_motion_state == NULL) || (p_out_cmd == NULL)) {
        return true;
    }

    if (s_tb_mode_reset_ticks > 0U) {
        p_out_cmd->sys_mode = ARA_MODE_INIT;
        p_out_cmd->emergency_stop = false;
        p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(TESTBENCH_DEMO_PARK_DEG);

        s_tb_mode_reset_ticks--;
        return true;
    }

    if (s_tb_wait_motion_ready) {
        if (!p_motion_state->motion_ready) {
            p_out_cmd->sys_mode = ARA_MODE_IDLE;
            p_out_cmd->emergency_stop = false;
            p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(TESTBENCH_DEMO_PARK_DEG);
            return true;
        }

        s_tb_wait_motion_ready = false;
        BSP_UART_Printf("[TB] Motion ready, sub-task released.\r\n");
    }

    return false;
}

/**
 * @brief  Print compact state snapshot.
 * @param  p_cmd Latest downlink command.
 * @param  p_motion_state Latest motion feedback.
 * @param  p_rc Pointer to formal RC semantic data, or NULL.
 * @param  p_dbg Pointer to debug RC analog data, or NULL.
 */
static void Testbench_PrintSnapshot(const DataHub_Cmd_t *p_cmd,
                                    const DataHub_State_t *p_motion_state,
                                    const RcControlData_t *p_rc,
                                    const RcDebugAnalogData_t *p_dbg)
{
    if ((p_cmd == NULL) || (p_motion_state == NULL)) {
        return;
    }

    BSP_UART_Printf("[TB] mode=%s cmd_mode=%d estop=%d tgt=%u cur=%u vel=%d foc=%d\r\n",
                    Testbench_GetModeName(s_tb_mode),
                    (int)p_cmd->sys_mode,
                    (int)p_cmd->emergency_stop,
                    (unsigned int)p_cmd->target_foc_angle,
                    (unsigned int)p_motion_state->current_foc_angle,
                    (int)p_motion_state->current_velocity,
                    (int)p_motion_state->foc_status);

    if (p_rc != NULL) {
        BSP_UART_Printf("[TB] rc: link=%d req=%d estop=%d arm=%d grip=%d aux=%u rst=%d\r\n",
                        (int)p_rc->is_link_up,
                        (int)p_rc->req_mode,
                        (int)p_rc->estop_state,
                        (int)p_rc->arm_cmd,
                        (int)p_rc->gripper_cmd,
                        (unsigned int)p_rc->aux_knob_val,
                        (int)p_rc->sys_reset_pulse);
    }

    if (p_dbg != NULL) {
        BSP_UART_Printf("[TB] dbg: link=%d req=%d estop=%d ch1=%d aux=%u rst=%d tgt_deg=%u\r\n",
                        (int)p_dbg->is_link_up,
                        (int)p_dbg->req_mode,
                        (int)p_dbg->estop_state,
                        (int)p_dbg->ch1_percent,
                        (unsigned int)p_dbg->aux_knob_val,
                        (int)p_dbg->sys_reset_pulse,
                        (unsigned int)s_tb_moddbg_target_deg);
    }
}

/**
 * @brief  Update onboard status LED according to current testbench state.
 * @param  p_cmd Latest downlink command.
 */
static void Testbench_UpdateStatusLed(const DataHub_Cmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    if (p_cmd->emergency_stop || (p_cmd->sys_mode == ARA_MODE_ERROR)) {
        if ((s_tb_low_freq_tick % 2U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else if (s_tb_mode == TESTBENCH_MODE_AUTO_FIXED) {
        if ((s_tb_low_freq_tick % 10U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else if (s_tb_mode == TESTBENCH_MODE_MENU) {
        if ((s_tb_low_freq_tick % 50U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else {
        if ((s_tb_low_freq_tick % 20U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    }
}

/**
 * @brief  Poll UART console and handle one-byte commands.
 */
static void Testbench_PollConsole(void)
{
    uint8_t ch;

    while (BSP_UART_Read(BSP_UART_DEBUG, &ch, 1U) == 1U) {

        if ((ch == '\r') || (ch == '\n')) {
            continue;
        }

        if (s_tb_mode == TESTBENCH_MODE_MENU) {
            switch (ch) {
                case '0':
                    Testbench_EnterMode(TESTBENCH_MODE_MOD_DEBUG);
                    break;

                case '1':
                    Testbench_EnterMode(TESTBENCH_MODE_BLDC_3POS);
                    break;

                case '2':
                    Testbench_EnterMode(TESTBENCH_MODE_RC_MANUAL);
                    break;

                case '3':
                    Testbench_EnterMode(TESTBENCH_MODE_AUTO_FIXED);
                    break;

                default:
                    Testbench_PrintMainMenu();
                    break;
            }
        } else {
            switch (ch) {
                case 'q':
                case 'Q':
                    Testbench_EnterMode(TESTBENCH_MODE_MENU);
                    break;

                case 'p':
                case 'P':
                    s_tb_print_snapshot_req = true;
                    break;

                default:
                    break;
            }
        }
    }
}

/**
 * @brief  Execute TB0: module debug with direct analog CH1 control.
 * @param  current_tick_ms Current system time in milliseconds.
 * @param  p_motion_state Latest motion feedback.
 * @param  p_out_cmd Output command toward Motion.
 * @param  p_out_dbg Output debug RC analog data.
 */
static void Testbench_RunModuleDebug(uint32_t current_tick_ms,
                                     const DataHub_State_t *p_motion_state,
                                     DataHub_Cmd_t *p_out_cmd,
                                     RcDebugAnalogData_t *p_out_dbg)
{
    int16_t step_deg;
    uint8_t roll_degree;

    if ((p_motion_state == NULL) || (p_out_cmd == NULL) || (p_out_dbg == NULL)) {
        return;
    }

    if (Testbench_RunModeEntryGuard(p_motion_state, p_out_cmd)) {
        memset(p_out_dbg, 0, sizeof(RcDebugAnalogData_t));
        p_out_dbg->is_link_up = false;
        p_out_dbg->req_mode = (uint8_t)ARA_MODE_IDLE;
        p_out_dbg->estop_state = (uint8_t)ESTOP_ACTIVE;
        p_out_dbg->ch1_percent = 0;
        p_out_dbg->aux_knob_val = 0U;
        p_out_dbg->sys_reset_pulse = false;
        return;
    }

    TaskRc_UpdateDebugAnalog(current_tick_ms, p_out_dbg);

    if (p_out_dbg->sys_reset_pulse) {
        p_out_cmd->sys_mode = ARA_MODE_INIT;
        p_out_cmd->emergency_stop = false;
        p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(TESTBENCH_DEMO_PARK_DEG);
        return;
    }

    if ((!p_out_dbg->is_link_up) || (p_out_dbg->estop_state == (uint8_t)ESTOP_ACTIVE)) {
        p_out_cmd->sys_mode = ARA_MODE_ERROR;
        p_out_cmd->emergency_stop = true;
        p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(s_tb_moddbg_target_deg);
        Testbench_CommandAuxActuators(0U, 0U);
        return;
    }

    step_deg = (int16_t)(p_out_dbg->ch1_percent / TESTBENCH_MODDBG_CH1_DIVISOR);
    if ((p_out_dbg->ch1_percent <= TESTBENCH_MODDBG_CH1_DEADBAND_PCT) &&
        (p_out_dbg->ch1_percent >= (-TESTBENCH_MODDBG_CH1_DEADBAND_PCT)))
    {
        step_deg = 0;
    }

    if (step_deg != 0) {
        int32_t next_target = (int32_t)s_tb_moddbg_target_deg + (int32_t)step_deg;
        if (next_target < (int32_t)TESTBENCH_DEMO_MIN_DEG) {
            next_target = (int32_t)TESTBENCH_DEMO_MIN_DEG;
        } else if (next_target > (int32_t)TESTBENCH_DEMO_MAX_DEG) {
            next_target = (int32_t)TESTBENCH_DEMO_MAX_DEG;
        }
        s_tb_moddbg_target_deg = (uint16_t)next_target;
    }

    roll_degree = Testbench_MapAuxToRollDegree(p_out_dbg->aux_knob_val);

    p_out_cmd->sys_mode = ARA_MODE_MANUAL;
    p_out_cmd->emergency_stop = false;
    p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(s_tb_moddbg_target_deg);

    Testbench_CommandAuxActuators(0U, roll_degree);
}

/**
 * @brief  Execute TB1: BLDC three-target closed-loop demo.
 * @param  p_motion_state Latest motion feedback.
 * @param  p_out_cmd Output command toward Motion.
 */
static void Testbench_RunBldcThreeTarget(const DataHub_State_t *p_motion_state,
                                         DataHub_Cmd_t *p_out_cmd)
{
    uint16_t target_deg;

    if ((p_motion_state == NULL) || (p_out_cmd == NULL)) {
        return;
    }

    if (Testbench_RunModeEntryGuard(p_motion_state, p_out_cmd)) {
        return;
    }

    target_deg = s_tb_bldc_targets_deg[s_tb_bldc_target_idx];

    p_out_cmd->sys_mode = ARA_MODE_MANUAL;
    p_out_cmd->emergency_stop = false;
    p_out_cmd->target_foc_angle = Testbench_DemoAngleToRaw(target_deg);

    Testbench_CommandAuxActuators(0U, 0U);

    if (Testbench_IsJointReachedDeg(p_motion_state,
                                    target_deg,
                                    TESTBENCH_BLDC_REACH_TOL_DEG))
    {
        s_tb_bldc_hold_ticks++;

        if ((s_tb_bldc_hold_ticks >= TESTBENCH_BLDC_HOLD_TICKS) &&
            (s_tb_bldc_target_idx < 2U))
        {
            s_tb_bldc_target_idx++;
            s_tb_bldc_hold_ticks = 0U;
            BSP_UART_Printf("[TB1] advance -> target %u deg\r\n",
                            (unsigned int)s_tb_bldc_targets_deg[s_tb_bldc_target_idx]);
        }
    } else {
        s_tb_bldc_hold_ticks = 0U;
    }
}

/**
 * @brief  Execute TB2: ELRS MANUAL demo.
 * @param  current_tick_ms Current system time in milliseconds.
 * @param  p_motion_state Latest motion feedback.
 * @param  p_out_cmd Output command toward Motion.
 * @param  p_out_rc Output RC semantic data.
 */
static void Testbench_RunRcManual(uint32_t current_tick_ms,
                                  const DataHub_State_t *p_motion_state,
                                  DataHub_Cmd_t *p_out_cmd,
                                  RcControlData_t *p_out_rc)
{
    if ((p_motion_state == NULL) || (p_out_cmd == NULL) || (p_out_rc == NULL)) {
        return;
    }

    if (Testbench_RunModeEntryGuard(p_motion_state, p_out_cmd)) {
        memset(p_out_rc, 0, sizeof(RcControlData_t));
        return;
    }

    TaskRc_Update(current_tick_ms, p_out_rc);

    if (p_out_rc->is_link_up) {
        p_out_rc->req_mode = ARA_MODE_MANUAL;
    }

    TaskManipulator_Update(p_out_rc, p_motion_state, p_out_cmd);
}

/**
 * @brief  Execute TB3: AUTO fixed-flow demo.
 * @param  current_tick_ms Current system time in milliseconds.
 * @param  p_motion_state Latest motion feedback.
 * @param  p_out_cmd Output command toward Motion.
 * @param  p_out_rc Output RC semantic data.
 */
static void Testbench_RunAutoFixed(uint32_t current_tick_ms,
                                   const DataHub_State_t *p_motion_state,
                                   DataHub_Cmd_t *p_out_cmd,
                                   RcControlData_t *p_out_rc)
{
    if ((p_motion_state == NULL) || (p_out_cmd == NULL) || (p_out_rc == NULL)) {
        return;
    }

    if (Testbench_RunModeEntryGuard(p_motion_state, p_out_cmd)) {
        memset(p_out_rc, 0, sizeof(RcControlData_t));
        return;
    }

    TaskRc_Update(current_tick_ms, p_out_rc);
    TaskManipulator_Update(p_out_rc, p_motion_state, p_out_cmd);
}

/**
 * @brief  High-frequency 1kHz physical thread.
 * @param  argument Unused.
 */
static void Thread_Testbench_HighFreq_Ctrl(void *argument)
{
    TickType_t last_wake_time;
    DataHub_Cmd_t in_cmd;
    DataHub_State_t out_state;

    (void)argument;

    memset(&in_cmd, 0, sizeof(DataHub_Cmd_t));
    memset(&out_state, 0, sizeof(DataHub_State_t));

    TaskMotion_Init();

    last_wake_time = xTaskGetTickCount();

    for (;;) {
        DataHub_ReadCmd(&in_cmd);
        TaskMotion_Update(&in_cmd, &out_state);
        DataHub_WriteState(&out_state);

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TESTBENCH_HIGH_FREQ_PERIOD_MS));
    }
}

/**
 * @brief  Low-frequency 50Hz physical thread.
 * @param  argument Unused.
 */
static void Thread_Testbench_LowFreq_Logic(void *argument)
{
    TickType_t last_wake_time;
    uint32_t current_tick_ms;
    DataHub_Cmd_t out_cmd;
    DataHub_State_t motion_state;
    RcControlData_t rc_data;
    RcDebugAnalogData_t rc_dbg;

    (void)argument;

    memset(&out_cmd, 0, sizeof(DataHub_Cmd_t));
    memset(&motion_state, 0, sizeof(DataHub_State_t));
    memset(&rc_data, 0, sizeof(RcControlData_t));
    memset(&rc_dbg, 0, sizeof(RcDebugAnalogData_t));

    TaskRc_Init();
    TaskManipulator_Init();
    Testbench_EnsureActuatorInit();

    last_wake_time = xTaskGetTickCount();
    Testbench_EnterMode(TESTBENCH_MODE_MENU);

    for (;;) {
        s_tb_low_freq_tick++;
        current_tick_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

        Testbench_PollConsole();
        DataHub_ReadState(&motion_state);

        out_cmd.sys_mode = ARA_MODE_IDLE;
        out_cmd.emergency_stop = false;
        out_cmd.target_foc_angle = Testbench_DemoAngleToRaw(TESTBENCH_DEMO_PARK_DEG);

        memset(&rc_data, 0, sizeof(RcControlData_t));
        memset(&rc_dbg, 0, sizeof(RcDebugAnalogData_t));

        switch (s_tb_mode) {
            case TESTBENCH_MODE_MOD_DEBUG:
                Testbench_RunModuleDebug(current_tick_ms,
                                         &motion_state,
                                         &out_cmd,
                                         &rc_dbg);
                break;

            case TESTBENCH_MODE_BLDC_3POS:
                Testbench_RunBldcThreeTarget(&motion_state, &out_cmd);
                break;

            case TESTBENCH_MODE_RC_MANUAL:
                Testbench_RunRcManual(current_tick_ms,
                                      &motion_state,
                                      &out_cmd,
                                      &rc_data);
                break;

            case TESTBENCH_MODE_AUTO_FIXED:
                Testbench_RunAutoFixed(current_tick_ms,
                                       &motion_state,
                                       &out_cmd,
                                       &rc_data);
                break;

            case TESTBENCH_MODE_MENU:
            default:
                Testbench_CommandAuxActuators(0U, 0U);
                break;
        }

        DataHub_WriteCmd(&out_cmd);
        Testbench_UpdateStatusLed(&out_cmd);

        if (((s_tb_low_freq_tick % TESTBENCH_LOG_INTERVAL_TICKS) == 0U) ||
            s_tb_print_snapshot_req)
        {
            if (s_tb_mode == TESTBENCH_MODE_MOD_DEBUG) {
                Testbench_PrintSnapshot(&out_cmd, &motion_state, NULL, &rc_dbg);
            } else if ((s_tb_mode == TESTBENCH_MODE_RC_MANUAL) ||
                       (s_tb_mode == TESTBENCH_MODE_AUTO_FIXED))
            {
                Testbench_PrintSnapshot(&out_cmd, &motion_state, &rc_data, NULL);
            } else {
                Testbench_PrintSnapshot(&out_cmd, &motion_state, NULL, NULL);
            }

            s_tb_print_snapshot_req = false;
        }

        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TESTBENCH_LOW_FREQ_PERIOD_MS));
    }
}

/* ============================================================================
 * 4. Public API
 * ========================================================================== */

void App_Testbench_Init(void)
{
    DataHub_Init();

    (void)xTaskCreate(Thread_Testbench_HighFreq_Ctrl,
                      "tb_ctrl_1k",
                      TESTBENCH_HIGH_FREQ_STACK,
                      NULL,
                      (tskIDLE_PRIORITY + 3U),
                      &s_tb_high_freq_task_handle);

    (void)xTaskCreate(Thread_Testbench_LowFreq_Logic,
                      "tb_logic_50",
                      TESTBENCH_LOW_FREQ_STACK,
                      NULL,
                      (tskIDLE_PRIORITY + 2U),
                      &s_tb_low_freq_task_handle);
}