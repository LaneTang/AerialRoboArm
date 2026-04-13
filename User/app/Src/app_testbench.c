/**
 * @file app_testbench.c
 * @brief Dual-thread on-board testbench implementation for ARA project.
 * @note  Strictly C99. Reuses formal L4 runnables and formal DataHub bridge.
 *        This module is intended for staged bring-up under the same
 *        dual-thread architecture as the production application.
 */

#include "app_testbench.h"
#include "datahub.h"

#include "task_motion.h"
#include "task_rc.h"
#include "task_manipulator.h"
#include "task_mock_vision.h"

#include "bsp_uart.h"
#include "dev_status.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <stddef.h>

/* ============================================================================
 * 1. Testbench Modes / Configuration
 * ========================================================================== */

typedef enum {
    TESTBENCH_MODE_IDLE = 0,
    TESTBENCH_MODE_MOTION_ONLY,
    TESTBENCH_MODE_RC_MANUAL,
    TESTBENCH_MODE_AUTO_MOCK
} TestbenchMode_t;

/**
 * @brief Testbench log verbosity level.
 */
typedef enum {
    TESTBENCH_LOG_BASIC = 0,      /**< Compact one-line status output. */
    TESTBENCH_LOG_VERBOSE         /**< Detailed multi-line module-wise output. */
} TestbenchLogLevel_t;

#define TESTBENCH_LOG_INTERVAL_TICKS        (5U)

#define TESTBENCH_HOME_ANGLE_RAW            (850U)
#define TESTBENCH_MID_ANGLE_RAW             (2100U)
#define TESTBENCH_MAX_ANGLE_RAW             (3700U)

#define TESTBENCH_MOTION_SEGMENT_TICKS      (50U)

/* ============================================================================
 * 2. Static Thread Handles / Shared State
 * ========================================================================== */

static TaskHandle_t s_tb_high_freq_task_handle = NULL;
static TaskHandle_t s_tb_low_freq_task_handle  = NULL;

static volatile TestbenchMode_t     s_tb_mode = TESTBENCH_MODE_IDLE;
static volatile MockScenario_t      s_tb_mock_scenario = MOCK_SCENARIO_HAPPY_PATH;
static volatile bool                s_tb_software_estop = false;
static volatile bool                s_tb_print_snapshot_req = false;
static volatile TestbenchLogLevel_t s_tb_log_level = TESTBENCH_LOG_BASIC;

static TestbenchMode_t     s_tb_last_announced_mode = TESTBENCH_MODE_IDLE;
static bool                s_tb_last_announced_estop = false;
static TestbenchLogLevel_t s_tb_last_announced_log_level = TESTBENCH_LOG_BASIC;

/* ============================================================================
 * 3. Private Helper Functions
 * ========================================================================== */

static const char *Testbench_GetModeName(TestbenchMode_t mode)
{
    switch (mode) {
        case TESTBENCH_MODE_IDLE:        return "IDLE";
        case TESTBENCH_MODE_MOTION_ONLY: return "MOTION_ONLY";
        case TESTBENCH_MODE_RC_MANUAL:   return "RC_MANUAL";
        case TESTBENCH_MODE_AUTO_MOCK:   return "AUTO_MOCK";
        default:                         return "UNKNOWN";
    }
}

static const char *Testbench_GetScenarioName(MockScenario_t scenario)
{
    switch (scenario) {
        case MOCK_SCENARIO_IDLE:         return "IDLE";
        case MOCK_SCENARIO_HAPPY_PATH:   return "HAPPY_PATH";
        case MOCK_SCENARIO_TARGET_LOST:  return "TARGET_LOST";
        case MOCK_SCENARIO_WIFI_JITTER:  return "WIFI_JITTER";
        case MOCK_SCENARIO_PC_ESTOP:     return "PC_ESTOP";
        default:                         return "UNKNOWN";
    }
}

static const char *Testbench_GetLogLevelName(TestbenchLogLevel_t level)
{
    switch (level) {
        case TESTBENCH_LOG_BASIC:   return "BASIC";
        case TESTBENCH_LOG_VERBOSE: return "VERBOSE";
        default:                    return "UNKNOWN";
    }
}

static void Testbench_PrintMenu(void)
{
    BSP_UART_Printf("\r\n=== ARA DUAL-THREAD TESTBENCH ===\r\n");
    BSP_UART_Printf("[h] Print menu\r\n");
    BSP_UART_Printf("[i] IDLE / quiet mode\r\n");
    BSP_UART_Printf("[m] MOTION_ONLY test\r\n");
    BSP_UART_Printf("[r] RC_MANUAL test\r\n");
    BSP_UART_Printf("[a] AUTO_MOCK test\r\n");
    BSP_UART_Printf("[1] Mock scenario: HAPPY_PATH\r\n");
    BSP_UART_Printf("[2] Mock scenario: TARGET_LOST\r\n");
    BSP_UART_Printf("[3] Mock scenario: WIFI_JITTER\r\n");
    BSP_UART_Printf("[4] Mock scenario: PC_ESTOP\r\n");
    BSP_UART_Printf("[e] Toggle software E-Stop\r\n");
    BSP_UART_Printf("[b] BASIC log mode\r\n");
    BSP_UART_Printf("[v] VERBOSE log mode\r\n");
    BSP_UART_Printf("[p] Print one-shot snapshot\r\n");
    BSP_UART_Printf("---------------------------------\r\n> ");
}

static void Testbench_UpdateStatusLed(uint32_t low_freq_tick_counter)
{
    if (s_tb_software_estop) {
        if ((low_freq_tick_counter % 2U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
        return;
    }

    if (s_tb_mode == TESTBENCH_MODE_AUTO_MOCK) {
        if ((low_freq_tick_counter % 10U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else if (s_tb_mode == TESTBENCH_MODE_IDLE) {
        if ((low_freq_tick_counter % 50U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else {
        if ((low_freq_tick_counter % 20U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    }
}

static void Testbench_PrintStateTransitions(void)
{
    if (s_tb_last_announced_mode != s_tb_mode) {
        BSP_UART_Printf("[TB] Mode -> %s\r\n", Testbench_GetModeName((TestbenchMode_t)s_tb_mode));

        if (s_tb_mode == TESTBENCH_MODE_IDLE) {
            BSP_UART_Printf("[TB] IDLE quiet mode: periodic logs suspended. Press 'h' for menu, 'p' for snapshot.\r\n");
        }

        s_tb_last_announced_mode = s_tb_mode;
    }

    if (s_tb_last_announced_estop != s_tb_software_estop) {
        BSP_UART_Printf("[TB] Software E-Stop -> %d\r\n", (int)s_tb_software_estop);
        s_tb_last_announced_estop = s_tb_software_estop;
    }

    if (s_tb_last_announced_log_level != s_tb_log_level) {
        BSP_UART_Printf("[TB] Log Level -> %s\r\n", Testbench_GetLogLevelName((TestbenchLogLevel_t)s_tb_log_level));
        s_tb_last_announced_log_level = s_tb_log_level;
    }
}

static void Testbench_ExecuteConsoleCommand(char cmd)
{
    switch (cmd) {
        case 'h':
        case 'H':
            Testbench_PrintMenu();
            break;

        case 'i':
        case 'I':
            s_tb_mode = TESTBENCH_MODE_IDLE;
            break;

        case 'm':
        case 'M':
            s_tb_mode = TESTBENCH_MODE_MOTION_ONLY;
            break;

        case 'r':
        case 'R':
            s_tb_mode = TESTBENCH_MODE_RC_MANUAL;
            break;

        case 'a':
        case 'A':
            s_tb_mode = TESTBENCH_MODE_AUTO_MOCK;
            break;

        case '1':
            s_tb_mock_scenario = MOCK_SCENARIO_HAPPY_PATH;
            (void)TaskMockVision_SetScenario(MOCK_SCENARIO_HAPPY_PATH);
            BSP_UART_Printf("[TB] Mock Scenario -> HAPPY_PATH\r\n");
            break;

        case '2':
            s_tb_mock_scenario = MOCK_SCENARIO_TARGET_LOST;
            (void)TaskMockVision_SetScenario(MOCK_SCENARIO_TARGET_LOST);
            BSP_UART_Printf("[TB] Mock Scenario -> TARGET_LOST\r\n");
            break;

        case '3':
            s_tb_mock_scenario = MOCK_SCENARIO_WIFI_JITTER;
            (void)TaskMockVision_SetScenario(MOCK_SCENARIO_WIFI_JITTER);
            BSP_UART_Printf("[TB] Mock Scenario -> WIFI_JITTER\r\n");
            break;

        case '4':
            s_tb_mock_scenario = MOCK_SCENARIO_PC_ESTOP;
            (void)TaskMockVision_SetScenario(MOCK_SCENARIO_PC_ESTOP);
            BSP_UART_Printf("[TB] Mock Scenario -> PC_ESTOP\r\n");
            break;

        case 'e':
        case 'E':
            s_tb_software_estop = !s_tb_software_estop;
            break;

        case 'b':
        case 'B':
            s_tb_log_level = TESTBENCH_LOG_BASIC;
            break;

        case 'v':
        case 'V':
            s_tb_log_level = TESTBENCH_LOG_VERBOSE;
            break;

        case 'p':
        case 'P':
            s_tb_print_snapshot_req = true;
            break;

        default:
            break;
    }
}

static void Testbench_PollConsole(void)
{
    uint8_t  rx_buf[32];
    uint16_t rx_len;
    uint16_t i;

    rx_len = BSP_UART_Read(BSP_UART_DEBUG, rx_buf, (uint16_t)sizeof(rx_buf));
    for (i = 0U; i < rx_len; i++) {
        Testbench_ExecuteConsoleCommand((char)rx_buf[i]);
    }
}

static void Testbench_BuildMotionOnlyCommand(uint32_t low_freq_tick_counter,
                                             DataHub_Cmd_t *p_out_cmd)
{
    uint32_t phase;

    if (p_out_cmd == NULL) {
        return;
    }

    p_out_cmd->sys_mode       = ARA_MODE_MANUAL;
    p_out_cmd->emergency_stop = false;

    phase = (low_freq_tick_counter / TESTBENCH_MOTION_SEGMENT_TICKS) % 3U;

    if (phase == 0U) {
        p_out_cmd->target_foc_angle = TESTBENCH_HOME_ANGLE_RAW;
    } else if (phase == 1U) {
        p_out_cmd->target_foc_angle = TESTBENCH_MID_ANGLE_RAW;
    } else {
        p_out_cmd->target_foc_angle = TESTBENCH_MAX_ANGLE_RAW;
    }
}

static void Testbench_BuildRcManualCommand(uint32_t current_tick_ms,
                                           DataHub_Cmd_t *p_out_cmd,
                                           RcControlData_t *p_rc_snapshot)
{
    AraVisionData_t dummy_vision;

    if ((p_out_cmd == NULL) || (p_rc_snapshot == NULL)) {
        return;
    }

    memset(p_rc_snapshot, 0, sizeof(RcControlData_t));
    memset(&dummy_vision, 0, sizeof(dummy_vision));

    TaskRc_Update(current_tick_ms, p_rc_snapshot);

    dummy_vision.seq_id         = 0U;
    dummy_vision.is_tracking    = false;
    dummy_vision.is_grabbable   = false;
    dummy_vision.pc_estop_req   = false;
    dummy_vision.target_roll    = 0;
    dummy_vision.target_dist_mm = 0U;

    TaskManipulator_Update(p_rc_snapshot, &dummy_vision, p_out_cmd);
}

static void Testbench_BuildAutoMockCommand(uint32_t current_tick_ms,
                                           DataHub_Cmd_t *p_out_cmd,
                                           RcControlData_t *p_rc_snapshot,
                                           AraVisionData_t *p_vision_snapshot)
{
    if ((p_out_cmd == NULL) || (p_rc_snapshot == NULL) || (p_vision_snapshot == NULL)) {
        return;
    }

    memset(p_rc_snapshot, 0, sizeof(RcControlData_t));
    memset(p_vision_snapshot, 0, sizeof(AraVisionData_t));

    p_rc_snapshot->is_link_up      = true;
    p_rc_snapshot->req_mode        = ARA_MODE_AUTO;
    p_rc_snapshot->estop_state     = ESTOP_RELEASED;
    p_rc_snapshot->arm_cmd         = ARM_CMD_HOLD;
    p_rc_snapshot->gripper_cmd     = GRIPPER_CMD_STOP;
    p_rc_snapshot->sys_reset_pulse = false;
    p_rc_snapshot->aux_knob_val    = 500U;

#if TESTBENCH_ENABLE_MOCK_VISION
    TaskMockVision_Update(current_tick_ms, p_vision_snapshot);
#else
    p_vision_snapshot->seq_id         = 0U;
    p_vision_snapshot->is_tracking    = false;
    p_vision_snapshot->is_grabbable   = false;
    p_vision_snapshot->pc_estop_req   = false;
    p_vision_snapshot->target_roll    = 0;
    p_vision_snapshot->target_dist_mm = 0U;
#endif

    TaskManipulator_Update(p_rc_snapshot, p_vision_snapshot, p_out_cmd);
}

static void Testbench_PrintBasicStatus(const DataHub_Cmd_t *p_cmd,
                                       const DataHub_State_t *p_motion_state)
{
    if ((p_cmd == NULL) || (p_motion_state == NULL)) {
        return;
    }

    BSP_UART_Printf("[TB] mode=%s scen=%s sestop=%d sys=%d cmd_estop=%d tgt=%u cur=%u vel=%d foc=%d\r\n",
                    Testbench_GetModeName((TestbenchMode_t)s_tb_mode),
                    Testbench_GetScenarioName((MockScenario_t)s_tb_mock_scenario),
                    (int)s_tb_software_estop,
                    (int)p_cmd->sys_mode,
                    (int)p_cmd->emergency_stop,
                    (unsigned int)p_cmd->target_foc_angle,
                    (unsigned int)p_motion_state->current_foc_angle,
                    (int)p_motion_state->current_velocity,
                    (int)p_motion_state->foc_status);
}

static void Testbench_PrintVerboseStatus(uint32_t low_freq_tick_counter,
                                         const DataHub_Cmd_t *p_cmd,
                                         const DataHub_State_t *p_motion_state,
                                         const RcControlData_t *p_rc_snapshot,
                                         const AraVisionData_t *p_vision_snapshot)
{
    int32_t pos_error = 0;

    if ((p_cmd == NULL) || (p_motion_state == NULL)) {
        return;
    }

    pos_error = (int32_t)p_cmd->target_foc_angle - (int32_t)p_motion_state->current_foc_angle;

    BSP_UART_Printf("[TB VERBOSE] tick=%lu mode=%s scen=%s sestop=%d\r\n",
                    (unsigned long)low_freq_tick_counter,
                    Testbench_GetModeName((TestbenchMode_t)s_tb_mode),
                    Testbench_GetScenarioName((MockScenario_t)s_tb_mock_scenario),
                    (int)s_tb_software_estop);

    BSP_UART_Printf("  [CTRL] sys=%d cmd_estop=%d target_raw=%u pos_err=%ld\r\n",
                    (int)p_cmd->sys_mode,
                    (int)p_cmd->emergency_stop,
                    (unsigned int)p_cmd->target_foc_angle,
                    (long)pos_error);

    BSP_UART_Printf("  [SENS] angle_raw=%u vel=%d foc_status=%d\r\n",
                    (unsigned int)p_motion_state->current_foc_angle,
                    (int)p_motion_state->current_velocity,
                    (int)p_motion_state->foc_status);

    if ((s_tb_mode == TESTBENCH_MODE_RC_MANUAL) && (p_rc_snapshot != NULL)) {
        BSP_UART_Printf("  [RC] link=%d req_mode=%d arm=%d grip=%d estop=%d aux=%u reset=%d\r\n",
                        (int)p_rc_snapshot->is_link_up,
                        (int)p_rc_snapshot->req_mode,
                        (int)p_rc_snapshot->arm_cmd,
                        (int)p_rc_snapshot->gripper_cmd,
                        (int)p_rc_snapshot->estop_state,
                        (unsigned int)p_rc_snapshot->aux_knob_val,
                        (int)p_rc_snapshot->sys_reset_pulse);
    }

    if ((s_tb_mode == TESTBENCH_MODE_AUTO_MOCK) && (p_vision_snapshot != NULL)) {
        BSP_UART_Printf("  [VISION] seq=%u tracking=%d grabbable=%d pc_estop=%d roll=%d dist=%u\r\n",
                        (unsigned int)p_vision_snapshot->seq_id,
                        (int)p_vision_snapshot->is_tracking,
                        (int)p_vision_snapshot->is_grabbable,
                        (int)p_vision_snapshot->pc_estop_req,
                        (int)p_vision_snapshot->target_roll,
                        (unsigned int)p_vision_snapshot->target_dist_mm);
    }

    if (s_tb_mode == TESTBENCH_MODE_MOTION_ONLY) {
        BSP_UART_Printf("  [SCRIPT] home=%u mid=%u max=%u segment_ticks=%u\r\n",
                        (unsigned int)TESTBENCH_HOME_ANGLE_RAW,
                        (unsigned int)TESTBENCH_MID_ANGLE_RAW,
                        (unsigned int)TESTBENCH_MAX_ANGLE_RAW,
                        (unsigned int)TESTBENCH_MOTION_SEGMENT_TICKS);
    }
}

static void Testbench_LogStatus(uint32_t low_freq_tick_counter,
                                const DataHub_Cmd_t *p_cmd,
                                const DataHub_State_t *p_motion_state,
                                const RcControlData_t *p_rc_snapshot,
                                const AraVisionData_t *p_vision_snapshot,
                                bool force_print)
{
    if ((p_cmd == NULL) || (p_motion_state == NULL)) {
        return;
    }

    if ((s_tb_mode == TESTBENCH_MODE_IDLE) && (!force_print)) {
        return;
    }

    if ((!force_print) && ((low_freq_tick_counter % TESTBENCH_LOG_INTERVAL_TICKS) != 0U)) {
        return;
    }

    if (s_tb_log_level == TESTBENCH_LOG_VERBOSE) {
        Testbench_PrintVerboseStatus(low_freq_tick_counter,
                                     p_cmd,
                                     p_motion_state,
                                     p_rc_snapshot,
                                     p_vision_snapshot);
    } else {
        Testbench_PrintBasicStatus(p_cmd, p_motion_state);
    }
}

/* ============================================================================
 * 4. Dual Physical Threads
 * ========================================================================== */

static void Thread_Testbench_HighFreq_Ctrl(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TESTBENCH_HIGH_FREQ_PERIOD_MS);

    DataHub_Cmd_t   in_cmd;
    DataHub_State_t out_state;

    (void)argument;

    for (;;)
    {
        DataHub_ReadCmd(&in_cmd);
        TaskMotion_Update(&in_cmd, &out_state);
        DataHub_WriteState(&out_state);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

static void Thread_Testbench_LowFreq_Logic(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(TESTBENCH_LOW_FREQ_PERIOD_MS);

    DataHub_Cmd_t   out_cmd;
    DataHub_State_t motion_state;
    RcControlData_t rc_snapshot;
    AraVisionData_t vision_snapshot;
    uint32_t        low_freq_tick_counter = 0U;
    bool            force_print = false;

    (void)argument;

    for (;;)
    {
        uint32_t current_tick_ms = (uint32_t)xTaskGetTickCount();

        low_freq_tick_counter++;
        force_print = false;

        memset(&out_cmd, 0, sizeof(out_cmd));
        memset(&motion_state, 0, sizeof(motion_state));
        memset(&rc_snapshot, 0, sizeof(rc_snapshot));
        memset(&vision_snapshot, 0, sizeof(vision_snapshot));

        Testbench_PollConsole();
        Testbench_PrintStateTransitions();

        switch (s_tb_mode)
        {
            case TESTBENCH_MODE_MOTION_ONLY:
                Testbench_BuildMotionOnlyCommand(low_freq_tick_counter, &out_cmd);
                break;

            case TESTBENCH_MODE_RC_MANUAL:
                Testbench_BuildRcManualCommand(current_tick_ms, &out_cmd, &rc_snapshot);
                break;

            case TESTBENCH_MODE_AUTO_MOCK:
                Testbench_BuildAutoMockCommand(current_tick_ms,
                                               &out_cmd,
                                               &rc_snapshot,
                                               &vision_snapshot);
                break;

            case TESTBENCH_MODE_IDLE:
            default:
                out_cmd.sys_mode         = ARA_MODE_IDLE;
                out_cmd.emergency_stop   = false;
                out_cmd.target_foc_angle = TESTBENCH_HOME_ANGLE_RAW;
                break;
        }

        if (s_tb_software_estop) {
            out_cmd.sys_mode       = ARA_MODE_ERROR;
            out_cmd.emergency_stop = true;
        }

        DataHub_WriteCmd(&out_cmd);
        DataHub_ReadState(&motion_state);

        if (s_tb_print_snapshot_req) {
            force_print = true;
            s_tb_print_snapshot_req = false;
        }

        Testbench_UpdateStatusLed(low_freq_tick_counter);
        Testbench_LogStatus(low_freq_tick_counter,
                            &out_cmd,
                            &motion_state,
                            &rc_snapshot,
                            &vision_snapshot,
                            force_print);

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

/* ============================================================================
 * 5. Public API
 * ========================================================================== */

void App_Testbench_Init(void)
{
    DataHub_Init();

    TaskMotion_Init();
    TaskRc_Init();
    TaskManipulator_Init();

#if TESTBENCH_ENABLE_MOCK_VISION
    TaskMockVision_Init();
    TaskMockVision_SetScenario(MOCK_SCENARIO_HAPPY_PATH);
#endif

    s_tb_mode                    = TESTBENCH_MODE_IDLE;
    s_tb_mock_scenario           = MOCK_SCENARIO_HAPPY_PATH;
    s_tb_software_estop          = false;
    s_tb_print_snapshot_req      = false;
    s_tb_log_level               = TESTBENCH_LOG_BASIC;
    s_tb_last_announced_mode     = TESTBENCH_MODE_IDLE;
    s_tb_last_announced_estop    = false;
    s_tb_last_announced_log_level = TESTBENCH_LOG_BASIC;

    Testbench_PrintMenu();
    BSP_UART_Printf("[TB] Default boot mode = IDLE quiet mode\r\n");

    xTaskCreate(Thread_Testbench_HighFreq_Ctrl,
                "TbHighFreq",
                TESTBENCH_HIGH_FREQ_STACK,
                NULL,
                configMAX_PRIORITIES - 1,
                &s_tb_high_freq_task_handle);

    xTaskCreate(Thread_Testbench_LowFreq_Logic,
                "TbLowFreq",
                TESTBENCH_LOW_FREQ_STACK,
                NULL,
                configMAX_PRIORITIES - 3,
                &s_tb_low_freq_task_handle);
}