/**
 * @file app_testbench.c
 * @brief Module-test-only dual-thread testbench for ARA project.
 * @note  Strictly C99. This revision only tests modules:
 *        - ELRS init / link / raw channels / semantic decode
 *        - Motor init / FOC align
 *        - Motor open-loop test
 *        - Motor closed-loop test
 *
 *        No demo business flow is implemented here.
 */

#include "app_testbench.h"
#include "task_rc.h"
#include "bsp_uart.h"
#include "dev_status.h"

#include "drv_as5600.h"
#include "drv_bldc_power.h"
#include "alg_voltage_foc.h"
#include "alg_pid.h"
#include "mod_actuator.h"
#include "bsp_pwm.h"

#include "FreeRTOS.h"
#include "task.h"

#include <string.h>

/* ============================================================================
 * 1. Testbench Top-Level Modes
 * ========================================================================== */

typedef enum {
    TESTBENCH_SLOT_MENU = 0,
    TESTBENCH_SLOT_RC,
    TESTBENCH_SLOT_MOTOR_ALIGN,
    TESTBENCH_SLOT_MOTOR_OPEN,
    TESTBENCH_SLOT_MOTOR_CLOSED,
    TESTBENCH_SLOT_RC_MANUAL,
    TESTBENCH_SLOT_ACTUATOR_CALIB,  // <--- 新增 Slot 5
    TESTBENCH_SLOT_RC_AUTO         // <--- 新增 Slot 6
} TestbenchSlot_t;

/* 新增自动流状态枚举 */
typedef enum {
    AUTO_STATE_IDLE = 0,
    AUTO_STATE_GOING_HOME,      // 前往 +15°
    AUTO_STATE_GOING_PICK,      // 前往 +120° (此阶段开启遥测)
    AUTO_STATE_DONE
} AutoSequenceState_t;

typedef enum {
    TESTBENCH_LOGFMT_SEMANTIC = 0,
    TESTBENCH_LOGFMT_RAW
} TestbenchLogFormat_t;

static RcControlData_t s_tb_rc_semantic;
static uint16_t        s_tb_rc_raw_channels[DRV_ELRS_MAX_CHANNELS]; // 旁路原始数据缓存
static int16_t         s_tb_rc_ch1_percent;                         // 算出的 CH1 百分比

/* ============================================================================
 * 新增：二进制遥测协议 (PID 动态响应)
 * ========================================================================== */
#pragma pack(push, 1)
typedef struct {
    uint8_t  header[2]; // 固定包头：0xAA, 0xBB
    int16_t  target;    // 目标角度 (ext_raw)
    int16_t  pos;       // 实际角度 (ext_raw)
    int16_t  uq;        // 输出力矩
    uint8_t  crc;       // 简单的异或校验和
} PidTelemetryFrame_t;
#pragma pack(pop)

static bool s_tb_binary_stream_active = false; // 二进制流开关

/* ============================================================================
 * 2. Motor Bench Internal Types
 * ========================================================================== */

typedef enum {
    TB_MOTOR_CMD_IDLE = 0,
    TB_MOTOR_CMD_ALIGN,
    TB_MOTOR_CMD_OPEN_LOOP,
    TB_MOTOR_CMD_CLOSED_LOOP,
    TB_MOTOR_CMD_ESTOP
} TbMotorCmdType_t;

typedef enum {
    TB_MOTOR_STATE_UNINIT = 0,
    TB_MOTOR_STATE_IDLE,
    TB_MOTOR_STATE_ALIGNING,
    TB_MOTOR_STATE_OPEN_LOOP,
    TB_MOTOR_STATE_CLOSED_LOOP,
    TB_MOTOR_STATE_ERROR
} TbMotorState_t;

typedef struct {
    TbMotorCmdType_t cmd;
    int16_t          open_loop_speed_raw_per_tick;
    int32_t          closed_loop_target_ext_raw;
} TbMotorCmd_t;

typedef struct {
    bool            initialized;
    bool            aligned;
    TbMotorState_t  state;
    uint16_t        raw_angle;
    int32_t         ext_raw;
    int16_t         velocity_raw_per_ms;
    int32_t         open_loop_speed_raw_per_tick;
    int32_t         closed_loop_target_ext_raw;
    int32_t         error_ext;       // 当前误差
    int16_t         pid_out_uq;      // PID 输出值
    int32_t         pid_integral;    // 积分累加值
    int16_t         kp, ki, kd;      // 当前增益参数
    AraStatus_t     status;
} TbMotorStateReport_t;

typedef struct {
    bool               initialized;
    bool               aligned;
    TbMotorState_t     state;
    uint32_t           uptime_ticks;
    uint32_t           state_ticks;
    uint16_t           foc_zero_offset;
    uint16_t           prev_raw_angle;
    int32_t            open_loop_phase_raw;
    int32_t            last_error_ext;
    int16_t            last_pid_out_uq;
    AraStatus_t        status;

    DrvAS5600_Context_t enc_driver;
    DrvBldc_Context_t   motor_driver;
    AlgFoc_Context_t    foc_algo;
    AlgPid_Context_t    pos_pid;

    /* --- 新增：用于彻底解决越界翻转的绝对连续坐标累加器 --- */
    int32_t            continuous_ext_raw;
} TbMotorBenchContext_t;

/* ============================================================================
 * 3. Constants
 * ========================================================================== */

#define TESTBENCH_LOG_INTERVAL_TICKS        (10U)   /* 10 * 20ms = 200ms = 5Hz */

#define TB_MOTOR_HOME_RAW_CALIB             (2200U)
#define TB_MOTOR_WRAP_THRESHOLD_RAW         (800U)
#define TB_MOTOR_EXT_MIN_RAW                (0)
#define TB_MOTOR_EXT_MAX_RAW                ((int32_t)((4096U + TB_MOTOR_WRAP_THRESHOLD_RAW) - TB_MOTOR_HOME_RAW_CALIB))

#define TB_MOTOR_POLE_PAIRS                 (7U)
#define TB_MOTOR_SENSOR_PRIME_TICKS         (20U)
#define TB_MOTOR_ALIGN_VOLTAGE_D_Q15        (16384)
#define TB_MOTOR_ALIGN_DURATION_TICKS       (2000U)

#define TB_MOTOR_OPEN_LOOP_UQ_Q15           (16384)
#define TB_MOTOR_OPEN_LOOP_SPEED_STEP       (1)
#define TB_MOTOR_OPEN_LOOP_SPEED_MAX        (20)

#define TB_MOTOR_HARDCODED_ZERO_OFFSET      (887U)

#define TB_MOTOR_PID_ERR_FULL_SCALE_EXT     (1024)
#define TB_MOTOR_CLOSED_LOOP_UQ_LIMIT_Q15   (12000)
#define TB_MOTOR_PID_POS_KP_Q8_8            (384)
#define TB_MOTOR_PID_POS_KI_Q8_8            (3)
#define TB_MOTOR_PID_POS_KD_Q8_8            (0)
#define TB_MOTOR_PID_INT_MAX                (2560000)
#define TB_MOTOR_CLOSED_LOOP_TARGET_STEP    (32)
#define TB_MOTOR_TARGET_REACHED_THD_EXT     (12)

/* ============================================================================
 * 4. Static Shared Objects
 * ========================================================================== */

static TaskHandle_t s_tb_high_freq_task_handle = NULL;
static TaskHandle_t s_tb_low_freq_task_handle = NULL;

static TestbenchSlot_t s_tb_slot = TESTBENCH_SLOT_MENU;
static TestbenchLogFormat_t s_tb_log_format = TESTBENCH_LOGFMT_SEMANTIC;
static uint32_t s_tb_low_freq_tick = 0U;
static bool s_tb_print_snapshot_req = false;

static TbMotorCmd_t s_tb_motor_cmd;
static TbMotorStateReport_t s_tb_motor_state;

static ModActuator_Context_t s_tb_actuator_ctx; // Actuator 上下文实例

/* 全局控制变量 */
static AutoSequenceState_t s_auto_fsm = AUTO_STATE_IDLE;
static uint32_t s_auto_hold_ticks = 0;
extern bool s_tb_binary_stream_active; // 引用之前定义的二进制流开关

/* ============================================================================
 * 5. Generic Helpers
 * ========================================================================== */

static int32_t Testbench_Abs32(int32_t value)
{
    return (value >= 0) ? value : (-value);
}

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

static int32_t Testbench_ClampExtRaw(int32_t ext_raw)
{
    return ext_raw;
}

/**
 * @brief 将绝对单调坐标 (ext_raw) 映射为 [0, 359] 的物理角度
 * @note  定义 ext_raw = 0 时为 0 度。严格使用定点整数乘除法。
 */
static uint16_t Testbench_MapExtToDegree(int32_t ext_raw)
{
    // 1. 将绝对坐标对一圈的原始量程(4096)取模
    int32_t offset_raw = ext_raw % 4096;

    // 2. 如果是负数（反转了），将其拉回正整数范围
    if (offset_raw < 0) {
        offset_raw += 4096;
    }

    // 3. 映射到 360 度 (定点数计算：先乘后除防精度丢失)
    return (uint16_t)((offset_raw * 360) / 4096);
}

static int16_t Testbench_ClampOpenLoopSpeed(int16_t speed)
{
    if (speed > TB_MOTOR_OPEN_LOOP_SPEED_MAX) {
        return TB_MOTOR_OPEN_LOOP_SPEED_MAX;
    }

    if (speed < (-TB_MOTOR_OPEN_LOOP_SPEED_MAX)) {
        return (int16_t)(-TB_MOTOR_OPEN_LOOP_SPEED_MAX);
    }

    return speed;
}

static int16_t Testbench_CalcVelocity(uint16_t curr, uint16_t prev)
{
    int32_t diff = (int32_t)curr - (int32_t)prev;

    if (diff > 2048) {
        diff -= 4096;
    } else if (diff < -2048) {
        diff += 4096;
    }

    return (int16_t)diff;
}

static int32_t Testbench_MapRawToExt(uint16_t raw_angle)
{
    int32_t raw_unwrapped;

    if (raw_angle < TB_MOTOR_WRAP_THRESHOLD_RAW) {
        raw_unwrapped = (int32_t)raw_angle + 4096;
    } else {
        raw_unwrapped = (int32_t)raw_angle;
    }

    return raw_unwrapped - (int32_t)TB_MOTOR_HOME_RAW_CALIB;
}

static uint16_t Testbench_MapExtToRaw(int32_t ext_raw)
{
    int32_t raw_unwrapped;

    ext_raw = Testbench_ClampExtRaw(ext_raw);
    raw_unwrapped = (int32_t)TB_MOTOR_HOME_RAW_CALIB + ext_raw;

    return Testbench_WrapRawAngle(raw_unwrapped);
}

static int16_t Testbench_ExtErrorToQ15(int32_t error_ext_raw)
{
    int32_t scaled;

    if (error_ext_raw >= TB_MOTOR_PID_ERR_FULL_SCALE_EXT) {
        return (int16_t)32767;
    }

    if (error_ext_raw <= (-TB_MOTOR_PID_ERR_FULL_SCALE_EXT)) {
        return (int16_t)(-32768);
    }

    scaled = (error_ext_raw * 32767) / TB_MOTOR_PID_ERR_FULL_SCALE_EXT;

    if (scaled > 32767) {
        scaled = 32767;
    } else if (scaled < -32768) {
        scaled = -32768;
    }

    return (int16_t)scaled;
}

/* ============================================================================
 * 6. Shared Motor Command / State Accessors
 * ========================================================================== */

static void Testbench_WriteMotorCmd(const TbMotorCmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_tb_motor_cmd = *p_cmd;
    taskEXIT_CRITICAL();
}

static void Testbench_ReadMotorCmd(TbMotorCmd_t *p_cmd)
{
    if (p_cmd == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *p_cmd = s_tb_motor_cmd;
    taskEXIT_CRITICAL();
}

static void Testbench_WriteMotorState(const TbMotorStateReport_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    s_tb_motor_state = *p_state;
    taskEXIT_CRITICAL();
}

static void Testbench_ReadMotorState(TbMotorStateReport_t *p_state)
{
    if (p_state == NULL) {
        return;
    }

    taskENTER_CRITICAL();
    *p_state = s_tb_motor_state;
    taskEXIT_CRITICAL();
}

/* ============================================================================
 * 7. String Helpers
 * ========================================================================== */

static const char *Testbench_GetSlotName(TestbenchSlot_t slot)
{
    switch (slot) {
        case TESTBENCH_SLOT_MENU:
            return "MENU";
        case TESTBENCH_SLOT_RC:
            return "RC";
        case TESTBENCH_SLOT_MOTOR_ALIGN:
            return "MOTOR_ALIGN";
        case TESTBENCH_SLOT_MOTOR_OPEN:
            return "MOTOR_OPEN";
        case TESTBENCH_SLOT_MOTOR_CLOSED:
            return "MOTOR_CLOSED";
        case TESTBENCH_SLOT_RC_MANUAL:
            return "RC_MANUAL_DEMO";
        case TESTBENCH_SLOT_ACTUATOR_CALIB:
            return "ACTUATOR_CALIB";
        case TESTBENCH_SLOT_RC_AUTO:
            return "RC_AUTO_STEP_TEST";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetLogFormatName(TestbenchLogFormat_t fmt)
{
    switch (fmt) {
        case TESTBENCH_LOGFMT_SEMANTIC:
            return "SEMANTIC";
        case TESTBENCH_LOGFMT_RAW:
            return "RAW";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetAraModeName(AraSysMode_t mode)
{
    switch (mode) {
        case ARA_MODE_INIT:
            return "INIT";
        case ARA_MODE_IDLE:
            return "IDLE";
        case ARA_MODE_MANUAL:
            return "MANUAL";
        case ARA_MODE_AUTO:
            return "AUTO";
        case ARA_MODE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetArmCmdName(AraArmCmd_t cmd)
{
    switch (cmd) {
        case ARM_CMD_HOLD:
            return "HOLD";
        case ARM_CMD_EXTEND:
            return "EXTEND";
        case ARM_CMD_RETRACT:
            return "RETRACT";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetGripperCmdName(AraGripperCmd_t cmd)
{
    switch (cmd) {
        case GRIPPER_CMD_STOP:
            return "STOP";
        case GRIPPER_CMD_OPEN:
            return "OPEN";
        case GRIPPER_CMD_CLOSE:
            return "CLOSE";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetEstopName(AraEStopState_t state)
{
    switch (state) {
        case ESTOP_RELEASED:
            return "RELEASED";
        case ESTOP_ACTIVE:
            return "ACTIVE";
        default:
            return "UNKNOWN";
    }
}

static const char *Testbench_GetStatusName(AraStatus_t status)
{
    switch (status) {
        case ARA_OK:
            return "ARA_OK";
        case ARA_ERROR:
            return "ARA_ERROR";
        case ARA_BUSY:
            return "ARA_BUSY";
        case ARA_TIMEOUT:
            return "ARA_TIMEOUT";
        case ARA_ERR_PARAM:
            return "ARA_ERR_PARAM";
        case ARA_ERR_DMA:
            return "ARA_ERR_DMA";
        case ARA_ERR_IO:
            return "ARA_ERR_IO";
        case ARA_ERR_NACK:
            return "ARA_ERR_NACK";
        case ARA_ERR_CRC:
            return "ARA_ERR_CRC";
        case ARA_ERR_DISCONNECTED:
            return "ARA_ERR_DISCONNECTED";
        case ARA_ERR_MATH:
            return "ARA_ERR_MATH";
        default:
            return "ARA_STATUS_UNKNOWN";
    }
}

static const char *Testbench_GetMotorStateName(TbMotorState_t state)
{
    switch (state) {
        case TB_MOTOR_STATE_UNINIT:
            return "UNINIT";
        case TB_MOTOR_STATE_IDLE:
            return "IDLE";
        case TB_MOTOR_STATE_ALIGNING:
            return "ALIGNING";
        case TB_MOTOR_STATE_OPEN_LOOP:
            return "OPEN_LOOP";
        case TB_MOTOR_STATE_CLOSED_LOOP:
            return "CLOSED_LOOP";
        case TB_MOTOR_STATE_ERROR:
            return "ERROR";
        default:
            return "UNKNOWN";
    }
}

/* ============================================================================
 * 8. Menu / Input Helpers
 * ========================================================================== */

static void Testbench_PrintMainMenu(void)
{
    BSP_UART_Printf("\r\n=== ARA MODULE TESTBENCH ===\r\n");
    BSP_UART_Printf("0. RC module monitor\r\n");
    BSP_UART_Printf("1. Motor init / align test\r\n");
    BSP_UART_Printf("2. Motor open-loop test\r\n");
    BSP_UART_Printf("3. Motor closed-loop test\r\n");
    BSP_UART_Printf("4. RC Manual Demo (SC=Mode, CH1=Speed/Pos)\r\n"); // 新增提示
    BSP_UART_Printf("5. Actuator Calib (CH3=Roll, CH8=Gripper)\r\n"); // <--- 新增
    BSP_UART_Printf("6. RC Auto Step Test (CH3=Trigger 15->120 deg)\r\n");
    BSP_UART_Printf("------------------------------\r\n");
    BSP_UART_Printf("h = print menu\r\n");
    BSP_UART_Printf("p = print one-shot snapshot\r\n");
    BSP_UART_Printf("v = toggle log format semantic/raw\r\n");
    BSP_UART_Printf("q = return to menu / idle\r\n");
    BSP_UART_Printf("------------------------------\r\n");
    BSP_UART_Printf("slot commands:\r\n");
    BSP_UART_Printf("a = motor align\r\n");
    BSP_UART_Printf("g = motor run current slot mode\r\n");
    BSP_UART_Printf("s = motor idle stop\r\n");
    BSP_UART_Printf("e = motor estop\r\n");
    BSP_UART_Printf("z = zero speed/target\r\n");
    BSP_UART_Printf("+/- = adjust open-loop speed or closed-loop target\r\n");
    // 【新增】：三个角度测试热键提示
    BSP_UART_Printf("j/k/l = (Closed-loop) Set target to -30/120/210 deg\r\n");
    BSP_UART_Printf("------------------------------\r\n");
    BSP_UART_Printf("log format = %s\r\n", Testbench_GetLogFormatName(s_tb_log_format));
    BSP_UART_Printf("Select: ");
}

static void Testbench_ToggleLogFormat(void)
{
    if (s_tb_log_format == TESTBENCH_LOGFMT_SEMANTIC) {
        s_tb_log_format = TESTBENCH_LOGFMT_RAW;
    } else {
        s_tb_log_format = TESTBENCH_LOGFMT_SEMANTIC;
    }

    BSP_UART_Printf("[TB] log format=%s\r\n", Testbench_GetLogFormatName(s_tb_log_format));
}

static void Testbench_EnterSlot(TestbenchSlot_t new_slot)
{
    TbMotorCmd_t cmd;

    s_tb_slot = new_slot;
    s_tb_print_snapshot_req = true;

    Testbench_ReadMotorCmd(&cmd);

    if (new_slot == TESTBENCH_SLOT_MENU) {
        Testbench_PrintMainMenu();
        return;
    }

    if (new_slot == TESTBENCH_SLOT_MOTOR_OPEN) {
        cmd.cmd = TB_MOTOR_CMD_IDLE;
        cmd.open_loop_speed_raw_per_tick = 0;
        Testbench_WriteMotorCmd(&cmd);
    } else if (new_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
        cmd.cmd = TB_MOTOR_CMD_IDLE;
        cmd.closed_loop_target_ext_raw = 0;
        Testbench_WriteMotorCmd(&cmd);
    }

    BSP_UART_Printf("\r\n[TB] enter slot=%s\r\n", Testbench_GetSlotName(new_slot));
}

static void Testbench_HandleMotorCommandInput(uint8_t ch)
{
    TbMotorCmd_t cmd;

    Testbench_ReadMotorCmd(&cmd);

    switch (ch) {
        case 'a':
        case 'A':
            cmd.cmd = TB_MOTOR_CMD_ALIGN;
            BSP_UART_Printf("[TB] motor cmd=ALIGN\r\n");
            break;

        case 'g':
        case 'G':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN) {
                cmd.cmd = TB_MOTOR_CMD_OPEN_LOOP;
                BSP_UART_Printf("[TB] motor cmd=OPEN_LOOP speed=%d raw_per_tick\r\n",
                                (int)cmd.open_loop_speed_raw_per_tick);
            } else if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.cmd = TB_MOTOR_CMD_CLOSED_LOOP;
                BSP_UART_Printf("[TB] motor cmd=CLOSED_LOOP target_ext=%ld\r\n",
                                (long)cmd.closed_loop_target_ext_raw);
            }
            break;

        case 's':
        case 'S':
            cmd.cmd = TB_MOTOR_CMD_IDLE;
            BSP_UART_Printf("[TB] motor cmd=IDLE\r\n");
            break;

        case 'e':
        case 'E':
            cmd.cmd = TB_MOTOR_CMD_ESTOP;
            BSP_UART_Printf("[TB] motor cmd=ESTOP\r\n");
            break;

        case 'z':
        case 'Z':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN) {
                cmd.open_loop_speed_raw_per_tick = 0;
                BSP_UART_Printf("[TB] open-loop speed=0\r\n");
            } else if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw = 0;
                BSP_UART_Printf("[TB] closed-loop target_ext=0\r\n");
            }
            break;

        case '+':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN) {
                cmd.open_loop_speed_raw_per_tick =
                        Testbench_ClampOpenLoopSpeed((int16_t)(cmd.open_loop_speed_raw_per_tick +
                                                               TB_MOTOR_OPEN_LOOP_SPEED_STEP));
                BSP_UART_Printf("[TB] open-loop speed=%d raw_per_tick\r\n",
                                (int)cmd.open_loop_speed_raw_per_tick);
            } else if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw =
                        Testbench_ClampExtRaw(cmd.closed_loop_target_ext_raw +
                                              TB_MOTOR_CLOSED_LOOP_TARGET_STEP);
                BSP_UART_Printf("[TB] closed-loop target_ext=%ld\r\n",
                                (long)cmd.closed_loop_target_ext_raw);
            }
            break;

        case '-':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN) {
                cmd.open_loop_speed_raw_per_tick =
                        Testbench_ClampOpenLoopSpeed((int16_t)(cmd.open_loop_speed_raw_per_tick -
                                                               TB_MOTOR_OPEN_LOOP_SPEED_STEP));
                BSP_UART_Printf("[TB] open-loop speed=%d raw_per_tick\r\n",
                                (int)cmd.open_loop_speed_raw_per_tick);
            } else if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw =
                        Testbench_ClampExtRaw(cmd.closed_loop_target_ext_raw -
                                              TB_MOTOR_CLOSED_LOOP_TARGET_STEP);
                BSP_UART_Printf("[TB] closed-loop target_ext=%ld\r\n",
                                (long)cmd.closed_loop_target_ext_raw);
            }
            break;

            /* --- 【新增】：快捷角度跳转指令 --- */
        case 'j':
        case 'J':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw = 341;
                BSP_UART_Printf("[TB] target_ext set to 341 (30 deg)\r\n");
            }
            break;

        case 'k':
        case 'K':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw = 1365;
                BSP_UART_Printf("[TB] target_ext set to 1365 (120 deg)\r\n");
            }
            break;

        case 'l':
        case 'L':
            if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
                cmd.closed_loop_target_ext_raw = 2389;
                BSP_UART_Printf("[TB] target_ext set to 2389 (210 deg)\r\n");
            }
            break;

        default:
            break;
    }

    Testbench_WriteMotorCmd(&cmd);
}

static void Testbench_PollConsole(void)
{
    uint8_t ch;

    while (BSP_UART_Read(BSP_UART_DEBUG, &ch, 1U) == 1U) {

        if ((ch == '\r') || (ch == '\n')) {
            continue;
        }

        BSP_UART_Printf("[TB] input echo=%c\r\n", (char)ch);

        switch (ch) {
            case '0':
                Testbench_EnterSlot(TESTBENCH_SLOT_RC);
                break;

            case '1':
                Testbench_EnterSlot(TESTBENCH_SLOT_MOTOR_ALIGN);
                break;

            case '2':
                Testbench_EnterSlot(TESTBENCH_SLOT_MOTOR_OPEN);
                break;

            case '3':
                Testbench_EnterSlot(TESTBENCH_SLOT_MOTOR_CLOSED);
                break;

            case '4':
                Testbench_EnterSlot(TESTBENCH_SLOT_RC_MANUAL);
                break;
            case '5':
                Testbench_EnterSlot(TESTBENCH_SLOT_ACTUATOR_CALIB);
                break;
            case '6':
                Testbench_EnterSlot(TESTBENCH_SLOT_RC_AUTO);
                break;

            case 'q':
            case 'Q':
                Testbench_EnterSlot(TESTBENCH_SLOT_MENU);
                break;

            case 'p':
            case 'P':
                s_tb_print_snapshot_req = true;
                break;

            case 'v':
            case 'V':
                Testbench_ToggleLogFormat();
                break;

            case 'h':
            case 'H':
                Testbench_PrintMainMenu();
                break;

            default:
                if ((s_tb_slot == TESTBENCH_SLOT_MOTOR_ALIGN) ||
                    (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN) ||
                    (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED))
                {
                    Testbench_HandleMotorCommandInput(ch);
                }
                break;
        }
    }
}

/* ============================================================================
 * 9. RC Logging
 * ========================================================================== */

static void Testbench_PrintRcSnapshotSemantic(void)
{
    RcControlData_t semantic;
    RcDebugAnalogData_t debug;
    uint16_t raw[DRV_ELRS_MAX_CHANNELS];
    AraStatus_t raw_status;
    bool rc_init;

    memset(&semantic, 0, sizeof(RcControlData_t));
    memset(&debug, 0, sizeof(RcDebugAnalogData_t));
    memset(raw, 0, sizeof(raw));

    TaskRc_Update((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), &semantic);
    TaskRc_UpdateDebugAnalog((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), &debug);
    raw_status = TaskRc_CopyRawChannels(raw, DRV_ELRS_MAX_CHANNELS);
    rc_init = TaskRc_IsInitialized();

    BSP_UART_Printf("[RC] init=%s link=%s raw_copy=%s mode=%s estop=%s arm=%s grip=%s reset=%d aux=%u ch1_pct=%d\r\n",
                    rc_init ? "OK" : "FAIL",
                    semantic.is_link_up ? "UP" : "DOWN",
                    Testbench_GetStatusName(raw_status),
                    Testbench_GetAraModeName(semantic.req_mode),
                    Testbench_GetEstopName(semantic.estop_state),
                    Testbench_GetArmCmdName(semantic.arm_cmd),
                    Testbench_GetGripperCmdName(semantic.gripper_cmd),
                    (int)semantic.sys_reset_pulse,
                    (unsigned int)semantic.aux_knob_val,
                    (int)debug.ch1_percent);
}

/* 增加 Slot 5 专用的打印函数 */
static void Testbench_PrintActuatorCalibSnapshot(void)
{
    RcDebugAnalogData_t debug;
    uint32_t tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    TaskRc_UpdateDebugAnalog(tick, &debug);

    BSP_UART_Printf("[CALIB] LINK:%s | CH3->Roll: %u deg | CH8->Gripper: %u deg\r\n",
                    debug.is_link_up ? "UP" : "DOWN",
                    (unsigned int)debug.roll_angle,
                    (unsigned int)debug.gripper_angle);
}

/* 增加专属日志回显 */
static void Testbench_PrintAutoSnapshot(void) {
    TbMotorStateReport_t motor;
    Testbench_ReadMotorState(&motor);

    BSP_UART_Printf("[AUTO] FSM:%d | CH3:%u | TGT:%ld | CUR:%ld | ERR:%ld | REC:%s\r\n",
                    s_auto_fsm, s_tb_rc_raw_channels[MOD_RC_IDX_CH3],
                    (long)motor.closed_loop_target_ext_raw, (long)motor.ext_raw,
                    (long)motor.error_ext, s_tb_binary_stream_active ? "ON" : "OFF");
}

static void Testbench_PrintRcManualSnapshot(void) {
    TbMotorStateReport_t motor;
    Testbench_ReadMotorState(&motor);

    uint16_t cur_deg = Testbench_MapExtToDegree(motor.ext_raw);
    uint16_t tgt_deg = Testbench_MapExtToDegree(motor.closed_loop_target_ext_raw);

    BSP_UART_Printf("[MANUAL] SYS:%s | LINK:%s | ES:%s | CH1:%d%% | TGT:%u deg | CUR:%u deg | ERR:%ld | UQ:%d\r\n",
                    Testbench_GetAraModeName(s_tb_rc_semantic.req_mode),
                    s_tb_rc_semantic.is_link_up ? "UP" : "DOWN",
                    Testbench_GetEstopName(s_tb_rc_semantic.estop_state),
                    (int)s_tb_rc_ch1_percent,
                    (unsigned int)tgt_deg,
                    (unsigned int)cur_deg,
                    (long)motor.error_ext,
                    (int)motor.pid_out_uq);
}

/**
 * @brief 将 RC 语义直接映射到电机控制指令 (MANUAL 模式演示)
 */
static void Testbench_UpdateRcBridge(void) {
    if (s_tb_slot != TESTBENCH_SLOT_RC_MANUAL) return;

    if (!s_tb_rc_semantic.is_link_up) return;

    TbMotorCmd_t cmd;
    Testbench_ReadMotorCmd(&cmd);

    // 逻辑：SC开关 (Gripper语义) 控制模式
    if (s_tb_rc_semantic.gripper_cmd == GRIPPER_CMD_OPEN) {
        cmd.cmd = TB_MOTOR_CMD_OPEN_LOOP;
        cmd.open_loop_speed_raw_per_tick = (int16_t)(s_tb_rc_ch1_percent / 10);
    } else if (s_tb_rc_semantic.gripper_cmd == GRIPPER_CMD_CLOSE) {
        cmd.cmd = TB_MOTOR_CMD_CLOSED_LOOP;
        // 映射：CH1 [-100, 100] -> [0, 4095] ext_raw
        cmd.closed_loop_target_ext_raw = ((int32_t)s_tb_rc_ch1_percent + 100) * 4095 / 200;
    } else {
        cmd.cmd = TB_MOTOR_CMD_IDLE;
    }

    Testbench_WriteMotorCmd(&cmd);
}

static void Testbench_PrintRcSnapshotRaw(void)
{
    RcControlData_t semantic;
    uint16_t raw[DRV_ELRS_MAX_CHANNELS];
    AraStatus_t raw_status;
    bool rc_init;

    memset(&semantic, 0, sizeof(RcControlData_t));
    memset(raw, 0, sizeof(raw));

    TaskRc_Update((uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS), &semantic);
    raw_status = TaskRc_CopyRawChannels(raw, DRV_ELRS_MAX_CHANNELS);
    rc_init = TaskRc_IsInitialized();

    BSP_UART_Printf("[RC] raw init=%d link=%d raw_status=%d\r\n",
                    (int)rc_init,
                    (int)semantic.is_link_up,
                    (int)raw_status);

    BSP_UART_Printf("[RC] ch01=%u ch02=%u ch03=%u ch04=%u ch05=%u ch06=%u ch07=%u ch08=%u\r\n",
                    (unsigned int)raw[0], (unsigned int)raw[1], (unsigned int)raw[2], (unsigned int)raw[3],
                    (unsigned int)raw[4], (unsigned int)raw[5], (unsigned int)raw[6], (unsigned int)raw[7]);

    BSP_UART_Printf("[RC] ch09=%u ch10=%u ch11=%u ch12=%u ch13=%u ch14=%u ch15=%u ch16=%u\r\n",
                    (unsigned int)raw[8], (unsigned int)raw[9], (unsigned int)raw[10], (unsigned int)raw[11],
                    (unsigned int)raw[12], (unsigned int)raw[13], (unsigned int)raw[14], (unsigned int)raw[15]);
}

/* ============================================================================
 * 10. Motor Bench Core
 * ========================================================================== */

static void TbMotor_StopPowerStage(TbMotorBenchContext_t *p_ctx)
{
    BSP_PWM_StopAll();
    DrvBldc_Enable(&p_ctx->motor_driver, false);
}

static AraStatus_t TbMotor_ApplyFocOutputs(TbMotorBenchContext_t *p_ctx)
{
    return DrvBldc_SetDuties(&p_ctx->motor_driver,
                             p_ctx->foc_algo.duty_a,
                             p_ctx->foc_algo.duty_b,
                             p_ctx->foc_algo.duty_c);
}

static int16_t TbMotor_RunPositionController(TbMotorBenchContext_t *p_ctx, int32_t pos_error_ext)
{
    int16_t error_q15;

    error_q15 = Testbench_ExtErrorToQ15(pos_error_ext);
    return AlgPid_Compute(&p_ctx->pos_pid, error_q15, 0);
}

static void TbMotor_InitBench(TbMotorBenchContext_t *p_ctx)
{
    AraStatus_t init_status;

    memset(p_ctx, 0, sizeof(TbMotorBenchContext_t));

    p_ctx->state = TB_MOTOR_STATE_UNINIT;
    p_ctx->status = ARA_OK;

    init_status = DrvAS5600_Init(&p_ctx->enc_driver, BSP_I2C_MOTION, NULL);
    if (init_status != ARA_OK) {
        p_ctx->status = init_status;
        p_ctx->state = TB_MOTOR_STATE_ERROR;
        return;
    }

    DrvBldc_Init(&p_ctx->motor_driver, BSP_GPIO_MOTOR_EN);
    AlgFoc_Init(&p_ctx->foc_algo, TB_MOTOR_POLE_PAIRS, BSP_PWM_MAX_DUTY);

    // 直接注入硬件标定零位！
    AlgFoc_SetZeroOffset(&p_ctx->foc_algo, TB_MOTOR_HARDCODED_ZERO_OFFSET);

    // 【保留之前的修复】：因为编码器与电机相序物理方向相反，必须设为 -1
    p_ctx->foc_algo.motor_direction = -1;

    // 欺骗状态机，告诉它系统已经对齐，跳过安全拦截
    p_ctx->aligned = true;

    AlgPid_Init(&p_ctx->pos_pid);
    AlgPid_SetGains(&p_ctx->pos_pid,
                    TB_MOTOR_PID_POS_KP_Q8_8,
                    TB_MOTOR_PID_POS_KI_Q8_8,
                    TB_MOTOR_PID_POS_KD_Q8_8,
                    TB_MOTOR_CLOSED_LOOP_UQ_LIMIT_Q15,
                    TB_MOTOR_PID_INT_MAX); // 此时宏应该是 2560000

    init_status = DrvAS5600_TriggerUpdate(&p_ctx->enc_driver);
    if ((init_status != ARA_OK) && (init_status != ARA_BUSY)) {
        p_ctx->status = init_status;
        p_ctx->state = TB_MOTOR_STATE_ERROR;
        return;
    }

    p_ctx->initialized = true;
    p_ctx->state = TB_MOTOR_STATE_IDLE;
    p_ctx->status = ARA_OK;
}

static void TbMotor_UpdateBench(TbMotorBenchContext_t *p_ctx,
                                const TbMotorCmd_t *p_cmd,
                                TbMotorStateReport_t *p_out_state)
{
    uint16_t current_raw;
    int16_t current_vel;
    int32_t current_ext;
    AraStatus_t trigger_status;

    if ((p_ctx == NULL) || (p_cmd == NULL) || (p_out_state == NULL)) {
        return;
    }

    p_ctx->uptime_ticks++;
    p_ctx->state_ticks++;

    current_raw = DrvAS5600_GetRawAngle(&p_ctx->enc_driver);

    /* --- 【修复的核心代码】：基于最短路径增量的绝对位置追踪 --- */
    if (p_ctx->uptime_ticks == 1) {
        // 第一帧：利用旧的映射公式算出一个初始基准位置
        p_ctx->continuous_ext_raw = Testbench_MapRawToExt(current_raw);
        current_vel = 0;
    } else {
        // 后续帧：通过带环形翻转处理的测速函数获取增量，完美跨越 0<->4095
        current_vel = Testbench_CalcVelocity(current_raw, p_ctx->prev_raw_angle);
        p_ctx->continuous_ext_raw += current_vel;
    }

    // 赋值给当前扩展坐标，保证 POS 打印和闭环计算使用的是单调平滑数据
    current_ext = p_ctx->continuous_ext_raw;
    p_ctx->prev_raw_angle = current_raw;
    /* -------------------------------------------------------- */

    trigger_status = DrvAS5600_TriggerUpdate(&p_ctx->enc_driver);
    if ((trigger_status != ARA_OK) && (trigger_status != ARA_BUSY)) {
        p_ctx->status = trigger_status;
        p_ctx->state = TB_MOTOR_STATE_ERROR;
    }

    if (!p_ctx->initialized) {
        p_ctx->state = TB_MOTOR_STATE_ERROR;
    }

    if (p_ctx->uptime_ticks < TB_MOTOR_SENSOR_PRIME_TICKS) {
        TbMotor_StopPowerStage(p_ctx);
        p_ctx->state = TB_MOTOR_STATE_IDLE;
    } else {
        switch (p_cmd->cmd)
        {
            case TB_MOTOR_CMD_IDLE:
            {
                TbMotor_StopPowerStage(p_ctx);
                p_ctx->state = TB_MOTOR_STATE_IDLE;
                if (p_ctx->status == ARA_OK) {
                    p_ctx->status = ARA_OK;
                }
                break;
            }

            case TB_MOTOR_CMD_ALIGN:
            {
                AraStatus_t apply_status;

                if (!p_ctx->aligned && (p_ctx->state != TB_MOTOR_STATE_ALIGNING)) {
                    p_ctx->state_ticks = 0U;
                    p_ctx->state = TB_MOTOR_STATE_ALIGNING;
                    AlgPid_Reset(&p_ctx->pos_pid);
                }

                if (!p_ctx->aligned) {
                    DrvBldc_Enable(&p_ctx->motor_driver, true);
                    AlgFoc_Run(&p_ctx->foc_algo, 0U, 0, TB_MOTOR_ALIGN_VOLTAGE_D_Q15);
                    apply_status = TbMotor_ApplyFocOutputs(p_ctx);

                    if (apply_status != ARA_OK) {
                        p_ctx->status = apply_status;
                        p_ctx->state = TB_MOTOR_STATE_ERROR;
                        break;
                    }

                    p_ctx->status = ARA_OK;

                    if (p_ctx->state_ticks >= TB_MOTOR_ALIGN_DURATION_TICKS) {
                        p_ctx->foc_zero_offset = current_raw;
                        AlgFoc_SetZeroOffset(&p_ctx->foc_algo, p_ctx->foc_zero_offset);
                        p_ctx->aligned = true;
                        TbMotor_StopPowerStage(p_ctx);
                        p_ctx->state = TB_MOTOR_STATE_IDLE;
                    }
                } else {
                    TbMotor_StopPowerStage(p_ctx);
                    p_ctx->state = TB_MOTOR_STATE_IDLE;
                }
                break;
            }

            case TB_MOTOR_CMD_OPEN_LOOP:
            {
                AraStatus_t apply_status;

                if (!p_ctx->aligned) {
                    TbMotor_StopPowerStage(p_ctx);
                    p_ctx->state = TB_MOTOR_STATE_IDLE;
                    break;
                }

                if (p_cmd->open_loop_speed_raw_per_tick == 0) {
                    TbMotor_StopPowerStage(p_ctx);
                    p_ctx->state = TB_MOTOR_STATE_IDLE;
                    break;
                }

                if (p_ctx->state != TB_MOTOR_STATE_OPEN_LOOP) {
                    p_ctx->open_loop_phase_raw = (int32_t)current_raw;
                    p_ctx->state = TB_MOTOR_STATE_OPEN_LOOP;
                }

                p_ctx->open_loop_phase_raw += (int32_t)p_cmd->open_loop_speed_raw_per_tick;

                DrvBldc_Enable(&p_ctx->motor_driver, true);
                AlgFoc_Run(&p_ctx->foc_algo,
                           Testbench_WrapRawAngle(p_ctx->open_loop_phase_raw),
                           TB_MOTOR_OPEN_LOOP_UQ_Q15,
                           0);
                apply_status = TbMotor_ApplyFocOutputs(p_ctx);

                if (apply_status != ARA_OK) {
                    p_ctx->status = apply_status;
                    p_ctx->state = TB_MOTOR_STATE_ERROR;
                } else {
                    p_ctx->status = ARA_OK;
                }
                break;
            }

            case TB_MOTOR_CMD_CLOSED_LOOP:
            {
                AraStatus_t apply_status;
                int32_t pos_error_ext;
                int16_t torque_uq;
                int32_t target_ext;

                if (!p_ctx->aligned) {
                    TbMotor_StopPowerStage(p_ctx);
                    p_ctx->state = TB_MOTOR_STATE_IDLE;
                    break;
                }

                target_ext = Testbench_ClampExtRaw(p_cmd->closed_loop_target_ext_raw);
                pos_error_ext = target_ext - current_ext;
                torque_uq = TbMotor_RunPositionController(p_ctx, pos_error_ext);

                DrvBldc_Enable(&p_ctx->motor_driver, true);

                /* --- 【保留修复】：去掉负号，使用正常的级联输出 --- */
                AlgFoc_Run(&p_ctx->foc_algo, current_raw, torque_uq, 0);
                /* ------------------------------------------------- */

                apply_status = TbMotor_ApplyFocOutputs(p_ctx);

                if (apply_status != ARA_OK) {
                    p_ctx->status = apply_status;
                    p_ctx->state = TB_MOTOR_STATE_ERROR;
                } else {
                    p_ctx->status = ARA_OK;
                    p_ctx->state = TB_MOTOR_STATE_CLOSED_LOOP;
                }

                p_ctx->last_error_ext = pos_error_ext;
                p_ctx->last_pid_out_uq = torque_uq;
                break;
            }

            case TB_MOTOR_CMD_ESTOP:
            default:
            {
                TbMotor_StopPowerStage(p_ctx);
                p_ctx->status = ARA_ERROR;
                p_ctx->state = TB_MOTOR_STATE_ERROR;
                break;
            }
        }
    }

    p_out_state->initialized = p_ctx->initialized;
    p_out_state->aligned = p_ctx->aligned;
    p_out_state->state = p_ctx->state;
    p_out_state->raw_angle = current_raw;
    p_out_state->ext_raw = current_ext;  /* POS 的数据源，现在已绝对连续 */
    p_out_state->velocity_raw_per_ms = current_vel;
    p_out_state->open_loop_speed_raw_per_tick = p_cmd->open_loop_speed_raw_per_tick;
    p_out_state->closed_loop_target_ext_raw = Testbench_ClampExtRaw(p_cmd->closed_loop_target_ext_raw);
    p_out_state->status = p_ctx->status;

    p_out_state->error_ext    = p_ctx->last_error_ext;
    p_out_state->pid_out_uq   = p_ctx->last_pid_out_uq;
    p_out_state->pid_integral = p_ctx->pos_pid.integral_sum;
    p_out_state->kp           = p_ctx->pos_pid.Kp;
    p_out_state->ki           = p_ctx->pos_pid.Ki;
    p_out_state->kd           = p_ctx->pos_pid.Kd;
}

static void Testbench_UpdateActuatorCalib(void)
{
    if (s_tb_slot != TESTBENCH_SLOT_ACTUATOR_CALIB) return;

    RcDebugAnalogData_t debug;
    uint32_t tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);

    TaskRc_UpdateDebugAnalog(tick, &debug);

    if (!debug.is_link_up) return;

    /* 架构师注：
     * 这里我们故意“越权”调用了底层的 DrvServo_SetAngle，而不是上层的 ModActuator_SetGripper。
     * 因为 SetGripper 是基于 0-100% 的业务插值，而我们现在的目的是“标定”，
     * 必须绕过插值，直接向舵机发送 0~180 的原始度数，来探测它到底在多少度会卡住！
     */
    DrvServo_SetAngle(&s_tb_actuator_ctx.servo_roll, debug.roll_angle);
    DrvServo_SetAngle(&s_tb_actuator_ctx.servo_gripper, debug.gripper_angle);
}

/* ============================================================================
 * 11. Motor Logging
 * ========================================================================== */

static void Testbench_PrintMotorSnapshotSemantic(void)
{
    TbMotorStateReport_t st;

    Testbench_ReadMotorState(&st);

    BSP_UART_Printf("[MOTOR] init=%s aligned=%s run=%s raw=%u ext=%ld vel=%d open_spd=%d target_ext=%ld status=%s\r\n",
                    st.initialized ? "OK" : "FAIL",
                    st.aligned ? "YES" : "NO",
                    Testbench_GetMotorStateName(st.state),
                    (unsigned int)st.raw_angle,
                    (long)st.ext_raw,
                    (int)st.velocity_raw_per_ms,
                    (int)st.open_loop_speed_raw_per_tick,
                    (long)st.closed_loop_target_ext_raw,
                    Testbench_GetStatusName(st.status));
}
static void Testbench_PrintClosedLoopSnapshotSemantic(void) {
    TbMotorStateReport_t st;
    Testbench_ReadMotorState(&st);

    // 调用新增的映射函数获取物理角度
    uint16_t degree = Testbench_MapExtToDegree(st.ext_raw);

    // 格式：[PID] POS/TGT | ERR | UQ/I | P/I/D | DEG | 状态
    BSP_UART_Printf("[PID] POS:%ld TGT:%ld | ERR:%ld | UQ:%d I:%ld | P:%d I:%d D:%d | DEG:%u | %s\r\n",
                    (long)st.ext_raw, (long)st.closed_loop_target_ext_raw,
                    (long)st.error_ext, (int)st.pid_out_uq, (long)st.pid_integral,
                    (int)st.kp, (int)st.ki, (int)st.kd,
                    (unsigned int)degree,
                    Testbench_GetStatusName(st.status));
}

static void Testbench_PrintMotorSnapshotRaw(void)
{
    TbMotorStateReport_t st;

    Testbench_ReadMotorState(&st);

    BSP_UART_Printf("[MOTOR] raw init=%d aligned=%d state=%d raw=%u ext=%ld vel=%d open_spd=%d tgt=%ld status=%d\r\n",
                    (int)st.initialized,
                    (int)st.aligned,
                    (int)st.state,
                    (unsigned int)st.raw_angle,
                    (long)st.ext_raw,
                    (int)st.velocity_raw_per_ms,
                    (int)st.open_loop_speed_raw_per_tick,
                    (long)st.closed_loop_target_ext_raw,
                    (int)st.status);
}

/* ============================================================================
 * 12. Snapshot Routing / LED
 * ========================================================================== */

static void Testbench_PrintCurrentSnapshot(void)
{
    if (s_tb_slot == TESTBENCH_SLOT_RC) {
        if (s_tb_log_format == TESTBENCH_LOGFMT_SEMANTIC) {
            Testbench_PrintRcSnapshotSemantic();
        } else {
            Testbench_PrintRcSnapshotRaw();
        }
    } else if (s_tb_slot == TESTBENCH_SLOT_MOTOR_CLOSED) {
        /* 闭环模式专属路由 */
        if (s_tb_log_format == TESTBENCH_LOGFMT_SEMANTIC) {
            Testbench_PrintClosedLoopSnapshotSemantic();
        } else {
            Testbench_PrintMotorSnapshotRaw();
        }
    } else if ((s_tb_slot == TESTBENCH_SLOT_MOTOR_ALIGN) ||
               (s_tb_slot == TESTBENCH_SLOT_MOTOR_OPEN))
    {
        /* 对齐与开环模式维持原有格式 */
        if (s_tb_log_format == TESTBENCH_LOGFMT_SEMANTIC) {
            Testbench_PrintMotorSnapshotSemantic();
        } else {
            Testbench_PrintMotorSnapshotRaw();
        }
    }
}

static void Testbench_UpdateAutoSequence(void) {
    if (s_tb_slot != TESTBENCH_SLOT_RC_AUTO) {
        s_auto_fsm = AUTO_STATE_IDLE;
        return;
    }

    TbMotorCmd_t cmd;
    TbMotorStateReport_t st;
    Testbench_ReadMotorCmd(&cmd);
    Testbench_ReadMotorState(&st);

    // 1. 监控 RC CH3 触发信号 (s_tb_rc_raw_channels 在上一步已实现全局缓存)
    // CH3 (Throttle) 映射到 100% 约为 1811
    bool trigger = (s_tb_rc_raw_channels[MOD_RC_IDX_CH3] > 1750);

    switch (s_auto_fsm) {
        case AUTO_STATE_IDLE:
            if (trigger) {
                s_auto_fsm = AUTO_STATE_GOING_HOME;
                BSP_UART_Printf("[AUTO] Triggered! Moving to Home (+15deg)\r\n");
            }
            break;

        case AUTO_STATE_GOING_HOME:
            cmd.cmd = TB_MOTOR_CMD_CLOSED_LOOP;
            cmd.closed_loop_target_ext_raw = 171; // +15°

            // 判断是否到位 (误差 < 12 且速度极低)
            if (Testbench_Abs32(st.error_ext) < 12 && Testbench_Abs32(st.velocity_raw_per_ms) < 2) {
                s_auto_hold_ticks++;
                if (s_auto_hold_ticks > 10) { // 稳定 200ms
                    s_auto_fsm = AUTO_STATE_GOING_PICK;
                    s_auto_hold_ticks = 0;
                    s_tb_binary_stream_active = true; // 【关键】：开始高频记录动态响应
                    BSP_UART_Printf("[AUTO] Home Reached. Step to Pick (+120deg) & Recording...\r\n");
                }
            } else {
                s_auto_hold_ticks = 0;
            }
            break;

        case AUTO_STATE_GOING_PICK:
            cmd.closed_loop_target_ext_raw = 1365; // +120°

            if (Testbench_Abs32(st.error_ext) < 12) {
                s_auto_hold_ticks++;
                if (s_auto_hold_ticks > 50) { // 到位后继续记录 1 秒以观察超调稳态
                    s_auto_fsm = AUTO_STATE_DONE;
                    s_tb_binary_stream_active = false; // 停止记录
                    BSP_UART_Printf("[AUTO] Sequence Done. Telemetry Stopped.\r\n");
                }
            }
            break;

        case AUTO_STATE_DONE:
            if (!trigger) { // 等待摇杆复位后才允许下次触发
                s_auto_fsm = AUTO_STATE_IDLE;
                cmd.cmd = TB_MOTOR_CMD_IDLE;
            }
            break;
    }

    Testbench_WriteMotorCmd(&cmd);
}

static void Testbench_UpdateStatusLed(void)
{
    TbMotorStateReport_t st;

    Testbench_ReadMotorState(&st);

    if (st.status != ARA_OK) {
        if ((s_tb_low_freq_tick % 20U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    } else {
        if ((s_tb_low_freq_tick % 100U) == 0U) {
            DEV_Status_LED_ToggleLed();
        }
    }
}

/* ============================================================================
 * 13. Physical Threads
 * ========================================================================== */

static void Thread_Testbench_HighFreq_Ctrl(void *argument)
{
    TickType_t last_wake_time;
    TbMotorBenchContext_t motor_ctx;
    TbMotorCmd_t motor_cmd;
    TbMotorStateReport_t motor_state;

    (void)argument;

    memset(&motor_cmd, 0, sizeof(TbMotorCmd_t));
    memset(&motor_state, 0, sizeof(TbMotorStateReport_t));

    TbMotor_InitBench(&motor_ctx);
    Testbench_WriteMotorState(&motor_state);

    last_wake_time = xTaskGetTickCount();

    uint32_t loop_tick = 0;

    for (;;) {
        Testbench_ReadMotorCmd(&motor_cmd);
        TbMotor_UpdateBench(&motor_ctx, &motor_cmd, &motor_state);
        Testbench_WriteMotorState(&motor_state);

        loop_tick++;

        // 仅在闭环模式下，且开启了二进制流，进行 200Hz 降采样发送
        if (motor_ctx.state == TB_MOTOR_STATE_CLOSED_LOOP && s_tb_binary_stream_active) {
            if (loop_tick % 5 == 0) { // 5ms = 200Hz
                PidTelemetryFrame_t frame;
                frame.header[0] = 0xAA;
                frame.header[1] = 0xBB;
                // 为了压缩数据，将 int32_t 的 ext_raw 强转为 int16_t
                // (前提是测试小范围响应，差值在 +/- 32767 内)
                frame.target = (int16_t) motor_state.closed_loop_target_ext_raw;
                frame.pos = (int16_t) motor_state.ext_raw;
                frame.uq = motor_state.pid_out_uq;

                // 计算 Checksum (XOR)
                uint8_t *p_data = (uint8_t *) &frame.target;
                frame.crc = 0;
                for (int i = 0; i < 6; i++) {
                    frame.crc ^= p_data[i];
                }

                // 调用非阻塞 DMA 发送
                BSP_UART_Send_DMA(BSP_UART_DEBUG, (uint8_t *) &frame, sizeof(frame));
            }
        }
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TESTBENCH_HIGH_FREQ_PERIOD_MS));
    }
}

static void Thread_Testbench_LowFreq_Logic(void *argument)
{
    TickType_t last_wake_time;
    TbMotorCmd_t motor_cmd;

    (void)argument;

    memset(&motor_cmd, 0, sizeof(TbMotorCmd_t));
    motor_cmd.cmd = TB_MOTOR_CMD_IDLE;
    motor_cmd.open_loop_speed_raw_per_tick = 0;
    motor_cmd.closed_loop_target_ext_raw = 0;
    Testbench_WriteMotorCmd(&motor_cmd);

    TaskRc_Init();
//    ModActuator_Init(&s_tb_actuator_ctx); // <--- 初始化舵机模块

    last_wake_time = xTaskGetTickCount();
    Testbench_EnterSlot(TESTBENCH_SLOT_MENU);

    for (;;) {
        s_tb_low_freq_tick++;
        Testbench_PollConsole();

        // 更新自动流状态机
        Testbench_UpdateAutoSequence();

        uint32_t tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
        TaskRc_Update(tick, &s_tb_rc_semantic);

        // <--- 运行标定桥接逻辑
//        Testbench_UpdateActuatorCalib();
//

        if (s_tb_rc_semantic.is_link_up) {
            TaskRc_CopyRawChannels(s_tb_rc_raw_channels, DRV_ELRS_MAX_CHANNELS);

            int32_t ch1_raw = (int32_t)s_tb_rc_raw_channels[MOD_RC_IDX_CH1];

            /* --- 【修复】：强制转换为有符号整数计算，防止下溢变异 --- */
            int32_t percent = ((ch1_raw - (int32_t)MOD_RC_VAL_MID) * 100) /
                              ((int32_t)MOD_RC_VAL_MAX - (int32_t)MOD_RC_VAL_MID);
            /* ----------------------------------------------------- */

            // 钳位到 [-100, 100]
            if (percent > 100) percent = 100;
            if (percent < -100) percent = -100;
            s_tb_rc_ch1_percent = (int16_t)percent;
        } else {
            s_tb_rc_ch1_percent = 0;
        }

        // 运行桥接逻辑
        Testbench_UpdateRcBridge();

        // 更新日志路由
        if (s_tb_print_snapshot_req || (s_tb_low_freq_tick % TESTBENCH_LOG_INTERVAL_TICKS == 0)) {
            if (s_tb_slot == TESTBENCH_SLOT_RC_MANUAL) {
                Testbench_PrintRcManualSnapshot();
            }
//            if (s_tb_slot == TESTBENCH_SLOT_ACTUATOR_CALIB) {
//                Testbench_PrintActuatorCalibSnapshot(); // <--- 专属日志路由
//            }
            else if (s_tb_slot == TESTBENCH_SLOT_RC_AUTO) {
                Testbench_PrintAutoSnapshot();
            } else {
                Testbench_PrintCurrentSnapshot();
            }
            s_tb_print_snapshot_req = false;
        }

        Testbench_UpdateStatusLed();
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(TESTBENCH_LOW_FREQ_PERIOD_MS));
    }
}

/* ============================================================================
 * 14. Public API
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