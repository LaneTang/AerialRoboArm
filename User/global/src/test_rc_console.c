///**
// * @file test_rc_console.c
// * @brief RC Controller Modular & Integration Test Implementation
// */
//
//#include "test_rc_console.h"
//#include "bsp_uart.h"
//#include "FreeRTOS.h"
//#include "task.h"
//
///* --- Includes for Modules to Test --- */
//#include "drv_elrs.h"           // L2
//#include "mod_rc_semantic.h"    // L3
//#include "datahub.h"            // L4 Blackboard
//#include "drv_bldc_power.h"     // L2 Actuator (For Integration Test)
//#include "alg_voltage_foc.h"
//
///* --- Configuration --- */
//#define CONSOLE_BUF_SIZE 64
//#define TEST_MOTOR_DUTY  420    // Higher starting duty for the lower-KV 2808
//#define TEST_COMMUTATION_STEP_MS 12U
//
///* --- Global Instances --- */
//static DrvElrs_Context_t       elrs_ctx;
//static ModRcSemantic_Context_t semantic_ctx;
//static DrvBldc_Context_t       bldc_ctx;
//static AlgFoc_Context_t        foc_ctx; // [新增] FOC 上下文
//
//static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
//static bool    is_init = false;
//
///* --- Control Mode --- */
//typedef enum {
//    MODE_IDLE = 0,
//    MODE_RAW_CHANNELS,   // 1. L2: Test CRSF Parsing
//    MODE_SEMANTIC,       // 2. L3: Test Logic Translation
//    MODE_DATAHUB,        // 3. L4: Test Blackboard & Failsafe
//    MODE_MOTOR_LINK      // 4. SYS: RC -> Motor Control
//} TestRcMode_e;
//
//static TestRcMode_e current_mode = MODE_IDLE;
//static RcControlData_t current_intent;
//static uint8_t motor_step_idx = 0;
//static uint32_t motor_step_ts = 0;
//static bool motor_output_enabled = false;
//
///* --- Prototypes --- */
//static void PrintMenu(void);
//static void ExecuteCommand(char cmd);
//static void GenerateDefaultChannels(uint16_t *chs);
//static void ApplyMotorStep(uint8_t step_idx, bool forward);
//static void SetMotorOutputEnabled(bool enabled);
//
///* ==========================================
// * Init
// * ========================================== */
//void TestRcConsole_Init(void) {
//    BSP_UART_Printf("\r\n=== ARA PLATFORM: RC INTEGRATION TEST ===\r\n");
//
//    /* 1. Init DataHub */
//    DataHub_Init();
//
//    /* 2. Init L2: ELRS Driver */
//    DrvElrs_Init(&elrs_ctx, BSP_UART_ELRS);
//
//    /* 3. Init L3: Semantic Logic */
//    uint16_t default_chs[DRV_ELRS_MAX_CHANNELS];
//    GenerateDefaultChannels(default_chs);
//    ModRcSemantic_Init(&semantic_ctx, default_chs);
//
//    /* 4. Init Motor Power Stage */
//    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);
//
//    /* 5. [新增] Init FOC Algorithm */
//    // 极对数 7, PWM ARR周期 1440
//    AlgFoc_Init(&foc_ctx, 7, BSP_PWM_MAX_DUTY);
//
//    SetMotorOutputEnabled(false);
//    is_init = true;
//    PrintMenu();
//}
//
///* ==========================================
// * Main Loop
// * ========================================== */
//void TestRcConsole_TaskLoop(void) {
//    if (!is_init) return;
//
//    /* 1. Handle UART Commands */
//    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
//    if (len > 0) {
//        for (uint16_t i = 0; i < len; i++) ExecuteCommand((char)rx_buffer[i]);
//    }
//
//    /* 2. Update L2 Hardware Driver (Process UART RingBuffer) */
//    uint32_t now_ms = (uint32_t)xTaskGetTickCount();
//    DrvElrs_Update(&elrs_ctx, now_ms);
//    bool is_link_up = DrvElrs_IsLinkUp(&elrs_ctx);
//
//    /* 3. Update L3 Semantic Logic */
//    if (is_link_up) {
//        ModRcSemantic_Process(&semantic_ctx, elrs_ctx.channels, now_ms, &current_intent);
//        current_intent.is_link_up = true;
//    } else {
//        /* Manual Failsafe Generation for Test Console */
//        current_intent.is_link_up = false;
//        current_intent.req_mode = ARA_MODE_IDLE;
//        current_intent.estop_state = ESTOP_ACTIVE;
//        current_intent.arm_cmd = ARM_CMD_HOLD;
//    }
//
//    /* Write to DataHub for MODE_DATAHUB test */
//    DataHub_WriteRcData(&current_intent);
//
//    /* FOC 运行参数 (由于目前没有 AS5600，我们采用强拖模式产生开环旋转角) */
//    static uint16_t virtual_mech_angle = 0; // 0-4095
//    int16_t target_uq = 0;                  // 目标力矩电压 (Q15: -32768 ~ 32767)
//
//    // 假设 VBUS = 12V. 我们限制最大输出电压到 15% VBUS 左右 (防烧电机)
//    // 32768 * 15% ≈ 4915 (与你的 ALIGN_VOLTAGE_Q15 类似)
//    const int16_t MAX_TEST_UQ = 4915;
//
//    /* 4. Execute Mode Logic & Print Telemetry (Decimated) */
//    static uint32_t print_ts = 0;
//    bool should_print = (now_ms - print_ts > 200); // 5Hz Print Rate
//
//    switch (current_mode) {
//        case MODE_IDLE:
//            SetMotorOutputEnabled(false);
//            break;
//
//        case MODE_RAW_CHANNELS:
//            if (should_print) {
//                BSP_UART_Printf("[L2] Link:%d | CH1:%04d | CH5:%04d | CH7:%04d | CH8:%04d | CH9:%04d | CH10:%04d\r\n",
//                                is_link_up,
//                                elrs_ctx.channels[0], // CH1 (Roll)
//                                elrs_ctx.channels[MOD_RC_IDX_SA],
//                        // elrs_ctx.channels[MOD_RC_IDX_SE], // CH6 (SE) temporarily hidden
//                                elrs_ctx.channels[MOD_RC_IDX_SC],
//                                elrs_ctx.channels[MOD_RC_IDX_SF],
//                                elrs_ctx.channels[MOD_RC_IDX_SB],
//                                elrs_ctx.channels[MOD_RC_IDX_SD]);
//            }
//            break;
//
//        case MODE_SEMANTIC:
//            if (should_print) {
//                BSP_UART_Printf("[L3] ModeReq:%d | ArmCmd:%d | Grip:%d | EStop:%d | Pulse:%d | Aux:%d\r\n",
//                                current_intent.req_mode,
//                                current_intent.arm_cmd,
//                                current_intent.gripper_cmd,
//                                current_intent.estop_state,
//                                current_intent.sys_reset_pulse,
//                                current_intent.aux_knob_val);
//            }
//            break;
//
//        case MODE_DATAHUB:
//            if (should_print) {
//                RcControlData_t dh_data;
//                AraStatus_t s = DataHub_ReadRcData(&dh_data);
//                BSP_UART_Printf("[L4] HubRead:%d | LinkUp:%d | EStop:%d\r\n",
//                                s, dh_data.is_link_up, dh_data.estop_state);
//            }
//            break;
//
//        case MODE_MOTOR_LINK:
//            /* Integration Test: RC -> Smooth FOC Motor */
//            if (current_intent.estop_state == ESTOP_ACTIVE) {
//                SetMotorOutputEnabled(false);
//                if (should_print) BSP_UART_Printf("[SYS] EMERGENY STOP ACTIVE!\r\n");
//            } else if (current_intent.req_mode == ARA_MODE_MANUAL) {
//                SetMotorOutputEnabled(true);
//
//                // 1. 获取遥控器 CH1 (右摇杆左右) 的原始值 (映射为百分比 -100 ~ 100)
//                // 注意：在 mod_rc_semantic 中内部有 map_ch1_percent，但外部没有暴露
//                // 为了平滑控制，我们直接读取 elrs_ctx 的原始通道值计算比例
//                int32_t ch1_raw = (int32_t) elrs_ctx.channels[0]; // CH1
//                int32_t ch1_delta = ch1_raw - MOD_RC_VAL_MID;    // 以 1500 为中点
//
//                // 死区过滤 (摇杆回中时有一点偏差)
//                if (ch1_delta > -50 && ch1_delta < 50) {
//                    ch1_delta = 0;
//                }
//
//                if (ch1_delta != 0) {
//                    // 2. 将摇杆量映射为转速 (角度增量)
//                    // 推得越多，加的角度越多，转得越快。
//                    // 增量范围大致在 +/- 1 到 +/- 20 之间 (根据你想要的丝滑程度调整)
//                    int16_t angle_step = (int16_t) (ch1_delta / 20);
//
//                    virtual_mech_angle = (virtual_mech_angle + angle_step) & 0x0FFF; // 模 4096
//
//                    // 3. 将摇杆量映射为 Uq (力矩)
//                    // 如果推满 (delta=500)，Uq = MAX_TEST_UQ
//                    target_uq = (int16_t) ((ch1_delta * MAX_TEST_UQ) / 500);
//
//                    if (should_print)
//                        BSP_UART_Printf("[SYS] Moving | Uq:%d | Ang:%d\r\n", target_uq, virtual_mech_angle);
//                } else {
//                    /* Hold 状态：提供一个微小的锁定 Uq 或者 Uq=0，角度不变 */
//                    target_uq = 1000; // 给一点点电流锁住位置
//                    if (should_print) BSP_UART_Printf("[SYS] Motor: HOLD (FOC Locked)\r\n");
//                }
//
//                // --- 执行 FOC 核心算法 ---
//                // 参数：上下文, 虚拟机械角度(0-4095), Uq(力矩电压), Ud(磁链电压=0)
//                AlgFoc_Run(&foc_ctx, virtual_mech_angle, target_uq, 0);
//
//                // --- 写入底层 PWM ---
//                DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
//            } else {
//                DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
//                if (should_print) BSP_UART_Printf("[SYS] Auto Mode: RC Ignored.\r\n");
//            }
//            break;
//    }
//
//    if (should_print) print_ts = now_ms;
//}
//
///* ==========================================
// * Helper Functions
// * ========================================== */
//static void PrintMenu(void) {
//    BSP_UART_Printf("\r\n--- RC Integration Test Menu ---\r\n");
//    BSP_UART_Printf("[r] L2: Raw CRSF Channels (Link Test)\r\n");
//    BSP_UART_Printf("[s] L3: Semantic Intent (Logic Test)\r\n");
//    BSP_UART_Printf("[d] L4: DataHub Failsafe Read (Hub Test)\r\n");
//    BSP_UART_Printf("[m] SYS: Manual Motor Linkage (E-Stop + Arm Test)\r\n");
//    BSP_UART_Printf("[i] Stop / Idle\r\n");
//    BSP_UART_Printf("--------------------------------\r\n> ");
//}
//
//static void ExecuteCommand(char cmd) {
//    switch (cmd) {
//        case 'r':
//            current_mode = MODE_RAW_CHANNELS;
//            BSP_UART_Printf("MODE: RAW CHANNELS. Please move RC sticks.\r\n");
//            break;
//        case 's':
//            current_mode = MODE_SEMANTIC;
//            BSP_UART_Printf("MODE: SEMANTIC LOGIC. Test buttons and switches.\r\n");
//            break;
//        case 'd':
//            current_mode = MODE_DATAHUB;
//            BSP_UART_Printf("MODE: DATAHUB. Turn off RC transmitter to test Failsafe.\r\n");
//            break;
//        case 'm':
//            current_mode = MODE_MOTOR_LINK;
//            BSP_UART_Printf("MODE: SYSTEM LINKAGE. CAUTION: Motor will move!\r\n");
//            break;
//        case 'i':
//            current_mode = MODE_IDLE;
//            SetMotorOutputEnabled(false);
//            BSP_UART_Printf("STOPPED.\r\n");
//            break;
//        case 'h': PrintMenu(); break;
//        default: break;
//    }
//}
//
//static void GenerateDefaultChannels(uint16_t *chs) {
//    for (int i = 0; i < DRV_ELRS_MAX_CHANNELS; i++) {
//        chs[i] = MOD_RC_VAL_MID;
//    }
//    chs[MOD_RC_IDX_SD] = MOD_RC_SW_LOW - 100; // Default to E-STOP ACTIVE
//}
//
//static void ApplyMotorStep(uint8_t step_idx, bool forward)
//{
//    static const uint16_t step_table[6][3] = {
//            { TEST_MOTOR_DUTY, 0, 0 },
//            { TEST_MOTOR_DUTY, TEST_MOTOR_DUTY, 0 },
//            { 0, TEST_MOTOR_DUTY, 0 },
//            { 0, TEST_MOTOR_DUTY, TEST_MOTOR_DUTY },
//            { 0, 0, TEST_MOTOR_DUTY },
//            { TEST_MOTOR_DUTY, 0, TEST_MOTOR_DUTY }
//    };
//
//    uint8_t index = step_idx % 6U;
//    if (!forward) {
//        index = (uint8_t)((6U - index) % 6U);
//    }
//
//    DrvBldc_SetDuties(&bldc_ctx,
//                      step_table[index][0],
//                      step_table[index][1],
//                      step_table[index][2]);
//}
//
//static void SetMotorOutputEnabled(bool enabled)
//{
//    if (motor_output_enabled == enabled) {
//        return;
//    }
//
//    motor_output_enabled = enabled;
//    DrvBldc_Enable(&bldc_ctx, enabled);
//    if (!enabled) {
//        BSP_PWM_StopAll();
//    }
//}
