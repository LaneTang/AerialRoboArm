///**
// * @file test_console.c
// * @brief L4 Closed-Loop Control System (Final Integration)
// */
//
//#include "test_console.h"
//#include "bsp_uart.h"
//#include "main.h"
//#include "FreeRTOS.h"
//#include "semphr.h"
//
///* --- Includes --- */
//#include "drv_as5600.h"
//#include "drv_bldc_power.h"
//#include "alg_pid.h"
//#include "alg_voltage_foc.h"
//
///* --- Configuration --- */
//#define CONSOLE_BUF_SIZE 64
//#define POLE_PAIRS       7
//#define PWM_PERIOD       1440
//
//// [PID Tuning] 初始保守参数
//// Q8.8 Format: 1.0 = 256.
//// 建议: Kp=0.5 (128), Ki=0.01 (2), Kd=0
//#define PID_KP           128
//#define PID_KI           10
//#define PID_KD           0
//#define VOLT_LIMIT       8000  // 绝对输出限制 (Safe for 12V supply - 32768 (using Q15) )
//
///* --- Global Instances --- */
//static DrvAS5600_Context_t as5600_ctx;
//static DrvBldc_Context_t   bldc_ctx;
//static AlgPid_Context_t    pid_vel_ctx;
//static AlgFoc_Context_t    foc_ctx;
//
//static SemaphoreHandle_t   sem_as5600_done = NULL;
//static uint8_t rx_buffer[CONSOLE_BUF_SIZE];
//static bool    is_init = false;
//
///* --- Runtime Variables --- */
//static float    velocity_filtered = 0.0f; // rad/s
//static uint16_t prev_angle_raw = 0;
//static uint32_t prev_velocity_ts = 0;
//
///* --- Control Mode --- */
//typedef enum {
//    MODE_IDLE = 0,
//    MODE_VELOCITY_CHECK, // 1. 手转测速
//    MODE_VOLTAGE_LOOP,   // 2. 闭环电压(力矩)
//    MODE_SPEED_LOOP      // 3. 闭环速度(PID)
//} ControlMode_e;
//
//static ControlMode_e current_mode = MODE_IDLE;
//static float         target_val = 0.0f; // Target Voltage or Velocity
//
///* --- Prototypes --- */
//static void AS5600_Callback_ISR(void);
//static void Auto_Align_Zero(void);
//static void Update_Velocity(uint16_t current_raw);
//static void Loop_Control_Task(void);
//static void PrintMenu(void);
//static void ExecuteCommand(char cmd);
//
///* ==========================================
// * Init
// * ========================================== */
//void TestConsole_Init(void) {
//    BSP_UART_Printf("\r\n=== ARA PLATFORM: L4 CLOSED LOOP ===\r\n");
//
//    sem_as5600_done = xSemaphoreCreateBinary();
//
//    // 1. Init Drivers
//    DrvAS5600_Init(&as5600_ctx, BSP_I2C_MOTION, AS5600_Callback_ISR);
//    DrvBldc_Init(&bldc_ctx, BSP_GPIO_MOTOR_EN);
//
//    // 2. Init Algorithms
//    AlgPid_Init(&pid_vel_ctx);
//    AlgPid_SetGains(&pid_vel_ctx, PID_KP, PID_KI, PID_KD, VOLT_LIMIT, 2000000); // MaxOut, MaxInt
//
//    AlgFoc_Init(&foc_ctx, POLE_PAIRS, PWM_PERIOD);
//
//    // 3. Auto Align (Find Zero Offset)
//    // 此时会强行给电机通电，对齐零点
//    Auto_Align_Zero();
//
//    is_init = true;
//    PrintMenu();
//}
//
//void TestConsole_TaskLoop(void) {
//    // 1. Handle UART Commands
//    uint16_t len = BSP_UART_Read(BSP_UART_DEBUG, rx_buffer, CONSOLE_BUF_SIZE);
//    if (len > 0) {
//        for (uint16_t i = 0; i < len; i++) ExecuteCommand((char)rx_buffer[i]);
//    }
//
//    // 2. Main Control Loop (模拟高频任务)
//    if (is_init) {
//        Loop_Control_Task();
//    }
//
//}
//
///* ==========================================
// * Control Logic
// * ========================================== */
//
//// 自动对齐零点
//static void Auto_Align_Zero(void) {
//    BSP_UART_Printf("[Align] Aligning Sensor Zero...\r\n");
//
//    // 1. 强行将电压矢量指向电角度 0 (U=High, V=Low, W=Low approx)
//    // 在 FOC 算法中，给 Angle=0, Uq=0, Ud=voltage 即可对齐到 D轴
//    // 这里简单起见，直接调用 FOC Run(Angle=0, Uq=0, Ud=500)
//    // 这里的 600 是对齐电压，必须足以克服摩擦力
//    DrvBldc_Enable(&bldc_ctx, true);
//    AlgFoc_Run(&foc_ctx, 0, 0, 4000);
//    DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
//
//    // 2. 等待稳定 (700ms)
//    HAL_Delay(700);
//
//    // 3. 读取当前传感器的绝对角度，这就是零点偏差
//    DrvAS5600_TriggerUpdate(&as5600_ctx);
//    HAL_Delay(2); // Wait for DMA
//    uint16_t zero_pos = DrvAS5600_GetRawAngle(&as5600_ctx);
//
//    // 4. 设置 Offest
//    AlgFoc_SetZeroOffset(&foc_ctx, zero_pos);
//
//    // 5. 释放
//    DrvBldc_SetDuties(&bldc_ctx, 0, 0, 0);
//    DrvBldc_Enable(&bldc_ctx, false);
//
//    BSP_UART_Printf("[Align] Done. Zero Offset = %d\r\n", zero_pos);
//
//    // 顺便初始化速度计算变量
//    prev_angle_raw = zero_pos;
//    prev_velocity_ts = HAL_GetTick();
//}
//
//// 速度计算 + 低通滤波
//static void Update_Velocity(uint16_t current_raw) {
//    uint32_t now = HAL_GetTick();
//    float dt = (now - prev_velocity_ts) * 0.001f;
//    if (dt <= 0.0f) return;
//
//    // 计算角度差 (处理 0-4095 回绕)
//    int32_t delta = (int32_t)current_raw - (int32_t)prev_angle_raw;
//    if (delta > 2048)  delta -= 4096;
//    if (delta < -2048) delta += 4096;
//
//    // 转为弧度: delta * (2PI / 4096)
//    float delta_rad = delta * 0.001534f;
//
//    // 原始速度
//    float raw_vel = delta_rad / dt;
//
//    // 低通滤波 (LPF): y = 0.9*y_old + 0.1*new
//    // 系数越小，滤波越强但延迟越大
//    velocity_filtered = 0.9f * velocity_filtered + 0.1f * raw_vel;
//
//    prev_angle_raw = current_raw;
//    prev_velocity_ts = now;
//}
//
//static uint8_t sensor_error_count = 0;
//
//static void Loop_Control_Task(void) {
//    // 1. 发起读取请求并捕获底层状态
//    AraStatus_t trig_status = DrvAS5600_TriggerUpdate(&as5600_ctx);
//
//    if (trig_status != ARA_OK) {
//        // 错误A：底层 I2C 忙或设备离线 (例如返回 ARA_ERR_PARAM 意味着未初始化成功)
//        BSP_UART_Printf("[Err] Sensor Trigger Failed! Code: %d\r\n", trig_status);
//        vTaskDelay(pdMS_TO_TICKS(500)); // 降频防刷屏
//        return;
//    }
//
//    // 2. 等待 DMA 中断释放信号量
//    if (xSemaphoreTake(sem_as5600_done, pdMS_TO_TICKS(10)) != pdTRUE) {
//        // 错误B：触发成功，但 DMA 中断没回来！
//        sensor_error_count++;
//        if(sensor_error_count > 5) {
//            BSP_UART_Printf("[Err] Sensor DMA Timeout! Interrupt missing?\r\n");
//            sensor_error_count = 0;
//            vTaskDelay(pdMS_TO_TICKS(500));
//        }
//        return;
//    }
//    sensor_error_count = 0;
//
//    // 3. 获取数据
//    uint16_t raw_angle = DrvAS5600_GetRawAngle(&as5600_ctx);
//    //BSP_UART_Printf("Raw Angle: %d\r\n", raw_angle);
//
//    // 2. 更新速度估计
//    Update_Velocity(raw_angle);
//
//    // 3. 状态机
//    switch (current_mode) {
//        case MODE_IDLE:
//            DrvBldc_Enable(&bldc_ctx, false);
//            break;
//
//        case MODE_VELOCITY_CHECK:
//            // 只读不转，打印数据在 Menu 循环里做
//            DrvBldc_Enable(&bldc_ctx, false);
//            break;
//
//        case MODE_VOLTAGE_LOOP: // 闭环 FOC 电压模式
//            DrvBldc_Enable(&bldc_ctx, true);
//            // 直接设定 Uq = target (例如 2V), Ud = 0
//            // 这里的 Angle 是真实的传感器角度！不是开环累加的！
//            AlgFoc_Run(&foc_ctx, raw_angle, (int16_t)target_val, 0);
//            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
//            break;
//
//        case MODE_SPEED_LOOP: // 闭环 FOC 速度模式
//            DrvBldc_Enable(&bldc_ctx, true);
//
//            // A. 运行 PID: 目标速度 vs 实际速度 -> 输出 Uq 电压
//            // 为了匹配 PID 的 Q15 接口，我们需要把 float 速度转为 int16 (需缩放)
//            // 这里简单处理：假设 1 rad/s = 100 units
//            int16_t t_vel_int = (int16_t)(target_val * 10.0f);
//            int16_t m_vel_int = (int16_t)(velocity_filtered * 10.0f);
//
//            int16_t u_q_cmd = AlgPid_Compute(&pid_vel_ctx, t_vel_int, m_vel_int);
//
//            // B. 运行 FOC
//            AlgFoc_Run(&foc_ctx, raw_angle, u_q_cmd, 0);
//            DrvBldc_SetDuties(&bldc_ctx, foc_ctx.duty_a, foc_ctx.duty_b, foc_ctx.duty_c);
//            break;
//    }
//
//    // 4. 打印遥测 (降频)
//    static uint32_t print_ts = 0;
//    if (HAL_GetTick() - print_ts > 200) {
//        print_ts = HAL_GetTick();
//        if (current_mode != MODE_IDLE) {
//            // 打印: 模式 | 目标值 | 实际速度 | 电角度 | Uq电压
//            BSP_UART_Printf("M:%d | Tgt:%.1f | Vel:%.2f | Ang:%d | Uq:%d\r\n",
//                            current_mode, target_val, velocity_filtered, foc_ctx.electric_angle,
//                            (current_mode==MODE_SPEED_LOOP)?AlgPid_Compute(&pid_vel_ctx,0,0) : (int16_t)target_val);
//            // 注意：上面 PID 打印为了省事可能不准，主要看 Vel
//        }
//    }
//}
//
///* ==========================================
// * Helper
// * ========================================== */
//static void AS5600_Callback_ISR(void) {
//    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//    if (sem_as5600_done != NULL) xSemaphoreGiveFromISR(sem_as5600_done, &xHigherPriorityTaskWoken);
//    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
//}
//
//static void PrintMenu(void) {
//    BSP_UART_Printf("\r\n--- L4 Control Menu ---\r\n");
//    BSP_UART_Printf("[v] Check Velocity Signal (Hand Spin)\r\n");
//    BSP_UART_Printf("[t] Torque/Voltage Mode (Uq=400)\r\n");
//    BSP_UART_Printf("[c] Closed Loop Speed (Target=10.0)\r\n");
//    BSP_UART_Printf("[s] Stop / Idle\r\n");
//    BSP_UART_Printf("[z] Re-Align Zero\r\n");
//    BSP_UART_Printf("-----------------------\r\n> ");
//}
//
//static void ExecuteCommand(char cmd) {
//    switch (cmd) {
//        case 'v':
//            current_mode = MODE_VELOCITY_CHECK;
//            target_val = 0;
//            BSP_UART_Printf("MODE: Velocity Check. Spin motor by hand.\r\n");
//            break;
//
//        case 't':
//            current_mode = MODE_VOLTAGE_LOOP;
//            target_val = 3000.0f; // 约 3.3V
//            // 重置 PID 状态防止积分累积
//            AlgPid_Reset(&pid_vel_ctx);
//            BSP_UART_Printf("MODE: Voltage FOC. Uq=3.1V. Motor should accelerate.\r\n");
//            break;
//
//        case 'c':
//            current_mode = MODE_SPEED_LOOP;
//            target_val = 20.0f; // 目标 20 rad/s
//            AlgPid_Reset(&pid_vel_ctx);
//            BSP_UART_Printf("MODE: Speed Loop. Target=20.0 rad/s\r\n");
//            break;
//
//        case 's':
//            current_mode = MODE_IDLE;
//            target_val = 0;
//            DrvBldc_Enable(&bldc_ctx, false);
//            BSP_PWM_StopAll();
//            BSP_UART_Printf("STOPPED.\r\n");
//            break;
//
//        case 'z':
//            Auto_Align_Zero();
//            break;
//
//        case 'h': PrintMenu(); break;
//        default: break;
//    }
//}