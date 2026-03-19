/**
 * @file task_motion.c
 * @brief Motion Control Task Implementation
 * @author ARA Project Coder
 */

#include "task_motion.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "semphr.h"
#include "bsp_gpio.h" // For LED debug or EN pin

/* --- Configuration Constants --- */
#define MOTION_LOOP_PERIOD_MS   (1)     // 1kHz Loop
#define MOTOR_POLE_PAIRS        (7)     // 2804/2808 listed motors are both 7 pole-pairs
#define ALIGN_VOLTAGE_Q15       (4915)  // ~15% of full-scale voltage, safer starting point for 2808
#define ALIGN_DURATION_TICKS    (2500)  // 2.5 seconds, gives the heavier rotor more settle time
#define SENSOR_TIMEOUT_TICKS    (2)     // 2ms timeout for I2C

/* --- Global Context --- */
static TaskMotion_Context_t ctx;

/* --- RTOS Objects --- */
static SemaphoreHandle_t sem_i2c_cplt;
static TaskHandle_t task_handle;

/* --- Private Variables --- */
static uint16_t prev_angle_raw = 0;

/* --- Callback Implementation --- */
/**
 * @brief ISR Context Callback from DrvAS5600
 */
static void Motion_SensorReadyCallback(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(sem_i2c_cplt, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/* --- Helper: Velocity Calculation --- */
static int16_t CalculateVelocity(uint16_t curr, uint16_t prev)
{
    // Simple differentiation with wrap-around handling
    // Range 0-4095. Half is 2048.
    int32_t diff = (int32_t)curr - (int32_t)prev;

    // Handle wrap-around
    if (diff > 2048) {
        diff -= 4096;
    } else if (diff < -2048) {
        diff += 4096;
    }

    // Filter could be added here (LPF)
    // For now, return raw delta (Speed proportional to counts/ms)
    // To convert to RPM: Speed * (60000 / 4096)
    return (int16_t)diff;
}

/* --- API Implementation --- */

void TaskMotion_Init(void)
{
    // 1. Create Synchronization Primitives
    sem_i2c_cplt = xSemaphoreCreateBinary();

    // 2. Initialize Components
    // Initialize BSP/L2 Drivers
    // Note: BSP_I2C_Init and BSP_PWM_Init are assumed called in main.c or here.
    // Here we initialize the Context Objects.

    // AS5600: Register the ISR callback
    DrvAS5600_Init(&ctx.enc_driver, BSP_I2C_MOTION, Motion_SensorReadyCallback);

    // BLDC: Enable Pin Control
    DrvBldc_Init(&ctx.motor_driver, BSP_GPIO_MOTOR_EN);

    // FOC Algorithm
    // Pole Pairs = 7, PWM Period = 3600 (Example from BSP)
    AlgFoc_Init(&ctx.foc_algo, MOTOR_POLE_PAIRS, BSP_PWM_MAX_DUTY);

    // PID Controllers (Default safe gains, needs tuning)
    // Kp=1.0(256), Ki=0.01, Kd=0
    AlgPid_Init(&ctx.vel_pid);
    AlgPid_SetGains(&ctx.vel_pid, 300, 8, 0, BSP_PWM_MAX_DUTY, BSP_PWM_MAX_DUTY);

    // 3. Initialize State
    ctx.state = MOTION_STATE_IDLE;
    ctx.target_velocity = 0;

    // 4. Create Task
//    xTaskCreate(TaskMotion_Entry, "MotionTask", 256, NULL, 5, &task_handle);
}

void TaskMotion_SetTargetVelocity(int16_t velocity_q15)
{
    // Atomic write (16-bit aligned access is atomic on Cortex-M3)
    ctx.target_velocity = velocity_q15;
}

void TaskMotion_EmergencyStop(void)
{
    ctx.state = MOTION_STATE_ERROR;
    DrvBldc_Enable(&ctx.motor_driver, false);
}

/* --- Main Task Loop --- */
void TaskMotion_Entry(void *argument)
{
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(MOTION_LOOP_PERIOD_MS);
    uint32_t align_counter = 0;

    xLastWakeTime = xTaskGetTickCount();

    // Loop Forever
    for (;;) {
        // 1. Precision Timing
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
        ctx.tick_count++;

        // 2. Sensor Acquisition (All States need sensor data)
        // Trigger DMA Read
        if (DrvAS5600_TriggerUpdate(&ctx.enc_driver) == ARA_OK) {
            // Wait for completion (Block Task)
            if (xSemaphoreTake(sem_i2c_cplt, SENSOR_TIMEOUT_TICKS) != pdTRUE) {
                // Timeout: Sensor Error
                TaskMotion_EmergencyStop();
                continue;
            }
        } else {
            // Bus Busy or Error
            continue;
        }

        // Get fresh angle
        uint16_t raw_angle = DrvAS5600_GetRawAngle(&ctx.enc_driver);

        // 3. State Machine
        switch (ctx.state) {
            case MOTION_STATE_IDLE:
                DrvBldc_Enable(&ctx.motor_driver, false);

                // Transition Logic: For demo, auto-start alignment after 1s
                if (ctx.tick_count > 1000) {
                    DrvBldc_Enable(&ctx.motor_driver, true);
                    ctx.state = MOTION_STATE_ALIGNMENT;
                    align_counter = 0;
                }
                break;

            case MOTION_STATE_ALIGNMENT:
                // Alignment Sequence:
                // Force U_d voltage vector to align rotor to electrical zero.
                // We fake the 'raw_angle' input to FOC as 0, so it applies vector at 0 deg.
                // Actually, we tell FOC the sensor is at 0, and we ask for U_d.

                // Step 1: Force field.
                // U_q = 0 (No Torque), U_d = ALIGN_VOLTAGE (Holding Flux)
                AlgFoc_Run(&ctx.foc_algo, 0, 0, ALIGN_VOLTAGE_Q15);

                // Update Hardware
                DrvBldc_SetDuties(&ctx.motor_driver,
                                  ctx.foc_algo.duty_a,
                                  ctx.foc_algo.duty_b,
                                  ctx.foc_algo.duty_c);

                align_counter++;

                // Step 2: Sample Zero Offset at end of alignment
                if (align_counter >= ALIGN_DURATION_TICKS) {
                    // Motor should be stationary at Electrical Zero now.
                    // The current raw sensor value IS the Zero Offset.
                    AlgFoc_SetZeroOffset(&ctx.foc_algo, raw_angle);

                    // Reset PID integration to avoid jumps
                    AlgPid_Reset(&ctx.vel_pid);
                    prev_angle_raw = raw_angle;

                    // Go!
                    ctx.state = MOTION_STATE_CLOSED_LOOP;
                }
                break;

            case MOTION_STATE_CLOSED_LOOP:
            {
                // A. Calculate Velocity
                int16_t velocity = CalculateVelocity(raw_angle, prev_angle_raw);
                prev_angle_raw = raw_angle;

                // B. Run Velocity PID
                // Input: Target Speed, Measured Speed. Output: Target Torque (U_q)
                int16_t u_q = AlgPid_Compute(&ctx.vel_pid, ctx.target_velocity, velocity);

                // C. Run FOC
                // Inputs: Actual Angle, U_q (Torque), U_d (Flux - usually 0)
                AlgFoc_Run(&ctx.foc_algo, raw_angle, u_q, 0);

                // D. Update Driver
                DrvBldc_SetDuties(&ctx.motor_driver,
                                  ctx.foc_algo.duty_a,
                                  ctx.foc_algo.duty_b,
                                  ctx.foc_algo.duty_c);
                break;
            }

            case MOTION_STATE_ERROR:
            default:
                DrvBldc_Enable(&ctx.motor_driver, false);
                // System is locked. Requires Reset.
                break;
        }

        osDelay(1);
    }
}
