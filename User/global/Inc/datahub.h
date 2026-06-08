/**
 * @file datahub.h
 * @brief Global single-writer / multi-reader blackboard (demo_v7).
 *
 * Single-writer (ControlTask) multi-reader (DebugTask, defaultTask) data
 * exchange. All RTOS tasks read from here for observability; only
 * ControlTask writes. This invariant is contractual, not enforced by code.
 *
 * Field semantics are tuned for the ST3215-HS smart joint era:
 *   - Motion state is expressed in encoder steps and degrees (not the old
 *     wrapped FOC angle). Multi-turn will be added post demo_v7.
 *   - Link health is a cross-channel concern (RC, vision, servo).
 *   - LED pattern is decided here and rendered by DebugTask.
 *
 * The cross-frequency-bridge role from demo_v6 is deprecated. DataHub is
 * still the only inter-task shared state container, but the semantics are
 * now "published observations" instead of "control bridge".
 */

#ifndef DATAHUB_H
#define DATAHUB_H

#include "ara_def.h"

/* =============================================================================
 * 1. Global system enums
 * ============================================================================= */

/**
 * @brief System operation mode.
 */
typedef enum {
    ARA_MODE_INIT = 0,   /**< Boot / servo bring-up. */
    ARA_MODE_IDLE,       /**< Safe idle, torque off, waiting for operator. */
    ARA_MODE_MANUAL,     /**< RC direct control. */
    ARA_MODE_AUTO,       /**< Vision-driven autonomous sequence (supervised). */
    ARA_MODE_ERROR       /**< Latched fault / E-Stop state. */
} AraSysMode_t;

/**
 * @brief RC-derived arm action intent (continuous axis collapsed into
 *        three-level semantics via hysteresis).
 */
typedef enum {
    ARM_CMD_HOLD = 0,
    ARM_CMD_EXTEND,
    ARM_CMD_RETRACT
} AraArmCmd_t;

/**
 * @brief RC-derived gripper intent.
 */
typedef enum {
    GRIPPER_CMD_STOP = 0,
    GRIPPER_CMD_OPEN,
    GRIPPER_CMD_CLOSE
} AraGripperCmd_t;

/**
 * @brief Operator E-Stop switch state.
 */
typedef enum {
    ESTOP_RELEASED = 0,
    ESTOP_ACTIVE
} AraEStopState_t;

/**
 * @brief LED display patterns. ControlTask decides; DebugTask renders.
 */
typedef enum {
    LED_PATTERN_OFF = 0,
    LED_PATTERN_BOOT_FAST_BLINK,   /**< Bring-up in progress. */
    LED_PATTERN_IDLE_SLOW_BLINK,
    LED_PATTERN_MANUAL_HEARTBEAT,  /**< Short pulse every 500 ms. */
    LED_PATTERN_AUTO_SOLID,
    LED_PATTERN_ERROR_SOS,         /**< Fault / E-Stop / link loss. */
    LED_PATTERN_FAULT_PENDING_RESET /**< Fault cleared, waiting SB reset. */
} AraLedPattern_t;

/* =============================================================================
 * 2. RC intent (produced by TaskRc, consumed by TaskArbiter)
 * ============================================================================= */

typedef struct {
    bool            is_link_up;
    AraSysMode_t    req_mode;        /**< Mode requested via RC mode switch. */
    AraEStopState_t estop_state;
    AraArmCmd_t     arm_cmd;
    AraGripperCmd_t gripper_cmd;
    bool            sys_reset_pulse; /**< Edge-triggered SB pulse. */
    uint16_t        aux_knob_val;    /**< SF, normalised [0, 1000]. */
    int16_t         ch1_percent;           /**< CH1 analog -100..+100, for manual angle. */
    int16_t         incremental_angle_deg; /**< Incremental target accumulated by CH1 steps. */
    uint8_t         roll_degree;           /**< CH8 wheel mapped to 0..180 degrees. */
    uint8_t         gripper_angle;         /**< CH4 spring axis mapped to 0..180 degrees. */
} RcControlData_t;

/* =============================================================================
 * 3. Vision intent (produced by TaskVision, consumed by TaskArbiter)
 * ============================================================================= */

typedef struct {
    bool     target_present;          /**< True when a fresh target is visible. */
    int16_t  target_angle_deg;        /**< Commanded joint angle in degrees. */
    uint16_t target_speed;            /**< Desired traversal speed, step/s. */
    uint8_t  confidence;              /**< 0..100; zero disables AUTO action. */
    uint32_t last_update_tick_ms;     /**< When the latest vision event arrived. */
} VisionIntent_t;

/* =============================================================================
 * 4. DataHub structure (single writer, multiple readers)
 * ============================================================================= */

/**
 * @brief Consolidated system observation published by ControlTask.
 */
typedef struct {
    /* --- Mode and safety --- */
    AraSysMode_t      current_mode;
    bool              estop_active;
    AraLedPattern_t   led_pattern;

    /* --- Arbitration outcome observability --- */
    AraSysMode_t      arbiter_mode;          /**< What the arbiter chose. */
    uint8_t           arbiter_reason_code;   /**< Short code for debug display. */

    /* --- Servo command side (published value, not the raw RC intent) --- */
    int16_t           servo_target_steps;    /**< Most recent commanded target. */
    int16_t           servo_target_angle_deg;
    uint16_t          servo_target_speed;
    uint8_t           servo_target_acc;
    bool              servo_torque_request;
    uint8_t           end_roll_deg;          /**< Commanded PA1 roll, 0..180. */
    uint8_t           end_gripper_deg;       /**< Commanded PA0 gripper, 0..180. */

    /* --- Servo feedback side --- */
    int16_t           servo_position_steps;
    int16_t           servo_position_angle_deg;
    int16_t           servo_velocity_steps;
    int16_t           servo_load;            /**< -1000..+1000. */
    uint8_t           servo_voltage_dv;      /**< 0.1 V units. */
    uint8_t           servo_temp_c;
    bool              servo_moving;
    AraStatus_t       servo_status;          /**< ARA_OK / ARA_BUSY / ARA_ERR_DISCONNECTED. */

    /* --- Link health --- */
    bool              rc_link_up;
    bool              vision_link_up;
    uint32_t          rc_last_ok_ms;
    uint32_t          vision_last_ok_ms;

    /* --- Timestamps --- */
    uint32_t          last_update_tick_ms;
    uint32_t          control_loop_count;
} DataHub_t;

/* =============================================================================
 * 5. API (single writer / multi reader, atomic)
 * ============================================================================= */

/**
 * @brief Initialise the hub with safe defaults. Call before scheduler start.
 */
void DataHub_Init(void);

/**
 * @brief Atomically publish a new full snapshot (ControlTask only).
 * @note  Uses taskENTER_CRITICAL. Total execution is well under 5 us.
 */
void DataHub_Publish(const DataHub_t *snapshot);

/**
 * @brief Atomically read the latest snapshot.
 */
void DataHub_Read(DataHub_t *out);

/**
 * @brief Quick helper: just read the LED pattern. Useful from DebugTask.
 */
AraLedPattern_t DataHub_GetLedPattern(void);

/**
 * @brief Quick helper: just read the current mode.
 */
AraSysMode_t DataHub_GetCurrentMode(void);

#endif /* DATAHUB_H */
