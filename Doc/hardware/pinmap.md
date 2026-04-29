# Hardware Pin Map

This document records the main STM32F103 pin assignments used by the AerialRoboArm electrical-control prototype. It is derived from the finalized `FOC_DEMO2.ioc` configuration and is intended as a wiring reference for the `demo_v6` firmware.

## Motor Driver: BLDC Power Stage / SimpleFOC Mini

The BLDC joint motor is driven through TIM1 three-phase PWM outputs and a dedicated enable pin.

| STM32 Pin | Function | Connected Signal | Notes |
| --- | --- | --- | --- |
| PA8 | TIM1_CH1 | IN1 | U-phase PWM input. |
| PA9 | TIM1_CH2 | IN2 | V-phase PWM input. |
| PA10 | TIM1_CH3 | IN3 | W-phase PWM input. |
| PA11 | GPIO_Output | EN | Power-stage enable, active high. |
| GND | Ground | GND | Common ground is required. |
| External supply | Power input | VCC | Motor-side supply; do not power this from STM32 3.3 V. |

## Position Feedback: AS5600 Magnetic Encoder

The AS5600 encoder is connected through I2C1 and provides joint position feedback for the BLDC axis.

| STM32 Pin | Function | Connected Signal | Notes |
| --- | --- | --- | --- |
| PB8 | I2C1_SCL | SCL | I2C clock. External pull-up is recommended. |
| PB9 | I2C1_SDA | SDA | I2C data. External pull-up is recommended. |
| 3.3 V | Power | VCC | AS5600 logic supply. |
| GND | Ground | GND | Common ground is required. |

## Communication and Auxiliary PWM

USART1 is used for the ELRS receiver. USART3 is used as the debug / console UART. TIM2 provides two auxiliary PWM channels for low-frequency actuator outputs.

| STM32 Pin | Function | Connected Target | Notes |
| --- | --- | --- | --- |
| PB6 | USART1_TX | ELRS RX / peripheral RX | USART1 remapped TX. |
| PB7 | USART1_RX | ELRS TX / peripheral TX | USART1 remapped RX, configured for ELRS link input. |
| PB10 | USART3_TX | Debug UART RX | Serial console output. |
| PB11 | USART3_RX | Debug UART TX | Serial console input. |
| PA0 | TIM2_CH1 | Servo / actuator 1 | Auxiliary PWM output. |
| PA1 | TIM2_CH2 | Servo / actuator 2 | Auxiliary PWM output. |

## Debug, Clock, and Status

| STM32 Pin | Function | Connected Target | Notes |
| --- | --- | --- | --- |
| PA13 | SYS_JTMS-SWDIO | Debug probe SWDIO | SWD data. |
| PA14 | SYS_JTCK-SWCLK | Debug probe SWCLK | SWD clock. |
| PC13 | GPIO_Output | Status LED | Board-level status indicator. |
| PD0 | RCC_OSC_IN | External HSE crystal | HSE input. |
| PD1 | RCC_OSC_OUT | External HSE crystal | HSE output. |

## Notes

- All external modules must share ground with the STM32 control board.
- Motor power and logic power should be treated as separate domains.
- The active firmware entry is the `demo_v6` testbench path, so these pins are documented for the final demo/testbench configuration rather than for a generic expansion board.
