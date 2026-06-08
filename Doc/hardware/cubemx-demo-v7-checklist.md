# CubeMX Checklist for demo_v7 Hardware Bring-Up

This checklist is for the day the ST3215 servo hardware arrives. It records the exact STM32CubeMX changes needed to move the firmware from the current mock-only `demo_v7` state to real USART2 half-duplex servo communication plus watchdog protection.

Use this document together with `FOC_DEMO2.ioc`.

## Goal

After completing this checklist, the generated project should:

- provide `huart2` for the ST3215 bus,
- configure USART2 for single-wire half-duplex at 1 Mbps,
- enable DMA for USART2 TX/RX,
- enable IWDG for runtime recovery,
- remove legacy TIM1 BLDC and I2C1 AS5600 peripheral generation,
- preserve existing `demo_v7` user code blocks.

## Current Baseline

At the time this document was written, the repository is in this state:

- `USART2` is not enabled in CubeMX yet,
- `IWDG` is not enabled in CubeMX yet,
- `TIM1` is still generated from the old BLDC path,
- `I2C1` is still generated from the old AS5600 path,
- `FreeRTOSConfig.h` in source has `configTOTAL_HEAP_SIZE = 5120`, but the `.ioc` still carries an older heap value and must be synced before code generation.

## Pre-Flight

Before opening CubeMX:

1. Make sure the git working tree is clean.
2. Confirm you are editing the intended branch.
3. Connect the hardware notes you plan to use for ST3215 wiring.
4. Plan to regenerate code only once after all CubeMX edits are complete.

## CubeMX Changes

Apply the following changes in `FOC_DEMO2.ioc`.

### 1. Enable USART2 for ST3215 Half-Duplex

Open:

`Pinout & Configuration -> Connectivity -> USART2`

Set:

- `Mode`: `Single Wire (Half Duplex)`
- `Baud Rate`: `1000000 Bits/s`
- `Word Length`: `8 Bits`
- `Parity`: `None`
- `Stop Bits`: `1`
- `Data Direction`: `Receive and Transmit`
- `Hardware Flow Control`: `Disable`

Pins:

- `PA2 -> USART2_TX`
- `PA3` should remain free in half-duplex mode

Why:

- The runtime ST3215 path expects a single-wire serial bus on `USART2`.
- Without this step, CubeMX will not generate `huart2`, and the real bus implementation cannot be bound in `bsp_uart.c`.

### 2. Add USART2 DMA Channels

Open:

`USART2 -> DMA Settings`

Add:

- `USART2_TX -> DMA1 Channel 7`, `Mode = Normal`, `Priority = High`
- `USART2_RX -> DMA1 Channel 6`, `Mode = Normal`, `Priority = High`

Why:

- The planned runtime path uses DMA-backed transmit and receive for the half-duplex transaction flow.

### 3. Enable USART2 Interrupt

Open:

`USART2 -> NVIC Settings`

Set:

- `USART2 global interrupt = Enabled`
- `Preemption Priority = 5`

Why:

- The firmware uses FreeRTOS and already places peripheral interrupts at priority `5`.
- Keeping USART2 aligned with USART1/USART3 avoids priority mismatches in the DMA + notify flow.

### 4. Enable IWDG

Open:

`Pinout & Configuration -> System Core -> IWDG`

Set:

- `Activated = Enabled`
- `Prescaler = 64`
- `Reload = 2812`

Expected window:

- LSI about `40 kHz`
- `40000 / 64 = 625 Hz`
- `2812 / 625 ~= 4.5 s`

Why:

- `App_Housekeeping` already contains conditional watchdog feeding logic and only needs CubeMX to generate the peripheral.
- This gives roughly `1.5 s` of margin over the current `3000 ms` heartbeat freshness threshold.

### 5. Disable Legacy TIM1 BLDC Generation

Open:

`Pinout & Configuration -> Timers -> TIM1`

Set:

- `Mode = Disabled`

Expected released pins:

- `PA8`
- `PA9`
- `PA10`
- `PB13`
- `PB14`
- `PB15`

Why:

- The old BLDC/FOC path has already been removed from the active firmware.
- Keeping TIM1 enabled only regenerates dead code and occupies pins unnecessarily.

### 6. Disable Legacy I2C1 AS5600 Generation

Open:

`Pinout & Configuration -> Connectivity -> I2C1`

Set:

- `Mode = Disabled`

Expected released pins:

- `PB8`
- `PB9`

Why:

- The AS5600 stack has already been removed from the active firmware.
- Keeping I2C1 enabled only regenerates unused init code and DMA/IRQ handlers.

### 7. Sync FreeRTOS Heap Before Generating

Open:

`Middleware -> FreeRTOS`

Set:

- `configMINIMAL_STACK_SIZE = 64`
- `configTOTAL_HEAP_SIZE = 5120`
- `configUSE_NEWLIB_REENTRANT = 1`

Why:

- The repository source already uses `5120` bytes in `Core/Inc/FreeRTOSConfig.h`.
- If the `.ioc` keeps the older heap value, CubeMX may overwrite the source file during generation and silently revert the RAM tuning.

## Generate Code

After all edits above are complete:

1. Save the `.ioc`.
2. Run `Generate Code`.
3. Do not manually edit generated files before the first rebuild.

## Expected Generated File Changes

After generation, expect changes in these files:

- `Core/Inc/main.h`
- `Core/Inc/stm32f1xx_hal_conf.h`
- `Core/Src/main.c`
- `Core/Src/usart.c`
- `Core/Src/stm32f1xx_hal_msp.c`
- `Core/Src/stm32f1xx_it.c`
- `Core/Src/i2c.c`
- `Core/Src/tim.c`

Typical expectations:

- `MX_USART2_UART_Init()` appears and is called from `main.c`.
- `huart2` is declared and initialized.
- USART2 DMA handles and NVIC setup are generated.
- `HAL_IWDG_MODULE_ENABLED` becomes enabled in `stm32f1xx_hal_conf.h`.
- `MX_I2C1_Init()` and `MX_TIM1_Init()` disappear from the startup path.

## Immediate Post-Generate Checks

Before touching any manual code, confirm:

1. `Core/Src/main.c` now calls `MX_USART2_UART_Init()`.
2. `Core/Src/main.c` no longer calls `MX_I2C1_Init()` or `MX_TIM1_Init()`.
3. `Core/Src/usart.c` contains a real `huart2` init block.
4. `Core/Inc/stm32f1xx_hal_conf.h` enables `HAL_IWDG_MODULE_ENABLED`.
5. Existing `USER CODE BEGIN/END` blocks still contain project logic such as `App_Control_InitDeps()`.

## Manual Code Follow-Up After CubeMX

Once the generated code looks correct, finish the firmware hookup in this order:

1. In `User/bsp/Src/bsp_uart.c`, declare `extern UART_HandleTypeDef huart2;`.
2. Bind `uart_ctx[BSP_UART_ST3215].huart = &huart2;`.
3. Replace the current ST3215 half-duplex stub with the real DMA-based implementation.
4. Flip `TASK_MOTION_USE_MOCK` to `0` in `User/task/Inc/task_motion.h`.
5. Rebuild the firmware.
6. Flash the board and verify Ping first, then the full RUNNING state path.

## Quick Bring-Up Validation

After flashing:

1. Confirm boot still reaches the scheduler.
2. Confirm watchdog does not reset the board during normal operation.
3. Probe `PA2` and verify USART2 traffic exists.
4. Confirm ST3215 Ping returns a valid response.
5. Confirm `servo_online` becomes true.
6. Confirm the system can leave mock-only behavior and enter real running control.

## Rollback Hint

If generation produces unexpected breakage:

1. Do not patch around it blindly.
2. Inspect the `.ioc` diff first.
3. Re-check that FreeRTOS heap settings were not reverted.
4. Re-check that TIM1 and I2C1 were actually disabled instead of partially left enabled.

This keeps the first hardware bring-up focused and reduces the chance of mixing CubeMX noise with real runtime bugs.
