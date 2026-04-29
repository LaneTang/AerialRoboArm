# My Work / 个人工作说明

[English](#english) | [中文](#中文)

## English

This document clarifies my personal contribution to the AerialRoboArm project. The repository is presented as an engineering portfolio case study for the electrical-control subsystem of my undergraduate thesis project. The thesis document itself is not included in this repository.

### Scope I Was Responsible For

I was responsible for the embedded electrical-control and firmware implementation side of the project:

- STM32F103 firmware architecture.
- FreeRTOS-based dual-rate runtime organization.
- Peripheral abstraction and device-driver integration.
- BLDC motor control pipeline with AS5600 position feedback.
- Fixed-point PID and voltage-FOC implementation and tuning.
- ELRS remote-control input parsing and semantic mapping.
- Serial testbench and demo workflow for staged hardware validation.
- Documentation cleanup for the final repository presentation.

### Firmware Architecture

I organized the firmware into layered responsibilities:

- `User/bsp`: MCU peripheral abstraction for UART, I2C, PWM, and GPIO.
- `User/drv`: device-level drivers for AS5600, BLDC power stage, ELRS, servo output, and status IO.
- `User/mod`: reusable modules and algorithms, including PID, voltage FOC, actuator abstraction, and RC semantic mapping.
- `User/task`: task-level runnables for motion control, RC processing, and manipulator logic.
- `User/app`: physical thread containers and the final demo/testbench runtime.

This structure was designed around the resource limits of STM32F103C8T6 and the need to keep high-frequency motor control separate from low-frequency business logic and observability.

### Motor-Control Chain

For the BLDC folding-arm axis, I implemented and integrated:

- AS5600 magnetic encoder acquisition over I2C.
- BLDC power-stage control through three-phase PWM outputs.
- Electrical alignment and zero-offset handling.
- Fixed-point PID position loop.
- Voltage-FOC output generation.
- Continuous `ext_raw` coordinate tracking to avoid wrap-around discontinuities around the AS5600 raw-angle boundary.

The final demo/testbench firmware validates motor alignment, open-loop output, closed-loop target tracking, and staged response observation.

### Remote-Control and Demo Workflow

I integrated ELRS remote-control input into the firmware and mapped raw channels into system-level semantics such as manual control, mode selection, safety behavior, and demo triggers.

The final testbench supports:

- RC monitor.
- Motor alignment test.
- Motor open-loop test.
- Motor closed-loop test.
- RC manual closed-loop demo.
- AUTO step test for closed-loop response recording.
- Serial snapshots and compact telemetry output.

### What This Repository Does Not Claim

This repository does not claim to implement a complete UAV flight-control system, product-grade safety layer, full autonomous perception pipeline, or end-to-end aerial grasping system. It focuses on the robotic-arm electrical-control subsystem and the finalized demonstration firmware.

## 中文

本文档用于说明我在 AerialRoboArm 项目中的个人工作内容。本仓库作为本人本科毕业设计电控子系统的工程作品集案例进行展示。毕业论文正文不包含在本仓库中。

### 我负责的范围

我负责项目中的嵌入式电控与固件实现部分：

- STM32F103 固件架构。
- 基于 FreeRTOS 的双频运行时组织。
- 外设抽象与设备驱动集成。
- 基于 AS5600 位置反馈的 BLDC 电机控制链路。
- 定点 PID 与电压 FOC 的实现和调试。
- ELRS 遥控输入解析与语义映射。
- 用于分阶段硬件验证的串口 testbench 与 demo 流程。
- 面向最终仓库展示的文档整理。

### 固件架构

我将固件组织为分层职责结构：

- `User/bsp`：UART、I2C、PWM、GPIO 等 MCU 外设抽象。
- `User/drv`：AS5600、BLDC 功率级、ELRS、舵机输出与状态 IO 等设备级驱动。
- `User/mod`：PID、电压 FOC、执行器抽象、RC 语义映射等可复用算法和功能模块。
- `User/task`：运动控制、RC 处理与 manipulator 逻辑等任务级 runnable。
- `User/app`：物理线程容器与最终 demo/testbench 运行时。

该结构围绕 STM32F103C8T6 的资源限制设计，目标是将高频电机控制与低频业务逻辑、观测输出隔离开。

### 电机控制链路

针对折叠臂 BLDC 主轴，我实现并集成了：

- 基于 I2C 的 AS5600 磁编码器采集。
- 通过三相 PWM 输出控制 BLDC 功率级。
- 电角对齐与零点偏置处理。
- 定点 PID 位置环。
- 电压 FOC 输出生成。
- 连续 `ext_raw` 坐标跟踪，用于避免 AS5600 原始角度跨边界时的跳变问题。

最终 demo/testbench 固件验证了电机对齐、开环输出、闭环目标跟踪与分阶段响应观测。

### 遥控输入与演示流程

我将 ELRS 遥控输入接入固件，并将原始通道映射为手动控制、模式选择、安全行为与 demo 触发等系统级语义。

最终 testbench 支持：

- RC 监视。
- 电机对齐测试。
- 电机开环测试。
- 电机闭环测试。
- RC 手动闭环 demo。
- 用于闭环响应记录的 AUTO 阶跃测试。
- 串口状态快照与紧凑遥测输出。

### 本仓库不声称覆盖的内容

本仓库不声称实现完整无人机飞控系统、产品级安全层、完整自主感知链路或端到端空中抓取系统。它聚焦于机械臂电控子系统与定稿演示固件。
