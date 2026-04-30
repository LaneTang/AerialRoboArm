# Final Demo / Testbench Notes

[English](#english) | [中文](#中文)

## English

The finalized `demo_v6` firmware is organized around an on-board serial testbench. It is intended to demonstrate and validate the main electrical-control chain of the AerialRoboArm prototype, rather than serve as a general-purpose robotic-arm firmware framework.

## Demo Scope

- BLDC electrical alignment.
- BLDC open-loop voltage-vector output test.
- BLDC closed-loop position-control test.
- AS5600 feedback acquisition and continuous `ext_raw` position tracking.
- ELRS remote-input decoding and semantic mapping.
- RC manual bridge from remote input to closed-loop motor target.
- AUTO step test for staged closed-loop response recording.
- Serial console snapshots and compact telemetry output.

## Media

| Material | Description |
| --- | --- |
| [Full system overview](../../Figures/DEMO/整机系统全览图.jpg) | Integrated physical prototype. |
| [Electrical-control prototype platform](../../Figures/DEMO/电控系统原型控制平台.jpg) | STM32-based control platform and wiring context. |
| [Encoder and joint motor installation](../../Figures/DEMO/编码器与关节电机安装实物图.jpg) | AS5600 encoder and BLDC joint motor installation. |
| [Closed-loop folding-arm demo video](../../Figures/DEMO/实机闭环控制折叠臂模拟动作演示.mp4) | Physical closed-loop motion demonstration. |
| [ELRS manual closed-loop control demo video](../../Figures/DEMO/基于%20ELRS%20遥控器的手动闭环控制演示.mp4) | Manual closed-loop operation through ELRS remote input. |

## 中文

定稿版本 `demo_v6` 固件围绕板载串口 testbench 组织。它用于展示并验证 AerialRoboArm 样机的电控主链路，而不是作为通用机械臂固件框架。

## 演示范围

- BLDC 电角对齐。
- BLDC 开环电压矢量输出测试。
- BLDC 闭环位置控制测试。
- AS5600 反馈采集与连续 `ext_raw` 位置跟踪。
- ELRS 遥控输入解析与语义映射。
- 从遥控输入到闭环电机目标的 RC 手动桥接。
- 用于分阶段闭环响应记录的 AUTO 阶跃测试。
- 串口状态快照与紧凑遥测输出。

## 媒体素材

| 素材 | 说明 |
| --- | --- |
| [整机系统全览图](../../Figures/DEMO/整机系统全览图.jpg) | 集成样机实物。 |
| [电控系统原型控制平台](../../Figures/DEMO/电控系统原型控制平台.jpg) | STM32 电控平台与接线环境。 |
| [编码器与关节电机安装实物图](../../Figures/DEMO/编码器与关节电机安装实物图.jpg) | AS5600 编码器与 BLDC 关节电机安装方式。 |
| [实机闭环控制折叠臂模拟动作演示](../../Figures/DEMO/实机闭环控制折叠臂模拟动作演示.mp4) | 实机闭环运动演示。 |
| [基于 ELRS 遥控器的手动闭环控制演示](../../Figures/DEMO/基于%20ELRS%20遥控器的手动闭环控制演示.mp4) | 基于 ELRS 遥控输入的手动闭环控制演示。 |
