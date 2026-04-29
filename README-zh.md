# AerialRoboArm

**基于 STM32F103、FreeRTOS、BLDC 电机控制、AS5600 位置反馈与 ELRS 遥控输入的轻量型无人机操作臂电控系统样机。**

[English version](./README.md)

本仓库展示本人本科毕业设计项目对应的定稿工程实现。毕业论文正文不包含在本仓库中；本仓库重点展示该项目背后的固件源码、电控系统架构、硬件上下文与最终演示材料。

<p align="center">
  <img src="Figures/DEMO/整机系统全览图.jpg" width="72%" alt="AerialRoboArm 整机系统全览" />
</p>

<p align="center">
  <img src="Figures/DEMO/电控系统原型控制平台.jpg" width="42%" alt="电控系统原型控制平台" />
  <img src="Figures/DEMO/编码器与关节电机安装实物图.jpg" width="42%" alt="AS5600 编码器与 BLDC 关节电机安装" />
</p>

## 概览

AerialRoboArm 是一套面向无人机挂载场景的轻量型可折叠机械臂电控系统样机。项目关注如何在 MCU 资源受限的条件下，构建一套紧凑的控制子系统，同时支持电机控制、遥控操作、分阶段验证与实际工程调试所需的可观测性。

定稿版本 `demo_v6` 被整理为一个工程型作品集案例，而不是通用机器人框架。它的核心价值在于完整的控制链路：STM32F103 外设抽象、FreeRTOS 任务组织、BLDC 电机控制、AS5600 位置反馈、ELRS 遥控语义映射，以及用于最终演示的板载串口测试台。

## One more thing

这个项目之所以能完成，很大程度上依赖于我自己构建的一套 AI-native 开发工作流：**DACMAS**，即 **Designer - APIer - Coder Multi-Agent System**。

实际开发中，我几乎没写什么代码。整个项目更接近一个由我作为 human-in-the-loop 调度者主导的 AI 辅助工程流程：我负责系统目标、上下文路由、接口审查、物理验证与最终决策，而不同 AI Chatbox 分别承担不同认知角色：

- **Designer**：维护全局架构、系统行为、状态机意图以及 L1-L5 分层边界。
- **APIer**：将设计简报转化为明确的 C 语言契约，尤其是 `.h` 头文件、数据结构、函数签名与接口约束。
- **Coder**：在接口契约限制下实现局部 `.c` 文件，关注嵌入式 C 细节、错误处理、栈安全、时序约束与硬件行为。

这个项目已经不仅仅是简单的“用了 AI 辅助写代码”了，这个项目是一个 **AI-native 的硬件项目**：通过契约驱动、上下文隔离，并通过AI辅助真实硬件调试完成。我的工程工作**只是**定义架构、约束智能体、维护仓库中的事实源、审查生成代码、编译烧录、观察实机失败，并把反馈送回正确的角色中继续迭代。

完整方法论见 [About AI Assist Pipeline - Share.md](./Doc/About%20AI%20Assist%20Pipeline%20-%20Share.md)。

## 我的工作

我负责项目中的嵌入式电控系统部分，主要工作包括：

- STM32F103 固件架构设计与 FreeRTOS 任务组织。
- UART、I2C、PWM、AS5600 反馈、ELRS 输入、BLDC 功率级输出与舵机输出相关的 BSP / Driver 集成。
- 面向折叠臂 BLDC 主轴的定点 PID 与电压 FOC 控制链路。
- 基于 AS5600 的跨圈安全 `ext_raw` 连续位置表达，用于闭环位置控制。
- ELRS 遥控通道解析，以及面向手动控制、模式控制和安全行为的语义映射。
- 串口 testbench 设计，支持电机对齐、开环测试、闭环测试、RC 手动控制、AUTO 阶跃测试与遥测观测。
- 与本科毕业设计定稿范围一致的仓库文档整理。

更详细的个人工作说明见 [Doc/MY_WORK.md](./Doc/MY_WORK.md)。

## 本仓库内容

本仓库包含 AerialRoboArm 电控子系统的定稿固件源码、整理后的设计说明、硬件上下文图片与演示媒体。本仓库不包含毕业论文正文。

主要内容包括：

- STM32CubeMX 生成的工程配置与 STM32F103 固件源码。
- `User/` 目录下的分层用户固件，包括 BSP、驱动、算法/模块、任务与应用/testbench 代码。
- 固件工程所需的 HAL、CMSIS 与 FreeRTOS 依赖。
- `Figures/` 目录下的硬件与系统图片。
- `Doc/` 目录下整理后的设计、硬件、演示与归档文档。

## 演示媒体

以下媒体文件用于展示最终样机与验证流程。

| 素材 | 说明 |
| --- | --- |
| [整机系统全览图](./Figures/DEMO/整机系统全览图.jpg) | 集成样机的实物整体视图。 |
| [电控系统原型控制平台](./Figures/DEMO/电控系统原型控制平台.jpg) | STM32 电控平台与接线环境。 |
| [编码器与关节电机安装实物图](./Figures/DEMO/编码器与关节电机安装实物图.jpg) | AS5600 编码器与 BLDC 关节电机安装方式。 |
| [实机闭环控制折叠臂模拟动作演示](./Figures/DEMO/实机闭环控制折叠臂模拟动作演示.mp4) | 实机闭环运动演示。 |
| [基于 ELRS 遥控器的手动闭环控制演示](./Figures/DEMO/基于%20ELRS%20遥控器的手动闭环控制演示.mp4) | 基于 ELRS 遥控输入的手动闭环控制演示。 |

## 最终演示范围

当前 `demo_v6` 固件是定稿的板载 testbench/demo 版本。它验证了项目使用的电控主链路：

- BLDC 电机电角对齐。
- 电机开环电压矢量测试。
- 电机闭环位置控制测试。
- AS5600 编码器反馈与连续 `ext_raw` 位置跟踪。
- ELRS 遥控输入解析与语义映射。
- 从 RC 输入到电机目标的 MANUAL 控制桥接。
- 用于分阶段闭环响应记录的 AUTO 阶跃测试。
- 串口控制台菜单、状态快照日志与紧凑遥测输出。

`Core/Src/main.c` 中的当前固件入口调用 `App_Testbench_Init()`，用于挂载双线程 demo/testbench 运行时。

## 系统架构

固件被组织为分层嵌入式系统。该设计将芯片级硬件访问、设备驱动、可复用模块、任务级 runnable 与应用/testbench 容器分离。

<p align="center">
  <img src="Figures/DEMO/软件架构图.png" width="82%" alt="AerialRoboArm 软件架构图" />
</p>

| 层级 | 职责 | 主要目录 |
| --- | --- | --- |
| L5 Application | 物理线程容器与 demo/testbench 编排 | `User/app` |
| L4 Task | Motion、RC 与 manipulator runnable | `User/task` |
| L3 Module / Algorithm | PID、电压 FOC、执行器抽象、RC 语义 | `User/mod` |
| L2 Driver | 设备级硬件驱动 | `User/drv` |
| L1 BSP | STM32 外设抽象 | `User/bsp` |

最终 demo 运行时采用双频结构：

- 1 kHz 高频控制线程，用于电机反馈、控制计算、FOC 输出与 PWM 更新。
- 50 Hz 低频逻辑线程，用于控制台输入、RC 处理、demo 状态转移、日志输出与 testbench 编排。

<p align="center">
  <img src="Figures/DEMO/单轴运动控制测试流程图.png" width="78%" alt="单轴运动控制测试流程图" />
</p>

## 硬件平台

样机基于资源受限的 STM32F103 控制平台构建。硬件选型强调低成本、轻量化，以及与无人机挂载机械臂场景的直接相关性。

| 组件 | 作用 |
| --- | --- |
| STM32F103C8T6 | 嵌入式主控。 |
| FreeRTOS | 双频运行时调度。 |
| BLDC 云台电机 | 折叠臂关节执行器。 |
| SimpleFOCMini / BLDC 功率级 | 三相电机驱动接口。 |
| AS5600 磁编码器 | 基于 I2C 的关节位置反馈。 |
| ELRS 接收机 | 低延迟遥控输入。 |
| 舵机输出 | 低频夹爪 / 辅助执行器输出。 |
| 定制 / 原型电控平台 | 电气集成与分阶段调试。 |

## 仓库结构

```text
Core/          STM32CubeMX 生成的核心源码、中断与系统文件
Drivers/       STM32 HAL 与 CMSIS 依赖
Middlewares/   FreeRTOS 中间件
User/app/      应用层与 demo/testbench 线程容器
User/task/     motion、RC 与 manipulator 任务级 runnable
User/mod/      PID、FOC、RC 语义等算法与功能模块
User/drv/      AS5600、BLDC 功率级、ELRS、舵机与状态 IO 设备驱动
User/bsp/      UART、I2C、PWM、GPIO 等 MCU 外设抽象
Doc/           整理后的文档、个人贡献说明、演示说明与归档资料
Figures/       README 与最终 demo 媒体素材
```

## 文档入口

有用的文档入口：

- [文档索引](./Doc/README.md)
- [个人工作与贡献边界](./Doc/MY_WORK.md)
- [最终演示说明](./Doc/demo/final-demo.md)
- [系统顶层设计约束](./Doc/design/00_SYSTEM_CONSTITUTION.md)
- [运行时架构](./Doc/design/01_RUNTIME_ARCHITECTURE.md)
- [引脚表](./Doc/hardware/pinmap.md)

历史记录和早期草案保留在 `Doc/archive/` 下，用于追溯项目演进。README 表示经过整理后的作品集级项目视图。

## 边界与限制

本仓库展示的是本科阶段工程样机，而不是产品级无人机系统。项目重点是机械臂电控子系统与最终 demo/testbench 固件。

本仓库不覆盖以下内容：

- 无人机飞控、姿态控制或导航闭环。
- 产品级故障恢复与安全认证。
- 完整自主感知、规划与端到端空中抓取。
- 毕业论文正文。

## 致谢

本项目基于 STM32 生态以及若干重要开源 / 厂商组件完成：

- [FreeRTOS](https://www.freertos.org/) 用于实时调度。
- STMicroelectronics HAL 与 CMSIS 包用于 STM32F1 支持。
- [SimpleFOC](https://simplefoc.com/) 及其社区资料作为 BLDC / FOC 学习参考。
- DengFOC 与相关社区教程在早期电机控制调试中提供了帮助。
