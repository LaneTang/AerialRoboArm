# AerialRoboArm (ARA) - 机载折叠机械臂控制系统

[![Generic badge](https://img.shields.io/badge/MCU-STM32-blue.svg)](https://www.st.com/)
[![Generic badge](https://img.shields.io/badge/RTOS-FreeRTOS-orange.svg)](https://www.freertos.org/)
[![Generic badge](https://img.shields.io/badge/Control-SimpleFOC-green.svg)](https://simplefoc.com/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)]()

## 项目简介 (Introduction)

**AerialRoboArm (ARA)** 是一个专为机载环境（如无人机挂载）设计的轻量化、可折叠机械臂实验平台。目前项目还在原型验证开发中，下文提到的各项设计与实现均为初版，未来将持续迭代优化。

本项目旨在解决空中抓取任务中对**载荷重量敏感**与**控制精度要求高**之间的矛盾。通过采用 **STM32** 作为主控核心，结合 **SimpleFOC** 矢量控制算法驱动无刷电机（BLDC），实现了机械臂在平面内的稳定位置闭环与快速响应。软件架构基于 **FreeRTOS** 实时操作系统，采用分层模块化设计，具备良好的扩展性与移植性。

##  核心特性 (Key Features)

* **轻量化设计**：采用 2804 云台电机直驱与连杆折叠机构，极致压缩自重。
* **高精度控制**：基于 FOC（磁场定向控制）算法与外环 PID 位置控制，实现平滑、无抖动的关节运动。
* **分层软件架构**：严格遵循 L1-L5 分层设计，实现业务逻辑与硬件驱动解耦。
* **实时多任务**：基于 FreeRTOS (Heap_4) 管理运动控制、传感器融合与人机交互任务。
* **低成本方案**：使用 SimpleFOCMini 驱动板与 AS5600 磁编码器，大幅降低硬件门槛。

---

## 系统架构 (System Architecture)

本项目软件遵循模块化分层设计原则，确保代码的高内聚低耦合。

![Software Architecture](Figures/README/SwArct.png)
ED --> BSP_GPIO

### 软件分层说明

* **L5 Application Layer (应用层)**
    * `Central Scheduler`: 中央调度器，负责任务编排与系统状态机管理。
* **L4 Business Logic Layer (业务逻辑层)**
    * `Motion Control`: 运动控制核心，包含 PID 闭环计算。
    * `Arm Manipulator`: 机械臂运动学解算。
    * `DataHub`: 数据交换中心，负责各任务间的高效通信。
* **L3 Functional Module Layer (功能模块层)**
    * `Motor Module`: 封装 SimpleFOC 库，提供统一的电机控制接口。
    * `Logger`: 基于缓冲区的日志系统。
    * `Cmd Parser`: 指令解析器，处理遥控或上位机指令。
* **L2 Hardware Driver Layer (硬件驱动层)**
    * 包含 AS5600 编码器、激光测距模块、ELRS 通信模块及舵机驱动。
* **L1 BSP Layer (板级支持包)**
    * 基于 HAL 库，封装 UART (DMA+RingBuf)、I2C (DMA+Mutex)、PWM 等底层操作。

---

## 机械与电子设计 (Mechanical & Electronics)

为了实现“机载轻量化”与“系统高集成度”的目标，本项目并未止步于现成的模块拼接，而是同步进行了定制化的机械结构建模与 PCB 电路设计。

### 机械结构优化 (Mechanical Optimization)

机械臂本体采用**连杆折叠机构**设计，旨在解决无人机挂载时的空间收纳痛点。

![Mechanical Design](Figures/README/mech_design.png)
*(注：上图展示了当前的连杆传动与齿轮减速方案)*

* **折叠收纳**：通过多连杆机构实现大范围的折叠与展开，收纳状态下重心贴近机身，减少对飞行姿态的影响。
* **齿轮传动**：自定义齿轮组设计，在保证 BLDC 电机高速响应的同时，提供足够的末端抓取力矩。
* **迭代开发**：目前结构处于持续迭代中，重点优化连杆的拓扑结构以减轻重量，并增加关键部位的机械强度。

### 定制化电路设计 (Custom PCB Design)

为了摆脱杜邦线连接带来的不稳定性并减小电控体积，项目自主设计了 **ARA电控调试板 (ARA Control Board V0.21)**。

![PCB Design](Figures/README/pcb_design.png)

* **高集成度**：板载集成了 TB6612 驱动芯片（用于夹爪直流电机控制）、AMS1117 电源管理及 IMU 惯导接口。
* **接口标准化**：引出了符合 SimpleFOCMini 接口定义的 PWM/Enable 排针，以及用于 Laser/Ultrasonic 传感器的专用接口。
* **抗干扰设计**：针对机载高频振动与电磁环境，优化了 PCB 走线布局，增强了信号完整性。

---

## 硬件环境 (Hardware Setup)

### 核心组件

| 组件 | 型号/参数 | 说明 |
| :--- | :--- | :--- |
| **主控芯片** | STM32 Series | 高性能 ARM Cortex-M 内核 |
| **关节电机** | 2804 BLDC | 无刷云台电机，低齿槽转矩 |
| **电机驱动** | SimpleFOCMini v1.0 | 3路 PWM 输入，最大支持 30V |
| **位置反馈** | AS5600 | 12-bit 磁编码器，I2C 接口 |
| **测距传感器** | Laser Sensor | 单点激光测距，用于对地/障碍感知 |
| **通信链路** | ELRS Receiver | 低延迟远距离遥控接收 |

### 接线说明 (Wiring)

**2804 电机与驱动板连接：**
* 电机三相线 (U/V/W) -> 驱动板 OUT (无顺序要求，软件可校准)。
* 编码器与电机机械连接，I2C 信号线接入 MCU。

**SimpleFOCMini 引脚定义：**

| 引脚 | 功能 | MCU 连接推荐 | 备注 |
| :--- | :--- | :--- | :--- |
| **IN1/2/3** | PWM 输入 | TIM_CHx | 用于控制电机三相电压 |
| **EN** | 使能输入 | GPIO Output | 高电平有效 |
| **VCC/GND** | 电源输入 | 12V DC | 推荐 3S 锂电池供电 |
| **3.3V** | 辅助输出 | - | 仅用于给编码器等低功耗设备供电 |

---

## 开发环境 (Development Environment)

* **IDE**: CLion (2025.x+)
* **Build System**: CMake
* **Config Tool**: STM32CubeMX
* **Compiler**: arm-none-eabi-gcc
* **Debugger**: OpenOCD / DapLink / ST-Link

### 快速开始 (Getting Started)

1.  **克隆仓库**
    ```bash
    git clone [https://github.com/LaneTang/AerialRoboArm.git](https://github.com/LaneTang/AerialRoboArm.git)
    ```
2.  **打开项目**
    使用 CLion 打开项目根目录，等待 CMake 加载完成。
3.  **配置硬件**
    根据您的 MCU 型号，在 `STM32CubeMX` 中配置相应的 PWM 输出引脚与 I2C DMA 通道。
4.  **编译与烧录**
    点击 Build 并在 Run/Debug Configuration 中配置好下载器，即可烧录。

---

## 控制与交互 (Control)

* **遥控模式 (ELRS)**: 通过遥控器摇杆直接映射机械臂角度与夹爪开合。
* **调试模式**: 连接串口 (Baud 115200)，使用 CLI 指令进行 PID 在线调参或状态监控。
* **状态指示**: 板载 LED 指示当前系统状态（初始化/校准中/就绪/故障）。

---

## 特别声明：现代化的开发体验 (Powered by Modern Toolchain)

本项目代码 **100% 在 CLion + STM32CubeMX 环境下开发完成**。

我们在开发过程中深刻体会到了现代化工具链带来的效率飞跃，在此特别推荐这套“黄金组合”：

* **优雅的现代化 UI**: 告别上个世纪的界面风格，CLion 提供了赏心悦目的 Dark Mode 与极简视觉设计，让嵌入式开发也能成为一种视觉享受。
* **极致的代码智能**: 得益于 JetBrains 强大的 IntelliSense 引擎，无论是代码补全、智能导航还是即时重构（Refactoring），都能让编码如丝般顺滑，大幅降低心智负担。
* **强大的调试能力**: 内置的 OpenOCD/GDB 调试界面直观清晰，支持变量实时监控、内存视图以及 RTOS 任务状态查看，让 Bug 无处遁形。
* **CMake 构建系统**: 拥抱通用的 CMake 构建方式，配合 STM32CubeMX 的代码生成，真正实现了从底层配置到上层逻辑开发的无缝衔接。

如果你还在忍受传统 IDE 匮乏的代码编辑体验（比如说uvision5），臃肿的软件架构带来的低性能与响应速度（比如说Stm32CubeIDE），低项目兼容性（比如说uvision5）和辣眼睛的UI（比如说uvision5），不妨尝试迁移到 CLion，开启全新的嵌入式开发体验！

[Clion开发stm32指路](https://zhuanlan.zhihu.com/p/145801160)

---

## 贡献与致谢 (Credits)

本项目核心控制算法基于优秀的开源项目 [SimpleFOC](https://github.com/simplefoc/Arduino-FOC)。

**致谢名单：**
* **SimpleFOC Community**: 提供了强大的 FOC 算法库与活跃的社区支持。
* **FreeRTOS**: 提供了稳定高效的实时操作系统，保证了多任务调度的实时性。

**特别鸣谢：**
* **灯哥开源 (DengFOC)**: 感谢灯哥在 Bilibili 制作的 [FOC 系列教程](https://space.bilibili.com/493192058) 为本项目提供了宝贵的入门指引。
* **DengFOC_Lib**: 参考了 [DengFOC](https://github.com/ToanTech/DengFOC_Lib) 的部分实现逻辑，对理解 FOC 底层原理帮助巨大。

---

*Document generated by ARA Dev Assistant.*