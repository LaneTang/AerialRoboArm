# 🚀 分支开发简报: `feature/demo_v4_vision_and_end_effector`

**📅 日期**: 2026-03-19
**🎯 核心目标**: 实现 ARA-VSP 视觉单向透传协议解析、1-DOF 折叠臂空间映射状态机，并完成末端执行器（夹爪与 Roll 轴姿态）的底层驱动与业务模块封装。
**👤 开发模式**: Cortex-M3 (STM32F103) 无 FPU 严格资源限制环境，纯定点数/整型运算。

## 🛠️ 核心功能提交清单 (Changelog)

### 1. L2 硬件驱动层: 通用 PWM 数字舵机驱动 (`drv_servo.c/.h`)
* **整型映射优化**：针对 500-2500$\mu s$ 脉宽对应 $0^\circ \sim 180^\circ$ 的硬件特性，推导并实现了纯整型无溢出公式 `Pulse = 500 + ((Angle * 100) / 9)`，彻底杜绝了浮点运算开销。
* **物理防堵转保护**：引入了软件级的限位保护 (`min_limit` / `max_limit`)。当传入角度越界时，会自动截断至安全范围并抛出 `DRV_SERVO_LIMIT_REACHED` 警告，防止舵机过载烧毁。
* **面向对象设计 (OOP)**：通过 `DrvServo_Context_t` 句柄封装硬件状态，无全局变量，可无缝复用于多路舵机。

### 2. L3 业务模块层: 末端执行器控制 (`mod_actuator.c/.h`)
* **硬件语义解耦**：将底层的 PWM 脉宽抽象为高层业务语义。向上层 L4 提供 `ModActuator_SetGripper(percent)` (0-100% 闭合度) 和 `ModActuator_SetRoll(degree)` (姿态角) 接口。
* **标定线性插值**：夹爪闭合度通过 `gripper_open_angle` 和 `gripper_close_angle` 两个基准点进行带符号 32 位整型线性插值映射，方便后续根据机械公差进行微调。
* **多实例聚合**：在 `ModActuator_Context_t` 中聚合了 `servo_gripper` 和 `servo_roll` 两个子实例，统一管理末端形态。

### 3. L3 协议层: ARA-VSP 解析器 (`mod_vsp_parser.c/.h`)
* **零阻塞状态机**：实现专为 UART DMA 环形缓冲区设计的逐字节解析器，支持 `0xAA 0x55` 帧头校验、`Seq_ID` 跟踪与 Checksum 验证，安全提取目标距离与姿态数据。

### 4. L4 决策层: 自主机械臂控制任务 (`task_manipulator.c/.h`)
* **四阶自主状态机**：构建 `IDLE` -> `SEEKING` -> `CONVERGING` -> `GRABBING` -> `RETRACTING` 闭环控制逻辑。
* **1-DOF 空间降维反解 (IK)**：采用 **离线标定查表 (LUT) + 纯整数线性插值** 算法，将 Z 轴深度 (mm) 极速映射为大臂关节的 FOC 电机角度。
* **频率墙穿透与防抖**：采用定点一阶低通滤波 (`LPF_ALPHA`) 将 20Hz 视觉阶跃信号平滑为 1000Hz 物理指令；结合 $5\text{mm} / 200\text{ms}$ 空间与时间双重防抖机制，保障抓取动作的鲁棒性。
  这是一次极其精准且切中要害的架构审查！作为您的 AI 开发助手，我完全赞同您的结论。

您指出的**“逻辑模块（Module）不等于系统线程（Thread）”**是资源受限的嵌入式开发中最核心的黄金法则。在 STM32F103 的 **20KB RAM** 物理极限下，盲目将 5 个逻辑模块映射为 5 个 FreeRTOS 任务，光是分配独立的栈空间（Stack）和任务控制块（TCB）就会瞬间吃干抹净内存，导致栈溢出和频繁的上下文切换开销，彻底毁掉 1000Hz FOC 电机控制的实时性。

您的**“线程降维与合并 (Thread Condensation)”**方案——即剥离出 **1000Hz 高频实时控制线程** 与 **50Hz 低频业务超级循环线程**——是非常经典且稳健的机载控制架构（类似 PX4/ArduPilot 的底层调度思想）。

以下是基于您最新的架构重构指示，对当前已完成代码的分析以及接下来的开发规划：

---

## 深度审查报告及排雷建议

**在逻辑层面，这种按功能划分任务的方案非常完美；但在 STM32F103C8T6（20KB RAM）的物理极限下，将设计简报中的 5 个逻辑 Task 全部映射为独立的 FreeRTOS 线程，是一个极其危险的架构反模式（Anti-pattern）。**

如果您严格按照现有的 1对1 映射关系（`Task_Motion`, `Task_RC`, `Task_Vision`, `Task_Manipulator`, `Task_Scheduler` 全部作为独立线程），您的系统大概率会在上电几分钟内因为**内存栈溢出（Stack Overflow）**或**调度器饥饿（Scheduler Starvation）**而崩溃。

以下是针对 FreeRTOS 线程部署方案的“挤水”审查与重构建议：

### 1. 为什么现有的独立线程方案不可行？

* **致命的 RAM 危机（栈空间消耗）**：FreeRTOS 中，每个线程都需要独立分配 Stack。为了保证不溢出，每个业务线程至少需要分配 128 到 256 字（512B - 1KB）的栈空间。5 个独立线程加上 FreeRTOS 内核的 Idle/Timer 线程，光是维护“线程的外壳”就会瞬间吃掉您 20KB RAM 中的 **6KB - 8KB**。再算上全局变量、DataHub、串口/I2C的 DMA 缓冲区，RAM 会被彻底榨干，完全没有余量应对中断嵌套。
* **昂贵的上下文切换（CPU 损耗）**：STM32F103 运行在 72MHz。`Task_Motion` 需要以 1000Hz 运行（每 1 毫秒打断一次系统）以执行 FOC 运算。如果剩下的视觉、遥控、决策任务还要在同级或低优先级互相抢占、频繁切换上下文，不仅浪费大量 CPU 时钟周期，还会增加共享数据（DataHub）锁冲突的概率。

### 2. 架构师重构方案：线程降维与合并 (Thread Condensation)

在资源极度受限的 MCU 上，**逻辑模块（Module）不等于系统线程（Thread）**。那些运行频率相同、没有严格抢占需求的业务，必须被合并到同一个线程里的“超级循环（Super-Loop）”中顺序执行。

我们需要将原本的 5 个线程，**强制压缩为 2 个核心物理线程**：

#### 物理线程一：`Thread_HighFreq_Ctrl` (高频控制线程)
* **角色**：系统的“小脑”与“肌肉”。
* **优先级**：`osPriorityRealtime` (最高级，绝不能被其他任务打断)。
* **运行频率**：1000Hz (1ms 周期)。
* **包揽的逻辑 Task**：
    * 仅包含原 `Task_Motion` 的核心内容。
    * 读取编码器 -> 执行 FOC 算法 -> 输出 PWM。
* **内存策略**：分配约 1KB 栈空间。内部严禁使用任何阻塞延时（`vTaskDelay`），严禁执行复杂的条件分支。

#### 物理线程二：`Thread_LowFreq_Logic` (低频业务线程)
* **角色**：系统的“大脑”与“感知”。
* **优先级**：`osPriorityNormal`。
* **运行频率**：50Hz (20ms 周期，完全契合视觉 20Hz 和遥控器 50Hz 的节拍)。
* **包揽的逻辑 Task**（在一个 `while(1)` 循环内顺序执行）：
    1.  **原 `Task_RC`**：检查 UART 环形队列，解析 CRSF 数据。
    2.  **原 `Task_VisionRx`**：检查 UART 环形队列，解析 9 字节 ARA-VSP 视觉透传协议。
    3.  **原 `Task_CentralScheduler`**：检查上一轮的时间戳，进行 500ms 断联超时判断，更新系统模式（Auto/Manual/Error）。
    4.  **原 `Task_ArmManipulator`**：执行机械臂“寻的-收敛-抓取”的核心状态机（FSM）逻辑。将计算出的最新目标位置通过结构体推给高频控制线程。
* **架构收益**：这四个模块因为是顺序执行的，**共享同一个栈空间（省下 2-3KB RAM）**，且它们之间的数据传递直接通过局部变量或结构体指针即可，**彻底消除了互相锁死（Deadlock）和上下文切换的开销**。

### 3. 数据交换 (DataHub) 的降级优化

一旦我们将低频任务合并进一个线程，原本设计在 `DataHub` 中用于防止多线程读写冲突的重度互斥锁（Mutex）就可以大大简化。
* 只有在 `Thread_LowFreq_Logic` 向 `Thread_HighFreq_Ctrl` 传递目标角度时，才需要使用最轻量级的 `taskENTER_CRITICAL()` 屏蔽一下中断，将几个整型变量快速拷贝过去，然后立即退出临界区。

---

### 📊 架构重构后的影响评估：因祸得福

万幸的是，我们前期坚持了**“严格分层”**和**“面向对象 (OOP) 上下文封装”**的设计原则，这次架构大地震对我们已写好的底层代码**几乎零破坏**：

1. **完全保留的资产 (L2/L3 层)**：
    * `mod_vsp_parser.c` (视觉解析)、`drv_elrs.c` (遥控解析) 都是纯状态机，不包含任何 `vTaskDelay` 或 RTOS 依赖。它们可以直接被放入 `Thread_LowFreq_Logic` 中轮询。
    * `mod_actuator.c` (末端执行器)、`drv_servo.c` (舵机驱动)、`alg_voltage_foc.h` (FOC 算法) 是纯粹的输入输出映射，可以在任何线程中被安全调用。
2. **需要重构的资产 (L4/L5 层)**：
    * **`task_manipulator.c` 降级**：之前我们把它写成了一个带 `for(;;)` 和 `vTaskDelayUntil` 的死循环任务。现在，需要剥离掉它的 FreeRTOS 外壳，将其降级为一个普通的业务函数（例如 `void Manipulator_Update(void)`），由 `Thread_LowFreq_Logic` 每 20ms 调用一次。
    * **`DataHub` 瘦身**：原计划的 Mutex 互斥锁必须废弃（Mutex 会引发上下文切换和优先级反转）。高频线程和低频线程之间的数据交换，必须改用极其轻量的 `taskENTER_CRITICAL()` / `taskEXIT_CRITICAL()` 保护的内存快拷。

---

### 📈 最新项目进度分析报告 (基于双线程架构)

**当前整体进度：约 70%**（底层与算法已就绪，顶层调度待重构）

| 模块层级 | 组件名称 | 状态 | 适配新架构的说明 |
| :--- | :--- | :--- | :--- |
| **L2 驱动层** | PWM/I2C/UART/Servo | 🟢 已完成 | 无 OS 依赖，直接可用。 |
| **L3 算法层** | FOC / PID 控制器 | 🟢 已完成 | 纯数学运算，直接供 `Thread_HighFreq` 调用。 |
| **L3 协议层** | VSP 视觉 / ELRS 遥控 | 🟢 已完成 | 纯状态机，直接供 `Thread_LowFreq` 轮询。 |
| **L3 业务层** | 夹爪与姿态控制模块 | 🟢 已完成 | 暴露百分比与角度接口，可被安全调用。 |
| **L4 决策层** | 机械臂自动抓取 FSM | 🟡 需微调 | 需剥离 `vTaskDelay`，改为被动调用的 `Update` 函数。 |
| **L4 数据枢纽** | DataHub (黑板模式) | 🔴 待重构 | 需将 Mutex 改为临界区保护，定义双线程握手结构体。 |
| **L5 调度层** | **低频业务线程** (50Hz) | 🔴 待开发 | 需建立主循环，集成解析、状态机与看门狗。 |
| **L5 控制层** | **高频控制线程** (1000Hz)| 🔴 待开发 | 需建立高优先级实时循环，执行 FOC 闭环。 |

---

### 🚧 接下来三步的开发规划 (The Action Plan)

为了稳妥地将现有积木拼装进您的双线程架构中，我们接下来的开发步骤如下：

* **Step 1：重构 `DataHub` (轻量级数据枢纽)**
    * 废弃耗时的 Mutex。
    * 建立 `DataHub_ReadTarget(...)` 和 `DataHub_WriteTarget(...)` 接口，使用 `taskENTER_CRITICAL()` 保护短短几微秒的结构体拷贝。
* **Step 2：构建大脑 - `Thread_LowFreq_Logic` (50Hz 超级循环)**
    * 创建该线程，设置 `osPriorityNormal`。
    * 在循环内依次调用：遥控解析 -> 视觉解析 -> 看门狗校验 -> 状态机更新（原 Task_Manipulator 逻辑） -> 将结果写入 DataHub。
    * 循环末尾使用 `vTaskDelayUntil` 保证严格的 20ms 节拍。
* **Step 3：构建肌肉 - `Thread_HighFreq_Ctrl` (1000Hz 实时循环)**
    * 创建该线程，设置 `osPriorityRealtime`。
    * 在循环内依次调用：读取 AS5600 编码器 -> 从 DataHub 获取 L4 下发的目标位置 -> 执行 PID 与 FOC -> 输出 PWM。
    * 绝对禁止在此时序内加入任何阻塞操作。
刀。您是否希望我立刻为您输出**“无锁、基于临界区保护的轻量级 DataHub (`datahub.c/.h`)”**的重构代码？这将为后续的两个物理线程奠定安全的通信基础。