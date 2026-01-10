
# 多智能体协作系统（Multi-Agent System）with HITL based on AI Chatbox

## Previous Work

目前业界主要有两个流派：**单体增强流（Copilot模式）** 和 **多智能体自治流（Agent模式）**

对于前两年比较火爆的单体增强流 如Copilot，基于**对单一AI注入大量上下文信息（包括项目背景、代码库、设计文档等）** 来让AI同时完成项目的逻辑设计和代码实现。

优点是操作简单，用户只需要和一个AI对话即可完成所有工作，对用户来说门槛较低。

这种方法的缺点是上下文信息过多时，AI很容易出现信息过载而导致幻觉。而且，对于这种单体AI来说，很考验prompt提示词的水平，AI对结构性语言的处理能力优秀，但是对于非结构化的自然语言理解能力较差，长上下文引出的注意力分散很容易导致AI在某些局部信息中出现幻觉而出错。

出错比如，

- AI在生成代码时忽略了某些关键的项目约束（比如内存限制、实时性要求等），从而生成了不符合要求的代码。
- AI在处理复杂的逻辑时，可能会出现逻辑错误，导致生成的代码无法正确实现预期的功能。
- AI在面对模糊或不完整的需求时，可能会做出错误的假设，从而生成不符合用户期望的代码。
- AI在处理多模块交互时，可能会忽略某些模块之间的依赖关系，导致生成的代码无法正确协同工作。

相比之下，**多智能体自治流（Agent模式）** 则是将复杂任务拆分成多个子任务，并分配给不同的AI智能体来完成。每个智能体专注于自己的子任务，并通过协作来完成整体任务。这种方式的优点是可以充分利用不同AI智能体的优势，从而提高整体任务的完成效率和质量；缺点是需要设计复杂的协作机制来确保各个智能体之间的有效沟通和协作。

对于一个完整项目而言，Copilot模式并不适用处理复杂的多模块协作和分层架构设计。

故在本项目中，我们参考**多智能体自治流（Agent模式）** 采用实现了一个人工模拟的**多智能体协作系统（Multi-Agent System）** 的思路，设计了一个包含三种角色的AI工作流： Designer - APIer - Coder 三角色工作流 具体如下。

## Initial Idea

本项目计划长时间使用三个AI Chatbox进行工作，一个AI Chatbox 1号以APIer mode进行API文档分析，生成和管理；一个AI Chatbox以coder mode进行具体的代码生成和功能调试；一个AI Chatbox以designer mode进行顶层框架的设计和维护。

我设想的工作流是这样的：在designer 顶层架构 项目需求固定之后，指示APIer 进入特定层级中具体模块的开发，APIer 根据项目需求生成并固定该模块的输入输出API（引用的下层API和向上暴露的API）与该模块/层级要实现的功能和特性，然后基于API需求让coder 进行具体的代码生成和调试工作

## Completed Idea - “Designer - APIer - Coder” Workflow

“Designer - APIer - Coder” 三角色工作流

- Designer (架构师):

    - 职责: 维护全局视角（Global Context），持有System Prompt中的架构规则（L1-L5分层），决定模块划分，处理业务需求变更。

    - 核心产出: 需求文档（PRD）、架构图、模块功能描述。

- APIer (接口工程师/技术负责人):

    - 职责: 承上启下。将Designer的模糊需求转化为精确的代码契约。它需要“压缩”上下文：它不需要知道具体的FOC数学公式（那是Coder的事），但它必须知道FOC模块需要暴露 `SetVelocity` 接口，且输入是 `float` 。

    - 核心产出: .h 头文件、API 签名定义、输入输出约束、数据结构定义。

-   Coder (实现工程师):

    - 职责: 戴着镣铐跳舞。它只关注局部上下文（Local Context）。它不需要知道整个机械臂是用来抓什么的，它只需要知道“实现这个函数，满足这个输入输出，性能要求是1ms”。

    - 核心产出: .c 源文件、单元测试代码、具体的算法实现。

同时，在 workflow 中规定 严格的**交接标准（Handover Protocol）**

**阶段 1：Designer -> APIer (输出：设计描述)**

Designer 不应该只给模糊的话，应该输出伪代码或结构化文本。

- Designer 输出样例:

        "模块：L2_AS5600。功能：读取角度。约束：非阻塞，超时返回错误码。依赖：L1_I2C。"

**阶段 2：APIer -> Coder (核心关键点：.h 文件即 Prompt)**

**这是整个工作流的灵魂。** APIer 的工作应该直接生成 `Draft .h` 文件。

- 操作: 让 APIer 生成完整的 `as5600_driver.h`，包含所有 `struct` 定义、`enum` 错误码、函数原型、以及详细的 Doxygen 注释（作为功能说明）。

- Prompt 传递: 当你启动 Coder Chatbox 时，你的 Prompt 应该是：

    "你是 Coder。这是 `bsp_i2c.h` (底层能力) 和 `as5600_driver.h` (你要实现的目标)。请创建 `as5600_driver.c` 来实现头文件定义的功能。注意：只能调用 bsp 接口。"

**阶段 3：Coder -> Designer (逆向反馈：Reviewer 角色)**

建议在 Coder 完成代码后，增加一个微小的步骤：**自检报告**。 Coder 输出代码的同时，要求它输出：“本实现占用的资源预估（堆栈/耗时）”。如果不满足性能指标，你作为人类需要把这个信息反馈回 Designer。

## Current Work

### ARA Project: HITL-Bridged Multi-Agent System Architecture

#### 1. 系统核心理念

本系统通过**物理隔绝上下文 (Physically Isolated Context)** 策略，将嵌入式开发流程拆解为**三个独立的认知域**。

用户（HITL）作为唯一的数据总线（Data Bus）和物理验证器（Physics Validator），连接三个运行不同模式的 AI Chatbox。

- **Vibe Layer (Designer)**: 意图驱动，关注“系统行为”。
- **Abstraction Layer (APIer)**: 契约驱动，关注“接口定义”。
- **Implementation Layer (Coder)**: 约束驱动，关注“代码实现”。

本系统有五大核心特性：

1. 物理级的上下文隔离 (Physical Context Isolation)

   - 防止思维污染
   - 极高的 Attention Density
   - 明确的认知边界

2. 人类作为“物理状态总线” (Human as the Physical State Bus)

   本地项目文件系统（Local File System）是唯一的 **SSOT (Single Source of Truth)**。

   - 被动式信息域隔离
   - 由HITL负责信息更新，开发者对项目的参与模式从微观干预转为高可控的宏观调度和编排
   - 人类作为唯一的上下文传递者，需要手动确保信息的准确传递和物理验证

3. 契约驱动的开发模式 (Contract-Driven Development)

   Designer 输出的是 结构化简报；APIer 输出的是 C语言头文件 (.h)；Coder 输出的是 源文件 (.c)。

   - 使用显式契约 (Explicit Contracts) 进行交接，而非自然语言的模糊传递。
   - 解耦实现的确定性
   - 可测试性：每个契约都是一个独立的测试边界。

4. 融合式编程体验 (Hybrid Vibe-Rigour Workflow)

   - 上游 (Designer) 是 Vibe Coding：您用自然语言描述“意图”、“感觉”、“行为模式”。

   - 下游 (Coder) 是 Rigorous Engineering：通过 Prompt 约束，强制 AI 进入严谨的嵌入式 C 模式（检查指针、管理堆栈、处理错误）。

5. 逆向反馈闭环 (Reverse Feedback Loop)
   
   编译报错 -> 反馈给 Coder；性能不达标 -> 反馈给 APIer；逻辑死锁 -> 反馈给 Designer。

   - 易重构性，高效迭代

#### 2. 角色定义与职责矩阵

| 角色 (Agent Mode) | 核心关注点 (Focus) | 输入 (Input) | 输出 (Output) | 认知边界 (Knowledge Boundary) |
|-------------------|-------------------|--------------|---------------|-------------------------------|
| Designer (Mode A) | Why & What(逻辑、状态、交互) | 用户自然语言描述(Vibe/Intent) | [Design Brief] (结构化设计简报) | 全局全知。了解L1-L5所有层级，但不产生代码。 |
| APIer (Mode B)    | Interface(类型、签名、封装) | [Design Brief](来自 Designer) | [Contract: .h] (C语言头文件) | 上下游衔接。了解底层BSP能力和上层需求，不涉及具体算法。 |
| Coder (Mode C)    | How(性能、指针、优化) | [Contract: .h]+ 依赖的 .h | [Impl: .c]+ [Self-Check] | 局部受限。仅可见当前模块接口和底层API，无视全局架构。 |


#### 3. 工作管线 (The Pipeline)

此流程为单向“瀑布流”，但在验证阶段包含逆向反馈回路。

**Phase 1: 意图锚定 (The Vibe Phase)**
1. 交互: 用户向 Designer 描述需求（例如：“我想让机械臂归零时，先快速动大臂，碰到限位后回退一点”）。

2. 处理: Designer 结合 L5 状态机规则，分析该需求涉及的模块（L4_Task 和 L2_Driver）。

3. 产出: Designer 输出 [Design Brief]，明确定义“L2_Driver 需要提供 Home() 接口”以及“L4_Task 需要处理的 Timeout 逻辑”。

**Phase 2: 契约固化 (The Contract Phase)**
1. HITL 介入: 用户复制 [Design Brief]，粘贴给 APIer。

2. 处理: APIer 检查 L1 BSP 能力（能否支持该操作？），设计符合 C 语言规范的 struct 和 enum。

3. 产出: APIer 输出 [Contract: xxx.h]。这是整个流程的法律文件，不可轻易更改。

**Phase 3: 逻辑填充 (The Engineering Phase)**
1. HITL 介入: 用户创建本地 .h 文件，将内容粘贴进去。然后将该 .h 内容 + 依赖的 BSP 头文件 粘贴给 Coder。

2. 处理: Coder 戴着“镣铐”跳舞，在不修改 .h 的前提下，编写 .c 实现。它会专注于：不要用递归、不要用动态内存、使用 LUT 优化三角函数。

3. 产出: Coder 输出 [Impl: xxx.c] 和 [Self-Check Report]。

**Phase 4: 物理验证 (The Reality Check)**
1. HITL 介入: 用户将代码合入 IDE，编译、烧录。

2. 验证: 观察机械臂动作。

   - 情况 A (成功): 动作符合预期。

   - 情况 B (编译失败): Coder 代码有误 -> 反馈给 Coder。

   - 情况 C (运行卡死): API 设计不合理（如阻塞了FOC） -> 反馈给 APIer 修改接口。

   - 情况 D (动作逻辑错): 归零顺序不对 -> 反馈给 Designer 修改逻辑。

#### 4. 实现

目前计划基于 Gemini Web Chatbox 的形式 ，通过创建一个专门的 Gem ，将 “Designer - APIer - Coder” 三角色工作流 手动模拟的 **多智能体协作系统** 以 Global Prompt 的形式实现

然后手动打开 三个 Chatbox 窗口，分别充当 **Designer** - **APIer** - **Coder** 三个角色， 通过复制粘贴的形式进行交互和工作流推进。

```md
# Role Definition
你是一名服务于“ARA机载机械臂平台”的嵌入式系统专家。
你运行在一个**多智能体协作系统 (Multi-Agent System)** 中。
**当前运行模式 (Operation Mode) 将由用户在对话开始时指定。**

# Project Context (Immutable Constraints)
1. **Target MCU**: STM32F103C8T6 (Cortex-M3, 72MHz, **No FPU**, 20KB RAM, 64KB Flash).
   - **Resource Alert**: RAM is critical. No recursion. Minimize stack depth. Avoid dynamic allocation (`malloc`) after initialization.
2. **Language**: **Strictly C (C99/C11)**.
   - **NO C++**. Use `typedef struct` and function pointers to simulate Object-Oriented behavior (Context/Handle pattern).
3. **OS**: FreeRTOS (Heap_4).
4. **BSP**: L1 Layer is frozen. **Direct HAL API access is FORBIDDEN**. You MUST use the `BSP_xxx` contracts defined below.

# Architectural Rules (The "Law")
1. **Layer Hierarchy**:
   - **L5 (Scheduler)**: System State Machine. Manages Mode (Init/Idle/Run/Error). No direct HW access.
   - **L4 (Tasks)**: Business Logic (e.g., `MotorTask`, `CommTask`). **Decision & Logging Center**.
   - **L3 (Modules)**: Pure Algorithms (FOC, PID, Protocol). **Stateless & Hardware Agnostic**.
   - **L2 (Drivers)**: Component Drivers (AS5600, DRV8313). Wraps L1 BSP. **Report Error Only**.
   - **L1 (BSP)**: I2C/UART/PWM Wrappers (Provided).

2. **Decoupling & Interaction**:
   - **Vertical**: L4 calls L3/L2. L2 calls L1. **No jumping layers**.
   - **Horizontal**: No direct calls between parallel modules (e.g., Motor Driver cannot call ELRS Driver). Use DataHub/L4 for exchange.

3. **Error Propagation**:
   - **Bottom (L1-L3)**: Must return `Status Enum`. **NEVER** `printf`, `while(1)`, or `HAL_Delay` on error.
   - **Top (L4-L5)**: Check return status. Decide strategy (Retry vs. Stop vs. Alert). Log errors here.

# Operation Modes (Protocol)

## MODE A: DESIGNER (Architect)
- **Goal**: Define logic, boundaries, and data flow.
- **Input**: User requirements.
- **Output**: **[Design Brief]** block.
  - Must specify: Module Name, Layer, Functionality, Dependencies, Error Strategy.
- **Behavior**: Maintain global view. Do NOT write code.

## MODE B: APIer (Interface Engineer)
- **Goal**: Convert Design Brief into C Header Contracts.
- **Input**: **[Design Brief]**.
- **Output**: **[Contract: xxx.h]** block.
  - Must include: `typedef struct` Context, `enum` ErrorCodes, Doxygen comments (blocking/non-blocking spec).
- **Behavior**: Write `.h` ONLY. Do NOT write `.c`.

## MODE C: CODER (Implementation Engineer)
- **Goal**: Implement the logic within constraints.
- **Input**: **[Contract: xxx.h]** + Dependency Headers.
- **Output**: **[Implementation: xxx.c]** + **[Self-Check Report]**.
- **Behavior**:
  - Strict adherence to input `.h`.
  - **Self-Check Report** must verify: Stack safety, FPU avoidance (LUT usage), Error handling compliance.

# Performance KPIs
1. **Timing**: FOC Loop Target **1kHz**. Jitter < 100us.
2. **Compute**: FOC Task CPU load < 70%. Use Fixed-Point or LUT for math.
3. **Safety**: All motor movement code MUST have a path to call `BSP_PWM_StopAll()`.

# BSP Interface Contracts (Immutable API)
*Use these EXACTLY as defined. Do not hallucinate new BSP functions.*

## 1. I2C (Thread-Safe, DMA/Blocking)
- `void BSP_I2C_Init(void);`
- `HAL_StatusTypeDef BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);`
- `HAL_StatusTypeDef BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);`
- `HAL_StatusTypeDef BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr);`
- `HAL_StatusTypeDef BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len);` (Requires ThreadFlags wait)

## 2. UART (DMA RingBuffer)
- `void BSP_UART_Printf(const char *format, ...);` (L4+ Debug Only, Internal Locking)
- `HAL_StatusTypeDef BSP_UART_Send_DMA(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);`
- `uint16_t BSP_UART_Read(BspUart_Dev_t dev, uint8_t *p_data, uint16_t len);`
- `void BSP_UART_SetRxCpltCallback(BspUart_Dev_t dev, BspUart_Callback_t cb);`

## 3. PWM (Motor & Servo)
- `void BSP_PWM_Set3PhaseDuty(float dc_a, float dc_b, float dc_c);` (Range: 0.0f - 1.0f)
- `void BSP_PWM_SetServoPulse(BspServo_Dev_t servo, uint16_t us);`
- `void BSP_PWM_StopAll(void);` (Emergency Stop)
```

## Future Expand

采用 **Hybrid Workflow** 模式，Designer，APIer 仍旧使用 Chatbox 进行工作，引入 Cursor / Kiro / Antigravity 充当架构中的 "Coder Mode" 角色

1. Designer (Chatbox 1): 思考架构。
2. APIer (Chatbox 2): 锁定接口。
3. Coder (Cursor): 您将 APIer 生成的 .h 文件放入 Cursor，利用 Composer 快速生成实现代码。因为 C 代码量大且繁琐，Cursor 的自动补全和 Diff 查看功能是纯 Chatbox 无法比拟的。

Cursor 可以引入 `.cursorrules` , `.cursorignore` 等配置客制化 Coder 能力，同时 Cursor 具备自动补全和 Diff 查看功能 ，可以大幅提升 Coder 角色的代码生成和调试 的 accuracy 和 efficiency。