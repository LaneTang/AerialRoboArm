# ARA-VSP (ARA Vision Simplex Protocol，ARA 视觉单向透传协议)

### **[Design Brief] 单向视觉事件协议与机载决策架构 (Simplex-Decision)**

#### 1. 架构职责重定义 (Boundary Shift)
* **PC 端 (感知节点)**：只做一件事——以固定帧率（如 20Hz）将“目标在哪里”告诉 MCU。**PC 端不知道机械臂当前的状态，也不下发任何抓取动作指令。**
* **STM32F103 (决策与执行节点)**：接收持续涌入的视觉流数据。L4 `Task_ArmManipulator` 结合当前自身电机的实际位置，独立决定何时追踪、何时悬停、何时闭合夹爪。

#### 2. 单向视觉事件控制帧设计 (Rx Only Protocol)
既然全由 MCU 决策，PC 只需要下发目标的空间特征。协议极其精简（9 字节），纯粹的数据流透传。

| 字节偏移 | 字段名称 | 数据类型 | 物理意义与业务逻辑 |
| :--- | :--- | :--- | :--- |
| `[0]` | `Header1` | `uint8_t` | 固定 `0xAA` |
| `[1]` | `Header2` | `uint8_t` | 固定 `0x55` |
| `[2]` | `Seq_ID` | `uint8_t` | 帧序号 (0-255)。**防乱序与卡顿**。 |
| `[3]` | `Perception_Flag`| `uint8_t` | **感知状态 (Bitmask)**<br>Bit 0: `Is_Tracking` (视野中找到目标)<br>Bit 1: `Is_Grabbable_Zone` (目标进入深度可抓取阈值)<br>Bit 2: `Emergency_Stop` (地面人为强制急停) |
| `[4]-[5]`| `Target_Roll` | `int16_t` | 目标绝对 Roll 角度 (PC计算好的定点数，如 $30.5^\circ$ 发送为 3050)。 |
| `[6]-[7]`| `Target_Dist` | `uint16_t`| 目标距离相机的 Z 轴深度距离 (单位：毫米)。 |
| `[8]` | `Checksum` | `uint8_t` | 校验和 (B2 到 B7 累加低 8 位)。防线噪。 |

#### 3. MCU 端的自主决策逻辑 (L4 State Machine - Auto Mode)
因为没有 PC 帮 MCU 做决定，L4 `Task_ArmManipulator` 必须实现**“基于误差收敛的自主触发”**。

1.  **`STATE_SEEKING` (寻的)**：
    * 读取 DataHub 中的最新视觉帧。若 `Is_Tracking == 1`，将 `Target_Roll` 和 `Target_Dist` 映射为 L3 电机的目标值，开始跟随。
2.  **`STATE_CONVERGING` (误差收敛 - 决策核心)**：
    * MCU 不断对比“视觉期望位置”与“电机编码器实际位置”。
    * **触发抓取的 AND 条件**：
        1. 视觉标记就绪：`Is_Grabbable_Zone == 1`。
        2. 机械臂到位：`abs(Current_Roll - Target_Roll) < ROLL_DEADBAND` 且 `abs(Current_Ext - Target_Ext) < EXT_DEADBAND`。
        3. **时间防抖**：上述两个条件必须**持续满足 N 个系统 Tick**（例如 200ms）。
    * 一旦满足，状态机自主跃迁至 `STATE_GRAB`。
3.  **`STATE_GRAB` (执行抓取)**：
    * 驱动夹爪闭合。**在此期间，完全忽略/丢弃新到达的视觉串口数据**，防止抓取过程中的视觉突变导致电机抽搐。
4.  **`STATE_RETRACT` (收回)**：
    * 夹紧后，折叠臂缩回至原点。回到 `STATE_SEEKING` 或进入 `IDLE` 等待。

#### 4. 单向链路的极端安全性与冗余设计 (Simplex Safety)

砍掉上行链路后，最危险的情况是“瞎子骑瞎马”（网络断了，但 MCU 认为还在执行）。必须在 MCU 端建立两道绝对防线：

* **第一道防线：通信看门狗 (Rx Timeout)**
  L5 `CentralScheduler` 必须维护一个定时器。每次 L2 串口 DMA 接收到有效解析的一帧，重置定时器。如果超过 **500ms** 定时器溢出，说明 Wi-Fi 断联或 Linux 板死机。系统必须立即清除 DataHub 中的 `Is_Tracking` 标志，并触发机械臂归位。
* **第二道防线：空间低通滤波 (Spatial LPF)**
  由于深度相机的检测框可能会因为光线变化产生几帧的“瞬移”（例如距离瞬间从 200mm 跳到 800mm）。MCU 不能直接把 `Target_Dist` 喂给位置环 PID，必须在 L4 层做一个简单的**一阶低通滤波或变化率限幅（Slew Rate Limiter）**，防止关节电机瞬间索取大电流导致 STM32 掉电重启。

### **[Design Brief] ARA-VSP 协议端到端实现架构方案**

#### 1. PC 端实现方案 (The Perception Pipeline)

PC 端的唯一目标是：**稳定、匀速、低延迟地向网络链路泵入 ARA-VSP 数据帧**。PC 端不应关心底层电机如何运动，只负责“所见即所得”的空间映射。

* **1.1 核心管线流转 (Data Pipeline)**
    * **采集阶段 (Ingest)**：通过网络接收微型 Linux 板传来的 RGB-D 视频流。
    * **解算阶段 (Inference)**：运行目标检测算法（如 YOLO），提取目标 2D 像素坐标 (u, v)；结合深度图 (Depth Map) 提取目标中心点的 Z 轴真实物理距离（毫米）。
    * **映射阶段 (Mapping)**：将像素偏差映射为 Roll 轴的角度需求。*例如：目标偏左 50 像素，结合相机 FOV 换算为需要 Roll 轴偏转 -15.5度。*
    * **封包阶段 (Packing)**：将计算出的浮点数进行定点化（乘 100 取整），组装为 9 字节的 ARA-VSP 帧，并计算校验和。

* **1.2 关键架构约束：固定心跳节拍 (Fixed Tick Rate)**
    * **严禁突发发送 (No Bursting)**：即使 PC 端的显卡算力极高，能跑到 100fps，也**绝不能**以 100Hz 的频率向 Wi-Fi 狂发指令。这会导致微型 Linux 板的 TCP/UDP 缓冲区溢出。
    * **恒定帧率**：PC 端必须设立一个独立的发送线程（Timer Thread），以严格的 **20Hz (50ms/帧)** 或 **30Hz (33ms/帧)** 的频率下发数据。如果视觉算法在 50ms 内没算出新结果，就重发上一帧（更新 Seq_ID）；如果算出了多帧，只取最新的一帧发送。这让 ARA-VSP 帧在物理意义上成为了 MCU 的**绝对生命心跳**。

#### 2. MCU 端实现方案 (The Decision & Execution Core)

STM32F103 的资源极其有限，因此其核心架构必须是**“异步接收 + 同步状态机 + 硬件级解耦”**。

* **2.1 L2/L1 接收层：零 CPU 开销的 DMA 环形缓冲区**
    * **痛点**：如果使用传统的 UART 中断，每收到 1 个字节打断一次 CPU，系统极易在高速通信中崩溃。
    * **设计**：在 L1 BSP 层，开启 `UART_DMA_RX`，指向一个容量为 64 字节（约 7 个完整帧）的环形缓冲区 (RingBuffer)。
    * **机制**：L2 Driver 层注册 DMA 空闲中断 (IDLE Interrupt) 或半满/全满中断。当产生中断时，L2 层从 RingBuffer 中“滑动窗口”寻找 `0xAA 0x55` 帧头，一旦校验和 `Checksum` 匹配，立即将这 9 个字节提取出来。

* **2.2 L4 数据集散中心：DataHub 的无锁更新**
    * 由于解包在 L2/ISR 中进行，而业务在使用在 L4 Task 中，存在并发读写风险。
    * **设计**：在 DataHub 中定义 `struct ARA_VSP_Payload`。考虑到这是一个仅有几个字节的小结构体，在 FreeRTOS 中更新它时，无需使用沉重的 Mutex（互斥锁），直接使用 `taskENTER_CRITICAL()` 和 `taskEXIT_CRITICAL()` 进行极短时间的临界区保护拷贝即可。
    * **时间戳记录**：每次更新 DataHub 时，必须同步记录当前的系统滴答时钟 `Last_Vision_Tick = xTaskGetTickCount()`。

* **2.3 L4 核心业务层：频率墙与低通滤波 (The Frequency Wall & LPF)**
  这是最核心的工程难点：**PC 视觉帧率只有 20Hz (50ms)，而 L3 的 FOC 电机控制任务必须跑在 1000Hz (1ms)。**
    * 如果直接把 20Hz 的阶跃信号喂给电机，电机会表现出严重的“顿挫感”（每 50ms 抽搐一次）。
    * **滤波设计**：在 L4 `Task_ArmManipulator` 中，我们必须在“目标指令”进入“电机执行”之前，加一道**一阶低通滤波器 (First-Order Low-Pass Filter)** 或 **斜率限制器 (Slew Rate Limiter)**。
    * *逻辑表现*：MCU 不会立即跳到 PC 给定的角度，而是以平滑的曲线在 50ms 内逼近目标值。这不仅消除了顿挫，还能过滤掉 PC 端视觉算法由于光线变化产生的瞬间“毛刺（检测框跳动）”。

* **2.4 L5 系统调度层：绝对的安全底线**
    * `Task_CentralScheduler` 独立运行在一个高优先级 Task 中（例如 10Hz）。
    * 它唯一的任务就是盯着 DataHub 里的 `Last_Vision_Tick`。
    * **架构法则**：一旦发现 `xTaskGetTickCount() - Last_Vision_Tick > 500` (即 500ms 未收到有效 VSP 帧)，L5 立即无视 L4 的所有状态，强制调用 L3 的 `Actuator_EmergencyStop()`，并清空 DataHub 中的 `Is_Tracking` 标志。这就彻底防死了 Wi-Fi 突然断开导致无人机带臂乱飞的灾难。


