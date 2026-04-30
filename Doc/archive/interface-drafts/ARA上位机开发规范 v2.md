# ARA-Link 上位机系统设计规范 V2.0

## 1. 通信链路选型 (Hardware Layer)

### 推荐方案：有线 USB-TTL (CP2102/CH340)

* **理由**：
1. **稳定性第一**：FOC电机控制调试对实时性要求极高（1kHz采样）。蓝牙（HC-05）带宽低且存在丢包和延迟抖动，会导致波形断裂，误导PID参数整定。
2. **资源限制**：STM32F103需要使用 DMA 发送数据以释放CPU算力。有线连接能稳定支持 **921600 bps** 甚至 **2 Mbps** 的波形率，而HC-05通常稳定在115200 bps，不够用。
3. **供电**：USB可以直接为控制电路（MCU+编码器）供电，方便调试。



### 备选方案（仅在必须无线时）：ESP32-C3 透明传输

* 如果机械臂必须在移动中调试，使用一块 ESP32-C3 通过 UART 连接 STM32，运行 `ESP-LINK` 或简单的 `TCP-UART 透传固件`。电脑端通过 TCP Socket 连接。
* *警告*：不要使用 HC-05，带宽太窄，无法同时支撑波形和日志。

**结论**：开发阶段强制使用**有线 USB-TTL** 连接 **USART1 (PA9/PA10)**。

---

## 2. 混合数据协议定义 (Hybrid Protocol)

为了在一个串口同时传输“二进制波形”和“文本日志”，我们需要定义一种**包头识别机制**。

**基本帧结构**：
`[帧头 2Byte] [功能字 1Byte] [数据长度 1Byte] [载荷 N Bytes] [校验和 1Byte]`

* **帧头 (Header)**: `0xAA 0x55` (作为数据流同步标记)

### 2.1 协议功能字 (Function ID)

| ID (Hex) | 功能类型 | 数据方向 | 说明 |
| --- | --- | --- | --- |
| **0x01** | **CMD_PLOT** | MCU -> PC | 高速波形数据 (纯二进制 float) |
| **0x02** | **CMD_LOG** | MCU -> PC | 文本日志信息 (ASCII String) |
| **0x10** | **CMD_SET_PARAM** | PC -> MCU | 参数修改指令 |

### 2.2 详细帧格式

#### A. 高速波形帧 (CMD_PLOT) - 固定长度

用于绘制曲线。MCU端需以 50Hz~200Hz 频率发送。

| Byte | 内容 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0-1 | 0xAA 0x55 | `uint16` | 帧头 |
| 2 | **0x01** | `uint8` | **功能字：波形** |
| 3 | 0x10 | `uint8` | 长度：16字节 (4个float) |
| 4-7 | Target Angle | `float` | 目标角度 |
| 8-11 | Actual Angle | `float` | 实际角度 |
| 12-15 | Velocity | `float` | 当前速度 |
| 16-19 | Current/Torque | `float` | 电流或电压 |
| 20 | Checksum | `uint8` | Sum校验 |

#### B. 文本日志帧 (CMD_LOG) - 不定长

用于替代 `printf`。
*注意：为了不阻塞电机控制，MCU端日志单条长度限制在 64 或 128 字节以内。*

| Byte | 内容 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0-1 | 0xAA 0x55 | `uint16` | 帧头 |
| 2 | **0x02** | `uint8` | **功能字：日志** |
| 3 | Length (N) | `uint8` | 字符串的长度 |
| 4 ... 3+N | Text Data | `char[]` | 纯文本内容 (如 "Motor Init OK") |
| 4+N | Checksum | `uint8` | Sum校验 |

#### C. 参数设置帧 (CMD_SET_PARAM) - PC发送

| Byte | 内容 | 类型 | 说明 |
| --- | --- | --- | --- |
| 0-1 | 0xAA 0x55 | `uint16` | 帧头 |
| 2 | **0x10** | `uint8` | **功能字：设参** |
| 3 | Param ID | `uint8` | 0x01=P_Gain, 0x02=I_Gain ... |
| 4-7 | Value | `float` | 参数值 |
| 8 | Checksum | `uint8` |  |

---

## 3. 上位机软件界面需求 (GUI Requirements)

请指示学弟/学妹按此布局设计软件界面：

### 3.1 界面布局 (Layout)

建议采用 **“左图右控下终端”** 的布局。

* **Zone A: 实时波形区 (Upper Left)**
* 组件：PyQtGraph PlotWidget。
* 逻辑：解析收到 `0x01` 包，将4个float数据分别压入4条曲线的Buffer。


* **Zone B: 参数控制区 (Right)**
* 组件：PID参数输入框、滑动条、"写入参数"按钮。
* 逻辑：点击按钮组包发送 `0x10` 指令。


* **Zone C: 调试终端 (Bottom / Log Console)**
* **这是你新增的需求**。
* 组件：`QTextEdit` 或 `QPlainTextEdit` (设为 ReadOnly)。
* 功能：
* **解析显示**：收到 `0x02` 包时，提取 Text Data 追加到文本框末尾。
* **自动滚动**：新消息到达时自动滚到底部。
* **时间戳**：上位机软件在接收到数据时，自动在行首加上 `[HH:MM:SS.ms]`。
* **一键清屏**：提供 Clear 按钮。





### 3.2 软件架构逻辑 (Software Architecture)

为了防止界面卡死，**必须使用多线程**：

1. **Thread 1 (ComPort Worker)**:
* 负责 `serial.read()`。
* **状态机解析**：寻找 `0xAA 0x55` 头。
* 读取功能字。
* 如果是 `0x01` -> 读16字节 -> 发射 `Signal_Plot_Data`。
* 如果是 `0x02` -> 读N字节 -> 发射 `Signal_Log_Text`。


2. **Thread 2 (Main GUI)**:
* 连接 `Signal_Plot_Data` -> 更新曲线。
* 连接 `Signal_Log_Text` -> `textEdit.append(str)`.



---

## 4. 给开发者的特别提示 (Tips for Student)

1. **关于 printf 的重定向**：
* 告诉学弟，我（嵌入式端）会重写 STM32 的 `printf` 或自定义一个 `LOG()` 宏。
* 底层逻辑是：调用 `LOG("Error %d", code)` -> 格式化字符串 -> 加上 `0xAA 0x55 0x02 ...` 协议头 -> DMA发送。
* 上位机**不需要**做正则匹配，只需要按协议解包即可。


2. **粘包处理 (Packet Sticking)**：
* 由于UART是流设备，一次 Read 可能会读到半个包，或者 1.5 个包。
* **必须**在接收线程中维护一个 `Buffer`，循环查找 `0xAA 0x55`，切分出完整的一帧进行处理，处理完的从 Buffer 移除。**不能假设每次 read 都是刚好一帧**。