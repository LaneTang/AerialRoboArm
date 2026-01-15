# FOC demo

本项目用于 FOC算法 学习

项目采用 

| 硬件类型 | 选型            |
|------|---------------|
| 主控芯片 | STM32F103C8T6 |
|      |               |

## 项目接线

根据你提供的 STM32F103C8Tx 引脚配置图（STM32CubeMX 截图），并结合我们之前开发的 ARA FOC 项目代码，这里是完整的**系统全局接线表**。

请务必按照此表检查你的硬件连接，特别是 **Enable 引脚** 和 **串口引脚**，因为它们与默认配置可能不同。

---

### 1. 电机驱动模块 (SimpleFOC Mini / DRV8313)

*注意：PWM 引脚对应 TIM1 的三个通道。*

| STM32 引脚 | 功能定义 | 连接到驱动板 (SimpleFOC Mini) | 备注 |
| --- | --- | --- | --- |
| **PA8** | TIM1_CH1 | **IN1** | U 相 PWM |
| **PA9** | TIM1_CH2 | **IN2** | V 相 PWM |
| **PA10** | TIM1_CH3 | **IN3** | W 相 PWM |
| **PA11** | GPIO_Output | **EN** (Enable) | **使能引脚** (注意修改代码定义) |
| **GND** | 地 | **GND** | **必须共地！** |
| - | - | **VM / VCC** | 接 12V 电源 (不要接 STM32!) |

> **⚠️ 代码修改提醒**：
> 你的图片中将 **PA11** 配置为了 GPIO 输出（即使能）。请检查你的 `main.h` 或 `drv_bldc_power.c`，确保宏定义匹配：
> `#define BSP_GPIO_MOTOR_EN  GPIO_PIN_11`
> `#define BSP_PORT_MOTOR_EN  GPIOA`

---

### 2. 磁编码器模块 (AS5600)

*注意：使用 I2C1 接口。*

| STM32 引脚 | 功能定义 | 连接到 AS5600 | 备注 |
| --- | --- | --- | --- |
| **PB8** | I2C1_SCL | **SCL** | 时钟线 (建议接上拉电阻) |
| **PB9** | I2C1_SDA | **SDA** | 数据线 (建议接上拉电阻) |
| **3.3V** | 电源 | **VCC / 3.3V** | AS5600 支持 3.3V 或 5V |
| **GND** | 地 | **GND** | **必须共地** |
| - | - | **DIR** | 悬空或接地 (决定方向) |
| - | - | **GPO** | 悬空 (不使用) |

---

### 3. 串口调试 (USB-TTL / 串口助手)

*注意：图片中使用的是 **USART3** (PB10/PB11)，而不是常见的 USART1 (PA9/PA10 被 PWM 占用了)。*

| STM32 引脚 | 功能定义 | 连接到 USB-TTL 模块 | 备注 |
| --- | --- | --- | --- |
| **PB10** | USART3_TX | **RXD** | STM32 发 -> 电脑收 |
| **PB11** | USART3_RX | **TXD** | STM32 收 <- 电脑发 |
| **GND** | 地 | **GND** | **必须共地** |
| **5V** | 供电 (可选) | **5V** | 如果 STM32 需要 USB 供电 |

---

### 4. 系统供电与调试 (SWD)

| STM32 引脚 | 功能 | 连接对象 | 备注 |
| --- | --- | --- | --- |
| **PA13** | SWDIO | ST-Link / DAPLink | 仿真器数据线 |
| **PA14** | SWCLK | ST-Link / DAPLink | 仿真器时钟线 |
| **PC13** | GPIO_Output | 板载 LED | 通常低电平点亮 (用于状态指示) |
| **NRST** | Reset | 复位按键 | 外部复位 |
| **PD0/PD1** | OSC_IN/OUT | 外部晶振 | 8MHz 晶振 (HSE) |

---

### ✅ 接线检查清单 (Checklist)

1. **共地 (Common Ground)**：这是最重要的一点。STM32 的 GND、驱动板的 GND、AS5600 的 GND、USB转串口的 GND，这四者必须最终连在一起。
2. **Enable 引脚**：图片显示是 **PA11**。请务必确认你的杜邦线是从 PA11 接到了驱动板的 EN 脚。
3. **电源电压**：
* 驱动板 VM 接 12V。
* STM32 和 AS5600 接 3.3V (或通过 USB 5V 经 LDO 降压)。
* **千万不要把 12V 接到 STM32 的任何引脚上，否则瞬间烧毁。**


4. **I2C 上拉**：如果 AS5600 读取不稳定，检查 PB8/PB9 是否在模块上有上拉电阻（通常模块自带，如果是裸芯片则需要加 4.7k 电阻到 3.3V）。

这张表是基于你提供的图片生成的，按照这个接线，配合我们刚才验证通过的 L3 算法代码，新板子一到即可直接工作！
## LOG 

### 25/11/17 晚

写个规划

初步完成FOC算法的理论知识，开始尝试**搭建测试项目**进行一次开发验证

1. OLED 屏幕驱动实现
2. （或）屏幕显示LOG任务实现
3. （或）串口收发LOG任务实现

完成上述模块之后，开始实现 FOC算法

特别鸣谢：https://github.com/Bjersgen/Simplefoc

提到的关于STM32与esp32/arduino的硬件差异在simpleFOC部署上导致的性能区别启发了我

### 25.11.28 night

在FOC_DEMO2中搭建了测试平台，测试freeRTOS框架下的I2C/UART bsp代码实现和drv_as5600/drv_bldc代码实现

done：
1. I2C1 DMA tx/rx 代码实现
2. UART3 DMA tx + DMA rx RingBuffer bsp代码实现和功能测试
3. AS5600 drv 代码实现且成功读取到reg数据
   1. I2C1 fast mode 下20cm杜邦线情况会导致直接无法读取reg数据，返回000
   2. 换了更短的杜邦线发现问题解决

undone:
1. BLDC drv 实现与测试
2. FOC mod 实现与测试