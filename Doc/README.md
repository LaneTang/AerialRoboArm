# FOC demo

本项目用于 FOC算法 学习

项目采用 

| 硬件类型 | 选型            |
|------|---------------|
| 主控芯片 | STM32F103C8T6 |
|      |               |


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