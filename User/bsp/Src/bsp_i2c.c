/**
 * @file bsp_i2c.c
 * @brief I2C Wrapper Implementation (Fixed Address Shift & Bus Recovery)
 * @author ARA Project Coder
 */

#include "bsp_i2c.h"
#include "stm32f1xx_hal.h"
#include "bsp_gpio.h" // 需要操作GPIO来复位总线

/* --- Hardware Resources --- */
extern I2C_HandleTypeDef hi2c1;

static I2C_HandleTypeDef* i2c_handles[BSP_I2C_NUM] = {
        [BSP_I2C_MOTION] = &hi2c1
};

static AraCallback_t i2c_rx_cplt_cbs[BSP_I2C_NUM] = { NULL };

/* --- Helper: Status Conversion --- */
static AraStatus_t HAL_To_ARA_Status(HAL_StatusTypeDef status) {
    switch (status) {
        case HAL_OK:      return ARA_OK;
        case HAL_BUSY:    return ARA_BUSY;
        case HAL_TIMEOUT: return ARA_TIMEOUT;
        default:          return ARA_ERR_IO;
    }
}

/* --- Helper: Bus Recovery for STM32F1 --- */
// 解决 F103 I2C 容易卡死在 BUSY 状态的问题
static void I2C_ClearBusyFlagErratum(I2C_HandleTypeDef *hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 强行关闭 I2C 外设
    __HAL_RCC_I2C1_FORCE_RESET();
    HAL_Delay(2);
    __HAL_RCC_I2C1_RELEASE_RESET();

    // 2. 将 SCL 和 SDA 重新初始化为 GPIO Output
    // 注意：这里假设是 PB6(SCL) 和 PB7(SDA)，请根据你的原理图确认！
    GPIO_InitStruct.Pin = GPIO_PIN_6 | GPIO_PIN_7;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD; // 开漏输出
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    // 3. 模拟 9 个时钟脉冲，解救被拉死的 SDA
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET); // SDA High
    for (int i = 0; i < 9; i++) {
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET); // SCL Low
        HAL_Delay(1); // bit delay
        HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);   // SCL High
        HAL_Delay(1);
    }

    // 4. 产生 STOP 信号
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_6, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

    // 5. 重新初始化 HAL I2C
    // 这通常由 main.c 中的 MX_I2C1_Init 完成，但这里我们需要确保它被重置
    // 由于我们没有引用 MX_I2C1_Init，这里依赖 HAL_I2C_Init
    // 实际上，只要 GPIO 释放了，再次调用 HAL_I2C_Init 就能恢复
    hi2c->Instance->CR1 |= I2C_CR1_SWRST; // 再次软复位确保干净
    hi2c->Instance->CR1 &= ~I2C_CR1_SWRST;
    HAL_I2C_Init(hi2c);
}

/* --- API Implementation --- */

void BSP_I2C_Init(void)
{
    // 在这里检查总线是否忙。如果是，执行解锁流程。
    // HAL_I2C_GetState 可能不准，直接看标志位
    if (__HAL_I2C_GET_FLAG(&hi2c1, I2C_FLAG_BUSY)) {
        I2C_ClearBusyFlagErratum(&hi2c1);
    }
}

AraStatus_t BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    // [Fix] 左移地址! HAL 库要求传入 8-bit 地址
    uint16_t target_addr = dev_addr << 1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(
            i2c_handles[dev],
            target_addr,       // 使用左移后的地址
            reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            p_data,
            len,
            10 // 10ms Timeout
    );

    return HAL_To_ARA_Status(status);
}

AraStatus_t BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    // [Fix] 左移地址!
    uint16_t target_addr = dev_addr << 1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(
            i2c_handles[dev],
            target_addr,
            reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            p_data,
            len,
            10
    );

    return HAL_To_ARA_Status(status);
}

AraStatus_t BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    // [Fix] 左移地址!
    uint16_t target_addr = dev_addr << 1;

    // 增加 Trial 次数到 3，Timeout 到 10ms，提高稳定性
    HAL_StatusTypeDef status = HAL_I2C_IsDeviceReady(i2c_handles[dev], target_addr, 3, 10);

    if (status == HAL_OK) {
        return ARA_OK;
    } else {
        return ARA_ERR_NACK;
    }
}

/* === DMA Support === */

void BSP_I2C_SetRxCpltCallback(BspI2c_Dev_t dev, AraCallback_t cb)
{
    if (dev < BSP_I2C_NUM) {
        i2c_rx_cplt_cbs[dev] = cb;
    }
}

AraStatus_t BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len)
{
    if (dev >= BSP_I2C_NUM) return ARA_ERR_PARAM;

    // [Fix] 左移地址!
    uint16_t target_addr = dev_addr << 1;

    HAL_StatusTypeDef status = HAL_I2C_Mem_Read_DMA(
            i2c_handles[dev],
            target_addr,
            reg_addr,
            I2C_MEMADD_SIZE_8BIT,
            p_data,
            len
    );

    if (status == HAL_ERROR) return ARA_ERR_DMA;
    return HAL_To_ARA_Status(status);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    for (int i = 0; i < BSP_I2C_NUM; i++) {
        if (i2c_handles[i] == hi2c) {
            if (i2c_rx_cplt_cbs[i] != NULL) {
                i2c_rx_cplt_cbs[i]();
            }
            return;
        }
    }
}