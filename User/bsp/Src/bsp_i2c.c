/**
  ******************************************************************************
  * @file    bsp_i2c.c
  * @brief   BSP I2C 实现文件
  * @note    需要 CubeMX 配置 I2C 开启 DMA 和 中断，且中断优先级 <= FreeRTOS SysCall
  ******************************************************************************
  */

#include "bsp_i2c.h"
#include "i2c.h" // CubeMX 生成的 I2C 句柄声明 (hi2c1, hi2c2)

/* --- 内部配置 --- */
#define I2C_MUTEX_TIMEOUT_MS        10   // 获取总线锁的超时时间
#define I2C_BLOCKING_TIMEOUT_MS     10   // 阻塞模式下的硬件传输超时
#define I2C_DMA_TIMEOUT_MS          20   // DMA 模式下等待中断的超时时间

/* --- 任务通知标志位 (Thread Flags) --- */
#define I2C_FLAG_CMPLT              0x01 // 传输完成
#define I2C_FLAG_ERROR              0x02 // 传输错误

/* --- 管理结构体 --- */
typedef struct {
    I2C_HandleTypeDef *hi2c;        // HAL I2C 句柄
    osMutexId_t       mutex;        // 互斥锁 ID
    osThreadId_t      task_notify;  // 记录发起 DMA 的任务句柄，用于中断唤醒
} BspI2c_Handle_t;

/* --- 静态实例管理 --- */
static BspI2c_Handle_t bsp_i2c_devs[BSP_I2C_NUM] = {
        // [0] BSP_I2C_MOTION (I2C1)
        {
                .hi2c = &hi2c1,
                .mutex = NULL,
                .task_notify = NULL
        },
        // [1] BSP_I2C_FUNC (I2C2)
        {
                .hi2c = &hi2c2,
                .mutex = NULL,
                .task_notify = NULL
        }
};

/* --- 互斥锁属性 (优先级继承，防止优先级反转) --- */
static const osMutexAttr_t i2c_mutex_attr = {
        .name = "I2C_Mutex",
        .attr_bits = osMutexPrioInherit,
};

/* ============================================================ */
/*                          初始化函数                           */
/* ============================================================ */
void BSP_I2C_Init(void) {
    for (int i = 0; i < BSP_I2C_NUM; i++) {
        // 创建互斥锁
        bsp_i2c_devs[i].mutex = osMutexNew(&i2c_mutex_attr);

        if (bsp_i2c_devs[i].mutex == NULL) {
            // 错误处理: Heap 不足等
            // Error_Handler();
        }
    }
}

/* ============================================================ */
/*                  阻塞模式实现 (Blocking)                       */
/* ============================================================ */

HAL_StatusTypeDef BSP_I2C_ReadMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len) {
    HAL_StatusTypeDef status;
    BspI2c_Handle_t *h = &bsp_i2c_devs[dev];

    if (osMutexAcquire(h->mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        return HAL_BUSY; // 无法获取锁
    }

    // 执行阻塞读取 (假设寄存器地址为 8bit，这也是绝大多数传感器的标准)
    status = HAL_I2C_Mem_Read(h->hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, len, I2C_BLOCKING_TIMEOUT_MS);

    osMutexRelease(h->mutex);
    return status;
}

HAL_StatusTypeDef BSP_I2C_WriteMem(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len) {
    HAL_StatusTypeDef status;
    BspI2c_Handle_t *h = &bsp_i2c_devs[dev];

    if (osMutexAcquire(h->mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        return HAL_BUSY;
    }

    status = HAL_I2C_Mem_Write(h->hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, len, I2C_BLOCKING_TIMEOUT_MS);

    osMutexRelease(h->mutex);
    return status;
}

HAL_StatusTypeDef BSP_I2C_IsDeviceReady(BspI2c_Dev_t dev, uint16_t dev_addr) {
    HAL_StatusTypeDef status;
    BspI2c_Handle_t *h = &bsp_i2c_devs[dev];

    if (osMutexAcquire(h->mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) return HAL_BUSY;

    status = HAL_I2C_IsDeviceReady(h->hi2c, dev_addr, 2, 2); // 尝试2次，超时2ms

    osMutexRelease(h->mutex);
    return status;
}

/* ============================================================ */
/* DMA 模式实现                          */
/* ============================================================ */

// 内部通用 DMA 处理函数，减少代码重复
static HAL_StatusTypeDef BSP_I2C_DMA_Handler(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len, uint8_t is_read) {
    HAL_StatusTypeDef status;
    BspI2c_Handle_t *h = &bsp_i2c_devs[dev];
    uint32_t flags;

    // 1. 获取锁 (保护总线)
    if (osMutexAcquire(h->mutex, I2C_MUTEX_TIMEOUT_MS) != osOK) {
        return HAL_BUSY;
    }

    // 2. 注册当前任务，准备接收中断通知
    h->task_notify = osThreadGetId();
    osThreadFlagsClear(I2C_FLAG_CMPLT | I2C_FLAG_ERROR); // 清除旧标志

    // 3. 启动 DMA 传输
    if (is_read) {
        status = HAL_I2C_Mem_Read_DMA(h->hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, len);
    } else {
        status = HAL_I2C_Mem_Write_DMA(h->hi2c, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, p_data, len);
    }

    // 启动失败 (例如 DMA 忙)
    if (status != HAL_OK) {
        h->task_notify = NULL;
        osMutexRelease(h->mutex);
        return status;
    }

    // 4. 挂起任务，等待 ISR 唤醒 (Zero-Wait 核心)
    flags = osThreadFlagsWait(I2C_FLAG_CMPLT | I2C_FLAG_ERROR, osFlagsWaitAny, I2C_DMA_TIMEOUT_MS);

    // 5. 唤醒后处理
    h->task_notify = NULL; // 擦除记录

    if (flags & osFlagsError) {
        // 超时 (I2C 硬件无响应)
        HAL_DMA_Abort(&hdma_i2c1_tx); // 终止硬件操作
        status = HAL_TIMEOUT;
    } else if (flags & I2C_FLAG_ERROR) {
        // 硬件报错 (NACK 等)
        status = HAL_ERROR;
    } else {
        // 传输成功
        status = HAL_OK;
    }

    // 6. 释放锁
    osMutexRelease(h->mutex);

    return status;
}

HAL_StatusTypeDef BSP_I2C_ReadMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len) {
    return BSP_I2C_DMA_Handler(dev, dev_addr, reg_addr, p_data, len, 1); // 1 = Read
}

HAL_StatusTypeDef BSP_I2C_WriteMem_DMA(BspI2c_Dev_t dev, uint16_t dev_addr, uint16_t reg_addr, uint8_t *p_data, uint16_t len) {
    return BSP_I2C_DMA_Handler(dev, dev_addr, reg_addr, p_data, len, 0); // 0 = Write
}


/* ============================================================ */
/* HAL 中断回调重写 (Weak Override)          */
/* ============================================================ */

// 辅助: 通过 hi2c 句柄找到对应的 BSP 结构体
static BspI2c_Handle_t* Get_BSP_Ctx(I2C_HandleTypeDef *hi2c) {
    if (hi2c->Instance == I2C1) return &bsp_i2c_devs[BSP_I2C_MOTION];
    if (hi2c->Instance == I2C2) return &bsp_i2c_devs[BSP_I2C_FUNC];
    return NULL;
}

// 接收完成回调 (Read DMA Complete)
void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BspI2c_Handle_t *h = Get_BSP_Ctx(hi2c);
    if (h && h->task_notify) {
        osThreadFlagsSet(h->task_notify, I2C_FLAG_CMPLT); // 唤醒任务: 成功
    }
}

// 发送完成回调 (Write DMA Complete)
void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    BspI2c_Handle_t *h = Get_BSP_Ctx(hi2c);
    if (h && h->task_notify) {
        osThreadFlagsSet(h->task_notify, I2C_FLAG_CMPLT); // 唤醒任务: 成功
    }
}

// 错误回调 (NACK, Bus Error)
void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    BspI2c_Handle_t *h = Get_BSP_Ctx(hi2c);
    if (h && h->task_notify) {
        osThreadFlagsSet(h->task_notify, I2C_FLAG_ERROR); // 唤醒任务: 失败
    }
}