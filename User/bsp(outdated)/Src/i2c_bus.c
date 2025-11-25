//
// Created by User on 2025/11/20.
//

#include "i2c_bus.h"


SemaphoreHandle_t xI2CMutex;

void I2C_Bus_Init(void)
{
    xI2CMutex = xSemaphoreCreateMutex();
}