#ifndef __OLED_H
#define __OLED_H

// 包含HAL库和FreeRTOS所需头文件
#include "main.h"      // 包含HAL库和I2C句柄定义
#include "stdlib.h"
#include "FreeRTOS.h"  // FreeRTOS核心
#include "semphr.h"    // 信号量和互斥量

// 外部声明CubeMX生成的I2C句柄
extern I2C_HandleTypeDef hi2c1;


/***************OLED IIC通信定义****************/
// I2C设备的7位地址，这里使用8位写地址 0x78
#define OLED_ADDRESS 	(0x3C << 1)
/*********************END**********************/

#define OLED_CMD  0	// 命令
#define OLED_DATA 1	// 数据

// 初始化和HAL库回调函数
void OLED_Init(void);
//void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c); // 用于DMA完成后的通知

// 保持对外接口函数
void OLED_ColorTurn(uint8_t i);
void OLED_DisplayTurn(uint8_t i);
void OLED_WR_Byte(uint8_t dat,uint8_t mode);
void OLED_DisPlay_On(void);
void OLED_DisPlay_Off(void);
void OLED_Refresh(void);          // DMA模式刷新
void OLED_Clear(void);
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t);
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode);
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r);
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode);
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode);
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode);
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode);
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode);
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode);

#endif