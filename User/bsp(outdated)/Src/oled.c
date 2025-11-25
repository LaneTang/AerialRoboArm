#include "oled.h"
#include "stdlib.h"
#include "oledfont.h" // 字体文件引用（用户需确保存在）

// 图像缓存区：128列 * 8页 = 1024字节
uint8_t OLED_GRAM[128][8];

// FreeRTOS同步对象定义
//SemaphoreHandle_t OLED_I2C_Mutex = NULL;     // I2C总线互斥量
static TaskHandle_t xOledTaskHandle = NULL;     // 全局变量：记录正在等待 OLED DMA 完成的任务句柄
// DMA信号量已移除

/**
 * @brief  [底层核心] I2C DMA 传输并同步等待 (Yield)
 * @note   此函数会挂起当前任务，直到 DMA 传输完成。
 * 请确保在调用前已初始化 FreeRTOS 调度器。
 */
static void I2C_DMA_Transmit_Sync(uint16_t DevAddress, uint8_t *pData, uint16_t Size)
{
    // 1. 记录当前任务句柄
    xOledTaskHandle = xTaskGetCurrentTaskHandle();

    // 2. 启动 DMA 传输
    if (HAL_I2C_Master_Transmit_DMA(&hi2c1, DevAddress, pData, Size) != HAL_OK)
    {
        // 如果启动失败（如总线忙），根据需求进行错误处理

        // 这里简单处理：如果失败，直接返回（或者可以选择自旋重试）
        return;
    }

    // 3. 进入阻塞态，等待DMA完成中断ISR发送TaskNotification唤醒任务继续执行
    // pdTRUE: 退出时清零通知值 (消耗掉这次通知)
    // portMAX_DELAY: 死等，直到完成。如果为了防止硬件死锁，可设置超时时间如 pdMS_TO_TICKS(100)
    uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(100));
}

// --- HAL 库中断回调重写 ---
// 放在 stm32f1xx_it.c 或本文件中均可，确保不要重复定义
void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c)
{
    // 检查是否是 OLED 使用的 I2C1
    if (hi2c->Instance == I2C1)
    {
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;

        // 1. 确认有任务在等待 (防止空指针)
        if (xOledTaskHandle != NULL)
        {
            // 2. 发送通知唤醒任务
            vTaskNotifyGiveFromISR(xOledTaskHandle, &xHigherPriorityTaskWoken);

            // 3. 清除句柄 (可选，视逻辑严谨性而定)
            xOledTaskHandle = NULL;
        }

        // 3. 如果唤醒的任务优先级更高，请求上下文切换
        portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
    }
}

// --- HAL I2C 封装函数 ---
/**
 * @brief 发送一个字节到OLED (命令或短数据)
 * @param dat 要发送的数据或命令
 * @param mode 0: 命令 (OLED_CMD), 1: 数据 (OLED_DATA)
 * 注意：此函数用于发送单个命令/数据，刷新函数 OLED_Refresh 不调用此函数。
 */
// 初始化阶段必须极其稳定，不要使用 DMA
void OLED_WR_Byte(uint8_t dat, uint8_t mode)
{
    uint8_t i2c_buf[2]; // 阻塞函数可以使用局部变量

    if(mode == OLED_CMD) i2c_buf[0] = 0x00;
    else i2c_buf[0] = 0x40;

    i2c_buf[1] = dat;

    // 使用阻塞式发送，超时 10ms，确保命令必达
    HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, i2c_buf, 2, 10);
}

/**
 * @brief OLED 全屏刷新 (重构核心)
 * @note  命令部分保持阻塞(因数据量小)，数据部分使用 DMA+Yield
 */
void OLED_Refresh(void)
{
    uint8_t i, n;

    // DMA 数据缓冲区 (必须 static)
    static uint8_t data_buf[129];
    // 命令缓冲区
    uint8_t cmd_buf[2];

    for(i = 0; i < 8; i++)
    {
        // 1. 发送命令 (阻塞式)
        cmd_buf[0] = 0x00;

        cmd_buf[1] = 0xb0 + i; // Set Page Start Address
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 10);

        cmd_buf[1] = 0x02;     // Set Lower Column
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 10);

        cmd_buf[1] = 0x10;     // Set Higher Column
        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 10);

        // 2. 准备数据
        data_buf[0] = 0x40;
        for(n = 0; n < 128; n++)
        {
            data_buf[n+1] = OLED_GRAM[n][i];
        }

        // 3. 发送数据 (DMA 异步 + 任务通知阻塞)
        // 这里才是真正需要节省 CPU 时间的地方
        I2C_DMA_Transmit_Sync(OLED_ADDRESS, data_buf, 129);
    }
}

//// 写入GRAM到OLED (使用阻塞式 I2C)
//void OLED_Refresh(void)
//{
//    uint8_t i, n;
//    HAL_StatusTypeDef status;
//
//    // 缓冲区必须足够大，包含控制字节(0x40)和128字节数据
//    // 使用 static 避免栈溢出
//    static uint8_t data_buf[129];
//
//    // 用于发送命令的缓冲区
//    uint8_t cmd_buf[2] = {0x00, 0x00}; // [0x00, Command]
//
//    // --- 在 Mutex 保护下进行 I2C 阻塞传输 ---
//    for(i=0;i<8;i++) // 8页
//    {
//        // 1.1 发送命令：设置页地址 (0xb0 + i)
//        cmd_buf[1] = 0xb0 + i;
//        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 5);
//
//        // 1.2 发送命令：设置低列地址 (0x02)
//        cmd_buf[1] = 0x02;
//        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 5);
//
//        // 1.3 发送命令：设置高列地址 (0x10)
//        cmd_buf[1] = 0x10;
//        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, cmd_buf, 2, 5);
//
//
//        // 1.4 准备数据缓冲区
//        data_buf[0] = 0x40; // 数据模式控制字节
//        for(n=0;n<128;n++)
//        {
//            data_buf[n+1] = OLED_GRAM[n][i];
//        }
//
//        // 1.5 使用阻塞式 HAL 函数发送 129 字节数据 (超时 50ms)
//        HAL_I2C_Master_Transmit(&hi2c1, OLED_ADDRESS, data_buf, 129, 50);
//
//        // 注意：这里没有错误处理，如果传输失败，OLED 可能显示异常。
//    }
//
//}

//OLED初始化
void OLED_Init(void)
{
//    // 1. 创建 FreeRTOS 同步对象
//    if(OLED_I2C_Mutex == NULL)
//    {
//        OLED_I2C_Mutex = xSemaphoreCreateMutex();
//    }
//
//    // 移除 DMA 信号量创建的代码
//
//    // 确保 Mutex 创建成功
//    if (OLED_I2C_Mutex == NULL)
//    {
//        // Mutex 创建失败，无法保护 I2C 总线，直接返回
//        return;
//    }

    // 等待I2C硬件初始化完成
    HAL_Delay(200);

    // 以下初始化命令通过 OLED_WR_Byte (Mutex Guarded) 发送
    OLED_WR_Byte(0xAE,OLED_CMD);//--turn off oled panel
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0x10,OLED_CMD);
    OLED_WR_Byte(0x40,OLED_CMD);
    OLED_WR_Byte(0x81,OLED_CMD);
    OLED_WR_Byte(0xCF,OLED_CMD);
    OLED_WR_Byte(0xA1,OLED_CMD);
    OLED_WR_Byte(0xC8,OLED_CMD);
    OLED_WR_Byte(0xA6,OLED_CMD);
    OLED_WR_Byte(0xA8,OLED_CMD);
    OLED_WR_Byte(0x3f,OLED_CMD);
    OLED_WR_Byte(0xD3,OLED_CMD);
    OLED_WR_Byte(0x00,OLED_CMD);
    OLED_WR_Byte(0xd5,OLED_CMD);
    OLED_WR_Byte(0x80,OLED_CMD);
    OLED_WR_Byte(0xD9,OLED_CMD);
    OLED_WR_Byte(0xF1,OLED_CMD);
    OLED_WR_Byte(0xDA,OLED_CMD);
    OLED_WR_Byte(0x12,OLED_CMD);
    OLED_WR_Byte(0xDB,OLED_CMD);
    OLED_WR_Byte(0x30,OLED_CMD);
    OLED_WR_Byte(0x20,OLED_CMD);
    OLED_WR_Byte(0x02,OLED_CMD);
    OLED_WR_Byte(0x8D,OLED_CMD);
    OLED_WR_Byte(0x14,OLED_CMD);
    OLED_Clear(); // 清屏 (内部调用 OLED_Refresh)
    OLED_WR_Byte(0xAF,OLED_CMD); // Display On
}

// -----------------------------------------------------------
// 以下绘图/显示函数保持与原代码逻辑一致，操作 GRAM 数组
// -----------------------------------------------------------

// 清屏
void OLED_Clear(void)
{
    uint8_t i,n;
    for(i=0;i<8;i++)
    {
        for(n=0;n<128;n++)
        {
            OLED_GRAM[n][i]=0;
        }
    }
    OLED_Refresh();//使用 阻塞式 I2C 模式刷新显示
}

// 描点
void OLED_DrawPoint(uint8_t x,uint8_t y,uint8_t t)
{
    uint8_t i,m,n;
    i=y/8;
    m=y%8;
    n=1<<m;
    if(t){OLED_GRAM[x][i]|=n;}
    else
    {
        OLED_GRAM[x][i]=~OLED_GRAM[x][i];
        OLED_GRAM[x][i]|=n;
        OLED_GRAM[x][i]=~OLED_GRAM[x][i];
    }
//	OLED_Refresh(); // 仅操作GRAM，不立即刷新
}

// 绘制直线
void OLED_DrawLine(uint8_t x1,uint8_t y1,uint8_t x2,uint8_t y2,uint8_t mode)
{
    uint16_t t;
    int xerr=0,yerr=0,delta_x,delta_y,distance;
    int incx,incy,uRow,uCol;
    delta_x=x2-x1; // 计算坐标差值
    delta_y=y2-y1;
    uRow=x1;// 起点坐标
    uCol=y1;
    if(delta_x>0)incx=1; // 设置步进方向
    else if (delta_x==0)incx=0;// 垂直线
    else {incx=-1;delta_x=-delta_x;}
    if(delta_y>0)incy=1;
    else if (delta_y==0)incy=0;// 水平线
    else {incy=-1;delta_y=-delta_x;}
    if(delta_x>delta_y)distance=delta_x; // 选取差值较大的作为距离
    else distance=delta_y;
    for(t=0;t<distance+1;t++)
    {
        OLED_DrawPoint(uRow,uCol,mode);// 画点
        xerr+=delta_x;
        yerr+=delta_y;
        if(xerr>distance)
        {
            xerr-=distance;
            uRow+=incx;
        }
        if(yerr>distance)
        {
            yerr-=distance;
            uCol+=incy;
        }
    }
//	OLED_Refresh(); // 仅操作GRAM，不立即刷新
}

// 绘制圆
void OLED_DrawCircle(uint8_t x,uint8_t y,uint8_t r)
{
    int a, b,num;
    a = 0;
    b = r;
    while(2 * b * b >= r * r)
    {
        OLED_DrawPoint(x + a, y - b,1);
        OLED_DrawPoint(x - a, y - b,1);
        OLED_DrawPoint(x - a, y + b,1);
        OLED_DrawPoint(x + a, y + b,1);

        OLED_DrawPoint(x + b, y + a,1);
        OLED_DrawPoint(x + b, y - a,1);
        OLED_DrawPoint(x - b, y - a,1);
        OLED_DrawPoint(x - b, y + a,1);

        a++;
        num = (a * a + b * b) - r*r;// 计算点到圆心的距离
        if(num > 0)
        {
            b--;
            a--;
        }
    }
//		OLED_Refresh(); // 仅操作GRAM，不立即刷新
}


// -----------------------------------------------------------
// 字符显示函数 (操作 GRAM 数组，依赖字体文件 oledfont.h)
// -----------------------------------------------------------

// 在指定位置显示一个字符, 包括部分刷新逻辑
void OLED_ShowChar(uint8_t x,uint8_t y,uint8_t chr,uint8_t size1,uint8_t mode)
{
    uint8_t i,m,temp,size2,chr1;
    uint8_t x0=x,y0=y;
    if(size1==8)size2=6;
    else size2=(size1/8+((size1%8)?1:0))*(size1/2);  // 得到一个字符对应占用的字节数
    chr1=chr-' ';  // 得到偏移后的数值
    for(i=0;i<size2;i++)
    {
        if(size1==8)
        {temp=asc2_0806[chr1][i];} // 调用0806字体
        else if(size1==12)
        {temp=asc2_1206[chr1][i];} // 调用1206字体
        else if(size1==16)
        {temp=asc2_1608[chr1][i];} // 调用1608字体
        else if(size1==24)
        {temp=asc2_2412[chr1][i];} // 调用2412字体
        else return;
        for(m=0;m<8;m++)
        {
            if(temp&0x01)OLED_DrawPoint(x,y,mode);
            else OLED_DrawPoint(x,y,!mode);
            temp>>=1;
            y++;
        }
        x++;
        if((size1!=8)&&((x-x0)==size1/2))
        {x=x0;y0=y0+8;}
        y=y0;
    }
}


// 显示字符串
void OLED_ShowString(uint8_t x,uint8_t y,uint8_t *chr,uint8_t size1,uint8_t mode)
{
    while((*chr>=' ')&&(*chr<='~'))// 判断是否是非法字符!
    {
        // OLED_ShowChar 内部不再调用 OLED_Refresh()
        OLED_ShowChar(x,y,*chr,size1,mode);
        if(size1==8)x+=6;
        else x+=size1/2;
        chr++;
    }
    OLED_Refresh(); // 字符串显示完毕后，统一刷新一次
}

// 计算 m 的 n 次方
uint32_t OLED_Pow(uint8_t m,uint8_t n)
{
    uint32_t result=1;
    while(n--)
    {
        result*=m;
    }
    return result;
}

// 显示数字
void OLED_ShowNum(uint8_t x,uint8_t y,uint32_t num,uint8_t len,uint8_t size1,uint8_t mode)
{
    uint8_t t,temp,m=0;
    if(size1==8)m=2;
    for(t=0;t<len;t++)
    {
        temp=(num/OLED_Pow(10,len-t-1))%10;
        if(temp==0)
        {
            OLED_ShowChar(x+(size1/2+m)*t,y,'0',size1,mode);
        }
        else
        {
            OLED_ShowChar(x+(size1/2+m)*t,y,temp+'0',size1,mode);
        }
    }
    OLED_Refresh(); // 数字显示完毕后，统一刷新一次
}

// 显示中文
void OLED_ShowChinese(uint8_t x,uint8_t y,uint8_t num,uint8_t size1,uint8_t mode)
{
    uint8_t m,temp;
    uint8_t x0=x,y0=y;
    uint16_t i,size3=(size1/8+((size1%8)?1:0))*size1;  // 得到一个字符对应占用的字节数
    for(i=0;i<size3;i++)
    {
        if(size1==16)
        {temp=Hzk1[num][i];}// 调用16*16字库
        else if(size1==24)
        {temp=Hzk2[num][i];}// 调用24*24字库
        else if(size1==32)
        {temp=Hzk3[num][i];}// 调用32*32字库
        else if(size1==64)
        {temp=Hzk4[num][i];}// 调用64*64字库
        else return;
        for(m=0;m<8;m++)
        {
            if(temp&0x01)OLED_DrawPoint(x,y,mode);
            else OLED_DrawPoint(x,y,!mode);
            temp>>=1;
            y++;
        }
        x++;
        if((x-x0)==size1)
        {x=x0;y0=y0+8;}
        y=y0;
    }
    OLED_Refresh(); // 中文显示完毕后，统一刷新一次
}

// 滚动显示 (注意：原代码是无限循环，在FreeRTOS中需谨慎使用或修改)
void OLED_ScrollDisplay(uint8_t num,uint8_t space,uint8_t mode)
{
    // 警告：原代码中包含一个无限循环 while(1)，在 FreeRTOS 任务中调用会导致任务永远不让出 CPU。
    // 如果您需要滚动效果，请将此逻辑放到一个独立的、低优先级的任务中，并使用 vTaskDelay 代替紧密循环。

    uint8_t i,n,t=0,m=0,r;
    while(1) // 应该修改为循环次数或在外部控制退出
    {
        if(m==0)
        {
            OLED_ShowChinese(128,24,t,16,mode); // 写入一个汉字直到超出OLED_GRAM[][]的范围
            t++;
        }
        if(t==num)
        {
            for(r=0;r<16*space;r++)      // 显示空格
            {
                for(i=1;i<128;i++) // 注意：GRAM 数组大小为 [128][8]，这里可能需要修正边界
                {
                    for(n=0;n<8;n++)
                    {
                        OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
                    }
                }
                OLED_Refresh();
            }
            t=0;
        }
        m++;
        if(m==16){m=0;}
        for(i=1;i<128;i++)   // 实际滚动
        {
            for(n=0;n<8;n++)
            {
                OLED_GRAM[i-1][n]=OLED_GRAM[i][n];
            }
        }
        OLED_Refresh();
        // 在 FreeRTOS 中，需要添加延时，例如 vTaskDelay(pdMS_TO_TICKS(10));
        break; // 临时退出，防止死循环
    }
}

// 显示图片
void OLED_ShowPicture(uint8_t x,uint8_t y,uint8_t sizex,uint8_t sizey,uint8_t BMP[],uint8_t mode)
{
    uint16_t j=0;
    uint8_t i,n,temp,m;
    uint8_t x0=x,y0=y;
    sizey=sizey/8+((sizey%8)?1:0);
    for(n=0;n<sizey;n++)
    {
        for(i=0;i<sizex;i++)
        {
            temp=BMP[j];
            j++;
            for(m=0;m<8;m++)
            {
                if(temp&0x01)OLED_DrawPoint(x,y,mode);
                else OLED_DrawPoint(x,y,!mode);
                temp>>=1;
                y++;
            }
            x++;
            if((x-x0)==sizex)
            {
                x=x0;
                y0=y0+8;
            }
            y=y0;
        }
    }
    OLED_Refresh(); // 图片显示完毕后，统一刷新一次
}