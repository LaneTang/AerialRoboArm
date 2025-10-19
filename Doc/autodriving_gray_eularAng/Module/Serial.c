
#include "ti_msp_dl_config.h"
#include "Serial.h"
#include <math.h>
#include <stdio.h>
#include "string.h"

char                RxDataBuf[128];
uint8_t             gRxAngDataBuf[128];
uint8_t             RxFlag_DEBUG;
uint8_t             RxFlag_JY60;

/*Transmitting*/
void UART_transmitByte(UART_Regs *uart, uint8_t byte) {
    DL_UART_Main_transmitDataBlocking(uart, byte);
}

void UART_transmitArray (UART_Regs *uart, uint8_t* Array, uint8_t length) {
    for(uint8_t i = 0; i < length; i++)
    {
        UART_transmitByte(uart, Array[i]);
    }
}

void UART_transmitString(UART_Regs *uart, char* String)
{
    uint8_t i;
    for(i = 0; String[i] != '\0'; i++)
    {
        UART_transmitByte(uart, String[i]);
    }
}

void UART_transmitNum(UART_Regs *uart, uint32_t Number, uint8_t length)
{
    uint8_t bits[length];
    // from LSB to MSB
    for(uint8_t i = 0; i < length; i++)
    {
        bits[length - i - 1] = Number % 10 + '0';   // ASCII + 0x30
        Number /= 10; 
    }
    UART_transmitArray(uart, bits, length);
}

void UART_transmitFloat(UART_Regs *uart, float Number, uint8_t intLength, uint8_t fLength) 
{
    // 发送整数部分
    UART_transmitNum(uart, Number, intLength);
    
    // 发送小数点
    UART_transmitArray(uart, (uint8_t[]){'.'}, 1);
    
    // 发送小数部分
    uint32_t decimalPart = Number * pow(10, fLength);   // 获取小数部分
    if (fLength > 0) {
        UART_transmitNum(uart, decimalPart, fLength);
    }
}

/* printf 重定向*/
int fputc(int c, FILE* stream)
{
	DL_UART_Main_transmitDataBlocking(UART_0_INST, c);
    return c;
}

int fputs(const char* restrict s, FILE* restrict stream)
{
    uint16_t i, len;
    len = strlen(s);
    for(i=0; i<len; i++)
    {
        DL_UART_Main_transmitDataBlocking(UART_0_INST, s[i]);
    }
    return len;
}

int puts(const char *_ptr)
{
    int count = fputs(_ptr, stdout);
    count += fputs("\n", stdout);
    return count;
}


/* ---Receiving--- */
Protocol type = Custom_Text;

/*Rx Processing*/
void RxFunc(void) {

    switch (type) {
        case Custom_Hex:
        {
            // OLED_ShowString(1, 1,  (uint8_t*)"KFC here!", 16);
            UART_transmitArray(UART_0_INST, (uint8_t* )RxDataBuf, RxLength);

            break;
        }
        case Custom_Text:
        {
            UART_transmitString(UART_0_INST, RxDataBuf);     // echo

            break;
        }
        case WT61:
        {
            break;
        }
        case JY60:
        {
            break;
        }
        default:
            break;
    }
    
    RxFlag_DEBUG = 0;
}

void UART_JY60_INST_IRQHandler(void) {
    static uint8_t RxState = 0;
    static uint8_t pRxData = 0;
    uint8_t RxData;

    switch (DL_UART_Main_getPendingInterrupt(UART_JY60_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            RxData = DL_UART_Main_receiveDataBlocking(UART_JY60_INST);

            // FSM-receive data
            if (RxState == 0)
            {
                if (RxData == 0x53)
                {
                    RxState = 1;
                    pRxData = 0;
                }
            }
            else if (RxState == 1)
            {
                gRxAngDataBuf[pRxData++] = RxData;
                if (pRxData >= 6)
                {
                    RxState = 2;
                }
            }
            else if (RxState == 2)
            {
                if (RxData == 0x55)
                {
                    RxState = 0;
                    RxFlag_JY60 = 1;
                }
            }
            
            break;

        default:
            break;
    }
}

/*Rx debug Callback*/
void UART_0_INST_IRQHandler(void) {
    static uint8_t RxState = 0;
    static uint8_t pRxData = 0;
    uint8_t RxData;

    switch (DL_UART_Main_getPendingInterrupt(UART_0_INST)) {
        case DL_UART_MAIN_IIDX_RX:
            RxData = DL_UART_Main_receiveDataBlocking(UART_0_INST);

            // FSM-receive data
            // 根据选择的协议参数对应不同的解包逻辑
            switch (type) {
                case Custom_Hex: {// 自己的hex包接收逻辑
                    if (RxState == 0) {         // wait for 1-st header
                        if (RxData == 0x2c) 
                            RxState = 1;
                    }
                    else if (RxState == 1) {    // wait for 2-nd header
                        if (RxData == 0x12) 
                            RxState = 2;
                    }
                    else if (RxState == 2) {    // Start accept Data
                        RxDataBuf[pRxData++] = RxData;
                        //DL_UART_Main_transmitDataBlocking(UART_0_INST, RxData);
                        if (pRxData >= RxLength)       // length per pack
                        {
                            RxState = 3;
                            pRxData = 0;
                        }
                    }
                    else if (RxState == 3) {    // wait for footer
                        if (RxData == 0x5B)    
                        {
                            RxState = 0;
                            RxFlag_DEBUG = 1;  // declare that done receiving 
                        }
                    } 

                    break;
                }
                // 自己的text包接收逻辑
                case Custom_Text:   {
                    if (RxState == 0) {          // wait for header
                        if (RxData == '@') 
                            RxState = 1;
                    }
                    else if (RxState == 1)                      // State 2 - Receiving Data
                    {
                        if (RxData == '\r')     // Done packet receiving
                            RxState = 2;
                        else
                            RxDataBuf[pRxData++] = RxData;
                    }
                    else if (RxState == 2)      // State 3 - Waiting for Footer
                    {
                        if(RxData == '\n')      // Is footer - 是包尾
                        {
                            RxState = 0;  
                            RxDataBuf[pRxData] = '\0';      // completing String
                            RxFlag_DEBUG = 1;  // declare that receiving done
                            pRxData = 0;
                        } 
                    }

                    break;
                }
                case WT61:    {
                    
                    break;
                }   

                default:    
                    break;   

            }
            

            break;

        default:
            break;
    }
}
