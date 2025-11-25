#ifndef __SERIAL_H
#define __SERIAL_H
#define RxLength 4

typedef enum{
    Custom_Hex,
    Custom_Text,
    WT61,
    JY60
}Protocol;

extern char             RxDataBuf[];
extern uint8_t          gRxAngDataBuf[];
extern uint8_t          RxFlag_DEBUG;         // 当接受完了一个数据的时候， Serial_RxFlag置1
uint8_t                 RxFlag_JY60;

void UART_transmitByte(UART_Regs *uart, uint8_t byte);
void UART_transmitArray (UART_Regs *uart, uint8_t* Array, uint8_t length);
void UART_transmitString(UART_Regs *uart, char* String);
void UART_transmitNum(UART_Regs *uart, uint32_t Number, uint8_t length);
void UART_transmitFloat(UART_Regs *uart, float Number, uint8_t intLength, uint8_t fLength);

void RxFunc(void);

#endif
