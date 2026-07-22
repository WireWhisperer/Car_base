#include "UART_tools.h"

volatile unsigned char uart_data = 0;
float yaw = 0.f;

void uart_pc_Init(void)
{
    //清除串口中断标志
    NVIC_ClearPendingIRQ(UART_PC_INST_INT_IRQN);
    //使能串口中断
    NVIC_EnableIRQ(UART_PC_INST_INT_IRQN);

    uart_pc_send_string("uart_PC start work\r\n");
}

//串口发送单个字符
void uart_pc_send_char(char ch)
{
    //当串口0忙的时候等待，不忙的时候再发送传进来的字符
    while( DL_UART_isBusy(UART_PC_INST) == true );
    //发送单个字符
    DL_UART_Main_transmitData(UART_PC_INST, ch);
}
//串口发送字符串
void uart_pc_send_string(char* str)
{
    //当前字符串地址不在结尾 并且 字符串首地址不为空
    while(*str!=0&&str!=0)
    {
        //发送字符串首地址中的字符，并且在发送完成之后首地址自增
        uart_pc_send_char(*str++);
    }
}

//串口的中断服务函数
void UART_PC_INST_IRQHandler(void)
{
    //如果产生了串口中断
    switch( DL_UART_getPendingInterrupt(UART_PC_INST) )
    {
        case DL_UART_IIDX_RX://如果是接收中断
            //将发送过来的数据保存在变量中
            uart_data = DL_UART_Main_receiveData(UART_PC_INST);
            //将保存的数据再发送出去
            uart_pc_send_char(uart_data);
            break;

        default://其他的串口中断
            break;
    }
}