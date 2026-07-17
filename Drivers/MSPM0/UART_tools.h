#ifndef UART_TOOLS_H
#define UART_TOOLS_H

#include "ti_msp_dl_config.h"

void uart_pc_send_char(char ch); //串口0发送单个字符
void uart_pc_send_string(char* str); //串口0发送字符串
void uart_pc_Init(void);

#endif