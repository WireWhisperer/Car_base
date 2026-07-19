/**
 * @file    UART_tools.h
 * @brief   串口工具 — PC 端调试串口 (UART0)
 *
 * 通过 MSPM0G3507 的 UART0 (PA10=TX, PA11=RX) 与 PC 通信,
 * 波特率 115200, 支持字符/字符串发送和中断接收回显。
 */

#ifndef UART_TOOLS_H
#define UART_TOOLS_H

#include "ti_msp_dl_config.h"

/**
 * @brief  初始化 PC 串口
 * @note   调用 SYSCFG_DL_init() 初始化 UART 硬件,
 *         使能 NVIC 中断, 并发送 "uart_PC start work\r\n" 确认启动
 */
void uart_pc_Init(void);

/**
 * @brief  串口发送单个字符 (阻塞)
 * @param  ch  待发送字符
 */
void uart_pc_send_char(char ch);

/**
 * @brief  串口发送字符串 (阻塞)
 * @param  str  待发送的以 '\0' 结尾的字符串
 */
void uart_pc_send_string(char *str);

#endif
