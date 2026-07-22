/**
 * @file    interrupt.h
 * @brief   中断服务函数与全局变量
 *
 * 集中管理 MSPM0G3507 的中断服务例程和跨模块共享的全局变量:
 *   - SysTick_Handler: 系统时钟 1ms 定时
 *   - GROUP1_IRQHandler: GPIOA 中断 (按键/编码器/MPU6050)
 *   - Motor_PID_INST_IRQHandler: PID 定时器中断 (50ms)
 *   - UART DMA 中断 (WIT/BNO08X, 条件编译)
 */

#ifndef _INTERRUPT_H_
#define _INTERRUPT_H_

#include "clock.h"
#include "mpu6050.h"
#include "wit.h"
#include "ti_msp_dl_config.h"
#include "UART_tools.h"
#include <stdio.h>
#include "Motor.h"

/*---- 按键状态 (按键中断置位, 应用程序轮询清零) ----*/

/** 按键 1 按下标志 (PA24) */
extern volatile uint8_t BUTTON_1_IS_PRESSED;

/** 按键 2 按下标志 (PA23) */
extern volatile uint8_t BUTTON_2_IS_PRESSED;

#endif  /* #ifndef _INTERRUPT_H_ */
