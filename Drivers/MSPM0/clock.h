/**
 * @file    clock.h
 * @brief   SysTick 时钟与延时函数
 *
 * 基于 Cortex-M0+ SysTick 定时器, 提供毫秒级延时和系统时钟计数。
 * SysTick 每秒产生 1000 次中断 (1kHz), tick_ms 每次自增。
 */

#ifndef _CLOCK_H_
#define _CLOCK_H_

/** 系统运行毫秒计数, SysTick_Handler 中每 1ms 自增 */
extern volatile unsigned long tick_ms;

/**
 * @brief  毫秒级阻塞延时
 * @param  num_ms  延时毫秒数
 * @return 始终返回 0
 */
int mspm0_delay_ms(unsigned long num_ms);

/**
 * @brief  获取当前系统毫秒时间戳
 * @param  count  输出参数, 接收当前 tick_ms 值
 * @return 0=成功, 1=count 指针为空
 */
int mspm0_get_clock_ms(unsigned long *count);

/**
 * @brief  初始化 SysTick 定时器
 * @note   配置 SysTick 每 1ms 产生一次中断 (CPUCLK_FREQ / 1000),
 *         中断优先级设为最高 (0)
 */
void SysTick_Init(void);

#endif  /* #ifndef _CLOCK_H_ */
