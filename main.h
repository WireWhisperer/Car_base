/**
 * @file    main.h
 * @brief   项目主头文件 — 聚合所有模块的依赖
 *
 * 智能车竞赛项目 (MSPM0G3507), 包含以下子系统:
 *   - 双电机 PWM 驱动 + 编码器 PID 速度闭环 (Motor)
 *   - 感为八路灰度传感器 I2C 驱动 (gray_with_MCU)
 *   - SSD1306 OLED 显示屏 (oled_hardware_i2c)
 *   - MPU6050 六轴传感器 + DMP 姿态解算 (mpu6050)
 *   - 维特智能 IMU (WIT)
 *   - PC 调试串口 115200bps (UART_tools)
 *   - SysTick 毫秒时钟 (clock)
 */

#ifndef _MAIN_H_
#define _MAIN_H_

#include "clock.h"
#include "oled_hardware_i2c.h"
#include "Motor.h"
#include "UART_tools.h"
#include <stdio.h>
#include "gray_with_MCU.h"
#include "Gyroscope.h"

#endif  /* #ifndef _MAIN_H_ */
