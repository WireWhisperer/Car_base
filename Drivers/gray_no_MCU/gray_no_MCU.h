/**
 * @file    gray_no_MCU.h
 * @brief   感为八路灰度传感器 GPIO 驱动 (非 I2C 模式)
 *
 * 通过 MSPM0 GPIO 模拟 3-to-8 多路选择器 (AD0/AD1/AD2) 依次读取
 * 8 路灰度传感器的数字输出 (OUT 引脚)。
 *
 * 硬件连接:
 *   AD0: PA0, AD1: PA2, AD2: PB9  (通道选择, OUTPUT)
 *   OUT: PB8  (传感器数字输出, INPUT)
 *
 * @note   此为 GPIO 轮询方式, 不使用 I2C。
 *         如需 I2C 控制模式请使用 gray_with_MCU 驱动。
 */

#ifndef GREY_NO_MCU_H
#define GREY_NO_MCU_H

#include "ti_msp_dl_config.h"

/** 循迹方向 */
typedef enum {
    Clockwise,       /**< 顺时针 */
    Anticlockwise    /**< 逆时针 */
} Track_Clock;

/**
 * @brief  读取 8 路灰度数字数据 (GPIO 轮询)
 *
 *         通过 AD0/AD1/AD2 依次使能 8 个通道, 读取 OUT 引脚电平,
 *         拼接为 8 位数据: bit7=OUT1, bit6=OUT2, ..., bit0=OUT8
 *         1=白场 (亮/高电平), 0=黑场 (灭/低电平)
 *
 * @return 8 位灰度数据
 */
uint8_t Get_Gray_Data(void);

/**
 * @brief  计算车身与轨道切线的偏差角度
 *
 *         读取 8 路灰度数据, 找出最左和最右黑线位置, 计算中点与传感器中心
 *         (4.5) 的偏差, 经反正切变换得到角度, 限幅至 [-11°, +11°]。
 *         全白 (0xFF) 表示脱线, 按循迹方向给出最大转角。
 *
 * @param  dir  循迹方向
 * @return 偏差角度 (度), 范围 [-11°, +11°]
 */
double get_track(Track_Clock dir);

#endif /* GREY_NO_MCU_H */
