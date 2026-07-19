/**
 * @file    wit.h
 * @brief   维特智能 (WIT) IMU 姿态传感器驱动
 *
 * 通过 UART DMA 接收 WIT 传感器的姿态数据包 (0x55 帧头, 11 字节一组),
 * 解析加速度、角速度、欧拉角和温度信息。
 *
 * 硬件连接: UART_WIT (SysConfig 配置, RX only, 波特率与模块一致)
 *
 * SysConfig 配置要点: 见文件头部注释
 */

#ifndef __WIT_H
#define __WIT_H

#include "ti_msp_dl_config.h"

/**
 * @brief WIT 传感器数据
 * @note  由 UART DMA 中断自动更新
 */
typedef struct {
    float   pitch;       /**< 俯仰角 (度) */
    float   roll;        /**< 横滚角 (度) */
    float   yaw;         /**< 偏航角 (度) */
    float   temperature; /**< 温度 (°C) */
    int16_t ax;          /**< X 轴加速度 (mg) */
    int16_t ay;          /**< Y 轴加速度 (mg) */
    int16_t az;          /**< Z 轴加速度 (mg) */
    int16_t gx;          /**< X 轴角速度 (°/s) */
    int16_t gy;          /**< Y 轴角速度 (°/s) */
    int16_t gz;          /**< Z 轴角速度 (°/s) */
    int16_t version;     /**< 固件版本号 */
} WIT_Data_t;

/** 全局 WIT 传感器数据, 中断中自动更新 */
extern WIT_Data_t wit_data;

/**
 * @brief  初始化 WIT 传感器 UART DMA 接收
 * @note   配置 DMA 通道从 UART RX FIFO 传输 32 字节到 wit_dmaBuffer,
 *         使能 UART 中断。实际数据解析在 UART_WIT_INST_IRQHandler 中完成
 */
void WIT_Init(void);

#endif /* #ifndef __WIT_H */
