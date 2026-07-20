/**
 * ============================================================
 *  Gyroscope.h  —  六轴陀螺仪姿态角驱动 (公共接口)
 * ============================================================
 *
 *  硬件平台: TI MSPM0G3507
 *  通信接口: UART_Gyro (UART3, PA13=RX, PA14=TX), 115200-8N1
 *            SysConfig 中已配好, SYSCFG_DL_init() 完成硬件初始化
 *
 *  协议概要（参考 6轴数据手册·串口通信）：
 *    - 数据帧: 11 字节，帧头 0x5A, 角度帧类型 0xBB
 *    - Yaw:   Byte[6]低 + Byte[7]高 → short → /32768 × 180°
 *    - Pitch: Byte[4]低 + Byte[5]高 → short → /32768 × 180°
 *    - Roll:  Byte[2]低 + Byte[3]高 → short → /32768 × 180°
 *    - 校验和: 前 10 字节累加的低 8 位
 * ------------------------------------------------------------
 */

#ifndef __GYROSCOPE_H__
#define __GYROSCOPE_H__

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================
 * 全局变量 — 姿态角 (ISR 中实时更新)
 *===========================================================================*/

/** 当前 Yaw 航向角 (度), -180° ~ +180° */
extern float g_gyro_yaw;

/** 当前 Pitch 俯仰角 (度), -180° ~ +180° */
extern float g_gyro_pitch;

/** 当前 Roll 横滚角 (度), -180° ~ +180° */
extern float g_gyro_roll;

/*============================================================================
 * 调试变量 — 在 Debug 中观察以下变量定位问题
 *===========================================================================*/

/** ISR 进入次数: 0=中断未触发 */
extern volatile uint16_t g_gyro_dbg_isr_cnt;

/** FeedByte 调用次数 (收到的总字节数) */
extern volatile uint16_t g_gyro_dbg_byte_cnt;

/** 成功解析完整角度帧的次数 */
extern volatile uint16_t g_gyro_dbg_frame_cnt;

/** 帧校验失败的次数 (帧头错/校验和错) */
extern volatile uint16_t g_gyro_dbg_err_cnt;

/** 最后一次收到的原始字节 */
extern volatile uint8_t  g_gyro_dbg_last_byte;

/** 当前接收缓冲区索引 (0~10) */
extern volatile uint8_t  g_gyro_dbg_rx_idx;

/** 接收缓冲区前 3 字节快照 (帧头/类型/数据0低) */
extern volatile uint8_t  g_gyro_dbg_buf_snap[3];

/*============================================================================
 * 公共 API
 *===========================================================================*/

/**
 * @brief 初始化陀螺仪驱动
 * @note UART3 硬件已由 SYSCFG_DL_init() 完成初始化。
 *       本函数使能 RX 中断并发送校准指令。
 */
void Gyroscope_Init(void);

/**
 * @brief 向解析器喂入一个字节 (在 UART RX 中断中调用)
 */
void Gyroscope_FeedByte(uint8_t byte);

/**
 * @brief 发送 Yaw 角归零指令 (解锁 → 归零 → 保存)
 */
void Gyroscope_CalibrateZero(void);

#ifdef __cplusplus
}
#endif

#endif /* __GYROSCOPE_H__ */
