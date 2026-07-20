/**
 * ============================================================
 *  Gyroscope.h  —  六轴陀螺仪姿态角驱动 (公共接口)
 * ============================================================
 *
 *  硬件平台: TI MSPM0G3507
 *  通信接口: UART_Gyro (UART3, PA13=RX), 115200-8N1
 *            SysConfig 配置: RX-only + FIFO + DMA + RX timeout 中断
 *
 *  功能：通过 UART DMA 接收陀螺仪数据帧，实时解析 Yaw/Pitch/Roll，
 *        更新到全局变量。
 *
 *  协议 (数据手册·6轴串口通信):
 *    - 数据帧: 11 字节, 帧头 0x5A, 角度类型 0xBB
 *    - Yaw:   Byte[6]L + Byte[7]H → int16 → /32768 * 180°
 *    - Pitch: Byte[4]L + Byte[5]H → int16 → /32768 * 180°
 *    - Roll:  Byte[2]L + Byte[3]H → int16 → /32768 * 180°
 *    - 校验和: 前 10 字节累加的低 8 位
 *
 *  使用方法:
 *    1. main() 中 SYSCFG_DL_init() 完成 UART/DMA 硬件初始化
 *    2. 调用 Gyroscope_Init() 启动 DMA 接收
 *    3. 直接读取 g_gyro_yaw / g_gyro_pitch / g_gyro_roll
 *
 *  注意: 当前 UART_Gyro 为 RX-only 模式, TX 不可用,
 *        校准指令 (解锁/归零/保存) 无法发送。
 *        如需校准, 请在 SysConfig 中将 direction 改为 "TX and RX"。
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

extern float g_gyro_yaw;    /**< Yaw  航向角 (度), -180° ~ +180° */
extern float g_gyro_pitch;  /**< Pitch 俯仰角 (度), -180° ~ +180° */
extern float g_gyro_roll;   /**< Roll  横滚角 (度), -180° ~ +180° */

/*============================================================================
 * 调试变量 — Debug 中 Add Watch 观察
 *===========================================================================*/

/** ISR 进入次数, 0 = 中断未触发 */
extern volatile uint16_t g_gyro_dbg_isr_cnt;

/** 成功解析的完整角度帧数 */
extern volatile uint16_t g_gyro_dbg_frame_cnt;

/** 帧校验失败次数 (帧头错/类型错/校验和错) */
extern volatile uint16_t g_gyro_dbg_err_cnt;

/** 本次 ISR 中 DMA 实际收到的字节数 */
extern volatile uint8_t  g_gyro_dbg_rx_size;

/** DMA 缓冲区前 3 字节快照 (看陀螺仪实际发来什么) */
extern volatile uint8_t  g_gyro_dbg_buf_snap[3];

/*============================================================================
 * 公共 API
 *===========================================================================*/

/**
 * @brief 初始化陀螺仪 DMA 接收
 *
 * - 清零全局变量和调试计数器
 * - 配置 DMA 通道 (DMA_Gyro) 从 UART_RX 搬运 32 字节到 gyro_dmaBuffer
 * - 使能 NVIC 中断 (UART 外设中断已由 SysConfig 生成代码使能)
 *
 * @note UART3 硬件 + FIFO + DMA 触发 + RX timeout 中断已由
 *       SYSCFG_DL_init() → SYSCFG_DL_UART_Gyro_init() 完成。
 *       请在 SYSCFG_DL_init() 之后调用本函数。
 */
void Gyroscope_Init(void);

#ifdef __cplusplus
}
#endif

#endif /* __GYROSCOPE_H__ */
