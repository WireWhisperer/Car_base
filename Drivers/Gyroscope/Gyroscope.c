/**
 * ============================================================
 *  Gyroscope.c  —  六轴陀螺仪姿态角驱动 (实现)
 * ============================================================
 *
 *  硬件平台: TI MSPM0G3507
 *  通信接口: UART_Gyro (UART3, PA13=RX, PA14=TX), 115200-8N1
 *
 *  调试方法: 在 CCS Debug 中观察全局变量:
 *    - g_gyro_dbg_isr_cnt  = 0 → ISR 未触发 (中断/NVIC 问题)
 *    - g_gyro_dbg_byte_cnt = 0 → RX 未收到任何字节 (接线/波特率)
 *    - g_gyro_dbg_err_cnt  > 0 → 有数据但不是有效帧 (协议不匹配)
 *    - g_gyro_dbg_frame_cnt = 0 → 无有效角度帧
 *    - g_gyro_dbg_buf_snap  → 查看陀螺仪实际发来的数据格式
 * ------------------------------------------------------------
 */

#include "Gyroscope.h"
#include "ti_msp_dl_config.h"
#include "clock.h"
#include <string.h>

/*============================================================================
 * 全局变量 — 姿态角 (ISR 中实时更新)
 *===========================================================================*/

float g_gyro_yaw   = 0.0f;
float g_gyro_pitch = 0.0f;
float g_gyro_roll  = 0.0f;

/*============================================================================
 * 全局变量 — 调试计数器 (Debug 中观察用)
 *===========================================================================*/

volatile uint16_t g_gyro_dbg_isr_cnt   = 0;   /**< ISR 进入次数 */
volatile uint16_t g_gyro_dbg_byte_cnt  = 0;   /**< FeedByte 调用次数 */
volatile uint16_t g_gyro_dbg_frame_cnt = 0;   /**< 有效帧解析次数 */
volatile uint16_t g_gyro_dbg_err_cnt   = 0;   /**< 帧校验失败次数 */
volatile uint8_t  g_gyro_dbg_last_byte = 0;   /**< 最后收到的原始字节 */
volatile uint8_t  g_gyro_dbg_rx_idx    = 0;   /**< 当前接收缓冲区索引 */
volatile uint8_t  g_gyro_dbg_buf_snap[3] = {0, 0, 0};  /**< 收满11字节时的前3字节快照 */

/*============================================================================
 * 协议常量
 *===========================================================================*/

#define FRAME_HEADER         0x5AU
#define FRAME_TYPE_ANGLE     0xBBU
#define FRAME_LEN            11

#define ANGLE_SCALE_DIVISOR  32768.0f
#define ANGLE_RANGE_DEGREES  180.0f

/*============================================================================
 * 模块内部状态
 *===========================================================================*/

static uint8_t  s_rx_buf[FRAME_LEN];
static uint8_t  s_rx_idx = 0;

/*============================================================================
 * 内部工具函数
 *===========================================================================*/

static inline int16_t merge_bytes_le(uint8_t low, uint8_t high)
{
    return (int16_t)(((uint16_t)high << 8) | (uint16_t)low);
}

static inline float raw_to_angle(int16_t raw)
{
    return (float)raw / ANGLE_SCALE_DIVISOR * ANGLE_RANGE_DEGREES;
}

static void send_command(const uint8_t cmd[5])
{
    for (int i = 0; i < 5; i++) {
        DL_UART_transmitDataBlocking(UART_Gyro_INST, cmd[i]);
    }
}

/*============================================================================
 * 数据帧解析器
 *===========================================================================*/

void Gyroscope_FeedByte(uint8_t byte)
{
    g_gyro_dbg_byte_cnt++;          /* 调试: 收到的总字节数 */
    g_gyro_dbg_last_byte = byte;    /* 调试: 记录最后收到的字节 */

    s_rx_buf[s_rx_idx++] = byte;

    /* 帧头校验 */
    if (s_rx_buf[0] != FRAME_HEADER) {
        g_gyro_dbg_err_cnt++;       /* 调试: 帧头错误 */
        s_rx_idx = 0;
        goto update_debug;
    }

    /* 等待收满 11 字节 */
    if (s_rx_idx < FRAME_LEN) {
        goto update_debug;
    }

    /* 仅处理角度帧 */
    if (s_rx_buf[1] != FRAME_TYPE_ANGLE) {
        g_gyro_dbg_err_cnt++;       /* 调试: 非角度帧类型 */
        s_rx_idx = 0;
        goto update_debug;
    }

    /* 校验和 */
    {
        uint8_t sum = 0;
        for (int i = 0; i < 10; i++) {
            sum += s_rx_buf[i];
        }
        if (sum != s_rx_buf[10]) {
            g_gyro_dbg_err_cnt++;   /* 调试: 校验和失败 */
            s_rx_idx = 0;
            goto update_debug;
        }
    }

    /* 解析角度 */
    {
        int16_t raw_roll  = merge_bytes_le(s_rx_buf[2], s_rx_buf[3]);
        int16_t raw_pitch = merge_bytes_le(s_rx_buf[4], s_rx_buf[5]);
        int16_t raw_yaw   = merge_bytes_le(s_rx_buf[6], s_rx_buf[7]);

        g_gyro_roll  = raw_to_angle(raw_roll);
        g_gyro_pitch = raw_to_angle(raw_pitch);
        g_gyro_yaw   = raw_to_angle(raw_yaw);

        g_gyro_dbg_frame_cnt++;     /* 调试: 成功解析一帧 */

        /* 调试: 保存成功帧的前3字节快照 */
        g_gyro_dbg_buf_snap[0] = s_rx_buf[0];
        g_gyro_dbg_buf_snap[1] = s_rx_buf[1];
        g_gyro_dbg_buf_snap[2] = s_rx_buf[2];
    }

    s_rx_idx = 0;

update_debug:
    /* 调试: 导出当前 rx_idx (即使出错退出, 也保留状态供分析) */
    g_gyro_dbg_rx_idx = s_rx_idx;
    /* 持续更新快照的前3字节 */
    g_gyro_dbg_buf_snap[0] = s_rx_buf[0];
    g_gyro_dbg_buf_snap[1] = s_rx_buf[1];
    g_gyro_dbg_buf_snap[2] = s_rx_buf[2];
}

/*============================================================================
 * 公共 API 实现
 *===========================================================================*/

void Gyroscope_Init(void)
{
    /* 清零内部状态 */
    memset(s_rx_buf, 0, sizeof(s_rx_buf));
    s_rx_idx       = 0;
    g_gyro_yaw     = 0.0f;
    g_gyro_pitch   = 0.0f;
    g_gyro_roll    = 0.0f;

    /* 清零调试计数器 */
    g_gyro_dbg_isr_cnt   = 0;
    g_gyro_dbg_byte_cnt  = 0;
    g_gyro_dbg_frame_cnt = 0;
    g_gyro_dbg_err_cnt   = 0;
    g_gyro_dbg_last_byte = 0;
    g_gyro_dbg_rx_idx    = 0;

    /* 使能 UART RX 中断 (与 UART_tools.c 中 uart_pc_Init 相同模式) */
    DL_UART_enableInterrupt(UART_Gyro_INST, DL_UART_INTERRUPT_RX);
    NVIC_ClearPendingIRQ(UART_Gyro_INST_INT_IRQN);
    NVIC_EnableIRQ(UART_Gyro_INST_INT_IRQN);

    /* 发送校准序列 */
    Gyroscope_CalibrateZero();
}

void Gyroscope_CalibrateZero(void)
{
    static const uint8_t CMD_UNLOCK[5]   = {0x55, 0xAA, 0x13, 0x8E, 0x5F};
    static const uint8_t CMD_YAW_ZERO[5] = {0x55, 0xAA, 0x0A, 0x04, 0x00};
    static const uint8_t CMD_SAVE[5]     = {0x55, 0xAA, 0x00, 0x00, 0x00};

    send_command(CMD_UNLOCK);
    mspm0_delay_ms(100);

    send_command(CMD_YAW_ZERO);
    mspm0_delay_ms(100);

    send_command(CMD_SAVE);
    mspm0_delay_ms(100);
}

/*============================================================================
 * UART_Gyro 中断服务程序
 *  (API 调用与 UART_tools.c 中 UART_PC_INST_IRQHandler 完全一致)
 *===========================================================================*/

void UART_Gyro_INST_IRQHandler(void)
{
    g_gyro_dbg_isr_cnt++;   /* 调试: ISR 每进入一次就加 1 */

    switch (DL_UART_getPendingInterrupt(UART_Gyro_INST)) {
        case DL_UART_IIDX_RX:
            {
                uint8_t rx_byte = DL_UART_Main_receiveData(UART_Gyro_INST);
                Gyroscope_FeedByte(rx_byte);
            }
            break;

        default:
            break;
    }
}
