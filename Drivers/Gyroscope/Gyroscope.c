/**
 * ============================================================
 *  Gyroscope.c  —  六轴陀螺仪姿态角驱动 (实现)
 * ============================================================
 *
 *  硬件平台: TI MSPM0G3507
 *  通信接口: UART_Gyro (UART3, PA13=RX), 115200-8N1
 *            RX-only + FIFO + DMA (DMA_Gyro) + RX timeout 中断
 *
 *  与 WIT/BNO08X 相同的 DMA 接收架构:
 *    - DMA 固定地址 (UART RXDATA) → 块地址 (gyro_dmaBuffer)
 *    - RX timeout 中断触发 ISR 解析数据
 *
 *  数据手册协议帧格式 (11字节):
 *   ┌──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┬──────┐
 *   │ [0]  │ [1]  │ [2]  │ [3]  │ [4]  │ [5]  │ [6]  │ [7]  │ [8]  │ [9]  │ [10] │
 *   │ 0x5A │ TYPE │ DL0  │ DH0  │ DL1  │ DH1  │ DL2  │ DH2  │ DL3  │ DH3  │ SUM  │
 *   └──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┴──────┘
 *   类型 0xBB (角度帧): 数据0=Roll, 数据1=Pitch, 数据2=Yaw, 数据3=保留
 *
 *  注意: 当前 UART_Gyro 为 RX-only, 无法发送校准命令。
 *        如需校准功能, SysConfig 中 direction 改为 "TX and RX" 并连接 TX 引脚。
 * ------------------------------------------------------------
 */

#include "Gyroscope.h"
#include "ti_msp_dl_config.h"

/*============================================================================
 * DMA 接收缓冲区 — 33 字节 (3 × 11 字节帧)
 * DMA 传输 32 字节, 留 1 字节余量给 ISR 中 FIFO 残留读取
 *===========================================================================*/

uint8_t gyro_dmaBuffer[33];

/*============================================================================
 * 全局变量 — 姿态角 (ISR 中实时更新)
 *===========================================================================*/

float g_gyro_yaw   = 0.0f;
float g_gyro_pitch = 0.0f;
float g_gyro_roll  = 0.0f;

/*============================================================================
 * 全局变量 — 调试计数器
 *===========================================================================*/

volatile uint16_t g_gyro_dbg_isr_cnt   = 0;
volatile uint16_t g_gyro_dbg_frame_cnt = 0;
volatile uint16_t g_gyro_dbg_err_cnt   = 0;
volatile uint8_t  g_gyro_dbg_rx_size   = 0;
volatile uint8_t  g_gyro_dbg_buf_snap[3] = {0, 0, 0};

/*============================================================================
 * 协议常量
 *===========================================================================*/

#define FRAME_HEADER          0x5AU
#define FRAME_TYPE_ANGLE      0xBBU
#define FRAME_LEN             11

#define ANGLE_SCALE_DIVISOR   32768.0f
#define ANGLE_RANGE_DEGREES   180.0f
#define DMA_TRANSFER_SIZE     32

/*============================================================================
 * 公共 API
 *===========================================================================*/

/**
 * @brief 初始化陀螺仪 DMA 接收
 *
 * 与 WIT_Init() / BNO08X_Init() 完全相同的模式:
 *   1. 清零全局变量和调试计数器
 *   2. 配置 DMA 通道: 固定地址 (UART RXDATA) → 块地址 (gyro_dmaBuffer)
 *   3. 使能 NVIC 中断
 *
 * @note UART 外设初始化 (FIFO/波特率/DMA触发/RX timeout中断) 已由
 *       SYSCFG_DL_UART_Gyro_init() 完成。
 */
void Gyroscope_Init(void)
{
    /* 清零姿态数据 */
    g_gyro_yaw   = 0.0f;
    g_gyro_pitch = 0.0f;
    g_gyro_roll  = 0.0f;

    /* 清零调试计数器 */
    g_gyro_dbg_isr_cnt   = 0;
    g_gyro_dbg_frame_cnt = 0;
    g_gyro_dbg_err_cnt   = 0;
    g_gyro_dbg_rx_size   = 0;
    g_gyro_dbg_buf_snap[0] = 0;
    g_gyro_dbg_buf_snap[1] = 0;
    g_gyro_dbg_buf_snap[2] = 0;

    /* 配置 DMA: UART RX FIFO → gyro_dmaBuffer (与 WIT/BNO08X 相同模式) */
    DL_DMA_setSrcAddr(DMA, DMA_Gyro_CHAN_ID,
        (uint32_t)(&UART_Gyro_INST->RXDATA));
    DL_DMA_setDestAddr(DMA, DMA_Gyro_CHAN_ID,
        (uint32_t)&gyro_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_Gyro_CHAN_ID, DMA_TRANSFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_Gyro_CHAN_ID);

    // 在使能 NVIC 前清除 UART3 的 pending 中断状态
      DL_UART_Main_clearInterruptStatus(UART_Gyro_INST,
          DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
      NVIC_ClearPendingIRQ(UART_Gyro_INST_INT_IRQN);
      DL_UART_Main_clearInterruptStatus(UART_Gyro_INST,
          DL_UART_MAIN_INTERRUPT_RX_TIMEOUT_ERROR);
      NVIC_ClearPendingIRQ(UART_Gyro_INST_INT_IRQN);

      // 清空 RX FIFO 中可能残留的数据
      uint8_t dummy[4];
      DL_UART_drainRXFIFO(UART_Gyro_INST, dummy, 4);
    /* 使能 NVIC 中断
     * (UART 外设层的 RX timeout 中断已由 SysConfig 生成代码使能) */
    NVIC_EnableIRQ(UART_Gyro_INST_INT_IRQN);
}

/*============================================================================
 * UART_Gyro 中断服务程序 — DMA + RX timeout
 *
 * 触发条件: RX 空闲超过 1 字节时间 (rxTimeoutValue=1)
 * 处理逻辑: 与 WIT/BNO08X ISR 完全相同的 DMA 环形接收模式
 *===========================================================================*/

#if defined(UART_Gyro_INST_IRQHandler)
void UART_Gyro_INST_IRQHandler(void)
{
    uint8_t checkSum;
    uint8_t packCnt = 0;

    g_gyro_dbg_isr_cnt++;   /* 调试: ISR 进入计数 */

    /* 1. 停 DMA, 计算本次收到多少字节 */
    DL_DMA_disableChannel(DMA, DMA_Gyro_CHAN_ID);
    uint8_t rxSize = DMA_TRANSFER_SIZE
                     - DL_DMA_getTransferSize(DMA, DMA_Gyro_CHAN_ID);

    /* 2. 检查 FIFO 中是否还有残留字节 */
    if (DL_UART_isRXFIFOEmpty(UART_Gyro_INST) == false) {
        gyro_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_Gyro_INST);
    }

    g_gyro_dbg_rx_size = rxSize;    /* 调试: 记录本次收了多少字节 */

    /* 3. 以 11 字节为单位解析帧 */
    while (rxSize >= FRAME_LEN) {
        uint8_t *pkt = &gyro_dmaBuffer[packCnt * FRAME_LEN];

        /* 3a. 校验和 */
        checkSum = 0;
        for (int i = 0; i < FRAME_LEN - 1; i++) {
            checkSum += pkt[i];
        }

        /* 3b. 帧头 + 类型 + 校验和 三重验证 */
        if ((pkt[0] == FRAME_HEADER) &&
            (pkt[1] == FRAME_TYPE_ANGLE) &&
            (checkSum == pkt[10])) {

            /* 3c. 解析 Roll / Pitch / Yaw (小端序) */
            int16_t raw_roll  = (int16_t)((pkt[3] << 8) | pkt[2]);
            int16_t raw_pitch = (int16_t)((pkt[5] << 8) | pkt[4]);
            int16_t raw_yaw   = (int16_t)((pkt[7] << 8) | pkt[6]);

            g_gyro_roll  = (float)raw_roll  / ANGLE_SCALE_DIVISOR * ANGLE_RANGE_DEGREES;
            g_gyro_pitch = (float)raw_pitch / ANGLE_SCALE_DIVISOR * ANGLE_RANGE_DEGREES;
            g_gyro_yaw   = (float)raw_yaw   / ANGLE_SCALE_DIVISOR * ANGLE_RANGE_DEGREES;

            g_gyro_dbg_frame_cnt++;     /* 调试: 有效帧 +1 */
            g_gyro_dbg_buf_snap[0] = pkt[0];
            g_gyro_dbg_buf_snap[1] = pkt[1];
            g_gyro_dbg_buf_snap[2] = pkt[2];
        } else {
            g_gyro_dbg_err_cnt++;       /* 调试: 无效帧 +1 */
            /* 保存第一个无效帧的前3字节供诊断 */
            if (g_gyro_dbg_err_cnt == 1) {
                g_gyro_dbg_buf_snap[0] = pkt[0];
                g_gyro_dbg_buf_snap[1] = pkt[1];
                g_gyro_dbg_buf_snap[2] = pkt[2];
            }
        }

        rxSize -= FRAME_LEN;
        packCnt++;
    }

    /* 4. 清空 FIFO 中残留字节 */
    {
        uint8_t dummy[4];
        DL_UART_drainRXFIFO(UART_Gyro_INST, dummy, 4);
    }

    /* 5. 重新启动 DMA */
    DL_DMA_setDestAddr(DMA, DMA_Gyro_CHAN_ID,
        (uint32_t)&gyro_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_Gyro_CHAN_ID, DMA_TRANSFER_SIZE);
    DL_DMA_enableChannel(DMA, DMA_Gyro_CHAN_ID);
}
#endif  /* UART_Gyro_INST_IRQHandler */
