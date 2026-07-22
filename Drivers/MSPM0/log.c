/**
 * @file    log.c
 * @brief   运行时日志系统实现 — 环形缓冲区 + 二进制帧格式
 *
 * 帧格式 (10 字节):
 *   Byte 0:    0xAA 帧头
 *   Byte 1:    type  (log_event_type_t)
 *   Byte 2-5:  timestamp (uint32_t, little-endian, 毫秒)
 *   Byte 6-7:  val_a (int16_t, little-endian)
 *   Byte 7-8:  val_b (int16_t, little-endian)
 *   Byte 9:    checksum = XOR(Byte1..Byte8)
 */

#include "log.h"
#include "clock.h"
#include "UART_tools.h"
#include "ti_msp_dl_config.h"

/*============================================================================
 * 环形缓冲区配置
 *===========================================================================*/

#define LOG_BUF_ENTRIES  256          /**< 缓冲区容量 (必须为 2 的幂) */
#define LOG_BUF_MASK     (LOG_BUF_ENTRIES - 1)

#define LOG_FRAME_SIZE   10           /**< 每帧字节数 */
#define LOG_FRAME_HEADER 0xAAU        /**< 帧同步头 */

/*============================================================================
 * 环形缓冲区
 *===========================================================================*/

static uint8_t  log_buffer[LOG_BUF_ENTRIES][LOG_FRAME_SIZE];
static volatile uint16_t log_write_idx = 0;   /**< ISR 写入位置 */
static volatile uint16_t log_read_idx  = 0;   /**< main 循环读取位置 */
static volatile bool     log_overflow  = false;/**< 缓冲区溢出标志 */

static uint32_t last_heartbeat_ms = 0;

/*============================================================================
 * 内部辅助函数
 *===========================================================================*/

/**
 * @brief 将二进制帧编码到 buffer 中
 */
static void log_encode(uint8_t *frame, uint8_t type,
                       uint32_t timestamp, int16_t val_a, int16_t val_b)
{
    frame[0] = LOG_FRAME_HEADER;                       /* 帧头 */
    frame[1] = type;                                    /* 类型 */

    /* 时间戳 — 小端序 */
    frame[2] = (uint8_t)(timestamp & 0xFF);
    frame[3] = (uint8_t)((timestamp >> 8) & 0xFF);
    frame[4] = (uint8_t)((timestamp >> 16) & 0xFF);
    frame[5] = (uint8_t)((timestamp >> 24) & 0xFF);

    /* val_a — 小端序 */
    frame[6] = (uint8_t)(val_a & 0xFF);
    frame[7] = (uint8_t)((val_a >> 8) & 0xFF);

    /* val_b — 小端序 */
    frame[8] = (uint8_t)(val_b & 0xFF);

    /* 校验和 = XOR(Byte1..Byte8) */
    uint8_t checksum = 0;
    for (int i = 1; i < 9; i++) {
        checksum ^= frame[i];
    }
    frame[9] = checksum;
}

/**
 * @brief 发送一帧原始数据到 UART (阻塞, 仅在 main 循环中调用)
 */
static void log_send_frame(const uint8_t *frame)
{
    for (int i = 0; i < LOG_FRAME_SIZE; i++) {
        uart_pc_send_char(frame[i]);
    }
}

/*============================================================================
 * 公共 API
 *===========================================================================*/

void log_init(void)
{
    log_write_idx = 0;
    log_read_idx  = 0;
    log_overflow  = false;
    last_heartbeat_ms = 0;

    /* 清零缓冲区 (调试时方便查看) */
    for (int i = 0; i < LOG_BUF_ENTRIES; i++) {
        for (int j = 0; j < LOG_FRAME_SIZE; j++) {
            log_buffer[i][j] = 0;
        }
    }
}

void log_event(log_event_type_t type, int16_t val_a, int16_t val_b)
{
    /* 关中断保护环形缓冲区写操作 */
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t next_write = (log_write_idx + 1) & LOG_BUF_MASK;

    /* 缓冲区满则覆盖最旧数据 */
    if (next_write == log_read_idx) {
        log_overflow = true;
        log_read_idx = (log_read_idx + 1) & LOG_BUF_MASK;
    }

    /* 编码并写入缓冲区 */
    log_encode(log_buffer[log_write_idx], (uint8_t)type,
               tick_ms, val_a, val_b);
    log_write_idx = next_write;

    /* 恢复中断状态 */
    if (!(primask & 0x1)) {
        __enable_irq();
    }
}

void log_state_change(uint8_t new_state, uint8_t old_state)
{
    log_event(LOG_STATE_CHANGE, (int16_t)new_state, (int16_t)old_state);
}

bool log_drain(void)
{
    /* 无数据则直接返回 */
    if (log_read_idx == log_write_idx) {
        return false;
    }

    /* 发送一条日志帧 */
    log_send_frame(log_buffer[log_read_idx]);
    log_read_idx = (log_read_idx + 1) & LOG_BUF_MASK;

    return true;
}

void log_heartbeat(void)
{
    /* 约每秒发一次心跳, 帮助 PC 端同步帧边界 */
    if (tick_ms - last_heartbeat_ms >= 1000) {
        last_heartbeat_ms = tick_ms;
        log_event(LOG_HEARTBEAT, 0, 0);
    }
}
