/**
 * @file    log.h
 * @brief   运行时日志系统 — 环形缓冲区 + 二进制格式 + PC 端 Python 解析
 *
 * 设计要点:
 *   - 固定 10 字节日志条目, 环形缓冲区存储, ISR 中非阻塞写入
 *   - main 循环中调用 log_drain() 通过 UART_PC 发送
 *   - 二进制格式: [0xAA] [type:1] [timestamp:4 LE] [val_a:2 LE] [val_b:2 LE] [chk:1]
 *   - PC 端用 Python 脚本解析/显示/绘图
 *
 * 使用方法:
 *   1. main() 中调用 log_init() 初始化
 *   2. 控制 ISR / 主循环中调用 log_event_xxx() 记录数据
 *   3. main() 的 while(1) 中周期调用 log_drain() 发送数据
 *   4. PC 端运行 python log_viewer.py COMx 115200 查看日志
 */

#ifndef _LOG_H_
#define _LOG_H_

#include <stdint.h>
#include <stdbool.h>

/*============================================================================
 * 日志条目类型
 *===========================================================================*/

typedef enum {
    LOG_HEARTBEAT    = 0x00,  /**< 心跳/同步标记 (每 ~1s 发一次, PC 端用于同步) */
    LOG_PID_SPEED    = 0x01,  /**< PID 当前速度   val_a=左轮 val_b=右轮 (mm/s * 1000) */
    LOG_PID_DUTY     = 0x02,  /**< PID 当前占空比  val_a=左轮 val_b=右轮 (* 1000) */
    LOG_TARGET_SPEED = 0x03,  /**< PID 目标速度   val_a=左轮 val_b=右轮 (mm/s * 1000) */
    LOG_GYRO_YAW     = 0x04,  /**< 陀螺仪偏航角   val_a=Yaw(度*100) val_b=Pitch(度*100) */
    LOG_GRAY_SENSOR  = 0x05,  /**< 灰度传感器     val_a=raw_data val_b=angle_miss(*100) */
    LOG_STATE_CHANGE = 0x06,  /**< 状态切换       val_a=新状态 val_b=旧状态 */
    LOG_BUTTON       = 0x07,  /**< 按键事件       val_a=按键编号 */
    LOG_ROTATE_PARAM = 0x08,  /**< Rotate 控制参数 val_a=error(*100) val_b=speed_sub(*10000) */
    LOG_CUSTOM       = 0x0F,  /**< 自定义事件     val_a/b 由调用者定义 */
} log_event_type_t;

/*============================================================================
 * 状态 ID (用于 LOG_STATE_CHANGE)
 *===========================================================================*/

typedef enum {
    STATE_IDLE       = 0,
    STATE_PATROL     = 1,
    STATE_RECT_TRACE = 2,
    STATE_ROTATE     = 3,
    STATE_MANUAL     = 4,
} car_state_t;

/*============================================================================
 * API
 *===========================================================================*/

/**
 * @brief 初始化日志系统
 * @note  清空环形缓冲区, 初始化内部状态
 */
void log_init(void);

/**
 * @brief 记录一条日志 (通用接口, ISR 安全)
 * @param type     日志类型 @see log_event_type_t
 * @param val_a    数据 A (含义取决于 type)
 * @param val_b    数据 B (含义取决于 type)
 * @note  在 ISR 中调用安全 (关中断保护), 若缓冲区满则丢弃
 */
void log_event(log_event_type_t type, int16_t val_a, int16_t val_b);

/**
 * @brief 记录状态切换 (便捷接口)
 * @param new_state  新状态
 * @param old_state  旧状态
 */
void log_state_change(uint8_t new_state, uint8_t old_state);

/**
 * @brief 发送缓冲区中的一条日志到 UART
 * @retval true  发送了一条日志
 * @retval false 缓冲区空, 没有数据可发
 * @note  在 main 循环中周期调用, 每次发送一条, 不阻塞主逻辑
 */
bool log_drain(void);

/**
 * @brief 立即发送心跳标记
 * @note  在 main 循环开始时调用, 帮助 PC 端识别数据帧边界
 */
void log_heartbeat(void);

#endif /* _LOG_H_ */
