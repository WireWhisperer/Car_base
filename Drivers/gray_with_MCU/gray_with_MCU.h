#ifndef GREY_WITH_MCU_H
#define GREY_WITH_MCU_H

#include "ti_msp_dl_config.h"
#include <stdint.h>
#include <stdbool.h>

/*===========================================================================
 * 感为八路灰度传感器 I2C 驱动
 * 基于《感为八路灰度传感器数据手册》第7章 I²C 通讯协议
 *
 * 硬件连接: MSPM0G3507 I2C1 → 灰度传感器
 *   SDA: PA16
 *   SCL: PA17
 *===========================================================================*/

// I2C 7位地址 (默认: AD0=0 AD1=0 无跳线帽)
// 出厂默认: 0b1001100 = 76 = 0x4C
// 出厂默认含R/W: 0b1001100X = 0x98/0x99
#define GW_GRAY_ADDR            0x4C

/*===========================================================================
 * 命令符定义 (详见协议文档第8章 命令符汇总表)
 *===========================================================================*/

// 读取数字数据 — 返回8路开关状态, 每bit一路, 1=白场(亮) 0=黑场(灭)
// bit0=OUT1 ... bit7=OUT8
#define CMD_DIGITAL_DATA        0xDD

// 连续通道模拟数据 — 一次性读取所有启用通道的8-bit模拟值
#define CMD_CONTINUOUS_ANA      0xB0

// 单通道模拟数据 — 0xB1~0xB8 对应通道 1~8
#define CMD_CH1_ANA             0xB1
#define CMD_CH2_ANA             0xB2
#define CMD_CH3_ANA             0xB3
#define CMD_CH4_ANA             0xB4
#define CMD_CH5_ANA             0xB5
#define CMD_CH6_ANA             0xB6
#define CMD_CH7_ANA             0xB7
#define CMD_CH8_ANA             0xB8

// 传输通道使能 (读写) — 默认 0xFF (全部启用)
#define CMD_CHANNEL_ENABLE      0xCE

// 通道归一化使能 (读写, V3.6+) — 默认 0x00 (关闭)
#define CMD_NORMALIZE_EN        0xCF

// 滞回比较器参数 GrayB (读写) — 0→1 阈值
#define CMD_GRAYB               0xD0

// 滞回比较器参数 GrayW (读写) — 1→0 阈值
#define CMD_GRAYW               0xD1

// Ping 网络诊断 — 发送 0xAA, 期望返回 0x66
#define CMD_PING                0xAA

// 读取错误信息 — 返回8位错误寄存器, 读取后自动清零
#define CMD_ERROR               0xDE

// 设备软件重启 — 重启后需重新 ping
#define CMD_REBOOT              0xC0

// 固件版本号查询 — 高4位=主版本 低4位=次版本
#define CMD_FIRMWARE_VERSION    0xC1

// 软件地址配置 (写) — V3.6+需连续发送两次
#define CMD_SET_ADDR            0xAD

//0：顺时针
typedef enum {
    Clockwise,
    Anticlockwise
} Track_Clock;

/*===========================================================================
 * 函数声明
 *===========================================================================*/

/**
 * @brief  初始化灰度传感器
 *
 *         先发送地址探测确认传感器在 I2C 总线上, 再通过 Ping 建立通信握手。
 *         若任一环节失败则返回 false, 调用方应据此决定后续行为。
 *
 *         I2C 控制器本身由 SysConfig 生成的 SYSCFG_DL_I2C_GRAY_init() 配置
 *         (Controller Mode, 400kHz, 已在 SYSCFG_DL_init() 中自动调用),
 *         本函数只负责传感器层面的初始化。
 *
 * @return true  初始化成功, 传感器通信正常
 * @return false 初始化失败, 传感器未应答 (检查供电/上拉/接线)
 */
bool Gray_Init(void);

/**
 * @brief  Ping 网络诊断
 *
 *         向传感器发送 CMD_PING (0xAA), 期望收到 0x66 应答。
 *         0xAA (10101010) 与 0x66 (01100110) 互为反相, 便于示波器观察波形。
 *         Ping 具有回滚特性——Ping 后传感器自动恢复之前的工作命令。
 *
 * @return true  通信正常 (收到 0x66)
 * @return false 通信失败 (超时或返回值不符)
 */
bool Gray_Ping(void);

/**
 * @brief  读取 8 路灰度数字数据
 *
 *         发送 CMD_DIGITAL_DATA (0xDD) 后读取 1 字节, 每 bit 对应一路:
 *           bit0 = OUT1, bit1 = OUT2, ..., bit7 = OUT8
 *           1 = 白场 (亮/高电平), 0 = 黑场 (灭/低电平)
 *
 *         采用协议 Method 2: 两次独立 I2C 传输 (写命令 → STOP → 读数据),
 *         每次调用都重新发送命令, 确保数据一致性。
 *
 * @return 8 位灰度数据, I2C 错误时返回 0
 */
uint8_t Get_Gray_Data(void);

/**
 * @brief  计算车身与轨道切线的偏差角度
 *
 *         读取 8 路灰度数据, 找出最左和最右黑线位置, 计算中点与传感器中心
 *         (4.5) 的偏差, 经反正切变换为角度值, 限幅至 [-11°, +11°]。
 *
 *         全白 (gray_status == 0xFF) 表示完全脱线, 按循迹方向给出最大转角:
 *           Clockwise → -11°,  Anticlockwise → +11°
 *
 * @param  dir  循迹方向 (Clockwise = 顺时针, Anticlockwise = 逆时针)
 * @return 偏差角度 (度), 范围 [-11°, +11°]
 */
double get_miss_theta(Track_Clock dir);

#endif /* GREY_WITH_MCU_H */
