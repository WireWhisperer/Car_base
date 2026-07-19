/**
 * @file    oled_hardware_i2c.h
 * @brief   SSD1306 OLED 显示屏驱动 (硬件 I2C)
 *
 * 基于 MSPM0 硬件 I2C 控制器 (I2C_OLED) 驱动 0.96 寸 SSD1306 OLED,
 * 支持 128x64 分辨率, 提供字符/数字/字符串/汉字/图片显示和屏幕控制。
 *
 * 硬件连接: I2C_OLED (SysConfig 配置, 建议 Fast Mode 400kHz, 地址 0x3C)
 *
 * SysConfig 配置要点: 见文件头部注释
 */

#ifndef __OLED_HARDWARE_I2C_H
#define __OLED_HARDWARE_I2C_H

#include "ti_msp_dl_config.h"

/** I2C 写命令标识 */
#define OLED_CMD   0
/** I2C 写数据标识 */
#define OLED_DATA  1

/*---- 基础控制 ----*/

/**
 * @brief  毫秒级延时 (封装 mspm0_delay_ms)
 * @param  ms  延时毫秒数
 */
void delay_ms(uint32_t ms);

/**
 * @brief  OLED 初始化
 * @note   上电后调用一次, 执行 SSD1306 标准初始化序列 (约 20 条命令)
 */
void OLED_Init(void);

/** @brief 开启 OLED 显示 */
void OLED_Display_On(void);

/** @brief 关闭 OLED 显示 (进入休眠) */
void OLED_Display_Off(void);

/** @brief 清屏 (全黑) */
void OLED_Clear(void);

/**
 * @brief  反显控制
 * @param  i  0=正常显示, 1=反色显示
 */
void OLED_ColorTurn(uint8_t i);

/**
 * @brief  屏幕旋转 180 度
 * @param  i  0=正常方向, 1=旋转 180°
 */
void OLED_DisplayTurn(uint8_t i);

/*---- 绘图 ----*/

/**
 * @brief  向 SSD1306 写入一个字节
 * @param  dat   数据
 * @param  mode  OLED_CMD (0)=命令, OLED_DATA (1)=数据
 */
void OLED_WR_Byte(uint8_t dat, uint8_t mode);

/**
 * @brief  设置显示坐标
 * @param  x  列地址 (0~127)
 * @param  y  页地址 (0~7, 每页 8 像素高)
 */
void OLED_Set_Pos(uint8_t x, uint8_t y);

/*---- 文字显示 ----*/

/**
 * @brief  在指定位置显示一个字符
 * @param  x      列地址 (0~127)
 * @param  y      行地址 (0~63)
 * @param  chr    要显示的字符 (ASCII)
 * @param  sizey  字体高度: 8=6x8, 16=8x16
 */
void OLED_ShowChar(uint8_t x, uint8_t y, uint8_t chr, uint8_t sizey);

/**
 * @brief  显示字符串
 * @param  x      起始列
 * @param  y      起始行
 * @param  chr    以 '\0' 结尾的字符串
 * @param  sizey  字体高度: 8=6x8, 16=8x16
 */
void OLED_ShowString(uint8_t x, uint8_t y, uint8_t *chr, uint8_t sizey);

/**
 * @brief  显示数字
 * @param  x      起始列
 * @param  y      起始行
 * @param  num    要显示的数字
 * @param  len    数字位数
 * @param  sizey  字体高度: 8=6x8, 16=8x16
 */
void OLED_ShowNum(uint8_t x, uint8_t y, uint32_t num, uint8_t len, uint8_t sizey);

/**
 * @brief  显示汉字
 * @param  x      起始列
 * @param  y      起始行
 * @param  no     汉字在字库中的索引
 * @param  sizey  字体高度 (目前仅支持 16)
 */
void OLED_ShowChinese(uint8_t x, uint8_t y, uint8_t no, uint8_t sizey);

/** @brief  幂运算 (m^n), 用于数字位拆分 */
uint32_t oled_pow(uint8_t m, uint8_t n);

/*---- 图片 ----*/

/**
 * @brief  显示 BMP 位图
 * @param  x      起始列
 * @param  y      起始行
 * @param  sizex  图片宽度 (像素)
 * @param  sizey  图片高度 (像素)
 * @param  BMP    位图数据 (1 bit/pixel, 纵向字节排列)
 */
void OLED_DrawBMP(uint8_t x, uint8_t y, uint8_t sizex, uint8_t sizey, uint8_t BMP[]);

/*---- I2C 总线恢复 ----*/

/**
 * @brief  I2C SDA 解锁 — 总线恢复
 * @note   当 SDA 被从机拉低锁死时, 通过 GPIO 模拟 SCL 时钟脉冲释放总线,
 *         最多 100 个周期, 然后重新初始化 I2C 外设
 */
void oled_i2c_sda_unlock(void);

#endif /* #ifndef __OLED_HARDWARE_I2C_H */
