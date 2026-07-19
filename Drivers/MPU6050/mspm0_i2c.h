/**
 * @file    mspm0_i2c.h
 * @brief   MSPM0 I2C 底层驱动 — 为 MPU6050 提供 I2C 读写接口
 *
 * 封装 MSPM0 DriverLib 的 I2C 控制器操作, 支持:
 *   - 纯写 (Controller TX): 地址 + 寄存器 + 数据
 *   - 组合写后读 (Combined Write-then-Read, Repeated START):
 *     地址 + 寄存器 + Repeated START + 读数据
 *   - 总线恢复 (SDA 解锁): 当 SDA 被从机锁死时通过 GPIO 模拟时钟脉冲释放
 *
 * 所有 I2C 操作均带超时保护 (I2C_TIMEOUT_MS = 10ms)
 */

#ifndef _MSPM0_I2C_H_
#define _MSPM0_I2C_H_

/**
 * @brief  I2C SDA 解锁 — 总线恢复
 * @note   当 I2C 通信超时时调用, 将 SCL 切换为 GPIO 输出并发送时钟脉冲
 *         直到 SDA 释放为高电平 (最多 100 个周期), 然后重新初始化 I2C 外设
 */
void mpu6050_i2c_sda_unlock(void);

/**
 * @brief  I2C 写数据
 * @param  slave_addr  7 位从机地址
 * @param  reg_addr    寄存器地址 (单字节)
 * @param  length      数据长度 (字节数)
 * @param  data        待写数据缓冲区
 * @return 0=成功, -1=超时 (已执行 SDA 解锁)
 */
int mspm0_i2c_write(unsigned char slave_addr,
                     unsigned char reg_addr,
                     unsigned char length,
                     unsigned char const *data);

/**
 * @brief  I2C 组合写后读 (Repeated START)
 * @param  slave_addr  7 位从机地址
 * @param  reg_addr    寄存器地址 (单字节, 先写入从机)
 * @param  length      读取长度 (字节数)
 * @param  data        接收缓冲区
 * @return 0=成功, -1=超时 (已执行 SDA 解锁)
 * @note   使用 MCTR.RD_ON_TXEMPTY 实现 TX→RX 自动切换,
 *         总线时序: S Addr_W A Reg A Sr Addr_R A Data... A P
 */
int mspm0_i2c_read(unsigned char slave_addr,
                    unsigned char reg_addr,
                    unsigned char length,
                    unsigned char *data);

#endif  /* #ifndef _MSPM0_I2C_H_ */
