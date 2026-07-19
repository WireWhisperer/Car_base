/**
 * @file    mpu6050.h
 * @brief   MPU6050 六轴传感器 DMP 姿态解算
 *
 * 通过 I2C 与 MPU6050 通信, 使用 InvenSense DMP (Digital Motion Processor)
 * 固件进行姿态解算, 输出欧拉角 (pitch / roll / yaw) 和原始传感器数据。
 *
 * 硬件连接:
 *   I2C:    I2C_MPU6050 (需在 SysConfig 中配置)
 *   INT:    GPIO_MPU6050_INT (下降沿触发, DMP 数据就绪)
 *
 * 依赖: inv_mpu (I2C 底层驱动), inv_mpu_dmp_motion_driver (DMP 固件)
 */

#ifndef _MPU6050_H_
#define _MPU6050_H_

/** 陀螺仪原始数据 (x, y, z) */
extern short gyro[3];

/** 加速度计原始数据 (x, y, z) */
extern short accel[3];

/** 欧拉角 — 俯仰角 (度) */
extern float pitch;

/** 欧拉角 — 横滚角 (度) */
extern float roll;

/** 欧拉角 — 偏航角 (度) */
extern float yaw;

/**
 * @brief  初始化 MPU6050 与 DMP
 * @note   初始化 I2C 通信 → 加载 DMP 固件 → 配置传感器 → 使能 FIFO,
 *         任一环节失败则触发系统复位 (SYSCFG_DL_resetDevice)。
 *         初始化后自动使能 DMP, 以 50Hz 输出四元数 + 原始数据。
 *         需在 GROUP1_IRQHandler 中调用 Read_Quad() 读取数据。
 */
void MPU6050_Init(void);

/**
 * @brief  从 DMP FIFO 读取最新数据并解算欧拉角
 * @return 0=成功, -1=FIFO 读取失败
 * @note   由 GPIO 中断 (INT 引脚下降沿) 触发调用,
 *         读取后更新全局变量 gyro[3], accel[3], pitch, roll, yaw
 */
int Read_Quad(void);

#endif  /* #ifndef _MPU6050_H_ */
