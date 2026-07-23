/**
 * @file    Motor.h
 * @brief   直流电机驱动与 PID 速度控制
 *
 * 基于 MSPM0G3507 TimerG PWM (TIMG6) 和 GPIO 方向控制,
 * 配合编码器脉冲计数和定时器中断实现双电机闭环速度控制。
 *
 * 硬件连接:
 *   PWM:  PB2 (TIMG6_CCP0, 右轮), PB3 (TIMG6_CCP1, 左轮)
 *   方向: PB24=LEFT_1, PB20=LEFT_2, PB6=RIGHT_1, PA7=RIGHT_2
 *   使能: PB19=EN
 *   编码器: PA25=LEFT_A, PA26=LEFT_B, PA27=RIGHT_A, PB7=RIGHT_B
 */

#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "math.h"
#include <stdio.h>
#include "UART_tools.h"

/*---- 物理常量 ----*/

/** PWM 周期计数值 (决定 PWM 分辨率) */
#define PWM_MOTOR_Period_Count  1000

/** 轮胎直径 (mm) */
#define TIRE_D                  65.0f

/** 编码器每圈正向脉冲数 */
#define ENCODER_WIRE_COUNT      260

/** PID 控制周期 (ms), 定时器中断间隔 */
#define PID_TIMER_PERIOD        10

/** 圆周率 */
#define PI                      3.14f

/*---- 类型定义 ----*/

/** 电机编号 */
typedef enum {
    MOTOR_LEFT  = 0,  /**< 左轮 */
    MOTOR_RIGHT = 1   /**< 右轮 */
} MotorId;

/** 电机 PID 状态信息 */
typedef struct {
    MotorId id;           /**< 电机编号 (左/右) */
    float   Current_Duty; /**< 当前占空比 [-1.0, 1.0] */
    float   Current_Speed;/**< 当前速度 (m/s) */
    float   Target_Speed; /**< 目标速度 (m/s) */
    float   Last_error;   /**< 上次速度误差 (增量式 PID) */
    bool    pid_en;       /**< PID 使能标志 */
} Motor_PID_info;

/** 左轮 PID 状态 (全局) */
extern Motor_PID_info Motor_Left_PID;

/** 右轮 PID 状态 (全局) */
extern Motor_PID_info Motor_Right_PID;

/*---- 初始化 ----*/

/**
 * @brief  初始化电机相关外设
 * @note   使能 PWM 定时器、电机驱动芯片、编码器 GPIO 中断、
 *         PID 定时器中断 (50ms 周期)
 */
void Motor_Init(void);

/*---- 占空比控制 (开环) ----*/

/**
 * @brief  设置单个电机占空比与方向
 * @param  id            电机编号 (MOTOR_LEFT / MOTOR_RIGHT)
 * @param  duty_desired  目标占空比 [-1.0, 1.0], 正=前进, 负=后退, 0=刹车
 * @note   占空比受 duty_max (0.5) 和 duty_min (0.0) 限幅
 */
void Motor_Set_Duty(MotorId id, float duty_desired);

/**
 * @brief  同时设置左右轮占空比
 * @param  left_duty_desired   左轮目标占空比 [-1.0, 1.0]
 * @param  right_duty_desired  右轮目标占空比 [-1.0, 1.0]
 */
void Motor_Set_Duty_Both(float left_duty_desired, float right_duty_desired);

/*---- 速度控制 (PID 闭环) ----*/

/**
 * @brief  设置单个电机目标速度 (使能 PID)
 * @param  id           电机编号
 * @param  target_speed 目标线速度 (m/s), 正=前进, 负=后退, 0=停止并关闭 PID
 */
void Motor_Set_Speed(MotorId id, float target_speed);

/**
 * @brief  同时设置左右轮目标速度
 * @param  left_speed   左轮目标速度 (m/s)
 * @param  right_speed  右轮目标速度 (m/s)
 */
void Motor_Set_Speed_Both(float left_speed, float right_speed);

/*---- PID 计算 (定时器中断调用) ----*/

/**
 * @brief  根据编码器脉冲更新左右轮当前速度 (m/s)
 * @note   由 PID 定时器中断 (50ms) 调用, 调用后清零编码器脉冲计数
 * @details 速度 = 脉冲数 * 轮胎周长 / 定时周期 / 编码器线数
 */
void calculate_Speed(void);

/**
 * @brief  增量式 PID 更新单个电机
 * @param  info  电机 PID 状态结构体指针
 * @note   由 Motor_PID_Update_Both() 调用, 内部调用 Motor_Set_Duty()
 */
void Motor_PID_Update_Single(Motor_PID_info *info);

/**
 * @brief  增量式 PID 更新两个电机
 * @note   由 PID 定时器中断调用, 先 calculate_Speed() 再更新双电机
 */
void Motor_PID_Update_Both(void);

#endif /* MOTOR_H */
