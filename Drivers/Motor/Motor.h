#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>
#include "ti_msp_dl_config.h"
#include "math.h"

#define PWM_MOTOR_Period_Count 1000  //PWM总计数
#define TIRE_D 48.5  //轮胎直径(单位mm)
#define ENCODER_WIRE_COUNT 260  //编码器每圈正脉冲数
#define PID_TIMER_PERIOD 50  //PID定时器每50ms触发一次中断
#define PI 3.14

// 电机编号左0右1
typedef enum {
    MOTOR_LEFT = 0,
    MOTOR_RIGHT = 1
} MotorId;

typedef struct{
    MotorId id;           //左or右
    float Current_Duty;   //当前占空比
    float Current_Speed;  //当前速度
    float Target_Speed;   //目标速度
    float Last_error;     //速度误差
    bool pid_en;          //是否开启pid
}Motor_PID_info;

// 初始化电机相关外设
void Motor_Init(void);
// 设置单个电机占空比和方向
void Motor_Set_Duty(MotorId id, float duty_desired);
// 同时设置左右轮占空比
void Motor_Set_Duty_Both(float left_duty_desired, float right_duty_desired); 
// 设置单个电机速度和方向
void Motor_Set_Speed(MotorId id, float target_speed);
// 同时设置左右轮速度
void Motor_Set_Speed_Both(float left_speed, float right_speed);

//更新电机速度mm/s根据电机转圈数
void calculate_Speed(void);
//使用增量式PID更新
void Motor_PID_Update_Single(Motor_PID_info* info);
//增量式PID更新两电机
void Motor_PID_Update_Both(void);

#endif 
