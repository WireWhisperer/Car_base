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

// 初始化电机相关外设
void Motor_Init(void);
// 设置单个电机速度和方向
void Motor_Set(MotorId id, float duty_desired) ;
// 同时设置左右轮
void Motor_SetBoth(float left_duty_desired, float right_duty_desired);
//更新电机速度m/s根据电机转圈数
void calculate_Speed(void);

#endif 
