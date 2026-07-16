#include "Motor.h"

// 初始化电机相关外设
void Motor_Init(void) {
    DL_TimerG_startCounter(PWM_MOTOR_INST);
    DL_GPIO_setPins(GPIO_MOTOR_EN_PORT, GPIO_MOTOR_EN_PIN);
}

// 设置单个电机速度和方向,duty_desired取值为-1至1
void Motor_Set(MotorId id, float duty_desired) 
{
    // 设置方向
    if (duty_desired == 0)  //停止
    {
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_2_PIN);
        return;
    }
    if (duty_desired < 0 )  //后退
    {
        if (duty_desired < -1) duty_desired = -1;
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_setPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
    }
    else if (duty_desired > 0)  //前进
    {
        if (duty_desired > 1) duty_desired = 1;
        DL_GPIO_setPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
    }
    float PWM_MOTOR_Counter_Compare_Value = PWM_MOTOR_Period_Count * fabsf(duty_desired);       
    DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_MOTOR_Counter_Compare_Value, GPIO_PWM_MOTOR_C0_IDX);
}

// 同时设置左右轮
void Motor_SetBoth(float left_duty_desired, float right_duty_desired) 
{     
    Motor_Set(MOTOR_LEFT, left_duty_desired);
    Motor_Set(MOTOR_RIGHT, right_duty_desired);
}