#include "Motor.h"

extern int16_t Motor_Left_roll;
extern int16_t Motor_Right_roll;

float Moter_Left_Speed = 0;
float Moter_Right_Speed = 0;

// 初始化电机相关外设
void Motor_Init(void) {
    //使能电机启动PWM
    DL_TimerG_startCounter(PWM_MOTOR_INST);
    DL_GPIO_setPins(GPIO_MOTOR_EN_PORT, GPIO_MOTOR_EN_PIN);
    NVIC_EnableIRQ(GPIO_MULTIPLE_GPIOA_INT_IRQN);
    
    //启动PID电机测速
    NVIC_EnableIRQ(Motor_PID_INST_INT_IRQN);  
    DL_TimerA_startCounter(Motor_PID_INST);
}

// 设置单个电机速度和方向,duty_desired取值为-1至1
void Motor_Set(MotorId id, float duty_desired) 
{
    float PWM_MOTOR_Counter_Compare_Value = 0;
    
    if (duty_desired == 0)  //停止
    {
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
        DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_2_PIN);
        return;
    }
    switch (id)
    {
        case MOTOR_LEFT:  //如果是左轮
            // 设置方向
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
            PWM_MOTOR_Counter_Compare_Value = PWM_MOTOR_Period_Count * fabsf(duty_desired);       
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_MOTOR_Counter_Compare_Value, GPIO_PWM_MOTOR_C0_IDX);
            break;
        case MOTOR_RIGHT:  //如果是右轮
            // 设置方向
            if (duty_desired < 0 )  //后退
            {
                if (duty_desired < -1) duty_desired = -1;
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
                DL_GPIO_setPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
            }
            else if (duty_desired > 0)  //前进
            {
                if (duty_desired > 1) duty_desired = 1;
                DL_GPIO_setPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
            }
            PWM_MOTOR_Counter_Compare_Value = PWM_MOTOR_Period_Count * fabsf(duty_desired);       
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_MOTOR_Counter_Compare_Value, GPIO_PWM_MOTOR_C1_IDX);
            break;
    }
    
}

// 同时设置左右轮
void Motor_SetBoth(float left_duty_desired, float right_duty_desired) 
{     
    Motor_Set(MOTOR_LEFT, left_duty_desired);
    Motor_Set(MOTOR_RIGHT, right_duty_desired);
}

//更新电机速度m/s根据电机转圈数
void calculate_Speed(void)
{
    float speed = 0;
    
    Moter_Left_Speed = (float)Motor_Left_roll * TIRE_D * PI / PID_TIMER_PERIOD / ENCODER_WIRE_COUNT; 
    Moter_Right_Speed = (float)Motor_Right_roll * TIRE_D * PI / PID_TIMER_PERIOD / ENCODER_WIRE_COUNT; 

    Motor_Left_roll = 0;
    Moter_Right_roll = 0;
}