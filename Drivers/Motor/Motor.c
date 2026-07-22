#include "Motor.h"

extern int32_t Motor_Left_roll;
extern int32_t Motor_Right_roll;
extern int32_t Motor_Left_Journey;
extern int32_t Motor_Right_Journey;

const float kp = 0.05;
const float ki = 0.01;
const float duty_max = 0.6;  //正向占空比限幅
const float duty_min = -0.6;  //反向占空比限幅
const float speed_max = 0.5;
const float speed_min = -0.5;

extern char buffer[100];

Motor_PID_info Motor_Left_PID = {MOTOR_LEFT, 0, 0, 0, false};
Motor_PID_info Motor_Right_PID = {MOTOR_RIGHT, 0, 0, 0, false};

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

// 设置单个电机占空比和方向,duty_desired取值为-1至1
void Motor_Set_Duty(MotorId id, float duty_desired) 
{
    float PWM_MOTOR_Counter_Compare_Value = 0;
    
    if (duty_desired > duty_max) duty_desired = duty_max;
    if (duty_desired < duty_min) duty_desired = duty_min;

    switch (id)
    {
        case MOTOR_LEFT:  //如果是左轮
            // 设置方向
            if (duty_desired < 0 )  //后退
            {
                DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
                DL_GPIO_setPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
            }
            else if (duty_desired > 0)  //前进
            {
                DL_GPIO_setPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
            }
            else 
            {
                DL_GPIO_clearPins(GPIO_MOTOR_LEFT_1_PORT, GPIO_MOTOR_LEFT_1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_LEFT_2_PORT, GPIO_MOTOR_LEFT_2_PIN);
            }
            Motor_Left_PID.Current_Duty = duty_desired;
            PWM_MOTOR_Counter_Compare_Value = PWM_MOTOR_Period_Count * fabsf(duty_desired);       
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_MOTOR_Counter_Compare_Value, GPIO_PWM_MOTOR_C1_IDX);
            break;
        case MOTOR_RIGHT:  //如果是右轮
            // 设置方向
            if (duty_desired < 0 )  //后退
            {
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
                DL_GPIO_setPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
            }
            else if (duty_desired > 0)  //前进
            {
                DL_GPIO_setPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
            }
            else
            {
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_1_PORT, GPIO_MOTOR_RIGHT_1_PIN);
                DL_GPIO_clearPins(GPIO_MOTOR_RIGHT_2_PORT, GPIO_MOTOR_RIGHT_2_PIN);
            }
            
            Motor_Right_PID.Current_Duty = duty_desired;
            PWM_MOTOR_Counter_Compare_Value = PWM_MOTOR_Period_Count * fabsf(duty_desired);       
            DL_TimerG_setCaptureCompareValue(PWM_MOTOR_INST, PWM_MOTOR_Counter_Compare_Value, GPIO_PWM_MOTOR_C0_IDX);
            break;
    }
    
}

// 同时设置左右轮占空比
void Motor_Set_Duty_Both(float left_duty_desired, float right_duty_desired) 
{     
    Motor_Set_Duty(MOTOR_LEFT, left_duty_desired);
    Motor_Set_Duty(MOTOR_RIGHT, right_duty_desired);
}

void Motor_Set_Speed(MotorId id, float target_speed)
{
    if (!target_speed)
    {
        switch (id) 
        {
            case MOTOR_LEFT:
                Motor_Left_PID.pid_en = false;
                Motor_Left_PID.Target_Speed = 0;
                Motor_Set_Duty(MOTOR_LEFT, 0);
                break;
            case MOTOR_RIGHT:
                Motor_Right_PID.pid_en = false;
                Motor_Right_PID.Target_Speed = 0;
                Motor_Set_Duty(MOTOR_RIGHT, 0);
                break;
        }
        return;
    }
    
    if (target_speed > speed_max) target_speed = speed_max;
    if (target_speed < speed_min) target_speed = speed_min;
    switch (id)
    {
        case MOTOR_LEFT:
            Motor_Left_PID.pid_en = true;
            Motor_Left_PID.Target_Speed = target_speed;
            break;
        case MOTOR_RIGHT:
            Motor_Right_PID.pid_en = true;
            Motor_Right_PID.Target_Speed = target_speed;
            break;
    }
}

void Motor_Set_Speed_Both(float left_speed, float right_speed)
{
    Motor_Set_Speed(MOTOR_LEFT, left_speed);
    Motor_Set_Speed(MOTOR_RIGHT, right_speed);
}

//更新电机速度m/s及总路程mm根据电机转圈数
void calculate_Speed(void)
{
    float speed = 0;
    float inc_left_journey = (float)Motor_Left_roll * TIRE_D * PI / ENCODER_WIRE_COUNT;  //单位毫米
    float inc_right_journey = (float)Motor_Right_roll * TIRE_D * PI / ENCODER_WIRE_COUNT;  //单位毫米

    Motor_Left_Journey += inc_left_journey;
    Motor_Right_Journey += inc_right_journey;

    Motor_Left_PID.Current_Speed = inc_left_journey / PID_TIMER_PERIOD; 
    Motor_Right_PID.Current_Speed = inc_right_journey / PID_TIMER_PERIOD; 

    Motor_Left_roll = 0;
    Motor_Right_roll = 0;
}

//使用增量式PID更新
void Motor_PID_Update_Single(Motor_PID_info* info)
{
    if (info->pid_en)
    {
        float current_error = info->Target_Speed - info->Current_Speed;

        info->Current_Duty += (float)(kp * (current_error - info->Last_error) + ki * current_error);
        switch (info->id)
        {
            case MOTOR_LEFT:
                Motor_Set_Duty(MOTOR_LEFT, info->Current_Duty);
                
                break;
            case MOTOR_RIGHT:
                Motor_Set_Duty(MOTOR_RIGHT, info->Current_Duty);
                
                break;
        }
        info->Last_error = current_error;

    }
}

//增量式PID更新两电机
void Motor_PID_Update_Both(void)
{
    Motor_PID_Update_Single(&Motor_Left_PID);
    Motor_PID_Update_Single(&Motor_Right_PID);
}
