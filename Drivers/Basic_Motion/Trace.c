#include "Trace.h"

extern float g_gyro_yaw;
extern char buffer[100];

//最小差速下限 (克服静摩擦力)
const float  min_SPEED_SUB= 0.05;

extern int32_t Motor_Left_Journey;
extern int32_t Motor_Right_Journey;

//新思路循迹
void Patrol_Trace(Track_Clock spin_dir,float left_speed, float right_speed, float speed_add, float yaw_current)
{
    
    float yaw_target = yaw_current + 180;
  
    if (yaw_target > 180) yaw_target -= 360;
    if (yaw_target < -180) yaw_target += 360;
    
    double angle_miss = 0;//灰度传感器的当前与切线偏移角度
    float yaw_missed = 0; //小车与最终终点切线角度
    uint8_t have_closed = 0;

    while(1)
    {
        angle_miss = get_miss_theta(spin_dir);
        yaw_missed = yaw_target - g_gyro_yaw;

        if (yaw_missed > 180) yaw_missed -= 360;
        if (yaw_missed < -180) yaw_missed += 360;

        if ((angle_miss > -12) && (angle_miss <= -9)) Motor_Set_Duty_Both(left_speed, right_speed + 4*speed_add);
        if ((angle_miss > -9) && (angle_miss <= -6)) Motor_Set_Duty_Both(left_speed, right_speed + 3*speed_add);
        if ((angle_miss > -6) && (angle_miss <= -3)) Motor_Set_Duty_Both(left_speed, right_speed + 2*speed_add);
        if ((angle_miss > -3) && (angle_miss <= -1)) Motor_Set_Duty_Both(left_speed, right_speed + 1*speed_add);
        if ((angle_miss > -1) && (angle_miss <= 1)) Motor_Set_Duty_Both(left_speed, right_speed);
        if ((angle_miss > 1) && (angle_miss <= 3)) Motor_Set_Duty_Both(left_speed + speed_add, right_speed);
        if ((angle_miss > 3) && (angle_miss <= 6)) Motor_Set_Duty_Both(left_speed + 2*speed_add, right_speed);
        if ((angle_miss > 6) && (angle_miss <= 9)) Motor_Set_Duty_Both(left_speed + 3*speed_add, right_speed);
        if ((angle_miss > 9) && (angle_miss <= 12)) Motor_Set_Duty_Both(left_speed + 4*speed_add, right_speed);

        //sprintf((char*)oled_buffer, "miss: %f", yaw_missed);
        //OLED_ShowString(0, 2, oled_buffer, 8);
        
        //借助陀螺仪判断是否快出线
        if ((yaw_missed > -10) && (yaw_missed < 10))
        {
            if ((Get_Gray_Data() == 0xff)) break;
            else if (!have_closed)
            {
                left_speed /= 3;
                right_speed /= 3;
                speed_add /= 3;
                have_closed = 1;
                Motor_Set_Duty_Both(0, 0);
                mspm0_delay_ms(100);
            }
        }
        
        mspm0_delay_ms(10);
    }

    Motor_Set_Duty_Both(0, 0);
    //Rotate(0.4, yaw_target);
    //DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
}

void Rotate(Track_Clock dir, float speed, float yaw_target)
{
    float left_speed = 0;
    float right_speed = 0;
    float speed_sub = 0;
    float error = 0;
    float yaw_current = 0;
    float last_error = 0;

    /* 速度限幅 */
    if (speed > 0.6f) speed = 0.6f;
    if (speed < 0.0f) speed = 0.0f;

    /* --- Bug Fix: yaw_target 归一化到 [-180, 180] ---
     * 原来写的是 "yaw_target > yaw_target+180" 永远为假, 归一化是空操作 */
    if (yaw_target > 180.0f)  yaw_target -= 360.0f;
    if (yaw_target < -180.0f) yaw_target += 360.0f;

    do
    {
        /* 取当前 yaw, 并解开角度环绕使它与 yaw_target 在同一圈 */
        yaw_current = g_gyro_yaw;
        if (yaw_current > yaw_target + 180.0f) yaw_current -= 360.0f;
        if (yaw_current < yaw_target - 180.0f) yaw_current += 360.0f;

        error = yaw_target - yaw_current;

        /* PD 控制器: P=0.5/180, D=0.0001 */
        speed_sub = 0.5f / 180.0f * error + 0.0001f * (error - last_error);

        if (speed_sub > 0.0f && speed_sub < min_SPEED_SUB) speed_sub = min_SPEED_SUB;
        if (speed_sub < 0.0f && speed_sub > -min_SPEED_SUB) speed_sub = -min_SPEED_SUB;

        if (error > 0.0f)
        {
            /* 目标在 CCW 方向: 右轮快 → 逆时针转 → yaw ↑ */
            left_speed  = speed;
            right_speed = speed + speed_sub;
        }
        else
        {
            /* 目标在 CW 方向(或已超过): 左轮快 → 顺时针转 → yaw ↓
             * speed_sub <= 0, 所以 -speed_sub >= 0, 左轮加速 */
            left_speed  = speed - speed_sub;   
            right_speed = speed;
        }

        Motor_Set_Speed_Both(left_speed, right_speed);

        last_error = error;

        sprintf((char*)buffer, "yaw=%lf \r\n", yaw_current);
        uart_pc_send_string(buffer);

        sprintf((char*)buffer, "speed_sub=%lf \r\n", speed_sub);
        uart_pc_send_string(buffer);

        mspm0_delay_ms(50);

    } while (error > 1.f || error < -1.f);   /* Fix: 双向判断, 防止 overshoot 后直接退出 */

    Motor_Set_Speed_Both(0, 0);
}

bool meet_Rect(void)
{
    uint8_t count = 0;
    uint8_t gray_status = Get_Gray_Data();
    bool flag = false;

    for (int i = 0; i < 8; i++) if (!((gray_status >> i) & 0x1)) count++; //count记有几个0

    if (count == 0 || count >= 3) flag = true;
    //if (count >= 3) flag = true;

    return flag;
}

//左摇右晃十分差劲
void Rect_PID_trace(Track_Clock dir, float speed, float speed_add)
{
    double angle_miss = 0;//灰度传感器的当前与切线偏移角度
    uint8_t have_closed = 0;

    while(1)
    {
        Motor_Set_Speed_Both(speed, speed);
        do
        {
            angle_miss = get_miss_theta(dir);

            if ((angle_miss > -12) && (angle_miss <= -9)) Motor_Set_Speed_Both(speed + 4*speed_add, speed);
            if ((angle_miss > -9) && (angle_miss <= -6)) Motor_Set_Speed_Both(speed + 3*speed_add, speed);
            if ((angle_miss > -6) && (angle_miss <= -3)) Motor_Set_Speed_Both(speed + 2*speed_add, speed);
            if ((angle_miss > -3) && (angle_miss <= -1)) Motor_Set_Speed_Both(speed + 1*speed_add, speed);
            if ((angle_miss > -1) && (angle_miss <= 1)) Motor_Set_Speed_Both(speed, speed);
            if ((angle_miss > 1) && (angle_miss <= 3)) Motor_Set_Speed_Both(speed, speed + speed_add);
            if ((angle_miss > 3) && (angle_miss <= 6)) Motor_Set_Speed_Both(speed, speed + 2*speed_add);
            if ((angle_miss > 6) && (angle_miss <= 9)) Motor_Set_Speed_Both(speed, speed + 3*speed_add);
            if ((angle_miss > 9) && (angle_miss <= 12)) Motor_Set_Speed_Both(speed, speed + 4*speed_add);
        }while(!meet_Rect());
        float spin_yaw_target = dir ? g_gyro_yaw + 90 : g_gyro_yaw - 90;
        Rotate(dir, 0.05, spin_yaw_target);
    }
    
}

void Rect_DUTY_trace(Track_Clock dir, float left_duty, float right_duty, float duty_add_left, float duty_add_right, float yaw_current)
{
    double angle_miss = 0;//灰度传感器的当前与切线偏移角度
    uint8_t have_closed = 0;
    Motor_Left_PID.pid_en = false;
    Motor_Right_PID.pid_en = false;

    Motor_Set_Duty_Both(left_duty, right_duty);
    do
    {
        angle_miss = get_miss_theta(dir);

        if ((angle_miss > -12) && (angle_miss <= -9)) Motor_Set_Duty_Both(left_duty - 2*duty_add_left, right_duty + 4*duty_add_right);
        if ((angle_miss > -9) && (angle_miss <= -6)) Motor_Set_Duty_Both(left_duty - duty_add_left, right_duty + 3*duty_add_right);
        if ((angle_miss > -6) && (angle_miss <= -3)) Motor_Set_Duty_Both(left_duty , right_duty + 2*duty_add_right);
        if ((angle_miss > -3) && (angle_miss <= -1)) Motor_Set_Duty_Both(left_duty, right_duty + duty_add_right);
        if ((angle_miss > -1) && (angle_miss <= 1)) Motor_Set_Duty_Both(left_duty, right_duty);
        if ((angle_miss > 1) && (angle_miss <= 3)) Motor_Set_Duty_Both(left_duty + duty_add_left, right_duty);
        if ((angle_miss > 3) && (angle_miss <= 6)) Motor_Set_Duty_Both(left_duty + 2*duty_add_left, right_duty);
        if ((angle_miss > 6) && (angle_miss <= 9)) Motor_Set_Duty_Both(left_duty + 3*duty_add_left, right_duty - duty_add_right);
        if ((angle_miss > 9) && (angle_miss <= 12)) Motor_Set_Duty_Both(left_duty + 4*duty_add_left, right_duty - 2*duty_add_right);
        mspm0_delay_ms(30);
     }while(!meet_Rect() || (Motor_Left_Journey < 800 && Motor_Right_Journey < 800));

    float spin_yaw_target = dir ? yaw_current + 90 : yaw_current - 90;
    if (spin_yaw_target > 180) spin_yaw_target -= 360;
    if (spin_yaw_target < -180) spin_yaw_target += 360;

    while(fabsf(g_gyro_yaw - spin_yaw_target) > 5)
    //while(meet_Rect())
    {
        angle_miss = get_miss_theta(dir);

        if ((angle_miss > -12) && (angle_miss <= -9)) Motor_Set_Duty_Both(left_duty - 4*duty_add_left, right_duty + 6*duty_add_right);
        if ((angle_miss > -9) && (angle_miss <= -6)) Motor_Set_Duty_Both(left_duty - 2*duty_add_left, right_duty + 4*duty_add_right);
        if ((angle_miss > -6) && (angle_miss <= -3)) Motor_Set_Duty_Both(left_duty , right_duty + 2*duty_add_right);
        if ((angle_miss > -3) && (angle_miss <= -1)) Motor_Set_Duty_Both(left_duty, right_duty + duty_add_right);
        if ((angle_miss > -1) && (angle_miss <= 1)) Motor_Set_Duty_Both(left_duty, right_duty);
        if ((angle_miss > 1) && (angle_miss <= 3)) Motor_Set_Duty_Both(left_duty + duty_add_left, right_duty);
        if ((angle_miss > 3) && (angle_miss <= 6)) Motor_Set_Duty_Both(left_duty + 2*duty_add_left, right_duty);
        if ((angle_miss > 6) && (angle_miss <= 9)) Motor_Set_Duty_Both(left_duty + 4*duty_add_left, right_duty - 2*duty_add_right);
        if ((angle_miss > 9) && (angle_miss <= 12)) Motor_Set_Duty_Both(left_duty + 6*duty_add_left, right_duty - 4*duty_add_right);
        mspm0_delay_ms(10);
    }
    
    // Motor_Set_Duty_Both(left_duty, right_duty);
    // mspm0_delay_ms(200);

    //DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    // float spin_yaw_target = dir ? yaw_current + 90 : yaw_current - 90;
    // Rotate(dir, 0.0, spin_yaw_target);
    Motor_Left_Journey = 0;
    Motor_Right_Journey = 0;
    // //DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    // DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
}