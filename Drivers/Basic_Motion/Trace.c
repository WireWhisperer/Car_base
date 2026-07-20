#include "Trace.h"

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
        //yaw_missed = yaw_target - yaw;

        if (yaw_missed > 180) yaw_missed -= 360;
        if (yaw_missed < -180) yaw_missed += 360;

        if ((angle_miss > -12) && (angle_miss <= -9)) Motor_Set_Speed_Both(left_speed, right_speed + 4*speed_add);
        if ((angle_miss > -9) && (angle_miss <= -6)) Motor_Set_Speed_Both(left_speed, right_speed + 3*speed_add);
        if ((angle_miss > -6) && (angle_miss <= -3)) Motor_Set_Speed_Both(left_speed, right_speed + 2*speed_add);
        if ((angle_miss > -3) && (angle_miss <= -1)) Motor_Set_Speed_Both(left_speed, right_speed + 1*speed_add);
        if ((angle_miss > -1) && (angle_miss <= 1)) Motor_Set_Speed_Both(left_speed, right_speed);
        if ((angle_miss > 1) && (angle_miss <= 3)) Motor_Set_Speed_Both(left_speed + speed_add, right_speed);
        if ((angle_miss > 3) && (angle_miss <= 6)) Motor_Set_Speed_Both(left_speed + 2*speed_add, right_speed);
        if ((angle_miss > 6) && (angle_miss <= 9)) Motor_Set_Speed_Both(left_speed + 3*speed_add, right_speed);
        if ((angle_miss > 9) && (angle_miss <= 12)) Motor_Set_Speed_Both(left_speed + 4*speed_add, right_speed);

        //sprintf((char*)oled_buffer, "miss: %f", yaw_missed);
        //OLED_ShowString(0, 2, oled_buffer, 8);
        
        /*借助陀螺仪判断是否快出线
        if ((yaw_missed > -10) && (yaw_missed < 10))
        {
            if ((Get_Gray_Data() == 0xff)) break;
            else if (!have_closed)
            {
                left_speed /= 3;
                right_speed /= 3;
                speed_add /= 3;
                have_closed = 1;
                Motor_Set_Speed_Both(0, 0);
                mspm0_delay_ms(100);
            }
        }
        */

        mspm0_delay_ms(10);
    }

    Motor_Set_Speed_Both(0, 0);
    Rotate(0.4, yaw_target);
    //DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
}

void Rotate(float speed, float yaw_target)
{

}