#include "main.h"

char buffer[100] = {0};

extern Motor_PID_info Motor_Left_PID;
extern Motor_PID_info Motor_Right_PID;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    uart_pc_Init();

    // MPU6050_Init();
    // OLED_Init();
    // WIT_Init();
    
    Motor_Init();
    Motor_Set_Speed_Both(500, 500);

    while (1) 
    {
        sprintf(buffer, "LeftSpeed:%f \n", Motor_Left_PID.Current_Speed);
        uart_pc_send_string(buffer);
        sprintf(buffer, "Lefterror:%f \n", Motor_Left_PID.Last_error);
        uart_pc_send_string(buffer);
        sprintf(buffer, "RightSpeed:%f \n", Motor_Right_PID.Current_Speed);
        uart_pc_send_string(buffer);
        sprintf(buffer, "Righterror:%f \n", Motor_Right_PID.Last_error);
        uart_pc_send_string(buffer);
        mspm0_delay_ms(200);
    }
}