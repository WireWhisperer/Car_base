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
    Motor_Set_Speed_Both(0.4, 0.4);

    while (1) 
    {

    }
}