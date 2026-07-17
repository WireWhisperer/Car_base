#include "main.h"

char buffer[100] = {0};
extern float Moter_Left_Speed;
extern float Moter_Right_Speed;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    uart_pc_Init();

    // MPU6050_Init();
    // OLED_Init();
    // WIT_Init();
    
    Motor_Init();
    Motor_SetBoth(0.3, 0.3);

    while (1) 
    {
        sprintf(buffer, "LeftSpeed:%f\n", Moter_Left_Speed);
        uart_pc_send_string(buffer);
        sprintf(buffer, "RightSpeed:%f\n", Moter_Right_Speed);
        uart_pc_send_string(buffer);
        mspm0_delay_ms(1000);
    }
}