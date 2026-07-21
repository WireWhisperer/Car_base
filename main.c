#include "main.h"

char buffer[100] = {0};

extern Motor_PID_info Motor_Left_PID;
extern Motor_PID_info Motor_Right_PID;

extern float g_gyro_yaw;
extern uint8_t BUTTON_1_IS_PRESSED;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    uart_pc_Init();
    Gray_Init();

    // MPU6050_Init();
    // OLED_Init();
    // WIT_Init();

    Motor_Init();

    DL_GPIO_enableInterrupt(GPIOA, GPIO_BUTTON_PIN_1_PIN | GPIO_BUTTON_PIN_2_PIN); 
    //Motor_Set_Speed_Both(0.4, 0.4);
    Gyroscope_Init();

    while(!BUTTON_1_IS_PRESSED);

    Rotate(Clockwise, 0.0, g_gyro_yaw+90);

    Motor_Set_Speed_Both(0, 0);
    
    BUTTON_1_IS_PRESSED = 0;

    while (1)
    {
        //double miss = get_miss_theta(Clockwise);

        //sprintf((char*)buffer, "yaw=%lf \r\n", g_gyro_yaw );
        //uart_pc_send_string(buffer);
        //mspm0_delay_ms(200);
    }
}
