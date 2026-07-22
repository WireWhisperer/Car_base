#include "main.h"

char buffer[100] = {0};

extern Motor_PID_info Motor_Left_PID;
extern Motor_PID_info Motor_Right_PID;

extern float g_gyro_yaw;

extern uint8_t BUTTON_1_IS_PRESSED;
extern uint8_t BUTTON_2_IS_PRESSED;

float data,data1;

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();
    uart_pc_Init();
    Gray_Init();

    // MPU6050_Init();
    //OLED_Init();
    // WIT_Init();
    Motor_Init();

    
    DL_GPIO_enableInterrupt(GPIOA, GPIO_BUTTON_PIN_1_PIN | GPIO_BUTTON_PIN_2_PIN); 
    //Motor_Set_Speed_Both(0.4, 0.4);
    Gyroscope_Init();
    
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    while(!BUTTON_1_IS_PRESSED);

    BUTTON_1_IS_PRESSED = 0;
    //Motor_Set_Duty_Both(0.2, 0.203);
    while (1)
    {
        // data = Get_Gray_Data();
        // data1 = get_miss_theta(Clockwise);
        Rect_DUTY_trace(Clockwise, 0.2, 0.203, 0.05, g_gyro_yaw);
    }
    Motor_Set_Duty_Both(0.0, 0.0);
}
