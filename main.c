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

    uint16_t task = SelectTasks();
    task = task*4;
    
    DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
    while (task > 0)
    {
        Rect_DUTY_trace(Anticlockwise, 0.2, 0.197, 0.03, 0.03, g_gyro_yaw);
        DL_GPIO_togglePins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
        task--;
    }
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    Motor_Set_Duty_Both(0.0, 0.0);

    while (1)
    {
        // data = Get_Gray_Data();
        // data1 = get_miss_theta(Clockwise);
        // mspm0_delay_ms(100);
    }
}

//选择圈数
uint8_t SelectTasks(void)
{
    uint8_t task = 0;
    while(!BUTTON_2_IS_PRESSED)
    {
        if (BUTTON_1_IS_PRESSED)
        {
            BUTTON_1_IS_PRESSED = 0;
            mspm0_delay_ms(150);
            if (BUTTON_1_IS_PRESSED)
            {
                if (task == 3) task = 0;
                else task++;
                switch (task)
                {
                    case 0:
                        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
                        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
                        break;
                    case 1:
                        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
                        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
                        break;
                    case 2:
                        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
                        DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
                        break;
                    case 3:
                        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
                        DL_GPIO_setPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
                        break;
                }
            }
        }
    }
    BUTTON_2_IS_PRESSED = 0;
    mspm0_delay_ms(150);
    while(BUTTON_2_IS_PRESSED);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_RED_PIN);
    DL_GPIO_clearPins(GPIO_LED_PORT, GPIO_LED_PIN_LED_GREEN_PIN);
    return (task+1);
}