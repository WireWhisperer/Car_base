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
    Motor_Set_Speed_Both(0.0, 0.0);

    if (!Gray_Init())
    {
        uart_pc_send_string("[main] Gray init failed - sensor disabled\r\n");
        while (1) { mspm0_delay_ms(500); }
    }

    while (1)
    {
        uint8_t gray_data = Get_Gray_Data();

        uint8_t gray_datas[8];
        for (int i = 0; i < 8; i++)
            gray_datas[i] = (gray_data >> i) & 0x1;

        sprintf((char*)buffer, "gray=%d %d %d %d %d %d %d %d \r\n",
            gray_datas[0], gray_datas[1],
            gray_datas[2], gray_datas[3],
            gray_datas[4], gray_datas[5],
            gray_datas[6], gray_datas[7]);
        uart_pc_send_string(buffer);
        mspm0_delay_ms(200);
    }
}
