#include "ti_msp_dl_config.h"
#include "main.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    // MPU6050_Init();
    // OLED_Init();
    // WIT_Init();
    
    Motor_Init();
    Motor_SetBoth(0.4, 0.4);

    while (1) 
    {
        
    }
}
