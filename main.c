#include "ti_msp_dl_config.h"
#include "main.h"

int main(void)
{
    SYSCFG_DL_init();
    SysTick_Init();

    // MPU6050_Init();
    // OLED_Init();
    // WIT_Init();
    
    while (1) 
    {
        
    }
}
