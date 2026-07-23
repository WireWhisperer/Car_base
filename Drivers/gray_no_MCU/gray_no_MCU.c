#include "gray_no_MCU.h"

const float Base_Volect = 0.2;
const float Excursion_1_Force = 0.01;
extern uint8_t oled_buffer[32];

bool have_passed_5us = 0;

void Gray_Init(void)
{
	NVIC_EnableIRQ(TIMER_GRAY_INST_INT_IRQN);
}

 /*读取传感器8bit数据,两次要间隔1ms*/
uint8_t Get_Gray_Data(void)
 {
	uint8_t ret = 0;
	uint32_t num[8] = {0};

	//读取数据
	
	/*for (int i = 0; i < 8; i++)
	{
		DL_GPIO_setPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
		DL_Timer_startCounter(TIMER_GRAY_INST);
		while(!have_passed_5us);
		have_passed_5us = false;
		DL_GPIO_clearPins(GPIO_GRAY_PORT, GPIO_GRAY_CLK_PIN);
		num[i] = DL_GPIO_readPins(GPIO_GRAY_PORT, GPIO_GRAY_DAT_PIN);
	}*/

	num[0] = DL_GPIO_readPins(GPIO_GRAY_D0_PORT, GPIO_GRAY_D0_PIN)? 1:0;
	num[1] = DL_GPIO_readPins(GPIO_GRAY_D1_PORT, GPIO_GRAY_D1_PIN)? 1:0;
	num[2] = DL_GPIO_readPins(GPIO_GRAY_D2_PORT, GPIO_GRAY_D2_PIN)? 1:0;
	num[3] = DL_GPIO_readPins(GPIO_GRAY_D3_PORT, GPIO_GRAY_D3_PIN)? 1:0;
	num[4] = DL_GPIO_readPins(GPIO_GRAY_D4_PORT, GPIO_GRAY_D4_PIN)? 1:0;
	num[5] = DL_GPIO_readPins(GPIO_GRAY_D5_PORT, GPIO_GRAY_D5_PIN)? 1:0;
	num[6] = DL_GPIO_readPins(GPIO_GRAY_D6_PORT, GPIO_GRAY_D6_PIN)? 1:0;
	num[7] = DL_GPIO_readPins(GPIO_GRAY_D7_PORT, GPIO_GRAY_D7_PIN)? 1:0;
	
	//高位在前,将num[0..7]拼入ret (num[0]=bit7, num[7]=bit0)
	ret = (num[0] ? 0x80 : 0)
		| (num[1] ? 0x40 : 0)
		| (num[2] ? 0x20 : 0)
		| (num[3] ? 0x10 : 0)
		| (num[4] ? 0x08 : 0)
		| (num[5] ? 0x04 : 0)
		| (num[6] ? 0x02 : 0)
		| (num[7] ? 0x01 : 0);

 	return ret;
 }

//获取循迹时所需旋转角度的函数
double get_miss_theta(Track_Clock dir)
{
    uint8_t D[8] = {0};
    uint8_t gray_status = Get_Gray_Data();
    uint8_t Zero_Left;
    uint8_t Zero_Right;
    float Zero_Mid;
    int i;

    for (int i = 0; i < 8; i++)
        D[i] = (gray_status >> i) & 0x1;

    for (i = 0; D[i]; i++);
    Zero_Left = i + 1;
    for (i = 7; D[i]; i--);
    Zero_Right = i + 1;
    Zero_Mid = (float)(Zero_Left + Zero_Right) / 2;
    double angle = atan((4.5 - Zero_Mid) / 16) * 180 / M_PI;

    if (gray_status == 0xff) angle = dir ? -11 : 11;

    if (angle > 11) angle = 11;
    if (angle < -11) angle = -11;

    return angle;
}

void TIMER_GRAY_INST_IRQHandler(void)
{
	switch(DL_Timer_getPendingInterrupt(TIMER_GRAY_INST))
    {
        case DL_TIMER_IIDX_LOAD:
			have_passed_5us = true;
            break;
        default:
            break;
    }
}