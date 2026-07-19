#include "gray_no_MCU.h"

const float Base_Volect = 0.2;
const float Excursion_1_Force = 0.01;
extern uint8_t oled_buffer[32];

 /*读取传感器8bit数据*/
uint8_t Get_Gray_Data(void)
 {
	uint8_t ret = 0;
	uint32_t num[8] = {0};

	//读取数据
	DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[0] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[1] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[2] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[3] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[4] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_clearPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[5] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_clearPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[6] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

	DL_GPIO_setPins(GPIO_GRAY_AD0_PORT, GPIO_GRAY_AD0_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD1_PORT, GPIO_GRAY_AD1_PIN);
	DL_GPIO_setPins(GPIO_GRAY_AD2_PORT, GPIO_GRAY_AD2_PIN);
	num[7] = (DL_GPIO_readPins(GPIO_GRAY_OUT_PORT, GPIO_GRAY_OUT_PIN));

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
double get_track(Track_Clock dir)                        //1234 5678
{
	uint8_t D[8] = {0};
	uint8_t gray_status = Get_Gray_Data();
	uint8_t Zero_Left;
	uint8_t Zero_Right;
	float Zero_Mid;
	int i;

	for (int i = 0; i < 8; i++)
	{
		D[i] = (gray_status >> i) & 0x1;
	}

	for (i = 0; D[i]; i++);
	Zero_Left = i + 1;
	for (i = 7; D[i]; i--);
	Zero_Right = i + 1;
	Zero_Mid = (float)(Zero_Left + Zero_Right) / 2;
	double angle = atan((4.5-Zero_Mid)/16) * 180 / M_PI;

	// if (gray_status == 0xff)
	// {
	// 	angle = dir ? 11 : -11;
	// }

	if (gray_status == 0xff) angle = dir ? -11: 11 ;

	if (angle > 11) angle = 11;
	if (angle < -11) angle = -11;

	return angle;
}