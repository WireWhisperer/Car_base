
#include "interrupt.h"


volatile uint8_t BUTTON_1_IS_PRESSED = 0;
volatile uint8_t BUTTON_2_IS_PRESSED = 0;

int16_t Motor_Left_roll = 0;
int16_t Motor_Right_roll = 0;

int16_t Motor_Left_Journey = 0;
int16_t Motor_Right_Journey = 0;

extern char buffer[100];

void SysTick_Handler(void)
{
    tick_ms++;
}

#if defined UART_BNO08X_INST_IRQHandler
void UART_BNO08X_INST_IRQHandler(void)
{
    uint8_t checkSum = 0;
    extern uint8_t bno08x_dmaBuffer[19];

    DL_DMA_disableChannel(DMA, DMA_BNO08X_CHAN_ID);
    uint8_t rxSize = 18 - DL_DMA_getTransferSize(DMA, DMA_BNO08X_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_BNO08X_INST) == false)
        bno08x_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_BNO08X_INST);

    for(int i=2; i<=14; i++)
        checkSum += bno08x_dmaBuffer[i];

    if((rxSize == 19) && (bno08x_dmaBuffer[0] == 0xAA) && (bno08x_dmaBuffer[1] == 0xAA) && (checkSum == bno08x_dmaBuffer[18]))
    {
        bno08x_data.index = bno08x_dmaBuffer[2];
        bno08x_data.yaw = (int16_t)((bno08x_dmaBuffer[4]<<8)|bno08x_dmaBuffer[3]) / 100.0;
        bno08x_data.pitch = (int16_t)((bno08x_dmaBuffer[6]<<8)|bno08x_dmaBuffer[5]) / 100.0;
        bno08x_data.roll = (int16_t)((bno08x_dmaBuffer[8]<<8)|bno08x_dmaBuffer[7]) / 100.0;
        bno08x_data.ax = (bno08x_dmaBuffer[10]<<8)|bno08x_dmaBuffer[9];
        bno08x_data.ay = (bno08x_dmaBuffer[12]<<8)|bno08x_dmaBuffer[11];
        bno08x_data.az = (bno08x_dmaBuffer[14]<<8)|bno08x_dmaBuffer[13];
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_BNO08X_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_BNO08X_CHAN_ID, (uint32_t) &bno08x_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_BNO08X_CHAN_ID, 18);
    DL_DMA_enableChannel(DMA, DMA_BNO08X_CHAN_ID);
}
#endif

#if defined UART_WIT_INST_IRQHandler
void UART_WIT_INST_IRQHandler(void)
{
    uint8_t checkSum, packCnt = 0;
    extern uint8_t wit_dmaBuffer[33];

    DL_DMA_disableChannel(DMA, DMA_WIT_CHAN_ID);
    uint8_t rxSize = 32 - DL_DMA_getTransferSize(DMA, DMA_WIT_CHAN_ID);

    if(DL_UART_isRXFIFOEmpty(UART_WIT_INST) == false)
        wit_dmaBuffer[rxSize++] = DL_UART_receiveData(UART_WIT_INST);

    while(rxSize >= 11)
    {
        checkSum=0;
        for(int i=packCnt*11; i<(packCnt+1)*11-1; i++)
            checkSum += wit_dmaBuffer[i];

        if((wit_dmaBuffer[packCnt*11] == 0x55) && (checkSum == wit_dmaBuffer[packCnt*11+10]))
        {
            if(wit_dmaBuffer[packCnt*11+1] == 0x51)
            {
                wit_data.ax = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 2.048; //mg
                wit_data.ay = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 2.048; //mg
                wit_data.az = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 2.048; //mg
                wit_data.temperature =  (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]) / 100.0; //°C
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x52)
            {
                wit_data.gx = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 16.384; //°/S
                wit_data.gy = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 16.384; //°/S
                wit_data.gz = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 16.384; //°/S
            }
            else if(wit_dmaBuffer[packCnt*11+1] == 0x53)
            {
                wit_data.roll  = (int16_t)((wit_dmaBuffer[packCnt*11+3]<<8)|wit_dmaBuffer[packCnt*11+2]) / 32768.0 * 180.0; //°
                wit_data.pitch = (int16_t)((wit_dmaBuffer[packCnt*11+5]<<8)|wit_dmaBuffer[packCnt*11+4]) / 32768.0 * 180.0; //°
                wit_data.yaw   = (int16_t)((wit_dmaBuffer[packCnt*11+7]<<8)|wit_dmaBuffer[packCnt*11+6]) / 32768.0 * 180.0; //°
                wit_data.version = (int16_t)((wit_dmaBuffer[packCnt*11+9]<<8)|wit_dmaBuffer[packCnt*11+8]);
            }
        }

        rxSize -= 11;
        packCnt++;
    }
    
    uint8_t dummy[4];
    DL_UART_drainRXFIFO(UART_WIT_INST, dummy, 4);

    DL_DMA_setDestAddr(DMA, DMA_WIT_CHAN_ID, (uint32_t) &wit_dmaBuffer[0]);
    DL_DMA_setTransferSize(DMA, DMA_WIT_CHAN_ID, 32);
    DL_DMA_enableChannel(DMA, DMA_WIT_CHAN_ID);
}
#endif

void GROUP1_IRQHandler(void)
{
    uint32_t gpioA = DL_GPIO_getEnabledInterruptStatus(GPIOA,
        GPIO_ENCODER_LEFT_A_PIN | GPIO_ENCODER_RIGHT_A_PIN | 
        GPIO_BUTTON_PIN_1_PIN | GPIO_BUTTON_PIN_2_PIN);

    if (gpioA & GPIO_BUTTON_PIN_1_PIN)
    {
        BUTTON_1_IS_PRESSED = 1;
        DL_GPIO_clearInterruptStatus(GPIO_BUTTON_PORT,
            GPIO_BUTTON_PIN_1_PIN);
        // uart_pc_send_string("BUTTON_1 was pressed\r\n");
    }
    else if (gpioA & GPIO_BUTTON_PIN_2_PIN)
    {
        BUTTON_2_IS_PRESSED = 1;
        DL_GPIO_clearInterruptStatus(GPIO_BUTTON_PORT,
            GPIO_BUTTON_PIN_2_PIN);
        // uart_pc_send_string("BUTTON_2 was pressed\r\n");
    }
    else if (gpioA & GPIO_ENCODER_LEFT_A_PIN)
    {
        Motor_Left_roll += (DL_GPIO_readPins(
            GPIO_ENCODER_LEFT_B_PORT,
            GPIO_ENCODER_LEFT_B_PIN) ? -1 : 1);
        DL_GPIO_clearInterruptStatus(GPIO_BUTTON_PORT,
            GPIO_ENCODER_LEFT_A_PIN);
        //sprintf(buffer, "Left:%d", Motor_Left_roll);
        //uart_pc_send_string(buffer);
    }
    else if (gpioA & GPIO_ENCODER_RIGHT_A_PIN)
    {
        Motor_Right_roll += (DL_GPIO_readPins(
            GPIO_ENCODER_RIGHT_B_PORT,
            GPIO_ENCODER_RIGHT_B_PIN) ? 1 : -1);
        DL_GPIO_clearInterruptStatus(GPIO_BUTTON_PORT,
            GPIO_ENCODER_RIGHT_A_PIN);
        // sprintf(buffer, "Right:%d", Motor_Right_roll);
        // uart_pc_send_string(buffer);
    }
    switch (DL_Interrupt_getPendingGroup(DL_INTERRUPT_GROUP_1)) {
        /* MPU6050 INT */
        #if defined GPIO_MPU6050_PORT
            #if defined GPIO_MPU6050_INT_IIDX
            case GPIO_MPU6050_INT_IIDX:
            #elif (GPIO_MPU6050_PORT == GPIOA) && (defined GPIO_MULTIPLE_GPIOA_INT_IIDX)
            case GPIO_MULTIPLE_GPIOA_INT_IIDX:
            #elif (GPIO_MPU6050_PORT == GPIOB) && (defined GPIO_MULTIPLE_GPIOB_INT_IIDX)
            case GPIO_MULTIPLE_GPIOB_INT_IIDX:
            #endif
                Read_Quad();
                break;
        #endif
    }
}

void Motor_PID_INST_IRQHandler(void)
{
    switch(DL_Timer_getPendingInterrupt(Motor_PID_INST))
    {
        case DL_TIMER_IIDX_LOAD:
            calculate_Speed();
            Motor_PID_Update_Both();
            break;
        default:
            break;
    }
}