#include "gray_with_MCU.h"
#include "clock.h"
#include <math.h>

#define I2C_TIMEOUT_MS  10
#define PING_MAX_RETRIES 100

/*===========================================================================
 * I2C 底层驱动 — 两次独立传输 (协议 Method 2)
 *===========================================================================*/

static int Gray_I2C_ReadReg(uint8_t cmd, uint8_t *data, uint8_t length)
{
    unsigned long start, cur;
    unsigned i;

    if (!length) return 0;

    /* Step 1: 写命令 */
    mspm0_get_clock_ms(&start);

    DL_I2C_fillControllerTXFIFO(I2C_GRAY_INST, &cmd, 1);
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_GRAY_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_GRAY_INST, GW_GRAY_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (!DL_I2C_getRawInterruptStatus(I2C_GRAY_INST,
               DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) return -1;
    }

    /* Step 2: 读数据 */
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        DL_I2C_INTERRUPT_CONTROLLER_RX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_GRAY_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_GRAY_INST, GW_GRAY_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_RX, length);

    i = 0;
    mspm0_get_clock_ms(&start);
    do {
        if (!DL_I2C_isControllerRXFIFOEmpty(I2C_GRAY_INST))
        {
            if (i < length)
                data[i++] = DL_I2C_receiveControllerData(I2C_GRAY_INST);
            else
                DL_I2C_receiveControllerData(I2C_GRAY_INST);
        }
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) return -1;
    } while (!DL_I2C_getRawInterruptStatus(I2C_GRAY_INST,
                  DL_I2C_INTERRUPT_CONTROLLER_RX_DONE));

    while (!DL_I2C_isControllerRXFIFOEmpty(I2C_GRAY_INST) && i < length)
        data[i++] = DL_I2C_receiveControllerData(I2C_GRAY_INST);

    DL_I2C_flushControllerTXFIFO(I2C_GRAY_INST);

    return (i == length) ? 0 : -1;
}

static int Gray_I2C_Probe(void)
{
    unsigned long start, cur;
    uint8_t dummy = 0x00;

    mspm0_get_clock_ms(&start);

    DL_I2C_fillControllerTXFIFO(I2C_GRAY_INST, &dummy, 1);
    DL_I2C_clearInterruptStatus(I2C_GRAY_INST,
        DL_I2C_INTERRUPT_CONTROLLER_TX_DONE);
    while (!(DL_I2C_getControllerStatus(I2C_GRAY_INST) &
             DL_I2C_CONTROLLER_STATUS_IDLE));
    DL_I2C_startControllerTransfer(I2C_GRAY_INST, GW_GRAY_ADDR,
        DL_I2C_CONTROLLER_DIRECTION_TX, 1);

    while (!DL_I2C_getRawInterruptStatus(I2C_GRAY_INST,
               DL_I2C_INTERRUPT_CONTROLLER_TX_DONE))
    {
        mspm0_get_clock_ms(&cur);
        if (cur >= (start + I2C_TIMEOUT_MS)) return -1;
    }
    return 0;
}

/*===========================================================================
 * 传感器 API
 *===========================================================================*/

bool Gray_Init(void)
{
    /* I2C 控制器由 SYSCFG_DL_I2C_GRAY_init() 配置 (SysConfig) */
    mspm0_delay_ms(100);

    if (Gray_I2C_Probe() != 0)
        return false;

    int retry = 0;
    while (!Gray_Ping())
    {
        if (++retry >= PING_MAX_RETRIES)
            return false;
        mspm0_delay_ms(10);
    }

    return true;
}

bool Gray_Ping(void)
{
    uint8_t status = 0;
    if (Gray_I2C_ReadReg(CMD_PING, &status, 1) != 0)
        return false;
    return (status == 0x66);
}

uint8_t Get_Gray_Data(void)
{
    uint8_t ret = 0;
    Gray_I2C_ReadReg(CMD_DIGITAL_DATA, &ret, 1);
    return ret;
}

/*===========================================================================
 * 循迹算法
 *===========================================================================*/

double get_track(Track_Clock dir)
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
