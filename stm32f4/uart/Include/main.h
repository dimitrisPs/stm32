#include <stm32f4xx_hal.h>
#include "buffer_circ_c.h"
#include "drv_uart.h"


typedef struct sensors
{
    int16_t acc[3];
    int16_t gyr[3];
    int16_t mag[3];
    int16_t tmpr;
    int16_t encoderLeft;
    int16_t ecnoderRight;
    int16_t batteryVoltage;
}sensors_t;
