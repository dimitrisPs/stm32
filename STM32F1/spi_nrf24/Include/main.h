
#ifndef __MAIN_H
#define __MAIN_H


#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"
#include "stm32f1xx_hal_uart.h"
#include <string.h>


void SystemClock_Config(void);

void initUart(UART_HandleTypeDef *uartTD);
void initSpi(SPI_HandleTypeDef *spiTD);
void initLed();


#endif /* __MAIN_H */
