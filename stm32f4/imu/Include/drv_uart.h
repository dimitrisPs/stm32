#ifndef DRV_UART_H_
#define DRV_UART_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"

typedef enum u_m
{
    UART_MODE_POLLING    ,
    UART_MODE_INTERRUPT  ,
    UART_MODE_DMA
}UART_MODE__T;
typedef enum u_p
{
    UART_PORT_EXT1      ,
    UART_PORT_XBEE      ,
    UART_PORT_GPS
}UART_PORT__T;



void uart_init(UART_HandleTypeDef *uart_htd,const UART_MODE__T uart_mode_x,const UART_PORT__T uart_port_x, const uint baudRate); 
//uart_init_intr
//uart_init_dma




//uart_deinit

//uart_set_baud

//uart_writeByte

//uart_writeLine

//uart_write

//uart_readByte

//uart_readLine

//uart_read

#endif //DRV_UART_H_
