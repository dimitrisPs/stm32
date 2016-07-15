#include "drv_uart.h"
#include "stm32f4xx_hal.h"
#include "stm32f4xx_hal_uart.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_rcc.h"

void uart_init(UART_HandleTypeDef *uart_htd,const UART_MODE__T uart_mode_x,const UART_PORT__T uart_port_x, const uint baudRate)
{
    //init stracture and handler
    GPIO_InitTypeDef ext1Pins={0};


    if (uart_port_x == UART_PORT_EXT1)
    {
        //ext	uart3 portC, pins tx:pc10,rx:pc11
        __HAL_RCC_GPIOC_CLK_ENABLE();
        __HAL_RCC_USART3_CLK_ENABLE();
        ext1Pins.Pin=GPIO_PIN_10 | GPIO_PIN_11;
        ext1Pins.Alternate=GPIO_AF7_USART3;
        uart_htd->Instance=USART3;
    }
    else if (uart_port_x == UART_PORT_XBEE)
    {
        //xbee 	uart4 portA, pins tx:pa0,rx:pa1
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_UART4_CLK_ENABLE();
        ext1Pins.Pin=GPIO_PIN_0 | GPIO_PIN_1;
        ext1Pins.Alternate=GPIO_AF8_UART4;
        uart_htd->Instance=UART4;
    }
    else if (uart_port_x == UART_PORT_GPS)
    {
        //gps 	uart2 portA, pins tx:pa2,rx:pa3
        __HAL_RCC_GPIOA_CLK_ENABLE();
        __HAL_RCC_USART2_CLK_ENABLE();
        ext1Pins.Pin=GPIO_PIN_2 | GPIO_PIN_3;
        ext1Pins.Alternate=GPIO_AF7_USART2;
        uart_htd->Instance=USART2;
    }
    else
        return; //wrong user port input

    ext1Pins.Pull=GPIO_NOPULL;
    ext1Pins.Mode= GPIO_MODE_AF_PP;
    ext1Pins.Speed=GPIO_SPEED_FAST;


    uart_htd->Init.BaudRate=baudRate;
    uart_htd->Init.HwFlowCtl=UART_HWCONTROL_NONE;
    uart_htd->Init.Mode=UART_MODE_TX_RX;
    uart_htd->Init.OverSampling=UART_OVERSAMPLING_16;
    uart_htd->Init.Parity=UART_PARITY_NONE;
    uart_htd->Init.StopBits=UART_STOPBITS_2;
    uart_htd->Init.WordLength=UART_WORDLENGTH_8B;

    if (uart_port_x == UART_PORT_EXT1)
        HAL_GPIO_Init(GPIOC,&ext1Pins);
    else//for gps and xbee ports 
        HAL_GPIO_Init(GPIOA,&ext1Pins);

    if (HAL_UART_Init(uart_htd) != HAL_OK)
    {
        while(1);
    }





    if (uart_mode_x==UART_MODE_POLLING)
    {
    }
    else if (uart_mode_x==UART_MODE_INTERRUPT)
    {
        //set interrupts
    }
    else if(uart_mode_x==UART_MODE_DMA)
    {
        //enable clocks dma
        //configure uart
        //set dma channels
        //set interrupts
    }
}
