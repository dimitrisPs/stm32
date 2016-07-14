#include "main.h"


const uint8_t aTxBuffer[] = "this is a sample programm    \n";
uint8_t aRxBuffer[40];

int main(void)
{
    UART_HandleTypeDef uart={0};
    SPI_HandleTypeDef spi1={0};
    HAL_Init();
    SystemClock_Config();
    initLed();
    HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
    initUart(&uart);
    initSpi(&spi1);


    nrf24_init(&spi1, GPIOA, GPIO_PIN_4);

    while(1)
    {
        nrf24_getStatus();
    }

}

void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};

    /* Configure PLL ------------------------------------------------------*/

    oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
    oscinitstruct.HSEState        = RCC_HSE_ON;
    oscinitstruct.LSEState        = RCC_LSE_OFF;
    oscinitstruct.HSIState        = RCC_HSI_OFF;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;

    //set clock to 72 MHz (rated). with 8MHz external crystal and pll multiplier
    // 9 thw system is clocked at 8*9=72 MHz
    oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
    oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL9;

    if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }

    /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
    clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
    clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV4;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
}

void initUart(UART_HandleTypeDef *uartTD)
{
    uartTD->Instance=USART1;
    uartTD->Init.BaudRate=9600;
    uartTD->Init.HwFlowCtl=UART_HWCONTROL_NONE;
    uartTD->Init.Mode=UART_MODE_TX_RX;
    uartTD->Init.OverSampling=UART_OVERSAMPLING_16;
    uartTD->Init.Parity=UART_PARITY_NONE;
    uartTD->Init.StopBits=UART_STOPBITS_1;
    uartTD->Init.WordLength=UART_WORDLENGTH_8B;
    if (HAL_UART_Init(uartTD)!=HAL_OK)
    {
        while (1)
        {
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
            HAL_Delay(5000);
            HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
            HAL_Delay(20000);
        }
    }
}
void initSpi(SPI_HandleTypeDef *spiTD)
{
    GPIO_InitTypeDef nrf24CsPin;

    nrf24CsPin.Pin=GPIO_PIN_4; //same as nss of spi1 (pa1)
    nrf24CsPin.Mode=GPIO_MODE_OUTPUT_PP;
    nrf24CsPin.Pull=GPIO_PULLUP;
    nrf24CsPin.Speed=GPIO_SPEED_FREQ_LOW;

    spiTD->Instance=SPI1;
    spiTD->Init.CLKPhase=SPI_PHASE_1EDGE;
    spiTD->Init.CLKPolarity=SPI_POLARITY_LOW;
    spiTD->Init.CRCCalculation=SPI_CRCCALCULATION_DISABLE;
    spiTD->Init.Mode=SPI_MODE_MASTER;
    spiTD->Init.NSS=SPI_NSS_SOFT;
    spiTD->Init.FirstBit=SPI_FIRSTBIT_MSB;
    spiTD->Init.DataSize=SPI_DATASIZE_8BIT;
    spiTD->Init.Direction=SPI_DIRECTION_2LINES;
    spiTD->Init.BaudRatePrescaler=SPI_BAUDRATEPRESCALER_8;//achiving >=10Mhz? for nrf24
    if (HAL_SPI_Init(spiTD)!=HAL_OK)
    {
        while(1);
    }
    HAL_GPIO_Init(GPIOA,&nrf24CsPin);
}

void initLed()
{
    GPIO_InitTypeDef ledGpio={0};

    __HAL_RCC_GPIOC_CLK_ENABLE();

    ledGpio.Mode=GPIO_MODE_OUTPUT_PP;
    ledGpio.Pin=GPIO_PIN_13;
    ledGpio.Speed=GPIO_SPEED_FREQ_LOW;
    ledGpio.Pull=GPIO_PULLUP;

    HAL_GPIO_Init(GPIOC,&ledGpio);
}
