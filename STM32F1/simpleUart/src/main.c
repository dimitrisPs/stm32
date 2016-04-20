#include "main.h"


const uint8_t aTxBuffer[] = " **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling ****  **** UART_TwoBoards_ComPolling **** ";

int main(void)
{
    UART_HandleTypeDef uart1;
    HAL_Init();

    /* Configure the system clock to 72 MHz */
    SystemClock_Config();

    initUart(&uart1);

    if (HAL_UART_Init(&uart1)!=HAL_OK)
    {
        //failed uart initialization;
        while(1);
    }

    while (1)
    {
        HAL_UART_Transmit(&uart1, aTxBuffer,40, 10);
    }
}

void SystemClock_Config(void)
{
    RCC_ClkInitTypeDef clkinitstruct = {0};
    RCC_OscInitTypeDef oscinitstruct = {0};

    /* Configure PLL ------------------------------------------------------*/

    oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSI;
    oscinitstruct.HSEState        = RCC_HSE_OFF;
    oscinitstruct.LSEState        = RCC_LSE_OFF;
    oscinitstruct.HSIState        = RCC_HSI_ON;
    oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
    oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
    oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSI_DIV2;
    oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL16;
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
    clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;
    if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
    {
        /* Initialization Error */
        while(1);
    }
}

void initUart(UART_HandleTypeDef *uartTD)
{
    //this is a fail safe, if the memory of State has garbage data
    //the uart init code wotn call mspUart.
    uartTD->State=0;

    uartTD->Instance = USART1;
    uartTD->Init.BaudRate=921600;
    uartTD->Init.HwFlowCtl=UART_HWCONTROL_NONE;
    uartTD->Init.Mode=UART_MODE_TX_RX;
    uartTD->Init.OverSampling=UART_OVERSAMPLING_16;
    uartTD->Init.Parity=UART_PARITY_NONE;
    uartTD->Init.StopBits=UART_STOPBITS_1;
    uartTD->Init.WordLength=UART_WORDLENGTH_8B;

    if (HAL_UART_Init(uartTD)!=HAL_OK)
    {
        //failed uart initialization;
        while(1);
    }
    return  ;
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
