#include "main.h"
#include "stm32f4xx_hal_uart.h"
#include <stdint.h>
#include "string.h"

#define CALIB_SAMPLES	2000
static void SystemClock_Config(void);
void perifSetup();

void Error_Handler();
void updateImu();
void gyro_calib();
void calGyr();

UART_HandleTypeDef ext1={0};
LSM9DS0_t inemo={0};
char str[256];
float gyr[3]={0};

int main()
{
	HAL_Init();
	SystemClock_Config();
	perifSetup();
	gyro_calib();
	HAL_UART_Transmit(&ext1, (uint8_t *)str,strlen(str) , 100);
	while (1)
	{
		updateImu();
		sprintf(str, "gyr ->  roll: %+-6.2f     pitch: %+-6.2f     yaw: %+-6.2f \n", gyr[0],gyr[1],gyr[2]);
		HAL_UART_Transmit(&ext1, (uint8_t *)str,strlen(str) , 100);
		HAL_Delay(100);
	}

}

void perifSetup()
{
	uart_init(&ext1,UART_MODE_POLLING,UART_PORT_EXT1,9600);
	LMS9DS0_Init_t confVals={G_SCALE_500DPS,A_SCALE_2G,M_SCALE_4GS,
							G_ODR_190_BW_25,A_ODR_200,M_ODR_100};
	LSM9DS0_Init( &inemo,MODE_I2C,SAD_G,SAD_XM);
	LSM9DS0_begin(&inemo,&confVals);

}
void updateImu()
{
	LSM9DS0_readGyro(&inemo);
	calGyr();
	LSM9DS0_readMag(&inemo);
	LSM9DS0_readAccel(&inemo);
	// sprintf(str, "acc ->  x: %-6d     y:%-6d     z:%-6d \n", inemo.ax,inemo.ay,inemo.az);
	// sprintf(str, "gyr ->  roll: %-6d     pitch:%-6d     yaw:%-6d \n", inemo.gx,inemo.gy,inemo.gz);
}



static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;

  /* Enable Power Control clock */
  __HAL_RCC_PWR_CLK_ENABLE();

  /* The voltage scaling allows optimizing the power consumption when the device is
     clocked below the maximum system frequency, to update the voltage scaling value
     regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /* Enable HSE Oscillator and activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
     clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;
  if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /* STM32F405x/407x/415x/417x Revision Z devices: prefetch is supported  */
  if (HAL_GetREVID() == 0x1001)
  {
    /* Enable the Flash prefetch */
    __HAL_FLASH_PREFETCH_BUFFER_ENABLE();
  }
}
void Error_Handler()
{
	while(1);
}


// void UART_RxCpltCallback(Uart_cb *uart)
// {
// 	uint len;
// 	UART_HandleTypeDef *huart;
// 	huart=&(uart->UART_Handler);
// 	if (huart->Instance==USART3)
// 	{
// 		len=Uart_read_line(uart,a,64);
// 		Uart_write(uart,a,len);
// 	}
//
// }

void gyro_calib()
{
	uint16_t calibCounts=0;
	inemo.gbias[0]=0;
	inemo.gbias[1]=0;
	inemo.gbias[2]=0;
	for (calibCounts=0;calibCounts<CALIB_SAMPLES;++calibCounts)
	{
		LSM9DS0_readGyro(&inemo);
		LSM9DS0_readGyro(&inemo);
		inemo.gbias[0]+=inemo.gx;
		inemo.gbias[1]+=inemo.gy;
		inemo.gbias[2]+=inemo.gz;
	}
	// sprintf(str, "gyro_com_offsets ->  roll: %6f     pitch:%6f     yaw:%6f \n", inemo.gbias[0],inemo.gbias[1],inemo.gbias[2]);
	// HAL_UART_Transmit(&ext1, (uint8_t *)str,strlen(str) , 100);

	inemo.gbias[0]/=(float)CALIB_SAMPLES;
	inemo.gbias[1]/=(float)CALIB_SAMPLES;
	inemo.gbias[2]/=(float)CALIB_SAMPLES;
	// sprintf(str, "gyro_raw_offsets ->  roll: %-6f     pitch:%-6f     yaw:%-6f \n", inemo.gbias[0],inemo.gbias[1],inemo.gbias[2]);
	// HAL_UART_Transmit(&ext1, (uint8_t *)str,strlen(str) , 100);

	inemo.gbias[0]*=inemo.gRes;
	inemo.gbias[1]*=inemo.gRes;
	inemo.gbias[2]*=inemo.gRes;
	sprintf(str, "gyro_raw_offsets ->  roll: %-6f     pitch:%-6f     yaw:%-6f \n", inemo.gbias[0],inemo.gbias[1],inemo.gbias[2]);

}
void calGyr()
{

	gyr[0]=(inemo.gx)*(float)inemo.gRes-inemo.gbias[0];
	gyr[1]=(inemo.gy)*(float)inemo.gRes-inemo.gbias[1];
	gyr[2]=(inemo.gz)*(float)inemo.gRes-inemo.gbias[2];
}
