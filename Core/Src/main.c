/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

	#include <string.h>
	#include "stdio.h"
	#include "mpu6050.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

	#define MY_DEBUG
#ifdef MY_DEBUG
	#define 	UART_DEBUG 			&huart4
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

	MPU6050_t MPU6050;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

	void UartDebug(char* _text) ;
	void I2C_ScanBusFlow(I2C_HandleTypeDef * _hi2c, UART_HandleTypeDef * _huart) ;

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_UART4_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

	char		debugString[0XFF]	= { 0 } ;

	#define 	DATE_as_int_str 	(__DATE__)
	#define 	TIME_as_int_str 	(__TIME__)
	sprintf(debugString,"\tBuild: %s. Time: %s.\r\n" ,	DATE_as_int_str , TIME_as_int_str ) ;
	UartDebug(debugString);

	I2C_ScanBusFlow(&hi2c1, &huart4);

	sprintf(debugString,"Connect and init MPU6050... " ) ;
	UartDebug(debugString);

	while (MPU6050_Init(&hi2c1) == 1);

	sprintf(debugString,"successful!\r\n\r\n" ) ;
	UartDebug(debugString);

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  MPU6050_Read_All(&hi2c1, &MPU6050);
	  HAL_Delay(100);
	  sprintf(debugString,"x.Raw -> %x\ty.Raw -> %x\tz.Raw -> %x\t ", MPU6050.Accel_X_RAW , MPU6050.Accel_Y_RAW , MPU6050.Accel_Z_RAW);
		UartDebug(debugString);

	  sprintf(debugString,"x -> %fx\ty -> %fx\tz -> %fx\t \n\r", MPU6050.Ax, MPU6050.Ay, MPU6050.Az);
		UartDebug(debugString);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Activate the Over-Drive mode
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

void UartDebug(char* _text) {
	#ifdef MY_DEBUG
		HAL_UART_Transmit(UART_DEBUG, (uint8_t*)_text, strlen(_text), 100);
	#endif
}	//***********************************************************************

//======================================================================
void I2C_ScanBusFlow(I2C_HandleTypeDef * _hi2c, UART_HandleTypeDef * _huart) {
	char DataChar[64];
	uint8_t device_serial_numb = 0;

	sprintf(DataChar,"\r\n\tStart scan I2C\r\n");
	HAL_UART_Transmit(_huart, (uint8_t *)DataChar, strlen(DataChar), 100);

	for ( uint8_t sbf = 0x07; sbf < 0x78; sbf++) {
		if (HAL_I2C_IsDeviceReady(_hi2c, sbf << 1, 10, 100) == HAL_OK) {
			device_serial_numb++;
			switch (sbf) {
				case 0x20: sprintf(DataChar,"%d) PCF-8574"			, device_serial_numb ) ; 	break ;
				case 0x23: sprintf(DataChar,"%d) BH-1750"			, device_serial_numb ) ; 	break ;
				case 0x27: sprintf(DataChar,"%d) FC113 "			, device_serial_numb ) ; 	break ;
				case 0x38: sprintf(DataChar,"%d) PCF-8574"			, device_serial_numb ) ; 	break ;
				case 0x57: sprintf(DataChar,"%d) AT24c32"			, device_serial_numb ) ; 	break ;
				case 0x50: sprintf(DataChar,"%d) AT24c256"			, device_serial_numb ) ; 	break ;
				case 0x68: sprintf(DataChar,"%d) DS3231 or MPU9250"	, device_serial_numb ) ; 	break ;
				case 0x76: sprintf(DataChar,"%d) BMP280"			, device_serial_numb ) ; 	break ;
				case 0x77: sprintf(DataChar,"%d) BMP180"			, device_serial_numb ) ; 	break ;
				default:   sprintf(DataChar,"%d) Unknown"			, device_serial_numb ) ;	break ;
			}// end switch
			char DataCharRes[96];
			sprintf(DataCharRes,"%s\tAdr: 0x%x\r\n", DataChar, sbf);
			HAL_UART_Transmit(_huart, (uint8_t *)DataCharRes, strlen(DataCharRes), 100);
			HAL_Delay(10);
		} //end if HAL I2C1
	} // end for sbf i2c1

	if (device_serial_numb == 0) {
		sprintf(DataChar,"---no devices found---\r\n");
		HAL_UART_Transmit(_huart, (uint8_t *)DataChar, strlen(DataChar), 100);
	}
	sprintf(DataChar,"\tEnd scan I2C\r\n");
	HAL_UART_Transmit(_huart, (uint8_t *)DataChar, strlen(DataChar), 100);
}// end void I2C_ScanBus

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
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
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
