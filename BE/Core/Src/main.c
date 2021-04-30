/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lib_lcd.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "MY_DHT22.h"
//à faire: supprimer mes deux fonctions et uliser leur transmit et leur receive pour coimmmuniquer directemdednt avec le perotable

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
static rgb_lcd lcdData;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
char buffer[100];

int len;

static const uint8_t SHT31_ADDR = 0x44 << 1; // Use 8-bit address
static const uint8_t REG_TEMP = 0x86;
static const uint8_t REG_HUM = 0x86;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
float TempC, Humidity;
float val, valH, temp, hum;
unsigned char uartData[50];
unsigned char data[4];
int state=0;

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
  MX_TIM2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
 /*Transmission 
  et reception en I2C
  	  	  	  	  	  */
  HAL_I2C_Master_Transmit(&hi2c1,0x44<<1,data,2,1000)
  HAL_Delay(800);
  HAL_I2C_Master_Receive(&hi2c1,SHT31_ADDR | 0x01,data,4,50);
  /*Transmission 
    et reception en UART1
    	  	  	  	  	  */
  HAL_UART_Receive(&huart1,SHT31_ADDR | 0x01,data,50);
  HAL_Delay(800);
  HAL_UART_Transmit(&huart1,SHT31_ADDR,data,50);


  lcd_init(&hi2c1, &lcdData); // initialise le lcd


   /* USER CODE END 2 */

   /* Infinite loop */
   /* USER CODE BEGIN WHILE */
  while (1)
  {UART_HandleTypeDef *huart;
	//sht31
	data[0]= 0x2C;
	data[1]=0x06;
	data[2]=0x00;
	data[3]=0x00;

    HAL_I2C_Master_Transmit(&hi2c1,SHT31_ADDR,data,sizeof(data),50);
    HAL_Delay(800);
	HAL_I2C_Master_Receive(&hi2c1,SHT31_ADDR | 0x01,data,4,50);

    HAL_UART_Transmit(&huart1,SHT31_ADDR,data,50);
    HAL_Delay(800);
	HAL_UART_Receive(&huart1,SHT31_ADDR | 0x01,data,50);

    val = data[0]<<8 | data[1];
	valH = data[0]<<8 | data[3];
	temp = -45 +175 * ((float)val/65535);
	hum = 100 * ((float)valH/65535);


	 sprintf(buffer,"La Temp en C: %u\r\n", ((unsigned int) temp));

		lcd_position(&hi2c1,0,0);
		lcd_print(&hi2c1, buffer);

	    sprintf(buffer,"L'Humidite : %u \r\n", ((unsigned int) hum));

		lcd_position(&hi2c1,1,1);
		lcd_print(&hi2c1, buffer);
		HAL_Delay(5000);
//afficher la température sur BLE Terminal
		void HAL_TIM_PeriodElapsedCallback( TIM_HandleTypeDef *htim)
			{
			if(htim == &htim2)
			{
			sprintf(buffer,"La Temp en C: %u\r\n", ((unsigned int) temp));
			len = strlen(buffer);
			HAL_UART_Transmit(&huart1,buffer,len,100);
			}
			}
// controle vocal

void HAL_UART_RxCpltCallback( UART_HandleTypeDef *huart)
	   {
	    if(huart == &huart1)
	    {
	    state=huart;
	    }
	    HAL_I2C_Master_Receive(&hi2c1,SHT31_ADDR | 0x01,data,4,50);
	   }
if(state==1){
	    	sprintf(buffer,"La Temp en C: %u\r\n", ((unsigned int) temp));

	    	lcd_position(&hi2c1,0,0);
	    	lcd_print(&hi2c1, buffer);

	    	sprintf(buffer,"L'Humidite : %u \r\n", ((unsigned int) hum));
	    	lcd_position(&hi2c1,1,1);
	    	lcd_print(&hi2c1, buffer);
	    	HAL_Delay(5000);
	    	}
	    else{

	    	if(state==2)
	    	{
	    		clearlcd();
	    	}

	    }

     /* USER CODE END WHILE */
     /* USER CODE BEGIN 3 */

     // Tell TMP102 that we want to read from the temperature register
  	/* data[0]=0x2C;
	 data[1]=0x06;
	 data[2]=0x00;
	 data[3]=0x00;*/

    /*HAL_I2C_Master_Transmit(&hi2c1,SHT31_ADDR,data,sizeof(data),50);
	HAL_Delay(800);
	HAL_I2C_Master_Receive(&hi2c1,SHT31_ADDR | 0x01,data,4,50);
    val = data[0]<<8 | data[1];
	valH = data[0]<<8 | data[3];
	temp = -45 +175 * ((float)val/65535);
	hum = 100 * ((float)valH/65535);
    sprintf(buffer,"La Temp en C: %u\r\n", ((unsigned int) temp));

	lcd_position(&hi2c1,0,0);
	lcd_print(&hi2c1, buffer);

    sprintf(buffer,"L'Humidite : %u \r\n", ((unsigned int) hum));

	lcd_position(&hi2c1,1,1);
	lcd_print(&hi2c1, buffer);
	HAL_Delay(5000);*/

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_I2C1;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_PCLK1;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
