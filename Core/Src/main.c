/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */


#define 	BUFFER_SIZE   	16
#define 	MAX_ADDR 		256


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c2;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

uint8_t data_in[BUFFER_SIZE];
uint8_t data_out[BUFFER_SIZE+2];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C2_Init(void);
static void MX_UART4_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


//-----------------------------
void check_eeprom ()
{

#define BUFFER   32

uint8_t dock_in[BUFFER];
uint8_t dock_out[BUFFER];


		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, RESET);
		HAL_Delay(2000);

		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, SET);
		HAL_Delay(500);

		uint8_t i=0;

		for(i=0; i<BUFFER; i++)
		{
			dock_in[i]=i;
			dock_out[i]=0;
		}
		dock_out[BUFFER-1]=13;

		for(i=0; i<BUFFER; i++)
		{
			dock_in[i]=i;
			dock_out[i]=0;
		}


/*
 *
 *
		HAL_I2C_Mem_Write(&hi2c2, 0xA0, 0xFFF, I2C_MEMADD_SIZE_16BIT, data_in, 1, 150); //  0xFFFFFFFFFFFFFFFF
		HAL_I2C_Mem_Read(&hi2c2, 0xA0, 0xFFF, I2C_MEMADD_SIZE_16BIT, data_out, 1, 150);
		HAL_UART_Transmit(&huart4, data_out, BUFFER, 150);
		data_out[15]=13;
		HAL_UART_Transmit(&huart4, data_out[15], 1, 150);
		HAL_Delay(1000);
 *
 *
 *
 *
 * */


		HAL_I2C_Mem_Write(&hi2c2, 0xA0, 0x01, I2C_MEMADD_SIZE_8BIT, dock_in, BUFFER, 100);
		HAL_Delay(500);

		HAL_I2C_Mem_Read(&hi2c2, 0xA0, 0x01, I2C_MEMADD_SIZE_8BIT, dock_out, BUFFER, 100);
		HAL_Delay(500);

		HAL_UART_Transmit_IT(&huart4, dock_out, BUFFER);
		HAL_Delay(500);


	/*
	uint8_t dataBuffer[10] = {0xFF};

	for(uint8_t i=0x00; i<0xFF; i++)
	{
		HAL_I2C_Mem_Write(&hi2c1, (0xA0 << 1), i, I2C_MEMADD_SIZE_8BIT, dataBuffer, 1, 100);
		//j++;
	}
	*/

}

//-------------------------------------
void erase_eeprom ()
{

#define 	BUFFER_SIZE   	16
#define 	MAX_ADDR 		256

uint8_t dock_in[BUFFER_SIZE];
uint8_t dock_out[BUFFER_SIZE];




		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, SET);
		HAL_Delay(2000);

		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, RESET);
		HAL_Delay(500);

		uint16_t i=0x00;

		for(i=0; i<BUFFER_SIZE; i++)
		{
			dock_in[i]=0xFF;
			dock_out[i]=0;
		}


		///-------general cycle------
		for(i=0x00; i<MAX_ADDR; i++)
		{
			HAL_I2C_Mem_Write(&hi2c2, 0xA0, i, I2C_MEMADD_SIZE_8BIT, dock_in, BUFFER_SIZE, 1000);
			HAL_Delay(500);

			HAL_I2C_Mem_Read(&hi2c2, 0xA0, i, I2C_MEMADD_SIZE_8BIT, dock_out, BUFFER_SIZE, 1000);
			HAL_UART_Transmit(&huart4, dock_out, BUFFER_SIZE, 1000);

			dock_out[1]=i;
			dock_out[2]=13;
			dock_out[0]=13;

			HAL_UART_Transmit(&huart4, dock_out, 3, 1000);

		}

		HAL_Delay(1500);

		HAL_GPIO_WritePin(D5_GPIO_Port, D5_Pin, SET);

		HAL_Delay(1500);



}




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
  MX_I2C2_Init();
  MX_UART4_Init();
  /* USER CODE BEGIN 2 */







  	  	  erase_eeprom();


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */



		//check_eeprom();

		//erase_eeprom();

// buffer-size data_in data_out



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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 9;
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
}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00702681;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, D5_Pin|PAG_CLKOUT_Pin|PAG_TXRXCLK_Pin|PAG_TXRXDATA_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GAS_RX_Pin|LAMP_Pin|PAG_MUX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PAG_SWD_GPIO_Port, PAG_SWD_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, PAG_SCK_Pin|PAG_MISO_Pin|PAG_MOSI_Pin|PAG_SLE_Pin
                          |PAG_CE_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D5_Pin PAG_CLKOUT_Pin PAG_TXRXCLK_Pin PAG_TXRXDATA_Pin */
  GPIO_InitStruct.Pin = D5_Pin|PAG_CLKOUT_Pin|PAG_TXRXCLK_Pin|PAG_TXRXDATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : GAS_RX_Pin LAMP_Pin PAG_MUX_Pin */
  GPIO_InitStruct.Pin = GAS_RX_Pin|LAMP_Pin|PAG_MUX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PAG_SWD_Pin */
  GPIO_InitStruct.Pin = PAG_SWD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PAG_SWD_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PAG_SCK_Pin PAG_MISO_Pin PAG_MOSI_Pin PAG_SLE_Pin
                           PAG_CE_Pin */
  GPIO_InitStruct.Pin = PAG_SCK_Pin|PAG_MISO_Pin|PAG_MOSI_Pin|PAG_SLE_Pin
                          |PAG_CE_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
