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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
SPI_HandleTypeDef hspi1;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void display_char(uint8_t axis){
	if(axis == 0){
		HAL_GPIO_WritePin(SEG7_1_GPIO_Port, SEG7_1_Pin, RESET); //X
		HAL_GPIO_WritePin(SEG7_B_GPIO_Port, SEG7_B_Pin, SET);
		HAL_GPIO_WritePin(SEG7_C_GPIO_Port, SEG7_C_Pin, SET);
		HAL_GPIO_WritePin(SEG7_F_GPIO_Port, SEG7_F_Pin, SET);
		HAL_GPIO_WritePin(SEG7_G_GPIO_Port, SEG7_G_Pin, SET);
		HAL_GPIO_WritePin(SEG7_E_GPIO_Port, SEG7_E_Pin, SET);
	}
	if (axis == 1){
		HAL_GPIO_WritePin(SEG7_1_GPIO_Port, SEG7_1_Pin, RESET); //Y
		HAL_GPIO_WritePin(SEG7_B_GPIO_Port, SEG7_B_Pin, SET);
		HAL_GPIO_WritePin(SEG7_C_GPIO_Port, SEG7_C_Pin, SET);
		HAL_GPIO_WritePin(SEG7_F_GPIO_Port, SEG7_F_Pin, SET);
		HAL_GPIO_WritePin(SEG7_G_GPIO_Port, SEG7_G_Pin, SET);
	}
	if(axis == 2){
		HAL_GPIO_WritePin(SEG7_1_GPIO_Port, SEG7_1_Pin, RESET); //Z
		HAL_GPIO_WritePin(SEG7_A_GPIO_Port, SEG7_A_Pin, SET);
		HAL_GPIO_WritePin(SEG7_B_GPIO_Port, SEG7_B_Pin, SET);
		HAL_GPIO_WritePin(SEG7_G_GPIO_Port, SEG7_G_Pin, SET);
		HAL_GPIO_WritePin(SEG7_E_GPIO_Port, SEG7_E_Pin, SET);
		HAL_GPIO_WritePin(SEG7_D_GPIO_Port, SEG7_D_Pin, SET);
	}
	HAL_Delay(1000);
}

void set_s7_output(float display_value, uint8_t seg_choice) //seg_choice in 0...3
{
	static GPIO_TypeDef* seg_binding_bitshift2seg_gpio_port[] =
	{
			SEG7_A_GPIO_Port,
			SEG7_B_GPIO_Port,
			SEG7_C_GPIO_Port,
			SEG7_D_GPIO_Port,
			SEG7_E_GPIO_Port,
			SEG7_F_GPIO_Port,
			SEG7_G_GPIO_Port,
			SEG7_P_GPIO_Port,
	};
	static uint16_t seg_binding_bitshift2seg_pin[] =
	{
			SEG7_A_Pin,
			SEG7_B_Pin,
			SEG7_C_Pin,
			SEG7_D_Pin,
			SEG7_E_Pin,
			SEG7_F_Pin,
			SEG7_G_Pin,
			SEG7_P_Pin,
	};
	static GPIO_TypeDef* seg_choice2seg_gpio_port[] =
	{
			SEG7_1_GPIO_Port,
			SEG7_2_GPIO_Port,
			SEG7_3_GPIO_Port,
			SEG7_4_GPIO_Port,
	};
	static uint16_t seg_choice2seg_pin[] =
	{
			SEG7_1_Pin,
			SEG7_2_Pin,
			SEG7_3_Pin,
			SEG7_4_Pin,
	};
	static double seg_choice2int_coeff[] =
		{
				1,
				.1,
				.01,
				.001,
		};
	// 0xXX: PGFEDCBA
	static uint8_t digit2seg_binding[] =
	{
			0b01000000, // 0
			0b01111001, // 1
			0b00100100, // 2
			0b00110000, // 3
			0b00011001, // 4
			0b00010010, // 5
			0b00000010, // 6
			0b01111000, // 7
			0b00000000, // 8
			0b00010000  // 9
	};

	// digit position choice
	for(uint8_t seg_ix = 0; seg_ix < 4; ++seg_ix)
	{
		GPIO_TypeDef* seg7_gpio_port = seg_choice2seg_gpio_port[seg_ix];
		uint16_t seg7_pin = seg_choice2seg_pin[seg_ix];
		HAL_GPIO_WritePin(seg7_gpio_port, seg7_pin, SET);
	}
	GPIO_TypeDef* seg7_gpio_port = seg_choice2seg_gpio_port[3-seg_choice];
	uint16_t seg7_pin = seg_choice2seg_pin[3-seg_choice];
	HAL_GPIO_WritePin(seg7_gpio_port, seg7_pin, RESET);

	// digit show choice
	double minus_space_shift;
	if(display_value < (double)(+0))
	{
		display_value *= -1;
		minus_space_shift = 1;
//		if(seg_choice == 0)
//		{
//			HAL_GPIO_WritePin(SEG7_A_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_B_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_C_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_D_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_E_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_F_GPIO_Port, SEG7_P_Pin, RESET);
//			HAL_GPIO_WritePin(SEG7_G_GPIO_Port, SEG7_P_Pin, SET);
//			HAL_GPIO_WritePin(SEG7_P_GPIO_Port, SEG7_P_Pin, RESET);
//			return;
//		}
	}
	else minus_space_shift = 1;

	uint8_t display_value_rank10 = (int)log10(display_value);
	if(display_value_rank10 < 0) display_value_rank10 = 0;
	if(display_value_rank10 > 3) display_value_rank10 = 3;
	int rank_coeff = pow(10, display_value_rank10);

	uint8_t digit = ((int) (display_value / (seg_choice2int_coeff[seg_choice] * rank_coeff * minus_space_shift))) % 10;
	uint8_t seg_binding = digit2seg_binding[digit];

	for(uint8_t bitshift = 0; bitshift < 7; ++bitshift)
	{
		uint8_t current_seg7_pin_state_flag_extractor = 1 << bitshift;
		GPIO_PinState current_seg7_pin_state;
		if(seg_binding & current_seg7_pin_state_flag_extractor)
			current_seg7_pin_state = RESET;
		else current_seg7_pin_state = SET;

		GPIO_TypeDef* current_seg7_gpio_port = seg_binding_bitshift2seg_gpio_port[bitshift];
		uint16_t current_seg7_pin = seg_binding_bitshift2seg_pin[bitshift];

		HAL_GPIO_WritePin(current_seg7_gpio_port,
				current_seg7_pin,
				current_seg7_pin_state);
	}

	if(seg_choice2int_coeff[seg_choice] * rank_coeff * minus_space_shift == 1)
		HAL_GPIO_WritePin(SEG7_P_GPIO_Port, SEG7_P_Pin, SET);
	else HAL_GPIO_WritePin(SEG7_P_GPIO_Port, SEG7_P_Pin, RESET);



}

void Com_SPI_Write(uint8_t add, uint8_t val){
    uint8_t DataReceived[2];
    uint8_t DataToTransmit[2];

    DataToTransmit[0]=add;
    DataToTransmit[1]=val;

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, DataToTransmit, DataReceived, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);
}

void Com_SPI_Read(uint8_t reg, uint8_t* rec){
    uint8_t DataReceived[2];
    uint8_t DataToTransmit[2];

    DataToTransmit[0]=LIS35_READ|reg;
    DataToTransmit[1]=0;

    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive(&hspi1, DataToTransmit, DataReceived, 2, HAL_MAX_DELAY);
    HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_SET);

    *rec = DataReceived[1];
}

uint8_t TxBuf[50];
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
  MX_DMA_Init();
  MX_SPI1_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  uint8_t seg_choice = 0;
  int delay_mgr_ctr = 0;
  Com_SPI_Write(LIS35_REG_CR1,0x47);
  uint8_t acceleration;
  double acc_converted;
  uint8_t pusch = 0;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(delay_mgr_ctr % 200 == 0)
	  {
		  Com_SPI_Read(LIS35_REG_OUTZ, &acceleration);

		  if(acceleration & 0x80) acceleration = ~acceleration - 1;
		  acc_converted = acceleration *0.02 /1.04 /1.018;

		  uint8_t TxSize;
		  TxSize = snprintf(TxBuf, sizeof(TxBuf), "Acceleration Z -> %f [g] \r\n", acc_converted);
		  HAL_UART_Transmit_DMA(&huart2, TxBuf, TxSize);
	  }

	  if(delay_mgr_ctr % 2 == 0)
	  {
		  set_s7_output(acc_converted  *9.8, seg_choice);
		  seg_choice = (++seg_choice) % 4;
	  }

	  ++delay_mgr_ctr;
	  HAL_Delay(1);
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */
	__HAL_RCC_SPI1_CLK_ENABLE();
  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, SEG7_G_Pin|SEG7_D_Pin|SEG7_E_Pin|SEG7_C_Pin
                          |SEG7_B_Pin|SEG7_F_Pin|SEG7_A_Pin|SEG7_P_Pin
                          |SEG7_1_Pin|SEG7_2_Pin|SEG7_3_Pin|SEG7_4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SEG7_G_Pin SEG7_D_Pin SEG7_E_Pin SEG7_C_Pin
                           SEG7_B_Pin SEG7_F_Pin SEG7_A_Pin SEG7_P_Pin
                           SEG7_1_Pin SEG7_2_Pin SEG7_3_Pin SEG7_4_Pin */
  GPIO_InitStruct.Pin = SEG7_G_Pin|SEG7_D_Pin|SEG7_E_Pin|SEG7_C_Pin
                          |SEG7_B_Pin|SEG7_F_Pin|SEG7_A_Pin|SEG7_P_Pin
                          |SEG7_1_Pin|SEG7_2_Pin|SEG7_3_Pin|SEG7_4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_NSS_Pin */
  GPIO_InitStruct.Pin = SPI1_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_NSS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Button_Pin */
  GPIO_InitStruct.Pin = Button_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Button_GPIO_Port, &GPIO_InitStruct);

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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
