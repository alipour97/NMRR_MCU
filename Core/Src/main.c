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
#include "ad717x.h"
#include "global.h"
#include "ad411x_regs.h"
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

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
uint8_t uart_buffer[UART_BUFFER_SIZE];
uint8_t tx_buffer[7+7+2*sizeof(uint32_t) * ADC_BUFFER_SIZE] = {0};

uint32_t time_buff[ADC_BUFFER_SIZE];
uint32_t adc_buff[ADC_BUFFER_SIZE];
uint32_t tq_buff[ADC_BUFFER_SIZE];

uint16_t adc_buff_idx = 0;

extern ad717x_st_reg ad4111_regs[];

ad717x_dev *pad717x_dev = NULL;
static ad717x_st_reg *ad717x_device_map = ad4111_regs;
static uint8_t ad717x_reg_count = sizeof(ad4111_regs) / sizeof(ad4111_regs[0]);

volatile enum SPI_STATUS spi_status = IDLE;
volatile enum ADC_SM adc_sm = ADC_IDLE;



struct spi_read spi_read_reg = {0,NULL};
struct spi_write spi_write_reg = {{0},0,NULL};
//uint8_t spi_read_reg = 0x00;
// Pointer to the struct representing the AD717x device


// Device setup
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI1_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */

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
  MX_DMA_Init();
  MX_USART2_UART_Init();
  MX_SPI1_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */
  ad717x_app_initialize();

  ad717x_configure_device_odr(pad717x_dev, 0, sps_100);
  HAL_Delay(10);
  ad717x_set_channel_status(pad717x_dev, 0, 1);
  HAL_Delay(10);
  ad717x_set_adc_mode(pad717x_dev, CONTINUOUS);

  union ad717x_analog_inputs AIN_0;
  AIN_0.analog_input_pairs = VIN0_VIN1;
  HAL_Delay(10);
  ad717x_connect_analog_input(pad717x_dev, 0, AIN_0);
  HAL_Delay(10);
  ad717x_assign_setup(pad717x_dev, 0, 0);
  HAL_Delay(10);
  ad717x_set_polarity(pad717x_dev, 1, 0);
  HAL_Delay(10);
  ad717x_set_reference_source(pad717x_dev, EXTERNAL_REF, 0);
  HAL_Delay(10);

  HAL_Delay(10);
  ad717x_enable_input_buffer(pad717x_dev, 1, 0, 0);
  HAL_Delay(10);
  // Initial Message for PC:
  char *init_msg = "{inf,\r\nWelcome to Pourostad Project,end}\r\n";
  HAL_UART_Transmit(PC_UART, (uint8_t*)init_msg, strlen(init_msg), 10);
  HAL_UART_Receive_DMA(PC_UART, uart_buffer, UART_BUFFER_SIZE);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  htim2.Instance->CNT = 0;
  HAL_TIM_Base_Start(&htim2);
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  if(uart_buffer[0]!='\0')
	  {
		  check_command();
	  }
	  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
	  {
		  HAL_Delay(50);
		  if(HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_RESET)
		  {
			  for(int i=0; i<ADC_BUFFER_SIZE; i++)
				  time_buff[i] = 16*i+3;
			  HAL_UART_Transmit(&huart2, (uint8_t*)time_buff, ADC_BUFFER_SIZE*4, 100);
		  }
	  }
//	  if(HAL_GPIO_ReadPin(DRY_GPIO_Port, DRY_Pin) == GPIO_PIN_RESET)
//	  {
//		  int32_t pData;
//		  AD717X_ReadData(pad717x_dev, &pData);
//		  float num = (float)pData / (1 << 31);
//		  char hexString[20];  // Buffer to store "0x" + 4 hex digits + null terminator
//			sprintf(hexString, "%.4f\r\n", num);  // Format as hex string with "0x" prefix
//			send_string(hexString);
//	  }
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
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 6;
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

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
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
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 90-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  huart2.Init.BaudRate = 1152000;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 1);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 2);
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : B1_Pin DRY_Pin */
  GPIO_InitStruct.Pin = B1_Pin|DRY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 2);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
int32_t ad717x_app_initialize(void)
{
	// Used to create the ad717x device
	ad717x_init_param ad717x_init = {
		0,
		ad717x_device_map,		// pointer to device register map
		ad717x_reg_count,		// number of device registers
		ID_AD4115,				// Active Device
		1,						// Reference Enable
		16,						// Channel Number
		8,						// Setup Number

	};

	// Initialze the device
	return (AD717X_Init(&pad717x_dev, ad717x_init));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == DRY_Pin) {
	  if(spi_status == READING)
	  {
			ad717x_st_reg *pReg = (ad717x_st_reg*)spi_read_reg.pReg;
			uint8_t Rx[8] = {0};
			HAL_SPI_Transmit(SPI, &spi_read_reg.Tx, 1, 100);

			HAL_SPI_Receive(SPI, Rx, pReg->size, 100);


			pReg->value = 0;
			for(int i = 0; i < pReg->size; i++) {
				pReg->value <<= 8;
				pReg->value += Rx[i];
			}
		  	spi_status = IDLE;
		  	if(pReg->addr == 0x04) //if it is read reg
		  	{

		  		if(adc_sm == ADC_IDLE)
		  		{
		  			float data = (((float) pReg->value / (1<<23))-1) * 25;
					char hexString[24];
					sprintf(hexString, "{inf,\r\n%.4f,end}\r\n", data);
					send_string(hexString);
		  		}
		  		else
		  		{
		  			adc_to_buf(pReg->value);
		  			adc_sm = ADC_IDLE;
		  			spi_status = SENDING;
		  		}

		  	}
		  	else
		  	{
				char hexString[24];  // Buffer to store "0x" + 4 hex digits + null terminator
				sprintf(hexString, "{inf,\r\n0x%04x,end}\r\n", (unsigned int)pReg->value);  // Format as hex string with "0x" prefix
				send_string(hexString);
		  	}
	  }
	  else if(spi_status == WRITING)
	  {
		  ad717x_st_reg *pReg = (ad717x_st_reg*)spi_write_reg.pReg;
		  HAL_SPI_Transmit(SPI, spi_write_reg.Tx, pReg->size + 1, 100);

		  send_string("{inf,\r\nwrite done,end}\r\n");
		  spi_status = IDLE;
		  AD717X_ReadRegister(pad717x_dev, pReg->addr);
	  }
	  else if(spi_status == TRIGGER)
	  {
		spi_status = READING;
	  }
	  else if (spi_status == GETID)
	  {
		uint8_t Tx = 0x47;
		uint8_t Rx[2] = {0};
		HAL_SPI_Transmit(SPI, &Tx, 1, 10);
		HAL_SPI_Receive(SPI, Rx, 2, 100);
		uint16_t receivedData = (Rx[0] << 8) | Rx[1];
		char hexString[22];  // Buffer to store "0x" + 4 hex digits + null terminator
		sprintf(hexString, "{inf,\r\n0x%04x,end}\r\n", receivedData);  // Format as hex string with "0x" prefix
		send_string(hexString);
		spi_status = IDLE;
	  }
	  else if(spi_status == SENDING)
	  {
		  adc_sm = ADC_READING;
		  AD717X_ReadRegister(pad717x_dev, 4);
//		  spi_status = SPI_NOP;
//		  uint8_t Tx = 0x47;
//		  uint8_t Rx[4] = {0};
////		  HAL_SPI_Transmit(SPI, &Tx, 1, 10);
////		  HAL_SPI_Receive(SPI, Rx, 4, 100);
//		  uint32_t receivedData = 0;
//		  for(int i = 0; i < 4; i++) {
//			  receivedData <<= 8;
//			  receivedData += Rx[i];
//		  }
//		  adc_to_buf(receivedData);
//		  spi_status = SENDING;
//		  char hexString[9];
//		  sprintf(hexString, "%lu\r\n", HAL_GetTick());
//		  send_string(hexString);
	  }
  } else {
      __NOP();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}
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
