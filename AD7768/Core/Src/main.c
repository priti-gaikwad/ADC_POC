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
#include "stm32h7xx_ll_spi.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CS1_LOW 	(1 << 19)        	// port pin no.19 RESET
#define CS1_HIGH	(1 << 3)			// port pin no.3
#define REG_READ	0x40
#define REG_WRITE	0x00
#define SPI2_TXF	(SPI2->SR & (1<<1))
#define SPI2_RXF	(SPI2->SR & (1<<0))
#define ADC_DRDY	(GPIOE->IDR & (1<<2))

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart4;

/* USER CODE BEGIN PV */

SPI_HandleTypeDef *hspip1 = &hspi1;
SPI_HandleTypeDef *hspip2 = &hspi2;
SPI_HandleTypeDef *hspip3 = &hspi3;
uint8_t tx_data[4] = {0};
uint8_t rx_data[9] = {0};
uint32_t s_data[100]  = {0};
uint32_t tx = 0;
uint32_t rx = 0;

uint16_t read_intf = 0;
uint16_t pow_n_con = 0;
uint16_t cont_conv = 0;
uint16_t decimate = 0;
uint16_t scratch = 0;
uint16_t SPI_DIAG = 0;
uint32_t data_24 = 0;
uint16_t ADC_DIAG = 0;
uint16_t cntr = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM3_Init(void);
static void MX_UART4_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

void SPI_rxtx(uint8_t* Txdata,uint8_t* Rxdata,uint16_t Size);
void SPI_TxRx1(const uint8_t *pTxData,uint8_t* Rxdata,uint8_t*  Size);
void SPI2_INIT(int len);
int8_t SPIM2_XFER(uint8_t *txb, uint8_t *rxb, int len);
void spi_tx_rx(SPI_HandleTypeDef *hspi,uint8_t *p8_t_data,uint8_t *p8_r_data,uint32_t u32_size);
void SPI_ll_init(void);
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

/* Configure the peripherals common clocks */
  PeriphCommonClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_SPI1_Init();
  MX_SPI2_Init();
  MX_TIM3_Init();
  MX_UART4_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */

//  SPI2_INIT(3);
   SPI_ll_init();
	/*** Timer 3 ch 2 --> 17 Mhz ***/
	//  TIM3->CCR2 = 8;
	//  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);

	/** RESET **/
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
//	  init_ad7768();

//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
//  		HAL_Delay(10);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

/* RESET trigger register = 0x1D */
	tx_data[0] = 0x1D;      //for write  operation
	tx_data[1] = 0x80;		// default value
	GPIOE->BSRR |= CS1_LOW;
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
	SPI_rxtx(tx_data, rx_data, 2);
	GPIOE->BSRR |= CS1_HIGH;

/* write interface format = 0x14*/
	tx_data[0] = 0x14;      //for write  operation
	tx_data[1] = 0x01;		// enable continuous read mode.
	GPIOE->BSRR |= CS1_LOW;
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
	SPI_rxtx(tx_data, rx_data, 2);
	GPIOE->BSRR |= CS1_HIGH;

/* Power and clock register = 0x15 */
	tx_data[0] = 0x15;      //for write  operation
	tx_data[1] = 0x22;		// mclk/4,PWRMODE
	GPIOE->BSRR |= CS1_LOW;
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
	SPI_rxtx(tx_data, rx_data, 2);
	GPIOE->BSRR |= CS1_HIGH;

/* Digital filter = 0x19*/
	tx_data[0] = 0x19;      //for write  operation
	tx_data[1] = 0x60;		// low FIR filter,X32
	GPIOE->BSRR |= CS1_LOW;
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 2, HAL_MAX_DELAY);
	SPI_rxtx(tx_data, rx_data, 2);
	GPIOE->BSRR |= CS1_HIGH;


/******* SYNC PULSE *******/
	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 0);

//	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
//	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);
	HAL_Delay(10);
	HAL_UART_Transmit(&huart4, "\ninput = 1902.3\n\r", 20, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, HAL_MAX_DELAY);
//	HAL_SPI_TransmitReceive(&hspi1, tx_data, rx_data, 3, HAL_MAX_DELAY);

while(1){



//	for(int i = 0; i < 100;i++)
//	{
		tx_data[0] = 0x00;      //for write  operation
		tx_data[1] = 0x00;
		tx_data[1] = 0x00;

		while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 1){};
		{
			HAL_SPI_Abort(&hspi1);
			SPI_rxtx(tx_data, rx_data, 3);
//			data_24 = ((rx_data[0] << 16) |(rx_data[3] << 8) | rx_data[6]);
			data_24 = ((rx_data[0] << 16) |(rx_data[1] << 8) | rx_data[2]);
//			sprintf(s_data,"%d, ",data_24);
//			HAL_UART_Transmit(&huart4, s_data, 15, HAL_MAX_DELAY);

		}
//		sprintf(s_data,"%d, ",data_24);
//					HAL_UART_Transmit(&huart4, s_data, 15, HAL_MAX_DELAY);
//		HAL_Delay(50);
//	}
//for(;;);

//	uint8_t data = 0x00;
//	tx_data[0] = 0x00;      //for write  operation
//	tx_data[1] = 0x00;
//	tx_data[1] = 0x00;
//
//	while(HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2) == 1)
//	{
//		HAL_SPI_Abort(&hspi1);
//		SPI_rxtx(tx_data, rx_data, 3);
//
//		data_24 = ((rx_data[0] << 16) |(rx_data[1] << 8) | rx_data[2]);
//		sprintf(s_data,"%d, ",data_24);
//		HAL_UART_Transmit(&huart4, s_data, 15, HAL_MAX_DELAY);
//		data++;
//		if(data > 100){
//			for(;;);
//			break;
//		}

//	}
HAL_Delay(50);
////GPIOE->BSRR |= CS1_HIGH;
//}
////}
//
}

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

  /*AXI clock gating */
  RCC->CKGAENR = 0xFFFFFFFF;

  /** Supply configuration update enable
  */
  HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_DIV1;
  RCC_OscInitStruct.HSICalibrationValue = 64;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 20;
  RCC_OscInitStruct.PLL.PLLN = 175;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_1;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_6) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_SPI3|RCC_PERIPHCLK_SPI2
                              |RCC_PERIPHCLK_SPI1;
  PeriphClkInitStruct.PLL2.PLL2M = 32;
  PeriphClkInitStruct.PLL2.PLL2N = 129;
  PeriphClkInitStruct.PLL2.PLL2P = 2;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 2;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_1;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL2;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi2.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi3.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 0x0;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi3.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi3.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi3.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi3.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi3.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi3.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi3.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi3.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 15;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK)
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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CS_GPIO_Port, CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RESET_GPIO_Port, RESET_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_3, GPIO_PIN_SET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CS_Pin */
  GPIO_InitStruct.Pin = CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : RESET_Pin */
  GPIO_InitStruct.Pin = RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PD3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void init_ad7768(void)
{
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
	HAL_Delay(100);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
	HAL_Delay(10);



}


void SPI_rxtx(uint8_t* Txdata,uint8_t* Rxdata,uint16_t Size)
{
	GPIOE->BSRR |= CS1_LOW;

//		LL_SPI_Enable(SPI1);
//		LL_SPI_Enable(SPI2);
//		LL_SPI_Enable(SPI3);
		LL_SPI_StartMasterTransfer(SPI1);
		LL_SPI_StartMasterTransfer(SPI2);
		LL_SPI_StartMasterTransfer(SPI3);
		for(int i=0;i<Size;i++)
		{
			while(!(LL_SPI_IsActiveFlag_TXP(SPI1) | LL_SPI_IsActiveFlag_TXP(SPI2) | LL_SPI_IsActiveFlag_TXP(SPI3)));
//			*((__IO uint8_t*)&SPI1->TXDR) = (uint8_t*)Txdata++;
//			*((__IO uint8_t*)&SPI1->TXDR) = *((__IO uint8_t*)&SPI2->TXDR) = *((__IO uint8_t*)&SPI3->TXDR) = (uint8_t*)0x00;
			*((__IO uint8_t*)&SPI1->TXDR) = (uint8_t*)0x00;
			*((__IO uint8_t*)&SPI2->TXDR) = (uint8_t*)0x11;
			*((__IO uint8_t*)&SPI3->TXDR) = (uint8_t*)0x22;
			while(!(LL_SPI_IsActiveFlag_RXP(SPI1) | LL_SPI_IsActiveFlag_RXP(SPI2) | LL_SPI_IsActiveFlag_RXP(SPI3)));
			*Rxdata++ = LL_SPI_ReceiveData8(SPI1);
			*Rxdata++ = LL_SPI_ReceiveData8(SPI2);
			*Rxdata++ = LL_SPI_ReceiveData8(SPI3);

		}

		LL_SPI_ClearFlag_EOT(SPI1);
		LL_SPI_ClearFlag_EOT(SPI2);
		LL_SPI_ClearFlag_EOT(SPI3);
	GPIOE->BSRR |= CS1_HIGH;
//	HAL_Delay(1);
}


void SPI_ll_init(void)
{
	LL_SPI_Enable(SPI1);
	LL_SPI_Enable(SPI2);
	LL_SPI_Enable(SPI3);
}



void SPI_TxRx1(const uint8_t *pTxData,uint8_t* Rxdata,uint8_t*  Size)
{
	GPIOE->BSRR |= CS1_LOW;
//	HAL_Delay(1);


	hspip1->State 		= HAL_SPI_STATE_BUSY_TX_RX;
	hspip1->ErrorCode	= HAL_SPI_ERROR_NONE;
	hspip1->pRxBuffPtr  = (uint8_t*)Rxdata;
	hspip1->pTxBuffPtr 	= (const uint8_t*)pTxData;
	hspip1->TxXferSize  = Size;

	SPI1->CR1 |= (1 << 0);
	//LL_SPI_SetDataWidth(SPI1,LL_SPI_DATAWIDTH_8BIT );

		while(Size > 0 )
		{
			SPI1->CR2 = 0x01;
				SPI1->CR1 |= (1 << 9);


//	int temp = SPI1->SR & (1<<1);
			if(SPI1->SR & (1<<1))
			{
				SPI1->TXDR = *((const uint32_t*)hspip1->pTxBuffPtr);

			if(SPI1->SR & (1<<1))
			{
				*Rxdata = *((__IO uint8_t*)&hspip1->Instance->RXDR);
			}

			Rxdata++;
			Size--;
			for(int i= 0;i<20;i++);
			}

	}

		SPI1->IFCR |= SPI_IFCR_TXTFC | SPI_IFCR_EOTC ;			// set EOTC
					SPI1->CR1 &= ~(1 << 0);

    GPIOE->BSRR |= CS1_HIGH;
    HAL_Delay(1);
  }



void spi_tx_rx(SPI_HandleTypeDef *hspi,uint8_t *p8_t_data,uint8_t *p8_r_data,uint32_t u32_size)
{
      uint16_t   initial_TxXferCount;
      uint16_t   initial_RxXferCount;

      initial_TxXferCount = u32_size;
      initial_RxXferCount = u32_size;

      /* Set the transaction information */
      hspi->State       = HAL_SPI_STATE_BUSY_TX_RX;
      hspi->ErrorCode   = HAL_SPI_ERROR_NONE;
      hspi->pRxBuffPtr  = (uint8_t *)p8_r_data;
      hspi->pTxBuffPtr  = (const uint8_t *)p8_t_data;

      hspi->RxXferCount = u32_size;
      hspi->RxXferSize  = u32_size;

      hspi->TxXferCount = u32_size;
      hspi->TxXferSize  = u32_size;

      /*Init field not used in handle to zero */
      hspi->RxISR       = NULL;
      hspi->TxISR       = NULL;

      __HAL_SPI_DISABLE(hspi);

      SPI_2LINES(hspi);

      //__HAL_SPI_DISABLE(hspi);

      MODIFY_REG(hspi->Instance->CR2, SPI_CR2_TSIZE, u32_size);

      __HAL_SPI_ENABLE(hspi);

      SET_BIT(hspi->Instance->CR1, SPI_CR1_CSTART);

      while ((initial_TxXferCount > 0UL) /*|| (initial_RxXferCount > 0UL)*/)
      {
          /* Check the TXP flag */
          if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_TXP)) && (initial_TxXferCount > 0UL))
          {
            *((__IO uint8_t *)&hspi->Instance->TXDR) = *((const uint8_t *)hspi->pTxBuffPtr);
            hspi->pTxBuffPtr += sizeof(uint8_t);
            hspi->TxXferCount--;
            initial_TxXferCount = hspi->TxXferCount;
          }

          /* Check the RXP flag */
          if ((__HAL_SPI_GET_FLAG(hspi, SPI_FLAG_RXP)) && (initial_RxXferCount > 0UL))
          {
            *((uint8_t *)hspi->pRxBuffPtr) = *((__IO uint8_t *)&hspi->Instance->RXDR);
            hspi->pRxBuffPtr += sizeof(uint8_t);
            hspi->RxXferCount--;
            initial_RxXferCount = hspi->RxXferCount;
          }
      }

      __HAL_SPI_CLEAR_EOTFLAG(hspi);
      __HAL_SPI_CLEAR_TXTFFLAG(hspi);

      __HAL_SPI_DISABLE(hspi);

      __HAL_SPI_DISABLE_IT(hspi, (SPI_IT_EOT | SPI_IT_TXP | SPI_IT_RXP | SPI_IT_DXP | SPI_IT_UDR | SPI_IT_OVR | \
                                  SPI_IT_FRE | SPI_IT_MODF));

      __HAL_SPI_CLEAR_MODFFLAG(hspi);

      __HAL_SPI_CLEAR_FREFLAG(hspi);

//      SPI_CloseTransfer(hspi);

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

	TIM_ClockConfigTypeDef sClockSourceConfig = {0};
	TIM_MasterConfigTypeDef sMasterConfig = {0};
	TIM_OC_InitTypeDef sConfigOC = {0};

	/* USER CODE BEGIN TIM3_Init 1 */

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
