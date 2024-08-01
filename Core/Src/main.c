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
#include "ssd1306.h"
#include "fonts.h"
#include "SA818.h"
#include <string.h>
#define SSD1306_INCLUDE_FONT_7x10
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SSD1306_USE_I2C

char buf[1024];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim16;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SA818_HandleTypeDef hsa818;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM16_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Settings for SA818
int rxFreq = 0;
int txFreq = 0;
int counter = 0;
uint8_t mode = 0; // 0 - standart mode; 1 - rx Freq configuration; 2 - tx Freq configuration

int32_t prevCounter = 0;

void Eeprom_Init() {
	//const char wmsg[] = "Some data";
	//char rmsg[sizeof(wmsg)];
	uint16_t wmsg[2];
	uint16_t rmsg[sizeof(wmsg)];
	memset(wmsg, 0, sizeof(wmsg));
	// HAL expects address to be shifted one bit to the left
	uint16_t devAddr = (0x50 << 1);
	uint16_t memAddr = 0x0100;
	HAL_StatusTypeDef status;

	//HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT,
	//   (uint8_t*)wmsg, sizeof(wmsg), HAL_MAX_DELAY);

	for (;;) {
		status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
		HAL_MAX_DELAY);
		if (status == HAL_OK)
			break;
	}

	HAL_I2C_Mem_Read(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT,
			(uint8_t*) rmsg, sizeof(rmsg), HAL_MAX_DELAY);
	rxFreq = rmsg[0];
	txFreq = rmsg[1];

}

void Eeprom_Write() {

	uint16_t wmsg[2];
	wmsg[0] = rxFreq;
	wmsg[1] = txFreq;

	uint16_t devAddr = (0x50 << 1);
	uint16_t memAddr = 0x0100;
	HAL_StatusTypeDef status;

	HAL_I2C_Mem_Write(&hi2c1, devAddr, memAddr, I2C_MEMADD_SIZE_16BIT,
			(uint8_t*) wmsg, sizeof(wmsg), HAL_MAX_DELAY);

	for (;;) {
		status = HAL_I2C_IsDeviceReady(&hi2c1, devAddr, 1,
		HAL_MAX_DELAY);
		if (status == HAL_OK)
			break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM16) {
		HAL_GPIO_TogglePin(Audio_Tx_GPIO_Port, Audio_Tx_Pin);
	}
	if (htim->Instance == TIM14) {

		if (counter == 0) {
			uint8_t str[] = "AT+DMOSETGROUP=0,145.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			HAL_TIM_Base_Start_IT(&htim16);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 145);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 1) {
			HAL_TIM_Base_Stop_IT(&htim16);
			uint8_t str[] = "AT+DMOSETGROUP=0,148.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 148);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 2) {
			uint8_t str[] = "AT+DMOSETGROUP=0,151.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 151);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 3) {
			uint8_t str[] = "AT+DMOSETGROUP=0,154.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 154);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 4) {
			uint8_t str[] = "AT+DMOSETGROUP=0,157.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 157);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 5) {
			uint8_t str[] = "AT+DMOSETGROUP=0,160.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 160);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 6) {
			uint8_t str[] = "AT+DMOSETGROUP=0,163.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 163);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 7) {
			uint8_t str[] = "AT+DMOSETGROUP=0,166.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 166);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 8) {
			uint8_t str[] = "AT+DMOSETGROUP=0,169.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 169);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else if (counter == 9) {
			uint8_t str[] = "AT+DMOSETGROUP=0,172.0000,150.0000,0000,4,0000\r\n"; //передача приём
			HAL_UART_Transmit(&huart1, str, strlen(str), 300);
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RxFreq = %d Mhz", 150);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);

			sprintf(buf, "TxFreq = %d MHz", 172);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);

			ssd1306_UpdateScreen();
		} else {
			counter = -1;
		}
		counter++;
		// Ваш код здесь, который выполняется раз в секунду
	}
}

void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		/*//SSD1306_ClearScreen();
		mode++;
		Eeprom_Write();
		//SA818_SetConfig(&hsa818, SA_BANDWIDTH_12_5KHZ, txFreq, rxFreq, SA_CTCSS_OFF, SA_CTCSS_OFF, SA_SQUELCH_OFF);
		if (mode == 4)
			mode = 0;
		ssd1306_Fill(Black);
		ssd1306_SetCursor(0, 0);
		if (mode == 1)
			sprintf(buf, "* RxFrequency = %d", rxFreq);
		else
			sprintf(buf, "  RxFrequency = %d", rxFreq);
		ssd1306_WriteString(buf, Font_7x10, White);
		ssd1306_SetCursor(0, 20);
		if (mode == 2)
			sprintf(buf, "* TxFrequency = %d", txFreq);
		else
			sprintf(buf, "  TxFrequency = %d", txFreq);
		ssd1306_WriteString(buf, Font_7x10, White);
		ssd1306_SetCursor(0, 40);
		if (mode == 3)
			sprintf(buf, "* Mode = %d", mode);
		else
			sprintf(buf, "  Mode = %d", mode);
		ssd1306_WriteString(buf, Font_7x10, White);
		ssd1306_UpdateScreen();
		*/
	}
}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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
	MX_TIM1_Init();
	MX_USART1_UART_Init();
	MX_TIM14_Init();
	MX_TIM16_Init();
	/* USER CODE BEGIN 2 */

	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	Eeprom_Init();
	ssd1306_Init();
	//ssd1306_Fill(Black);

	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	ssd1306_UpdateScreen();

	/*SSD1306_Init();
	 SSD1306_ClearScreen();
	 while(SSD1306_IsReady() == 0);
	 for (uint8_t  i = 0; i < 8; i++)
	 {
	 SSD1306_DrawFilledRect(i * 16, i * 16 + 8, 16, 48);
	 SSD1306_UpdateScreen();
	 while(SSD1306_IsReady() == 0);

	 //HAL_Delay(25);
	 }

	 */

	/*SA818_Init(&hsa818, &huart1);

	 if (SA818_Begin(&hsa818) == 0) {
	 Error_Handler();
	 }

	 SA818_SetConfig(&hsa818, SA_BANDWIDTH_12_5KHZ, txFreq, rxFreq, SA_CTCSS_OFF,
	 SA_CTCSS_OFF, SA_SQUELCH_OFF);

	 SA818_SetVolume(&hsa818, SA_VOLUME_DEFAULT);

	 //SA818_SetFilters(&hsa818, SA_FILTER_ON, SA_FILTER_ON, SA_FILTER_ON);*/
	HAL_TIM_Base_Start_IT(&htim14);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		/*int32_t currCounter = __HAL_TIM_GET_COUNTER(&htim1);
		 currCounter = 32767 - ((currCounter - 1) & 0xFFFF) / 2;
		 if (currCounter > 32768 / 2) {
		 currCounter = currCounter - 32768;
		 }
		 if (currCounter != prevCounter) {
		 int32_t delta = currCounter - prevCounter;
		 prevCounter = currCounter;

		 if ((delta > -10) && (delta < 10)) {

		 ssd1306_Fill(Black);
		 ssd1306_SetCursor(0, 0);
		 if (mode == 1) {
		 rxFreq += delta;
		 if (rxFreq < 0)
		 rxFreq = 0;
		 sprintf(buf, "* RxFrequency = %d", rxFreq);
		 } else
		 sprintf(buf, "  RxFrequency = %d", rxFreq);
		 ssd1306_WriteString(buf, Font_7x10, White);
		 ssd1306_SetCursor(0, 20);
		 if (mode == 2) {
		 txFreq += delta;
		 if (txFreq < 0)
		 txFreq = 0;
		 sprintf(buf, "* TxFrequency = %d", txFreq);
		 } else
		 sprintf(buf, "  TxFrequency = %d", txFreq);
		 ssd1306_WriteString(buf, Font_7x10, White);
		 ssd1306_SetCursor(0, 40);
		 if (mode == 3)
		 sprintf(buf, "* Mode = %d", mode);
		 else
		 sprintf(buf, "  Mode = %d", mode);
		 ssd1306_WriteString(buf, Font_7x10, White);
		 ssd1306_UpdateScreen();
		 }
		 }*/

		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Configure the main internal regulator output voltage
	 */
	HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief I2C1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C1_Init(void) {

	/* USER CODE BEGIN I2C1_Init 0 */

	/* USER CODE END I2C1_Init 0 */

	/* USER CODE BEGIN I2C1_Init 1 */

	/* USER CODE END I2C1_Init 1 */
	hi2c1.Instance = I2C1;
	hi2c1.Init.Timing = 0x00303D5B;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Analogue filter
	 */
	if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE)
			!= HAL_OK) {
		Error_Handler();
	}

	/** Configure Digital filter
	 */
	if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief TIM1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM1_Init(void) {

	/* USER CODE BEGIN TIM1_Init 0 */

	/* USER CODE END TIM1_Init 0 */

	TIM_Encoder_InitTypeDef sConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM1_Init 1 */

	/* USER CODE END TIM1_Init 1 */
	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 65535;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
	sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC1Filter = 0;
	sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
	sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
	sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
	sConfig.IC2Filter = 0;
	if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM1_Init 2 */

	/* USER CODE END TIM1_Init 2 */

}

/**
 * @brief TIM14 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM14_Init(void) {

	/* USER CODE BEGIN TIM14_Init 0 */

	/* USER CODE END TIM14_Init 0 */

	/* USER CODE BEGIN TIM14_Init 1 */

	/* USER CODE END TIM14_Init 1 */
	htim14.Instance = TIM14;
	htim14.Init.Prescaler = 160 - 1;
	htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim14.Init.Period = 65535;
	htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim14) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM14_Init 2 */

	/* USER CODE END TIM14_Init 2 */

}

/**
 * @brief TIM16 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM16_Init(void) {

	/* USER CODE BEGIN TIM16_Init 0 */

	/* USER CODE END TIM16_Init 0 */

	/* USER CODE BEGIN TIM16_Init 1 */

	/* USER CODE END TIM16_Init 1 */
	htim16.Instance = TIM16;
	htim16.Init.Prescaler = 9;
	htim16.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim16.Init.Period = 1000 - 1;
	htim16.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim16.Init.RepetitionCounter = 0;
	htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim16) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM16_Init 2 */

	/* USER CODE END TIM16_Init 2 */

}

/**
 * @brief USART1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_USART1_UART_Init(void) {

	/* USER CODE BEGIN USART1_Init 0 */

	/* USER CODE END USART1_Init 0 */

	/* USER CODE BEGIN USART1_Init 1 */

	/* USER CODE END USART1_Init 1 */
	huart1.Instance = USART1;
	huart1.Init.BaudRate = 115200;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
	huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
	huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
	if (HAL_UART_Init(&huart1) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8)
			!= HAL_OK) {
		Error_Handler();
	}
	if (HAL_UARTEx_DisableFifoMode(&huart1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN USART1_Init 2 */

	/* USER CODE END USART1_Init 2 */

}

/**
 * @brief GPIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(LCD_GPIO_Port, LCD_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Audio_Tx_Pin | LED_Pin | WP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : LCD_Pin */
	GPIO_InitStruct.Pin = LCD_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(LCD_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : Audio_Rx_Pin */
	GPIO_InitStruct.Pin = Audio_Rx_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(Audio_Rx_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pins : Audio_Tx_Pin LED_Pin WP_Pin */
	GPIO_InitStruct.Pin = Audio_Tx_Pin | LED_Pin | WP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_Pin */
	GPIO_InitStruct.Pin = SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
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
