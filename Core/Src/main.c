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
#define FREQ 800
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SSD1306_USE_I2C
int counter = 0;
int counted = 0;
int pageNum = 1;
int rssiBuf[9] = { 0 };
char buf[1024];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
SA818_HandleTypeDef hsa818;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	counter++;

	if (counter == 1) {
		uint8_t str[] = "AT+DMOSETGROUP=0,148.0000,150.0000,0000,4,0000\r\n"; //передача приём

		HAL_UART_Transmit(&huart1, str, strlen(str), 300);

		uint8_t str2[] = "RSSI?\r\n";
		uint8_t rxbuf[100];
		HAL_UART_Transmit(&huart1, str2, strlen(str2), 300);
		HAL_UART_Receive(&huart1, rxbuf, sizeof(rxbuf), 300);

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
		uint8_t str[] = "AT+DMOSETGROUP=0,151.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,154.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,157.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,160.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,163.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,166.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,169.0000,150.0000,0000,4,0000\r\n";
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
		uint8_t str[] = "AT+DMOSETGROUP=0,172.0000,150.0000,0000,4,0000\r\n";
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
	} else if (counter == 10) {
		//Вернуть на изначальный канал (вывести результаты уровней, рестар после нажатия на кнопку)
		counter = 0;
		counted = 1;
		HAL_TIM_Base_Stop(&htim14);
		HAL_ADC_Start_IT(&hadc1);
	}

}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
		pageNum+=3;
		if (pageNum == 10)
			pageNum = 1;
	}
}

volatile uint32_t adcValue = 0;
volatile uint32_t sampleCount = 0;
volatile uint32_t zeroCrossings = 0;
volatile uint32_t prevAdcValue = 0;

void ADC_IRQHandler(void) {
	HAL_ADC_IRQHandler(&hadc1);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
	if (hadc->Instance == ADC1) {
		adcValue = HAL_ADC_GetValue(hadc);

		if ((prevAdcValue < 250 && adcValue >= 250)
				|| (prevAdcValue >= 250 && adcValue < 250)) {
			zeroCrossings++;
		}

		prevAdcValue = adcValue;
		sampleCount++;

		if (sampleCount >= 10000) {
			HAL_ADC_Stop_IT(hadc);
		}
	}
}

float calculateFrequency(void) {
	float timePeriod = (sampleCount / 43199.0);
	float frequency = zeroCrossings / (2 * timePeriod);
	return frequency;
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
	MX_USART1_UART_Init();
	MX_TIM14_Init();
	MX_ADC1_Init();
	/* USER CODE BEGIN 2 */

	ssd1306_Init();
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);

	sprintf(buf, "RxFreq = %d Mhz", 150);

	ssd1306_WriteString(buf, Font_7x10, White);
	ssd1306_SetCursor(0, 20);
	HAL_Delay(1);

	sprintf(buf, "TxFreq = %d MHz", 145);
	ssd1306_WriteString(buf, Font_7x10, White);
	ssd1306_SetCursor(0, 40);

	ssd1306_UpdateScreen();

	uint8_t str[] = "AT+DMOSETGROUP=0,150.0000,145.0000,0000,4,0000\r\n";
	HAL_UART_Transmit(&huart1, str, strlen(str), 300);

	HAL_ADC_Start_IT(&hadc1);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		if (!counted) {
			if (sampleCount >= 10000) {
				int freq = calculateFrequency();
				ssd1306_Fill(Black);
				ssd1306_SetCursor(0, 0);
				sprintf(buf, "RxFreq = %d Hz", freq);
				ssd1306_WriteString(buf, Font_7x10, White);
				sampleCount = 0;
				zeroCrossings = 0;
				if ((freq > FREQ - 5) && (freq < FREQ + 5)) {
					ssd1306_SetCursor(0, 20);
					sprintf(buf, "A meander has been");
					ssd1306_WriteString(buf, Font_7x10, White);
					ssd1306_SetCursor(0, 30);
					sprintf(buf, "found");
					ssd1306_WriteString(buf, Font_7x10, White);
					HAL_TIM_Base_Start_IT(&htim14);
				} else
					HAL_ADC_Start_IT(&hadc1);
				ssd1306_UpdateScreen();
			}
		} else {
			ssd1306_Fill(Black);
			ssd1306_SetCursor(0, 0);
			sprintf(buf, "RSSI: [%d] %d dbm", pageNum, rssiBuf[pageNum-1]);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 20);
			sprintf(buf, "RSSI: [%d] %d dbm", pageNum+1, rssiBuf[pageNum]);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_SetCursor(0, 40);
			sprintf(buf, "RSSI: [%d] %d dbm", pageNum+2, rssiBuf[pageNum+1]);
			ssd1306_WriteString(buf, Font_7x10, White);
			ssd1306_UpdateScreen();
		}
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
 * @brief ADC1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_ADC1_Init(void) {

	/* USER CODE BEGIN ADC1_Init 0 */

	/* USER CODE END ADC1_Init 0 */

	ADC_ChannelConfTypeDef sConfig = { 0 };

	/* USER CODE BEGIN ADC1_Init 1 */

	/* USER CODE END ADC1_Init 1 */

	/** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc1.Init.LowPowerAutoWait = DISABLE;
	hadc1.Init.LowPowerAutoPowerOff = DISABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	hadc1.Init.SamplingTimeCommon1 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.SamplingTimeCommon2 = ADC_SAMPLETIME_1CYCLE_5;
	hadc1.Init.OversamplingMode = DISABLE;
	hadc1.Init.TriggerFrequencyMode = ADC_TRIGGER_FREQ_HIGH;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure Regular Channel
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_REGULAR_RANK_1;
	sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN ADC1_Init 2 */

	/* USER CODE END ADC1_Init 2 */

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
	huart1.Init.BaudRate = 9600;
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
