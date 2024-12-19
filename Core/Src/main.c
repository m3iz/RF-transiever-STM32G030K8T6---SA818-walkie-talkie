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
uint8_t pwr = 1;
int pageNum = 1;
int rssiBuf[9] = { 0 };
char buf[1024];
uint8_t sfreq = 145, ffreq = 172, step = 9, fstep = 3, mode = 0; irqm = 0;
uint8_t rxBuffer[1024];
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_usart1_rx;

/* USER CODE BEGIN PV */
SA818_HandleTypeDef hsa818;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM14_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void delay_200ms() {
	// Это количество тактов для задержки в 200 миллисекунд при 16 МГц
	unsigned long delay_count = 320000;

	while (delay_count > 0) {
		delay_count--;
	}
}

void getRssi() {
	int index = 0;
	int i = 0;

	while (rxBuffer[i] != '\0' && index < 9) {
		if (rxBuffer[i] == '=') {
			i++;
			while (isspace(rxBuffer[i])) {
				i++;
			}
			if (isdigit(rxBuffer[i])
					|| (rxBuffer[i] == '-' && isdigit(rxBuffer[i + 1]))) {
				rssiBuf[index] = atoi((char*) &rxBuffer[i]);
				if (rssiBuf[index] < 20)
					rssiBuf[index] = 0;
				index++;
				// Пропустить число
				while (isdigit(rxBuffer[i]) || rxBuffer[i] == '-') {
					i++;
				}
			}
		} else {
			i++;
		}
	}
}

void nextStep(int txfreq, int rxfreq) {

	uint8_t str[100];
	sprintf(str, "AT+DMOSETGROUP=0,%d.0000,%d.0000,0000,4,0000\r\n", txfreq,
			rxfreq);
	HAL_UART_Transmit(&huart1, str, strlen(str), 300);
	delay_200ms();
	uint8_t str2[] = "RSSI?\r\n";
	HAL_UART_Transmit(&huart1, str2, strlen(str2), 300);
	/*ssd1306_Fill(Black);
	 ssd1306_SetCursor(0, 0);
	 sprintf(buf, "TxFreq = %d Mhz", txfreq);
	 ssd1306_WriteString(buf, Font_7x10, White);
	 ssd1306_SetCursor(0, 20);

	 sprintf(buf, "RxFreq = %d MHz", rxfreq);
	 ssd1306_WriteString(buf, Font_7x10, White);
	 ssd1306_SetCursor(0, 40);

	 ssd1306_UpdateScreen();*/
}

void Eeprom_RW(uint8_t rw) {
	if (rw == 1) {
		uint16_t wmsg[3];
		wmsg[0] = sfreq;
		wmsg[1] = ffreq;
		wmsg[2] = step;
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
	} else {
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
		sfreq = rmsg[0];
		ffreq = rmsg[1];
		step = rmsg[2];
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	counter++;

	if (counter < 10) {
		nextStep(sfreq + counter * fstep, sfreq + counter * fstep);
	} else if (counter == 10) {
		getRssi();
		counter = 0;
		counted = 1;
		HAL_TIM_Base_Stop(&htim14);
		//HAL_ADC_Start_IT(&hadc1);
	}

}

void HAL_GPIO_EXTI_Rising_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_0) {

		//HAL_GPIO_TogglePin(LCD_GPIO_Port, LCD_Pin);
		HAL_GPIO_TogglePin(ONF_GPIO_Port, ONF_Pin);

	}

}
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_5) {
		if (mode < 5) {
			mode++;

		}
		pageNum += 3;
		if (pageNum == 13)
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

void drawMenu() {
	Eeprom_RW(1);
	ssd1306_Fill(Black);
	ssd1306_SetCursor(0, 0);
	if (mode == 1)
		sprintf(buf, "* Start: %d Mhz", sfreq);
	else
		sprintf(buf, "Start: %d Mhz", sfreq);
	ssd1306_WriteString(buf, Font_7x10, White);
	ssd1306_SetCursor(0, 20);
	if (mode == 2)
		sprintf(buf, "* Stop: %d MHz", ffreq);
	else
		sprintf(buf, "Stop: %d MHz", ffreq);
	ssd1306_WriteString(buf, Font_7x10, White);

	ssd1306_SetCursor(0, 40);
	if (mode == 3)
		sprintf(buf, "* Steps: %d", step);
	else
		sprintf(buf, "Steps: %d", step);
	ssd1306_WriteString(buf, Font_7x10, White);

	ssd1306_UpdateScreen();
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
	MX_DMA_Init();
	MX_I2C1_Init();
	MX_USART1_UART_Init();
	MX_TIM14_Init();
	MX_ADC1_Init();
	MX_TIM1_Init();
	/* USER CODE BEGIN 2 */
	HAL_UART_Receive_DMA(&huart1, rxBuffer, 300);
	ssd1306_Init();
	Eeprom_RW(0);
	uint8_t str[] = "AT+DMOSETGROUP=0,145.0000,145.0000,0000,4,0000\r\n";

	HAL_UART_Transmit(&huart1, str, strlen(str), 300);
	HAL_Delay(200);
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	int32_t prevCounter = 0;

	while (1) {
		if (mode < 4) {
			drawMenu();
			int32_t currCounter = __HAL_TIM_GET_COUNTER(&htim1);
			currCounter = 32767 - ((currCounter - 1) & 0xFFFF) / 2;
			if (currCounter > 32768 / 2) {
				currCounter = currCounter - 32768;
			}
			if (currCounter != prevCounter) {
				int32_t delta = currCounter - prevCounter;
				prevCounter = currCounter;
				if ((delta > -10) && (delta < 10)) {
					if (mode == 1) {
						sfreq += delta;
					} else if (mode == 2) {
						ffreq += delta;
					} else if (mode == 3) {
						step += delta;
					}
				}
			}
		} else if (mode == 4) {
			fstep = (ffreq - sfreq) / step;
			HAL_ADC_Start_IT(&hadc1);
		}
		if (!counted) {
			if (sampleCount >= 10000) {
				int freq = calculateFrequency();
				sampleCount = 0;
				zeroCrossings = 0;
				if ((freq > FREQ - 5) && (freq < FREQ + 5)) {
					ssd1306_Fill(Black);
					ssd1306_SetCursor(0, 0);
					sprintf(buf, "Analyzing...");
					ssd1306_WriteString(buf, Font_7x10, White);
					ssd1306_UpdateScreen();
					HAL_TIM_Base_Start_IT(&htim14);
				} else
					HAL_ADC_Start_IT(&hadc1);
			}
		} else {
			if (pageNum == 10) {

				ssd1306_Fill(Black);

				const int16_t RSSI_MIN = -160;
				const int8_t RSSI_MAX = -30;

				const uint8_t DISPLAY_WIDTH = 128;
				const uint8_t DISPLAY_HEIGHT = 64;

				size_t rssiBufSize = sizeof(rssiBuf) / sizeof(rssiBuf[0]);

				const uint8_t STEP_X = (DISPLAY_WIDTH - 1) / (rssiBufSize - 1); // Корректировка шага

				const uint8_t AXIS_MARGIN = 10; // Отступ от границы экрана для осей

				ssd1306_Line(0, DISPLAY_HEIGHT - AXIS_MARGIN - 5,
						DISPLAY_WIDTH - 1, DISPLAY_HEIGHT - AXIS_MARGIN - 5,
						White);

				ssd1306_Line(AXIS_MARGIN - 8, 0, AXIS_MARGIN - 8,
						DISPLAY_HEIGHT - 1, White);

				for (size_t i = 0; i < rssiBufSize; i++) {

					int rssiValue = (int) (1.113 * rssiBuf[i] - 160);

					uint8_t y = (uint8_t) ((rssiValue - RSSI_MIN)
							* (DISPLAY_HEIGHT - AXIS_MARGIN)
							/ (RSSI_MAX - RSSI_MIN));
					y = DISPLAY_HEIGHT - AXIS_MARGIN - y;

					uint8_t x = i * STEP_X;

					if (i > 0) {
						int prevRssiValue = (int) (1.113 * rssiBuf[i - 1] - 160);
						uint8_t prev_y = (uint8_t) ((prevRssiValue - RSSI_MIN)
								* (DISPLAY_HEIGHT - AXIS_MARGIN)
								/ (RSSI_MAX - RSSI_MIN));
						prev_y = DISPLAY_HEIGHT - AXIS_MARGIN - prev_y; // �?нвертируем значение Y для предыдущей точки

						uint8_t prev_x = (i - 1) * STEP_X;

						ssd1306_Line(prev_x, prev_y, x, y, White);
					}
				}

				for (size_t i = 0; i < rssiBufSize; i++) {
					uint8_t x = i * STEP_X;

					uint8_t cursorX = (x < 3) ? 0 : (x - 3); // Корректируем значение, если x < 3
					ssd1306_SetCursor(cursorX, DISPLAY_HEIGHT - 10); // Устанавливаем курсор на 10 пикселей выше нижней границы дисплея
					snprintf(buf, sizeof(buf), "%d", (int) (i + 1)); // Форматируем индекс массива (начиная с 1)
					ssd1306_WriteString(buf, Font_7x10, White); // Выводим индекс на дисплей
				}

				// Обновление экрана
				ssd1306_UpdateScreen();

			} else {
				ssd1306_Fill(Black);
				ssd1306_SetCursor(0, 0);
				sprintf(buf, "RSSI: [%d] %d dbm", pageNum,
						(int) (1.113 * rssiBuf[pageNum - 1] - 160));
				ssd1306_WriteString(buf, Font_7x10, White);
				ssd1306_SetCursor(0, 20);
				sprintf(buf, "RSSI: [%d] %d dbm", pageNum + 1,
						(int) (1.113 * rssiBuf[pageNum] - 160));
				ssd1306_WriteString(buf, Font_7x10, White);
				ssd1306_SetCursor(0, 40);
				sprintf(buf, "RSSI: [%d] %d dbm", pageNum + 2,
						(int) (1.113 * rssiBuf[pageNum + 1] - 160));
				ssd1306_WriteString(buf, Font_7x10, White);
				ssd1306_UpdateScreen();
			}
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
 * Enable DMA controller clock
 */
static void MX_DMA_Init(void) {

	/* DMA controller clock enable */
	__HAL_RCC_DMA1_CLK_ENABLE();

	/* DMA interrupt init */
	/* DMA1_Channel1_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
	HAL_GPIO_WritePin(GPIOB, LCD_Pin | ONF_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA, Audio_Tx_Pin | LED_Pin | WP_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LCD_Pin ONF_Pin */
	GPIO_InitStruct.Pin = LCD_Pin | ONF_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/*Configure GPIO pins : Audio_Tx_Pin LED_Pin WP_Pin */
	GPIO_InitStruct.Pin = Audio_Tx_Pin | LED_Pin | WP_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin : PWR_Pin */
	GPIO_InitStruct.Pin = PWR_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(PWR_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : SW_Pin */
	GPIO_InitStruct.Pin = SW_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(SW_GPIO_Port, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI0_1_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(EXTI0_1_IRQn);

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
