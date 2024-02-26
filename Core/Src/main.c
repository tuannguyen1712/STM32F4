/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "stdint.h"
#include "SSD1306/ssd1306.h"
#include "SSD1306/menu.h"
#include "string.h"
#include "DHT11/dht.h"
#include "MQ135/MQ135.h"
#include "DS1307/DS1307.h"
#include "W25Q32/W25Q32.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define FLASH_SECTOR		4096			// 4KB
#define FLASH_ARRAY_SIZE	1024			// 1024 * FLASH_SECTOR_SIZE
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
bool rx_spi_flg;
uint64_t sys_tick = 0;					// 1ms
uint64_t last_tick = 0;
uint64_t get_inf_tick = 0;
bool btn1_status = false;
uint8_t menu_ind0 = 0;
uint8_t last_meu_ind0 = 0;
uint8_t menu_ind1 = 0;
uint8_t last_meu_ind1 = 0;
uint8_t menu_ind2 = 0;
uint8_t last_meu_ind2 = 0;
uint8_t menu_lv = 0;
uint8_t last_menu_lv = 0;
uint8_t menu_i[3];
uint8_t last_menu_i[3];

uint8_t uart_buff_cnt = 0;
uint8_t chr;						// get byte from uart
uint8_t tx_buf[100];
uint8_t rx_buf[100];
uint8_t uart_buf_overflow = 0;
bool rx_done = 0;
uint64_t uart_last_receive = 0;
const uint8_t UART_TIMEOUT = 1;		// uart timeout

char *DayOfWeek[] = { "Sunday", "Monday", "Tuesday", "Wednesday", "Thursday",
		"Friday", "Saturday" };
uint8_t *day;
uint8_t temp[200];
uint8_t rsp[] = "\tOK\n";
uint32_t ADC_val;
int dow_u, sec_u, min_u, hrs_u, date_u, mon_u;
int year_u;

Air_param_t aqi1;
DS1307_param_t ds1307;
dht11_t dht11;

uint8_t rx_spi[64];
uint16_t fl_i = 0;				// flash array index
uint8_t value[100];
/* byte 0: temperature
 * byte 1: humidity
 * byte 2(hi), 3(lo): acetone
 * byte 4(hi), 5(lo): alcohol
 * byte 6(hi), 7(lo): CO
 * byte 8(hi), 9(lo): CO2
 * byte 10(hi), 11(lo): NH4
 * byte 12(hi), 13(lo): Toluen
 */
uint16_t dataReg[8];

uint8_t er_dht1[] = "ER1 - Can't get DHT11 value";
uint8_t er_mq1[] = "ER2 - Can't get MQ135 value";
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM2_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM3_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
void Update_Screen(I2C_HandleTypeDef *hi2c);
void Display_First_Screen(I2C_HandleTypeDef *hi2c);
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
void Update_Screen(I2C_HandleTypeDef *hi2c);

void getValue();
void saveValue();				// write value to W25Q32
void readValue(uint16_t index);				// read value from W25Q32

void HandleCommand();
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart);
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi);
uint8_t* checkDay_i(uint8_t dow);
uint8_t checkDay_a(uint8_t *Day);

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc);
void getADC_value(ADC_HandleTypeDef *hadc, uint32_t Timeout, uint32_t *ADC_val);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
	MX_TIM2_Init();
	MX_USART1_UART_Init();
	MX_ADC1_Init();
	MX_I2C2_Init();
	MX_TIM3_Init();
	MX_SPI1_Init();
	/* USER CODE BEGIN 2 */
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start(&htim3);
	ssd1306_Init(&hi2c1);
	W25Q32_Init(&hspi1, GPIOB, GPIO_PIN_0);
	init_dht11(&dht11, &htim3, GPIOB, GPIO_PIN_13);
	DS1307_config(&hi2c2);
	DS1307_settime(&hi2c2, 00, 33, 14, 6, 8, 9, 2023);
	HAL_UART_Receive_IT(&huart1, &chr, sizeof(chr));
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, 0);
	Display_First_Screen(&hi2c1);
	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		HandleCommand();
		if (sys_tick - get_inf_tick >= 10000) {
			DS1307_gettime(&hi2c2, &ds1307);
			day = checkDay_i(ds1307.dow);
			sprintf((char*) tx_buf, "[%d]\n%-9s %02d:%02d:%02d %02d/%02d/%d\n",
					(int) (sys_tick / 1000), day, ds1307.hour, ds1307.min,
					ds1307.sec, ds1307.date, ds1307.month, ds1307.year);
			HAL_UART_Transmit(&huart1, tx_buf, strlen((char*) tx_buf), 200);
			memset(tx_buf, 0, strlen((char*) tx_buf));
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_13);
			getADC_value(&hadc1, 1000, &ADC_val);
			readDHT11(&dht11);
//		sprintf((char*) temp, "%d %d aaa\n", (int) dht11.temperature,
//				(int) dht11.humidty);
//		HAL_UART_Transmit(&huart1, temp, strlen((char*) temp), 200);
//		memset(tx_buf, 0, strlen((char*) tx_buf));
			if (dht11.temperature == 0 || dht11.humidty == 0) {
				getAQI_val(&aqi1, ADC_val);
				if (ADC_val == 0) {
					HAL_UART_Transmit(&huart1, er_mq1, strlen((char*) er_mq1),
							200);
				} else {
					HAL_UART_Transmit(&huart1, er_dht1, strlen((char*) er_dht1),
							200);
					sprintf((char*) temp,
							"Acetone: %d  Alcohol: %d  CO: %d  CO2: %d  NH4: %d  Toluene: %d  ADC value: %d\n\n",
							(int) aqi1.Acetone, (int) aqi1.Alcohol,
							(int) aqi1.CO, (int) aqi1.CO2, (int) aqi1.NH4,
							(int) aqi1.Toluene, (int) ADC_val);
					HAL_UART_Transmit(&huart1, temp, strlen((char*) temp), 200);
					memset(temp, 0, sizeof(temp));
				}
			} else {
				if (ADC_val == 0) {
					HAL_UART_Transmit(&huart1, er_mq1, strlen((char*) er_mq1),
							200);
				} else {
					getAQI_Correctval(&aqi1, dht11.temperature, dht11.humidty,
							ADC_val);
					sprintf((char*) temp,
							"Acetone: %d  Alcohol: %d  CO: %d  CO2: %d  NH4: %d  Toluene: %d  ADC value: %d\nTemperature: %d  Humidity: %d\n\n",
							(int) aqi1.Acetone, (int) aqi1.Alcohol,
							(int) aqi1.CO, (int) aqi1.CO2, (int) aqi1.NH4,
							(int) aqi1.Toluene, (int) ADC_val,
							(int) dht11.temperature, (int) dht11.humidty);
					HAL_UART_Transmit(&huart1, temp, strlen((char*) temp), 200);
					memset(temp, 0, sizeof(temp));
				}
			}
			saveValue();
			get_inf_tick = sys_tick;
		}
		Update_Screen(&hi2c1);
	}
	/* USER CODE END WHILE */

	/* USER CODE BEGIN 3 */

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
	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK) {
		Error_Handler();
	}

	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
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
	hi2c1.Init.ClockSpeed = 100000;
	hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c1.Init.OwnAddress1 = 0;
	hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c1.Init.OwnAddress2 = 0;
	hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C1_Init 2 */

	/* USER CODE END I2C1_Init 2 */

}

/**
 * @brief I2C2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_I2C2_Init(void) {

	/* USER CODE BEGIN I2C2_Init 0 */

	/* USER CODE END I2C2_Init 0 */

	/* USER CODE BEGIN I2C2_Init 1 */

	/* USER CODE END I2C2_Init 1 */
	hi2c2.Instance = I2C2;
	hi2c2.Init.ClockSpeed = 100000;
	hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c2.Init.OwnAddress1 = 0;
	hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c2.Init.OwnAddress2 = 0;
	hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
	if (HAL_I2C_Init(&hi2c2) != HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN I2C2_Init 2 */

	/* USER CODE END I2C2_Init 2 */

}

/**
 * @brief SPI1 Initialization Function
 * @param None
 * @retval None
 */
static void MX_SPI1_Init(void) {

	/* USER CODE BEGIN SPI1_Init 0 */

	/* USER CODE END SPI1_Init 0 */

	/* USER CODE BEGIN SPI1_Init 1 */

	/* USER CODE END SPI1_Init 1 */
	/* SPI1 parameter configuration*/
	hspi1.Instance = SPI1;
	hspi1.Init.Mode = SPI_MODE_MASTER;
	hspi1.Init.Direction = SPI_DIRECTION_2LINES;
	hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
	hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi1.Init.NSS = SPI_NSS_SOFT;
	hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
	hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi1.Init.CRCPolynomial = 10;
	if (HAL_SPI_Init(&hspi1) != HAL_OK) {
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
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 16;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 999;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM2_Init 2 */

	/* USER CODE END TIM2_Init 2 */

}

/**
 * @brief TIM3 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 16;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 65535;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
		Error_Handler();
	}
	/* USER CODE BEGIN TIM3_Init 2 */

	/* USER CODE END TIM3_Init 2 */

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
	if (HAL_UART_Init(&huart1) != HAL_OK) {
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
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0, GPIO_PIN_SET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);

	/*Configure GPIO pin : PC13 */
	GPIO_InitStruct.Pin = GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pin : PA3 */
	GPIO_InitStruct.Pin = GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pins : PB0 PB13 */
	GPIO_InitStruct.Pin = GPIO_PIN_0 | GPIO_PIN_13;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

	/* EXTI interrupt init*/
	HAL_NVIC_SetPriority(EXTI3_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	/* USER CODE BEGIN MX_GPIO_Init_2 */
	/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void Update_Screen(I2C_HandleTypeDef *hi2c) {
	if (last_meu_ind0 != menu_ind0) {
		ssd1306_Fill(Black);
		ssd1306_DrawBitmap(0, 0, menu[menu_ind0], 128, 64, White);
		ssd1306_UpdateScreen(hi2c);
		last_meu_ind0 = menu_ind0;
	}
}

void Display_First_Screen(I2C_HandleTypeDef *hi2c) {
	ssd1306_Fill(Black);
	ssd1306_DrawBitmap(0, 0, menu[0], 128, 64, White);
	ssd1306_UpdateScreen(hi2c);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	UNUSED(GPIO_Pin);
	if (GPIO_Pin == GPIO_PIN_3 && sys_tick - last_tick >= 200) {
		menu_ind0 = (menu_ind0 + 1) % 4;
		last_tick = sys_tick;
//		switch (menu_lv) {
//		case 0:
//			menu_i[0] = (menu_i[0] + 1) % 4;
//			break;
//		case 1:
//			if (menu_i[0] == 3)
//				menu_i[1] = (menu_i[1] + 1) % 6;
//			break;
	}
//	} else if (GPIO_Pin == GPIO_PIN_4 && sys_tick - last_tick >= 200) {
//		menu_ind0--;
//		if (menu_ind0 < 0)
//			menu_ind0 = menu_len - 1;
//		last_tick = sys_tick;
//	} else if (GPIO_Pin == GPIO_PIN_5 && sys_tick - last_tick >= 200) {
//		switch (last_menu_lv) {
//		case 0:
//			menu_lv++;
//			break;
//		case 1:
//		if ()
//	}
//} else if (GPIO_Pin == GPIO_PIN_6 && sys_tick - last_tick >= 200) {
//	menu_ind0--;
//	if (menu_ind0 < 0)
//		menu_ind0 = menu_len - 1;
//	last_tick = sys_tick;
//}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == htim2.Instance) {
		sys_tick++;
		if (sys_tick >= 0xFFFFFFFFFFFFFFFF)
			sys_tick = 0;
	}
}

unsigned char* checkDay_i(uint8_t dow) {
	return (unsigned char*) DayOfWeek[dow - 1];
}

uint8_t checkDay_a(uint8_t *Day) {
	uint8_t i;
	for (i = 0; i < 7; i++) {
		if (strcmp((char*) Day, (char*) DayOfWeek[i]) == 0)
			break;
	}
	return i + 1;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == huart1.Instance) {
		rx_done = true;
		if (uart_buff_cnt < sizeof(rx_buf)) {
			rx_buf[uart_buff_cnt] = chr;
		} else
			uart_buf_overflow = 1;
		uart_last_receive = sys_tick;
		uart_buff_cnt++;
		HAL_UART_Receive_IT(&huart1, &chr, sizeof(chr));
	}
}

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
	if (hspi->Instance == hspi1.Instance) {
		rx_spi_flg = 1;
	}
}

void getADC_value(ADC_HandleTypeDef *hadc, uint32_t Timeout, uint32_t *ADC_val) {
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, 1000);
	*ADC_val = HAL_ADC_GetValue(&hadc1);
	HAL_ADC_Stop(&hadc1);
}
//void getValue() {
//	value[0] = dht11.temperature;
//	value[1] = dht11.humidty;
//	value[2] = aqi1.Acetone >> 8;
//	value[3] = aqi1.Acetone & 0xFF;
//	value[4] = aqi1.Alcohol >> 8;
//	value[5] = aqi1.Alcohol & 0xFF;
//	value[6] = aqi1.CO >> 8;
//	value[7] = aqi1.CO & 0xFF;
//	value[8] = aqi1.CO2 >> 8;
//	value[9] = aqi1.CO2 & 0xFF;
//	value[10] = aqi1.NH4 >> 8;
//	value[11] = aqi1.NH4 & 0xFF;
//	value[12] = aqi1.Toluene >> 8;
//	value[13] = aqi1.Toluene & 0xFF;
//}

void saveValue() {
	value[0] = dht11.temperature;
	value[1] = dht11.humidty;
	value[2] = (uint8_t) (aqi1.Acetone >> 8);
	value[3] = (uint8_t) (aqi1.Acetone & 0xFF);
	value[4] = (uint8_t) (aqi1.Alcohol >> 8);
	value[5] = (uint8_t) (aqi1.Alcohol & 0xFF);
	value[6] = (uint8_t) (aqi1.CO >> 8);
	value[7] = (uint8_t) (aqi1.CO & 0xFF);
	value[8] = (uint8_t) (aqi1.CO2 >> 8);
	value[9] = (uint8_t) (aqi1.CO2 & 0xFF);
	value[10] = (uint8_t) (aqi1.NH4 >> 8);
	value[11] = (uint8_t) (aqi1.NH4 & 0xFF);
	value[12] = (uint8_t) (aqi1.Toluene >> 8);
	value[13] = (uint8_t) (aqi1.Toluene & 0xFF);
	W25Q32_erase4k(fl_i * FLASH_SECTOR);
	W25Q32_WriteData(value, fl_i * FLASH_SECTOR, 14);
	fl_i = (fl_i + 1) % FLASH_ARRAY_SIZE;
}
void readValue(uint16_t index) {
	W25Q32_ReadData(rx_spi, index * FLASH_SECTOR, 14);
	dataReg[0] = rx_spi[0];
	dataReg[1] = rx_spi[1];
	dataReg[2] = (uint16_t) (rx_spi[2] << 8) + (uint16_t) rx_spi[3];
	dataReg[3] = (uint16_t) (rx_spi[4] << 8) + (uint16_t) rx_spi[5];
	dataReg[4] = (uint16_t) (rx_spi[6] << 8) + (uint16_t) rx_spi[7];
	dataReg[5] = (uint16_t) (rx_spi[8] << 8) + (uint16_t) rx_spi[9];
	dataReg[6] = (uint16_t) (rx_spi[10] << 8) + (uint16_t) rx_spi[11];
	dataReg[7] = (uint16_t) (rx_spi[12] << 8) + (uint16_t) rx_spi[13];
}

void HandleCommand() {
	if ((sys_tick - uart_last_receive > 1 && uart_buff_cnt > 2)
			|| uart_buf_overflow) {
		if (strncmp((char*) rx_buf, "time:", 5) == 0) {
			sscanf((char*) rx_buf + 5, "%d %d:%d:%d %d/%d/%d", &dow_u, &hrs_u,
					&min_u, &sec_u, &date_u, &mon_u, &year_u);
			DS1307_settime(&hi2c2, (uint8_t) sec_u, (uint8_t) min_u,
					(uint8_t) hrs_u, (uint8_t) dow_u, (uint8_t) date_u,
					(uint8_t) mon_u, (uint16_t) year_u);
			sprintf((char*) tx_buf, "%d OK\n", (int) (sys_tick / 1000));
			HAL_UART_Transmit(&huart1, tx_buf, strlen((char*) tx_buf), 200);
			memset(tx_buf, 0, sizeof(tx_buf));
		} else if (strncmp((char*) rx_buf, "read:", 5) == 0) {
			int ind;
			sscanf((char*) rx_buf + 5, "%d", &ind);
			readValue(ind);
			sprintf((char*) tx_buf, "\n[%d] %d %d %d %d %d %d %d %d\n",
					(int) (sys_tick / 1000), dataReg[0], dataReg[1], dataReg[2],
					dataReg[3], dataReg[4], dataReg[5], dataReg[6], dataReg[7]);
			HAL_UART_Transmit(&huart1, tx_buf, strlen((char*) tx_buf), 200);
			memset(tx_buf, 0, sizeof(tx_buf));
		} else {
			sprintf((char*) tx_buf, "%d Incorrect command\n",
					(int) (sys_tick / 1000));
			HAL_UART_Transmit(&huart1, tx_buf, strlen((char*) tx_buf), 200);
			memset(tx_buf, 0, sizeof(tx_buf));
		}
		memset(rx_buf, 0, sizeof(rx_buf));
		uart_buff_cnt = 0;
	}
}

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
