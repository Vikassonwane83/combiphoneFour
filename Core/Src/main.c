/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define CHANNELS                2               // number of ADC channels scanned (PA0, PA1)
#define SAMPLES_PER_CHANNEL     32              // total samples per channel per full buffer (must be even)
#define ADC_MAX                 4095.0f         // 12-bit ADC
#define VREF                    3.3f

#define BUFFER_LENGTH           (CHANNELS * SAMPLES_PER_CHANNEL)  // DMA buffer length (half = BUFFER_LENGTH/2)
#if (SAMPLES_PER_CHANNEL % 2) != 0
#error "SAMPLES_PER_CHANNEL must be even for half-buffer processing"
#endif
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc;
DMA_HandleTypeDef hdma_adc;

/* USER CODE BEGIN PV */

static uint16_t adc_buffer[BUFFER_LENGTH]; // DMA circular buffer (16-bit words)
volatile float adc_voltage[CHANNELS]; // latest computed voltages (after calibration)
volatile uint8_t adc_data_ready = 0;

volatile uint16_t adc_raw[2];



/* Simple linear calibration placeholders (per-channel) */
static float cal_gain[CHANNELS] = { 1.0f, 1.0f };   // multiply
static float cal_offset[CHANNELS] = { 0.0f, 0.0f };   // add (volts)

extern ADC_HandleTypeDef hadc;
extern DMA_HandleTypeDef hdma_adc;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Send_String(char *str);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_DrawBar(uint8_t row, uint8_t value);
void LCD_LoadCustomChars(void);

static void process_adc_half(uint32_t start_index, uint32_t length);
void start_adc_dma_processing(void);

void LCD_PrintFloat(uint8_t row, float val);
uint8_t map_voltage(float v, float max_v);
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
	MX_DMA_Init();
	MX_ADC_Init();
	/* USER CODE BEGIN 2 */

	LCD_Init();
	LCD_LoadCustomChars();
	HAL_Delay(10);

	HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_raw, 2);

//	start_adc_dma_processing();
//     LCD_Send_String("Hello World!");
//     HAL_Delay(2000);
//
//     LCD_Clear();
//     LCD_Send_String("STM32F030K6");
//     HAL_Delay(2000);
//
//     LCD_Clear();
//     LCD_Send_String("LCD OK!");

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {

		uint16_t raw0 = adc_raw[0];
		uint16_t raw1 = adc_raw[1];

		adc_voltage[0] = (raw0 * VREF) / ADC_MAX;
		adc_voltage[1] = (raw1 * VREF) / ADC_MAX;

		uint8_t bar0 = (uint8_t) ((adc_voltage[0] / VREF) * 99.0f);
		uint8_t bar1 = (uint8_t) ((adc_voltage[1] / VREF) * 99.0f);

		/* Display */
		LCD_DrawBar(0, bar0);
		LCD_DrawBar(1, bar1);

//		if (adc_data_ready) {
//			float v0 = adc_voltage[0];
//			float v1 = adc_voltage[1];
//
//			LCD_PrintFloat(0, v0);
//			LCD_PrintFloat(1, v1);
//
//			uint8_t bar1 = map_voltage(v0, 2.5f);
//			uint8_t bar2 = map_voltage(v1, 2.5f);
//
//			LCD_DrawBar(0, bar1);
//			LCD_DrawBar(1, bar2)
//		}

		HAL_Delay(50);   // slow enough to avoid flicker
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSI14;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.HSI14CalibrationValue = 16;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC_Init(void)
{

  /* USER CODE BEGIN ADC_Init 0 */

  /* USER CODE END ADC_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC_Init 1 */

  /* USER CODE END ADC_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc.Instance = ADC1;
  hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc.Init.Resolution = ADC_RESOLUTION_12B;
  hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
  hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc.Init.LowPowerAutoWait = DISABLE;
  hadc.Init.LowPowerAutoPowerOff = DISABLE;
  hadc.Init.ContinuousConvMode = ENABLE;
  hadc.Init.DiscontinuousConvMode = DISABLE;
  hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc.Init.DMAContinuousRequests = DISABLE;
  hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  if (HAL_ADC_Init(&hadc) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel to be converted.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC_Init 2 */

  /* USER CODE END ADC_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
//  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
//  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, RS_Pin|EN_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, D4_Pin|D5_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : RS_Pin EN_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = RS_Pin|EN_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : D4_Pin D5_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

void LCD_Enable(void) {
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
}
void LCD_Send4Bits(uint8_t data) {
	HAL_GPIO_WritePin(GPIOA, D4_Pin,
			(data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, D5_Pin,
			(data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, D6_Pin,
			(data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, D7_Pin,
			(data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_SendCmd(uint8_t cmd) {
	HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_RESET);

	LCD_Send4Bits(cmd >> 4);
	LCD_Enable();

	LCD_Send4Bits(cmd & 0x0F);
	LCD_Enable();
}
void LCD_SendData(uint8_t data) {
	HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_SET);

	LCD_Send4Bits(data >> 4);
	LCD_Enable();

	LCD_Send4Bits(data & 0x0F);
	LCD_Enable();
}

void LCD_Init(void) {
	HAL_Delay(50);

	LCD_Send4Bits(0x03);
	LCD_Enable();
	HAL_Delay(5);

	LCD_Send4Bits(0x02);
	LCD_Enable();
	HAL_Delay(5);

	LCD_SendCmd(0x28); // 4-bit mode, 2 line
	LCD_SendCmd(0x0C); // display on, cursor off
	LCD_SendCmd(0x06); // auto-increment
	LCD_Clear();
}

void LCD_Clear(void) {
	LCD_SendCmd(0x01);
	HAL_Delay(2);
}

void LCD_Send_String(char *str) {
	while (*str)
		LCD_SendData(*str++);
}
//void LCD_DrawBar(uint8_t row, uint8_t value)
//{
//    if (value > 99) value = 99;
//
//    // Correct rounding
//    uint8_t filled = (value * 14 + 99) / 100;
//
//    if (filled > 14) filled = 14;
//    uint8_t empty = 14 - filled;
//
//    // Set cursor
//    if (row == 0) LCD_SendCmd(0x80);
//    else          LCD_SendCmd(0xC0);
//
//    // Draw filled
//    for (uint8_t i = 0; i < filled; i++)
//        LCD_SendData('#');
//
//    // Draw empty
//    for (uint8_t i = 0; i < empty; i++)
//        LCD_SendData(' ');
//
//    // Percentage (2 chars)
//    LCD_SendData((value / 10) + '0');
//    LCD_SendData((value % 10) + '0');
//}

void LCD_DrawBar(uint8_t row, uint8_t value) {
	if (value > 99)
		value = 99;

	uint8_t filled = (value * 14 + 99) / 100;
	if (filled > 14)
		filled = 14;

	uint8_t empty = 14 - filled;

	// Set cursor
	if (row == 0)
		LCD_SendCmd(0x80);
	else
		LCD_SendCmd(0xC0);

	// Filled (black blocks)
	for (uint8_t i = 0; i < filled; i++)
		LCD_SendData(0x00);       // custom block

	// Empty (just space)
	for (uint8_t i = 0; i < empty; i++)
		LCD_SendData(' ');

	// Percentage
	LCD_SendData((value / 10) + '0');
	LCD_SendData((value % 10) + '0');
}

void LCD_LoadCustomChars(void) {
	uint8_t fullBlock[8] = { 0b11111, 0b11111, 0b11111, 0b11111, 0b11111,
			0b11111, 0b11111, 0b11111 };

	// Select CGRAM address 0
	LCD_SendCmd(0x40);

	for (int i = 0; i < 8; i++)
		LCD_SendData(fullBlock[i]);

	// Return to DDRAM
	LCD_SendCmd(0x80);
}

void LCD_PrintFloat(uint8_t row, float val)
{
    char buf[16];

    // Convert float into integer + fraction (3 decimals)
    int scaled = (int)(val * 1000);     // example: 12.345 → 12345
    int int_part = scaled / 1000;       // 12
    int frac_part = scaled % 1000;      // 345

    // Generate final string
    sprintf(buf, "V:%d.%03d", int_part, frac_part);

    // Select row
    if (row == 0)
        LCD_SendCmd(0x80);     // First line
    else
        LCD_SendCmd(0xC0);     // Second line

    // Send to LCD
    LCD_Send_String(buf);
}


/* ---------------- Callbacks used by HAL ----------------
 HAL will call these functions when ADC-DMA reaches half or full transfer.
 Make sure CubeMX has DMA interrupts enabled for ADC DMA.
 ------------------------------------------------------ */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef *hadc_ptr) {
	/* Process first half of the circular buffer */
	/* start_index = 0, length = BUFFER_LENGTH/2 samples (interleaved channels) */
	(void) hadc_ptr;
	process_adc_half(0, BUFFER_LENGTH / 2);
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc_ptr) {
	/* Process second half of the circular buffer */
	/* start_index = BUFFER_LENGTH/2, length = BUFFER_LENGTH/2 samples */
	(void) hadc_ptr;
	process_adc_half(BUFFER_LENGTH / 2, BUFFER_LENGTH / 2);
}

/* ---------------- Process one half of the circular buffer ------------
 start_index: index in adc_buffer where this half begins (0 or BUFFER_LENGTH/2)
 length: number of half-buffer elements (should be BUFFER_LENGTH/2)
 Buffer layout (interleaved): [ch0_sample0, ch1_sample0, ch0_sample1, ch1_sample1, ...]
 ------------------------------------------------------------------- */
static void process_adc_half(uint32_t start_index, uint32_t length) {
	// length must be multiple of CHANNELS
	uint32_t samples_per_channel_in_half = length / CHANNELS;
	uint32_t sum[CHANNELS];
	memset(sum, 0, sizeof(sum));

	for (uint32_t i = 0; i < samples_per_channel_in_half; ++i) {
		for (uint32_t ch = 0; ch < CHANNELS; ++ch) {
			uint32_t idx = start_index + i * CHANNELS + ch;
			sum[ch] += adc_buffer[idx]; // safe: adc_buffer elements are 16-bit
		}
	}

	// compute average for this half and convert to voltage
	for (uint32_t ch = 0; ch < CHANNELS; ++ch) {
		float avg_raw = (float) sum[ch] / (float) samples_per_channel_in_half; // raw ADC mean
		float voltage = (avg_raw / ADC_MAX) * VREF;
		// apply simple linear calibration
		voltage = voltage * cal_gain[ch] + cal_offset[ch];
		// store the averaged value (this will be updated twice per full buffer cycle)
		adc_voltage[ch] = voltage;
	}

	// signal that new data is ready (main loop can read adc_voltage[])
	adc_data_ready = 1;
}

/* ---------------- Helper to start ADC + DMA ---------------- */
void start_adc_dma_processing(void) {
	// Clear flags & buffer (optional)

//	HAL_ADC_Stop(&hadc);
//	HAL_ADC_Stop_DMA(&hadc);

	memset((void*) adc_buffer, 0, sizeof(adc_buffer));
	adc_data_ready = 0;

	// Start ADC in DMA circular mode: number of uint32_t words = BUFFER_LENGTH
	// HAL expects a pointer to uint32_t but our buffer is 16-bit sized; casting is correct.
	if (HAL_ADC_Start_DMA(&hadc, (uint32_t*) adc_buffer, BUFFER_LENGTH)
			!= HAL_OK) {
		// Handle error
		Error_Handler();
	}
}

uint8_t map_voltage(float v, float max_v)
{
//    int scaled = (int)((v / max_v) * 99.0f);
	int scaled = (int)((v * 99.0f / max_v) + 0.5f);
    if (scaled < 0) scaled = 0;
    if (scaled > 99) scaled = 99;
    return (uint8_t)scaled;
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
