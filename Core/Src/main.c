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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
/* USER CODE BEGIN PFP */
void LCD_Init(void);
void LCD_Clear(void);
void LCD_Send_String(char *str);
void LCD_SendCmd(uint8_t cmd);
void LCD_SendData(uint8_t data);
void LCD_DrawBar(uint8_t row, uint8_t value);
void LCD_LoadCustomChars(void);
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
  /* USER CODE BEGIN 2 */

  LCD_Init();
  LCD_LoadCustomChars();
     HAL_Delay(10);

     /* ---- Display Text ---- */
     uint8_t value = 0;
     int8_t dir = 1;
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
  while (1)
  {
//	  LCD_Clear();
//	          LCD_Send_String("Loop Running");
//	          HAL_Delay(1000);
//
//	          LCD_Clear();
//	          LCD_Send_String("STM32F030K6");
//	          HAL_Delay(1000);

	  LCD_DrawBar(0, value);   // update only line 1 (no blinking)

	      value += dir;

	      if (value == 99) dir = -1;   // start decreasing
	      if (value == 0)  dir = +1;   // start increasing

	      LCD_DrawBar(1, value);   // update only line 1 (no blinking)

	      	      value += dir;

	      	      if (value == 99) dir = -1;   // start decreasing
	      	      if (value == 0)  dir = +1;   // start increasing

	      HAL_Delay(1);   // slow enough to avoid flicker
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
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
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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

void LCD_Enable(void)
{
    HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(GPIOB, EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
}
void LCD_Send4Bits(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOA, D4_Pin, (data & 0x01) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOA, D5_Pin, (data & 0x02) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, D6_Pin, (data & 0x04) ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOB, D7_Pin, (data & 0x08) ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void LCD_SendCmd(uint8_t cmd)
{
    HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_RESET);

    LCD_Send4Bits(cmd >> 4);
    LCD_Enable();

    LCD_Send4Bits(cmd & 0x0F);
    LCD_Enable();
}
void LCD_SendData(uint8_t data)
{
    HAL_GPIO_WritePin(GPIOB, RS_Pin, GPIO_PIN_SET);

    LCD_Send4Bits(data >> 4);
    LCD_Enable();

    LCD_Send4Bits(data & 0x0F);
    LCD_Enable();
}

void LCD_Init(void)
{
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

void LCD_Clear(void)
{
    LCD_SendCmd(0x01);
    HAL_Delay(2);
}

void LCD_Send_String(char *str)
{
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

void LCD_DrawBar(uint8_t row, uint8_t value)
{
    if (value > 99) value = 99;

    uint8_t filled = (value * 14 + 99) / 100;
    if (filled > 14) filled = 14;

    uint8_t empty = 14 - filled;

    // Set cursor
    if (row == 0) LCD_SendCmd(0x80);
    else          LCD_SendCmd(0xC0);

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

void LCD_LoadCustomChars(void)
{
    uint8_t fullBlock[8] = {
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111,
        0b11111
    };

    // Select CGRAM address 0
    LCD_SendCmd(0x40);

    for (int i = 0; i < 8; i++)
        LCD_SendData(fullBlock[i]);

    // Return to DDRAM
    LCD_SendCmd(0x80);
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
