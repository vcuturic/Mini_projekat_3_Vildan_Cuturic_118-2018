/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
#include <string.h>

#define Output0 GPIO_PIN_0
#define Output1 GPIO_PIN_1
#define Output2 GPIO_PIN_2
#define Output3 GPIO_PIN_3
#define Output4 GPIO_PIN_4
#define Output5 GPIO_PIN_5
#define Output6 GPIO_PIN_6

#define R1_PORT GPIOB
#define R1_PIN GPIO_PIN_3

#define R2_PORT GPIOB
#define R2_PIN GPIO_PIN_4

#define R3_PORT GPIOB
#define R3_PIN GPIO_PIN_5

#define R4_PORT GPIOB
#define R4_PIN GPIO_PIN_6

#define C1_PORT GPIOA
#define C1_PIN GPIO_PIN_0

#define C2_PORT GPIOB
#define C2_PIN GPIO_PIN_0

#define C3_PORT GPIOB
#define C3_PIN GPIO_PIN_12

#define VALID_CODE_PORT GPIOB
#define VALID_CODE_PIN GPIO_PIN_10

#define INVALID_CODE_PORT GPIOB
#define INVALID_CODE_PIN GPIO_PIN_11

// PA0 - last bit set to 0, USED for input
int values[] = {0b11011110, 0b11111110, 0b00001110, 0b11111010, 0b11011010, 0b11001100, 0b10011110, 0b10110110, 0b00001100, 0b01111110};

char inputCode[3];
char validCode[3] = "118";
int codeDifference = 1;
int numbersEntered = 0; // Used not to turn on red diode if user did not enter 3 numbers
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

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void read_keypad (void) {
 // SET ROW 1 TO HIGH ALL OTHERS TO LOW
  HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_SET);
  HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET);

  if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN)) {
	  // 1
	  GPIOA -> ODR = ~values[8];
	  while(HAL_GPIO_ReadPin(C1_PORT, C1_PIN));
	  numbersEntered++;
	  strcat(inputCode, "1");
  }

  if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN)) {
	  // 2
	  GPIOA -> ODR = ~values[7];
	  while(HAL_GPIO_ReadPin(C2_PORT, C2_PIN));
	  numbersEntered++;
	  strcat(inputCode, "2");
  }

  if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN)) {
	  // 3
	  GPIOA -> ODR = ~values[6];
	  while(HAL_GPIO_ReadPin(C3_PORT, C3_PIN));
	  numbersEntered++;
	  strcat(inputCode, "3");
  }

  // SET ROW 2 TO HIGH ALL OTHERS TO LOW
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN)) {
	  // 4
	  GPIOA -> ODR = ~values[5];
	  while(HAL_GPIO_ReadPin(C1_PORT, C1_PIN));
	  numbersEntered++;
	  strcat(inputCode, "4");
	}

	if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN)) {
	  // 5
	  GPIOA -> ODR = ~values[4];
	  while(HAL_GPIO_ReadPin(C2_PORT, C2_PIN));
	  numbersEntered++;
	  strcat(inputCode, "5");
	}

	if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN)) {
	  // 6
	  GPIOA -> ODR = ~values[3];
	  while(HAL_GPIO_ReadPin(C3_PORT, C3_PIN));
	  numbersEntered++;
	  strcat(inputCode, "6");
	}

	// SET ROW 3 TO HIGH ALL OTHERS TO LOW
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_SET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_RESET);

	if(HAL_GPIO_ReadPin(C1_PORT, C1_PIN)) {
	  // 7
	  GPIOA -> ODR = ~values[2];
	  while(HAL_GPIO_ReadPin(C1_PORT, C1_PIN));
	  numbersEntered++;
	  strcat(inputCode, "7");
	}

	if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN)) {
	  // 8
	  GPIOA -> ODR = ~values[1];
	  while(HAL_GPIO_ReadPin(C2_PORT, C2_PIN));
	  numbersEntered++;
	  strcat(inputCode, "8");
	}

	if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN)) {
	  // 9
	  GPIOA -> ODR = ~values[0];
	  while(HAL_GPIO_ReadPin(C3_PORT, C3_PIN));
	  numbersEntered++;
	  strcat(inputCode, "9");
	}

	// SET ROW 3 TO HIGH ALL OTHERS TO LOW
	HAL_GPIO_WritePin(R1_PORT, R1_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R2_PORT, R2_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R3_PORT, R3_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(R4_PORT, R4_PIN, GPIO_PIN_SET);

	// *

	if(HAL_GPIO_ReadPin(C2_PORT, C2_PIN)) {
	  // 0
	  GPIOA -> ODR = ~values[9];
	  while(HAL_GPIO_ReadPin(C2_PORT, C2_PIN));
	  numbersEntered++;
	  strcat(inputCode, "0");
	}

	if(HAL_GPIO_ReadPin(C3_PORT, C3_PIN)) {
	  // 0
	  GPIOA -> ODR = ~values[6];
	  while(HAL_GPIO_ReadPin(C3_PORT, C3_PIN));
	  numbersEntered++;
	  strcat(inputCode, "0");
	}
	// #
}
void checkCode() {
	if(numbersEntered == 3){
		codeDifference = strcmp(validCode, inputCode);
		if(codeDifference == 0) {
			HAL_GPIO_WritePin(VALID_CODE_PORT, VALID_CODE_PIN, GPIO_PIN_RESET);
		}
		else {
			HAL_GPIO_WritePin(INVALID_CODE_PORT, INVALID_CODE_PIN, GPIO_PIN_RESET);
		}
		// reset code input
		numbersEntered = 0;
		strcpy(inputCode, "");
		HAL_Delay(2000); // WAIT 2 seconds then turn off diodes
		HAL_GPIO_WritePin(VALID_CODE_PORT, VALID_CODE_PIN, GPIO_PIN_SET);
		HAL_GPIO_WritePin(INVALID_CODE_PORT, INVALID_CODE_PIN, GPIO_PIN_SET);
	}
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
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  read_keypad();
	  checkCode();
	  HAL_Delay(100);
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
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6, GPIO_PIN_SET);

  /*Configure GPIO pin : PA0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 PB11 PB3 PB4
                           PB5 PB6 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

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
