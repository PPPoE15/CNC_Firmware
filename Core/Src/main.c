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
UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

/*
   PulsePerRound(PPM)                                          2048
----------------------- = PulsePerMilimeter (PPM)              ----  = 204.8
MilimetersPerRound(MPR)                                         10

                                       mm                                 
Speed [PPS] - PulsePerSecond =  PPM * ---   
                                       s
*/       

uint16_t WorkSpeed = 2048 / 10 * 50;   // 10240 PPS
uint16_t FreeSpeed = 2048 / 10 * 100;  // 20480 PPS

uint32_t X_pos = 0;
uint32_t Y_pos = 0;
uint32_t Z_pos = 0;


uint8_t X_dir = 1; //1 - cw, 0 - ccw
uint8_t Y_dir = 1;
uint8_t Z_dir = 1;


uint8_t str[3];
uint8_t TxD[1];
uint8_t current_command = 0;
uint8_t message = '0';  // zero state message
/*
message = '0' - zero state message
message = '1' - ready to next frame message
message = '2' - ready to get x-coord
message = '3' - ready to get y-coord
message = '4' - program if completed
message = '5' - error message
*/



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */


void send_message(uint8_t message)
{
	
	TxD[0] = message;
	HAL_UART_Transmit(&huart1, TxD, 1, 10); // send ready command
	HAL_Delay(10);
}

void CNC_Init (void)
{
	uint8_t request[1];  // ready-message
	while ( HAL_UART_Receive(&huart1, request, 1, 10) != HAL_OK ){}  // waiting request from PC
	if (request[0] == '1'){
		send_message('1');
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	}
}



uint8_t CNC_Frame(uint32_t x, uint32_t y){
	HAL_Delay(500);
	//message = '1'; // ready-message
	//HAL_UART_Transmit(&huart1, &message, 1, 10); // send ready command
	send_message('1');
	return(1);
}

uint8_t CNC_WorkMove(uint32_t x, uint32_t y){
	HAL_Delay(500);
	//message = '1'; // ready-message
	//HAL_UART_Transmit(&huart1, &message, 1, 10); // send ready command
	send_message('1');
	return(1);
}

uint8_t CNC_FreeMove(uint32_t x, uint32_t y){
	HAL_Delay(500);
	//message = '1'; // ready-message
	//HAL_UART_Transmit(&huart1, &message, 1, 10); // send ready command
	send_message('1');
	return(1);
}

void compressor_on(void)
{
	HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_RESET);
	send_message('1');  // ready-message
}

void compressor_off(void)
{
	HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);
	send_message('1');  // ready-message
}

void CNC_DeInit(void)
{
	compressor_off();
}

void CNC_Main(void)  // main CNC cycle
{ 
	uint8_t str[3];
	uint8_t coord[6];	
	uint32_t x_coord;  
	uint32_t y_coord;
	
	while (1)
	{	
		while ( HAL_UART_Receive(&huart1, str, 1, 10) != HAL_OK ){} // waiting for command from PC
			
		current_command = str[0];

		switch(current_command)
		{
			case '1': case '3':  //  get a coordinate command (free or work moving) G00 = '1' / G01 = '3'
					send_message('2');  // ready to get x-coord
					while ( HAL_UART_Receive(&huart1, coord, 6, 10) != HAL_OK ){}
					x_coord = 100000*(coord[0]- '0') + 10000*(coord[1]- '0') + 1000*(coord[2]- '0') + 100*(coord[3]- '0') + 10*(coord[4]- '0') + (coord[5]- '0'); // write a x-coord variable
						
					send_message('3'); // ready to get y-coord
					while ( HAL_UART_Receive(&huart1, coord, 6, 10) != HAL_OK ){}
					y_coord = 100000*(coord[0]- '0') + 10000*(coord[1]- '0') + 1000*(coord[2]- '0') + 100*(coord[3]- '0') + 10*(coord[4]- '0') + (coord[5]- '0'); // write a x-coord variable
						
					switch(current_command)
					{
						case '1':
							CNC_FreeMove(x_coord, y_coord);
							break;
						
						case '3':
							CNC_WorkMove(x_coord, y_coord);
							break;
					}
					break;
						
			case '0':
					send_message('1'); // ready-message
					break;

			case '2':  // compressor on
					compressor_on();
					break;
			
			case '4':  // compressor off
					compressor_off();
					break;
			
			case '5':  // final of programm
					send_message('4'); // program is completed
					break;
			
			default:
					send_message('5'); // error
					break;
		}
	}
}


void X_updatePosition(void){
	if(X_dir == 1){
		X_pos ++;
	}
	else{
		X_pos --;
	}
}

void Y_updatePosition(void){
	if(Y_dir == 1){
		Y_pos ++;
	}
	else{
		Y_pos --;
	}
}

void Z_updatePosition(void){
	if(Z_dir == 1){
		Z_pos ++;
	}
	else{
		Z_pos --;
	}
}



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
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		CNC_Init();
		CNC_Main();
		CNC_DeInit();
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
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

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
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
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
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LED1_Pin|LED2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : X_hall_Pin Y_hall_Pin Z_hall_Pin */
  GPIO_InitStruct.Pin = X_hall_Pin|Y_hall_Pin|Z_hall_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : LED1_Pin LED2_Pin */
  GPIO_InitStruct.Pin = LED1_Pin|LED2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : Air_Pin */
  GPIO_InitStruct.Pin = Air_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Air_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);

  HAL_NVIC_SetPriority(EXTI3_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI3_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
