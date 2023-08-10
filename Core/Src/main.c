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
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define X_factor 60
#define Y_factor 75
#define buf_size 5000
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim10;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */

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

uint32_t x_coord;  
uint32_t y_coord;
uint32_t x_prev = 0;
uint32_t y_prev = 0;


uint8_t X_dir = 1;
uint8_t Y_dir = 1;
uint8_t Z_dir = 1;
int32_t X_pos = 0;
int32_t Y_pos = 0;
int32_t Z_pos = 0;

uint8_t flag = 0; // useles
uint32_t feed_rate = 3000; 
uint32_t free_speed = 7000;
uint32_t X_period = 500;
uint32_t Y_period = 500;
uint32_t f = 1000000; // dont use
int32_t X_e;
int32_t Y_e;
uint8_t X_home;
uint8_t Y_home;
uint8_t command [buf_size];
uint32_t X_buf [buf_size];
uint32_t Y_buf [buf_size];
uint16_t X_period_buf[buf_size];
uint16_t Y_period_buf[buf_size];
uint16_t frame = 0;
uint8_t X_dir_buf[buf_size];
uint8_t Y_dir_buf[buf_size];

uint32_t cycles_count = 0;
uint8_t ProgrammIsDone = 0;
uint8_t LoadingIsDone = 0;
uint8_t IsEmerg = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM10_Init(void);
/* USER CODE BEGIN PFP */

void X_driver(void)
{		
		if (X_dir == 1){
				GPIOA->ODR |= (1<<7); //dirX
				X_pos++;
		}
		else {
				GPIOA->ODR &= ~(1<<7);
				X_pos--;

		}

		GPIOA->ODR |= (1<<4); //stepX
		HAL_Delay(1);
		GPIOA->ODR &= ~(1<<4);
}

void Y_driver(void)
{
		if (X_dir == 1){
			GPIOC->ODR |= (1<<4); //dirY
			Y_pos++;
		}
		else {
			GPIOC->ODR &= ~(1<<4);
			Y_pos--;
		}

		GPIOA->ODR |= (1<<5); //stepY
		HAL_Delay(1);
		GPIOA->ODR &= ~(1<<5);
}

void Z_driver(void)
{
		if (Z_dir == 1){
			GPIOC->ODR |= (1<<5); //dirZ
			Z_pos++;
		}
		else {
			GPIOC->ODR &= ~(1<<5);
			Z_pos--;
		}

		GPIOA->ODR |= (1<<6); //stepZ
		HAL_Delay(1);
		GPIOA->ODR &= ~(1<<6);
}

static inline void calculate_period(int32_t x, int32_t y, uint32_t speed)
{
	float fdt;
	if(x < 0){ x*= -1;}
	if(y < 0){ y*= -1;}
	fdt = hypot(x, y);
	fdt = fdt * 1000000 / speed; // fdt = f * sqrtf(x*x + y*y) / feed_rate;
	X_period = fdt / x;
	Y_period = fdt / y;
}

void send_message(uint8_t message)
{
	TxD[0] = message;
	HAL_UART_Transmit(&huart1, TxD, 1, 10); // send ready command
}

void Driver_Init(void)
{
	TIM6->DIER |= TIM_DIER_UIE; // enable interrupt from tim6
	TIM7->DIER |= TIM_DIER_UIE; // enable interrupt from tim7
	TIM10->DIER |= TIM_DIER_UIE; // enable interrupt from tim10
	TIM6->CR1 &= ~TIM_CR1_CEN;	// disable tim6
	TIM7->CR1 &= ~TIM_CR1_CEN;	// disable tim7
	TIM10->CR1 &= ~TIM_CR1_CEN;	// disable tim10
	GPIOB->ODR |= (1 << 0); // enable stepper driver
}

void Driver_DeInit(void)
{
	TIM6->DIER &= ~TIM_DIER_UIE; // enable interrupt from tim6	
	TIM7->DIER &= ~TIM_DIER_UIE; // enable interrupt from tim7
	TIM10->DIER &= ~TIM_DIER_UIE; // enable interrupt from tim10
	TIM6->CR1 &= ~TIM_CR1_CEN;	// disable tim6
	TIM7->CR1 &= ~TIM_CR1_CEN;	// disable tim7
	TIM10->CR1 &= ~TIM_CR1_CEN;	// disable tim10
	GPIOB->ODR &= ~(1 << 0); // disable stepper driver
}

void CNC_FreeMoving(uint32_t x, uint32_t y)
{
	
	X_e = x - X_pos; // x-axis deviation
	Y_e = y - Y_pos; // y-axis deviation
	calculate_period(X_e, Y_e, free_speed);
	
	if(X_e < 0) { X_dir = 0; }
	else { X_dir = 1; }
	if(Y_e < 0) { Y_dir = 0; }
	else { Y_dir = 1; }

	TIM6->ARR = X_period; // X_period
	TIM7->ARR = Y_period; // Y_period
	
	
	if (X_e != 0) { TIM6->CR1 |= TIM_CR1_CEN; }	// enable tim6
	if (Y_e != 0) { TIM7->CR1 |= TIM_CR1_CEN; }	// enable tim7
	
	
	while(X_e != 0 || Y_e != 0){  //  (X_e >= 5 || X_e <= -5 || Y_e >= 5 || Y_e <= -5)
		X_e = x - X_pos; // x-axis deviation
		Y_e = y - Y_pos; // y-axis deviation	
		if (X_e == 0) { TIM6->CR1 &= ~TIM_CR1_CEN; }	// disable tim6
		if (Y_e == 0) { TIM7->CR1 &= ~TIM_CR1_CEN; }	// disable tim7
		flag = 2;
	}
}


void CNC_Init (void)
{
	Driver_Init();
	if( (GPIOB->IDR & (1 << 5)) != 0 ) { // high level GPIOE 3 - end of y
			Y_home = 0; // not home
		}	
		else{ Y_home = 1; } // at home
		if( (GPIOB->IDR & (1 << 6)) != 0 ) { // high level GPIOE 3 - end of x
			X_home = 0; // not home
		}	
		else{ X_home = 1; } // at home
		
		X_dir = 0;
		Y_dir = 0;
		X_period = 300;
		Y_period = 300;
		TIM6->ARR = X_period; // X_period
		TIM7->ARR = Y_period; // Y_period
		if(X_home != 1) { TIM6->CR1 |= TIM_CR1_CEN; }
		if(Y_home != 1) { TIM7->CR1 |= TIM_CR1_CEN; }
		
	while(X_home != 1 || Y_home != 1){
		if( (GPIOB->IDR & (1 << 5)) != 0 ) { // high level GPIOE 3 - end of y
			Y_home = 0; // not home
		}	
		else{ // at home
			Y_home = 1;
			TIM7->CR1 &= ~TIM_CR1_CEN;
		}
		
		if( (GPIOB->IDR & (1 << 6)) != 0 ) { // high level GPIOE 3 - end of x
			X_home = 0; // not home
		}	
		else{ // at home 
			X_home = 1; 
			TIM6->CR1 &= ~TIM_CR1_CEN;
		}
	}
	
	HAL_Delay(500);
	
	frame = 0;
	X_pos = 0;
	Y_pos = 0;
	flag = 0;
	ProgrammIsDone = 0;
	LoadingIsDone = 0;
	CNC_FreeMoving(7168, 14*2048);
	Driver_DeInit();
	X_pos = 0;
	Y_pos = 0;
	uint8_t request[1];  // ready-message
	
	while ( HAL_UART_Receive(&huart1, request, 1, 10) != HAL_OK ){}  // waiting request from PC
	if (request[0] == '1'){
		//Driver_Init();
		send_message('1');
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		HAL_Delay(10);
		//HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
		//Driver_Init();
	}
}

void CNC_Moving(uint32_t x, uint32_t y)
{
	
	X_e = x - X_pos; // x-axis deviation
	Y_e = y - Y_pos; // y-axis deviation
	//calculate_period(X_e, Y_e);
	
	if(X_e < 0) { X_dir = 0; }
	else { X_dir = 1; }
	if(Y_e < 0) { Y_dir = 0; }
	else { Y_dir = 1; }
//	X_dir = X_dir_buf[frame];
//	Y_dir = Y_dir_buf[frame];
	
	X_period = X_period_buf[frame];
	Y_period = Y_period_buf[frame];
	TIM6->ARR = X_period_buf[frame]; // X_period
	TIM7->ARR = Y_period_buf[frame]; // Y_period
	
	
	if (X_e != 0) { TIM6->CR1 |= TIM_CR1_CEN; }	// enable tim6
	if (Y_e != 0) { TIM7->CR1 |= TIM_CR1_CEN; }	// enable tim7
	
	
	while(X_e != 0 || Y_e != 0){  //  (X_e >= 5 || X_e <= -5 || Y_e >= 5 || Y_e <= -5)
		X_e = x - X_pos; // x-axis deviation
		Y_e = y - Y_pos; // y-axis deviation	
		if (X_e == 0) { TIM6->CR1 &= ~TIM_CR1_CEN; }	// disable tim6
		if (Y_e == 0) { TIM7->CR1 &= ~TIM_CR1_CEN; }	// disable tim7
		flag = 2;
	}
}

void compressor_on(void)
{
	HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_RESET);
	//send_message('1');  // ready-message
}

void compressor_off(void)
{
	HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);
	//send_message('1');  // ready-message
}

void CNC_DeInit(void)
{
	compressor_off();
	Driver_DeInit();
}

void Gcode_Loader(void)  // main CNC cycle
{ 
	uint8_t str[3];
	uint8_t frame_data[13];	
	
	while (LoadingIsDone == 0)
	{	
		while ( HAL_UART_Receive(&huart1, frame_data, 13, 10) != HAL_OK ){} // waiting for command from PC
		command[frame] = frame_data[0];
		switch(frame_data[0])
		{
			case '1':  //  get a coordinate command (free or work moving) G00 = '1' / G01 = '3'
					x_coord = 100000*(frame_data[1]- '0') + 10000*(frame_data[2]- '0') + 1000*(frame_data[3]- '0') + 100*(frame_data[4]- '0') + 10*(frame_data[5]- '0') + (frame_data[6]- '0'); // write a x-coord variable
					X_buf [frame] = x_coord;
					y_coord = 100000*(frame_data[7]- '0') + 10000*(frame_data[8]- '0') + 1000*(frame_data[9]- '0') + 100*(frame_data[10]- '0') + 10*(frame_data[11]- '0') + (frame_data[12]- '0'); // write a x-coord variable
					Y_buf [frame] = y_coord;
					
					calculate_period(x_coord - x_prev,y_coord - y_prev, free_speed);
					X_period_buf[frame] = X_period;
					Y_period_buf[frame] = Y_period;
			
//					if(x_coord - x_prev < 0) { X_dir = 0; }
//					else { X_dir = 1; }
//					if(y_coord - y_prev < 0) { Y_dir = 0; }
//					else { Y_dir = 1; }
//					X_dir_buf[frame] = X_dir;
//					Y_dir_buf[frame] = Y_dir;
					x_prev = x_coord;
					y_prev = y_coord;
					
					break;
						
			case '3':
					x_coord = 100000*(frame_data[1]- '0') + 10000*(frame_data[2]- '0') + 1000*(frame_data[3]- '0') + 100*(frame_data[4]- '0') + 10*(frame_data[5]- '0') + (frame_data[6]- '0'); // write a x-coord variable
					X_buf [frame] = x_coord;
					y_coord = 100000*(frame_data[7]- '0') + 10000*(frame_data[8]- '0') + 1000*(frame_data[9]- '0') + 100*(frame_data[10]- '0') + 10*(frame_data[11]- '0') + (frame_data[12]- '0'); // write a x-coord variable
					Y_buf [frame] = y_coord;
					
					calculate_period(x_coord - x_prev,y_coord - y_prev, feed_rate);
					X_period_buf[frame] = X_period;
					Y_period_buf[frame] = Y_period;
			
//					if(x_coord - x_prev < 0) { X_dir = 0; }
//					else { X_dir = 1; }
//					if(y_coord - y_prev < 0) { Y_dir = 0; }
//					else { Y_dir = 1; }
//					X_dir_buf[frame] = X_dir;
//					Y_dir_buf[frame] = Y_dir;
					x_prev = x_coord;
					y_prev = y_coord;
					break;
			case '0':
					//send_message('1'); // ready-message
					break;

			case '2':  // compressor on
					// compressor_on();
					//send_message('1'); // ready-message
					break;
			
			case '4':  // compressor off
					// compressor_off();
					//send_message('1'); // ready-message
					break;
			
			case '5':  // final of programm
					LoadingIsDone = 1;
					CNC_DeInit();
					//send_message('4'); // program is completed
					break;
			
			default:
					//send_message('5'); // error
					//CNC_DeInit();
					break;
		}
		frame++;
	}
}

void CNC_Main(void)  // main CNC cycle
{ 
	frame = 0;
	Driver_Init();
	HAL_Delay(100);
	while (ProgrammIsDone == 0)
	{	
		
		switch(command[frame])
		{
			case '1': case '3':  //  get a coordinate command (free or work moving) G00 = '1' / G01 = '3'
					CNC_Moving(X_buf[frame], Y_buf[frame]);

					break;
						
			case '0':
					//send_message('1'); // ready-message
					break;

			case '2':  // compressor on
					compressor_on();
					//send_message('1'); // ready-message
					break;
			
			case '4':  // compressor off
					compressor_off();
					//send_message('1'); // ready-message
					break;
			
			case '5':  // final of programm
					
					CNC_DeInit();
					ProgrammIsDone = 1;
					send_message('4'); // program is completed
					break;
			
			default:
					send_message('5'); // error
					CNC_DeInit();
					break;
		}
		frame++;
		
	}
}

void E_stop(void){
	TIM6->CR1 &= ~TIM_CR1_CEN;	// disable tim6
	TIM7->CR1 &= ~TIM_CR1_CEN;	// disable tim7
	IsEmerg = 1;
	Driver_DeInit();
	while (IsEmerg == 1){
		if( (GPIOB->IDR & (1 << 3)) != 0 ) { // high level GPIOE 3 - E_stop is active
			IsEmerg = 1;
		}	
		else{ IsEmerg = 0; } // E_stop is non-active
	}
	Driver_Init();
	TIM6->CR1 |= TIM_CR1_CEN;	// enable tim6
	TIM7->CR1 |= TIM_CR1_CEN;	// enable tim7
	
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
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM10_Init();
  /* USER CODE BEGIN 2 */
	//Driver_Init();
	CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; // ????????? TRACE
	DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk; // ????????? ??????? ??????
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	
		CNC_Init();
		
		Gcode_Loader();
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
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
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 83;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 500;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief TIM7 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM7_Init(void)
{

  /* USER CODE BEGIN TIM7_Init 0 */

  /* USER CODE END TIM7_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM7_Init 1 */

  /* USER CODE END TIM7_Init 1 */
  htim7.Instance = TIM7;
  htim7.Init.Prescaler = 83;
  htim7.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim7.Init.Period = 500;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim7) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim7, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM7_Init 2 */

  /* USER CODE END TIM7_Init 2 */

}

/**
  * @brief TIM10 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM10_Init(void)
{

  /* USER CODE BEGIN TIM10_Init 0 */

  /* USER CODE END TIM10_Init 0 */

  /* USER CODE BEGIN TIM10_Init 1 */

  /* USER CODE END TIM10_Init 1 */
  htim10.Instance = TIM10;
  htim10.Init.Prescaler = 83;
  htim10.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim10.Init.Period = 500;
  htim10.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim10.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim10) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM10_Init 2 */

  /* USER CODE END TIM10_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, stepX_Pin|stepY_Pin|stepZ_Pin|DirX_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, DirY_Pin|DirZ_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Enable_GPIO_Port, Enable_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : stepX_Pin stepY_Pin stepZ_Pin DirX_Pin */
  GPIO_InitStruct.Pin = stepX_Pin|stepY_Pin|stepZ_Pin|DirX_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : DirY_Pin DirZ_Pin */
  GPIO_InitStruct.Pin = DirY_Pin|DirZ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : Enable_Pin Air_Pin */
  GPIO_InitStruct.Pin = Enable_Pin|Air_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : E_stop_Pin */
  GPIO_InitStruct.Pin = E_stop_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(E_stop_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : END_sense_Y_Pin END_sense_X_Pin */
  GPIO_InitStruct.Pin = END_sense_Y_Pin|END_sense_X_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
