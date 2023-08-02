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
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;

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

uint32_t X_duty_A;
uint32_t X_duty_B;
uint32_t X_duty_C;
uint32_t Y_duty_A;
uint32_t Y_duty_B;
uint32_t Y_duty_C;

uint16_t X_step_A = 0;
uint16_t X_step_B = 171;
uint16_t X_step_C = 341;

uint16_t Y_step_A = 0;
uint16_t Y_step_B = 171;
uint16_t Y_step_C = 341;

uint8_t X_dir = 1;
uint8_t Y_dir = 1;
int32_t X_pos = 0;
int32_t Y_pos = 0;



uint16_t sine_LookUp[] = {
		8400,8503,8606,8709,8812,8915,9018,9121,9223,9326,9428,9530,9633,9734,9836,9938,
10039,10140,10240,10341,10441,10541,10640,10740,10838,10937,11035,11133,11230,11327,11423,11519,
11615,11710,11804,11898,11991,12084,12177,12269,12360,12450,12540,12630,12718,12807,12894,12981,
13067,13152,13237,13321,13404,13486,13568,13649,13729,13808,13887,13964,14041,14117,14192,14266,
14340,14412,14484,14554,14624,14693,14761,14827,14893,14958,15022,15085,15147,15208,15268,15327,
15384,15441,15497,15551,15605,15657,15709,15759,15808,15856,15903,15949,15994,16037,16079,16121,
16161,16199,16237,16274,16309,16343,16376,16408,16438,16468,16496,16523,16548,16573,16596,16618,
16639,16658,16676,16693,16709,16724,16737,16749,16760,16769,16777,16784,16790,16794,16797,16799,
16800,16799,16797,16794,16790,16784,16777,16769,16760,16749,16737,16724,16709,16693,16676,16658,
16639,16618,16596,16573,16548,16523,16496,16468,16438,16408,16376,16343,16309,16274,16237,16199,
16161,16121,16079,16037,15994,15949,15903,15856,15808,15759,15709,15657,15605,15551,15497,15441,
15384,15327,15268,15208,15147,15085,15022,14958,14893,14827,14761,14693,14624,14554,14484,14412,
14340,14266,14192,14117,14041,13964,13887,13808,13729,13649,13568,13486,13404,13321,13237,13152,
13067,12981,12894,12807,12718,12630,12540,12450,12360,12269,12177,12084,11991,11898,11804,11710,
11615,11519,11423,11327,11230,11133,11035,10937,10838,10740,10640,10541,10441,10341,10240,10140,
10039,9938,9836,9734,9633,9530,9428,9326,9223,9121,9018,8915,8812,8709,8606,8503,
8400,8297,8194,8091,7988,7885,7782,7679,7577,7474,7372,7270,7167,7066,6964,6862,
6761,6660,6560,6459,6359,6259,6160,6060,5962,5863,5765,5667,5570,5473,5377,5281,
5185,5090,4996,4902,4809,4716,4623,4531,4440,4350,4260,4170,4082,3993,3906,3819,
3733,3648,3563,3479,3396,3314,3232,3151,3071,2992,2913,2836,2759,2683,2608,2534,
2460,2388,2316,2246,2176,2107,2039,1973,1907,1842,1778,1715,1653,1592,1532,1473,
1416,1359,1303,1249,1195,1143,1091,1041,992,944,897,851,806,763,721,679,
639,601,563,526,491,457,424,392,362,332,304,277,252,227,204,182,
161,142,124,107,91,76,63,51,40,31,23,16,10,6,3,1,
0,1,3,6,10,16,23,31,40,51,63,76,91,107,124,142,
161,182,204,227,252,277,304,332,362,392,424,457,491,526,563,601,
639,679,721,763,806,851,897,944,992,1041,1091,1143,1195,1249,1303,1359,
1416,1473,1532,1592,1653,1715,1778,1842,1907,1973,2039,2107,2176,2246,2316,2388,
2460,2534,2608,2683,2759,2836,2913,2992,3071,3151,3232,3314,3396,3479,3563,3648,
3733,3819,3906,3993,4082,4170,4260,4350,4440,4531,4623,4716,4809,4902,4996,5090,
5185,5281,5377,5473,5570,5667,5765,5863,5962,6060,6160,6259,6359,6459,6560,6660,
6761,6862,6964,7066,7167,7270,7372,7474,7577,7679,7782,7885,7988,8091,8194,8297};


	
uint8_t flag = 0; // useles
uint32_t feed_rate = 3000; 
uint32_t X_period = 500;
uint32_t Y_period = 500;
uint32_t f = 1000000; // dont use
int32_t X_e;
int32_t Y_e;

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
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM6_Init(void);
static void MX_TIM7_Init(void);
static void MX_TIM8_Init(void);
/* USER CODE BEGIN PFP */

void X_driver(void)
{		
		if (X_dir == 1){
				X_step_A++;
				X_step_B++;
				X_step_C++;
				X_pos++;
			
				if (X_step_A >= 512) {X_step_A = 0;}
				if (X_step_B >= 512) {X_step_B = 0;}
				if (X_step_C >= 512) {X_step_C = 0;}
		}
		else {
				X_step_A--;
				X_step_B--;
				X_step_C--;
				X_pos--;
			
				if (X_step_A >= 512) {X_step_A = 511;}
				if (X_step_B >= 512) {X_step_B = 511;}
				if (X_step_C >= 512) {X_step_C = 511;}
		}
			
		if (X_step_A >= 512) {X_step_A = 0;}
		if (X_step_B >= 512) {X_step_B = 0;}
		if (X_step_C >= 512) {X_step_C = 0;}
		
		X_duty_A = sine_LookUp[X_step_A];
		X_duty_B = sine_LookUp[X_step_B];
		X_duty_C = sine_LookUp[X_step_C];
		
		
		X_duty_A *= X_factor;
		X_duty_B *= X_factor;
		X_duty_C *= X_factor;
		if(X_period_buf[frame] <= 700){
			X_duty_A /= X_period_buf[frame];
			X_duty_B /= X_period_buf[frame];
			X_duty_C /= X_period_buf[frame];
		}
		else{
			X_duty_A /= 700;
			X_duty_B /= 700;
			X_duty_C /= 700;
		}
				
		TIM1 -> CCR1 = X_duty_A;
		TIM1 -> CCR2 = X_duty_B;
		TIM1 -> CCR3 = X_duty_C;
}

void Y_driver(void)
{
		if (Y_dir == 1){
				Y_step_A++;
				Y_step_B++;
				Y_step_C++;
				Y_pos++;
			
				if (Y_step_A >= 512) {Y_step_A = 0;}
				if (Y_step_B >= 512) {Y_step_B = 0;}
				if (Y_step_C >= 512) {Y_step_C = 0;}
		}
		else {
				Y_step_A--;
				Y_step_B--;
				Y_step_C--;
				Y_pos--;
			
				if (Y_step_A >= 512) {Y_step_A = 511;}
				if (Y_step_B >= 512) {Y_step_B = 511;}
				if (Y_step_C >= 512) {Y_step_C = 511;}
		}
		
		Y_duty_A = sine_LookUp[Y_step_A];
		Y_duty_B = sine_LookUp[Y_step_B];
		Y_duty_C = sine_LookUp[Y_step_C];
		
		Y_duty_A *= Y_factor;
		Y_duty_B *= Y_factor;
		Y_duty_C *= Y_factor;
		if(Y_period_buf[frame] <= 700){
			Y_duty_A /= Y_period_buf[frame];
			Y_duty_B /= Y_period_buf[frame];
			Y_duty_C /= Y_period_buf[frame];
		}
		else{
			Y_duty_A /= 700;
			Y_duty_B /= 700;
			Y_duty_C /= 700;
		}
		
		TIM8 -> CCR1 = Y_duty_A;
		TIM8 -> CCR2 = Y_duty_B;
		TIM8 -> CCR3 = Y_duty_C;
}

static inline void calculate_period(int32_t x, int32_t y)
{
	float fdt;
	if(x < 0){ x*= -1;}
	if(y < 0){ y*= -1;}
	fdt = hypot(x, y);
	fdt = fdt * 1000000 / feed_rate; // fdt = f * sqrtf(x*x + y*y) / feed_rate;
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
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3); // turn on complementary channel
	
	TIM6->DIER |= TIM_DIER_UIE; // enable interrupt from tim6
	
	
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_2); // turn on complementary channel
	HAL_TIM_PWM_Start(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim8, TIM_CHANNEL_3); // turn on complementary channel
	
	TIM7->DIER |= TIM_DIER_UIE; // enable interrupt from tim7

	TIM6->CR1 &= ~TIM_CR1_CEN;	// disable tim6
	TIM7->CR1 &= ~TIM_CR1_CEN;	// disable tim7
}

void Driver_DeInit(void)
{
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_2); // turn on complementary channel
	HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim1, TIM_CHANNEL_3); // turn on complementary channel
	
	TIM6->DIER &= ~TIM_DIER_UIE; // enable interrupt from tim6
	
	
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_1); // turn on complementary channel
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_2); // turn on complementary channel
	HAL_TIM_PWM_Stop(&htim8, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Stop(&htim8, TIM_CHANNEL_3); // turn on complementary channel
	
	TIM7->DIER &= ~TIM_DIER_UIE; // enable interrupt from tim6

	TIM6->CR1 &= ~TIM_CR1_CEN;	// disable tim6
	TIM7->CR1 &= ~TIM_CR1_CEN;	// disable tim7
}

void CNC_Init (void)
{
	frame = 0;
	X_pos = 0;
	Y_pos = 0;
	flag = 0;
	ProgrammIsDone = 0;
	LoadingIsDone = 0;
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
	
	TIM6->ARR = X_period_buf[frame]; // X_period
	TIM7->ARR = Y_period_buf[frame]; // Y_period
	
	if (X_e != 0) { TIM6->CR1 |= TIM_CR1_CEN; }	// enable tim6
	if (Y_e != 0) { TIM7->CR1 |= TIM_CR1_CEN; }	// enable tim7
	
	
	while(X_e != 0 || Y_e != 0){  //  (X_e >= 5 || X_e <= -5 || Y_e >= 5 || Y_e <= -5)
		DWT->CYCCNT = 0; // Iaioeyai n?ao?ee
		X_e = x - X_pos; // x-axis deviation
		Y_e = y - Y_pos; // y-axis deviation	
		if (X_e == 0) { TIM6->CR1 &= ~TIM_CR1_CEN; }	// disable tim6
		if (Y_e == 0) { TIM7->CR1 &= ~TIM_CR1_CEN; }	// disable tim7
		flag = 2;
		cycles_count = DWT->CYCCNT; // ?eoaai n?ao?ee oaeoia
	}
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
			case '1': case '3':  //  get a coordinate command (free or work moving) G00 = '1' / G01 = '3'
					x_coord = 100000*(frame_data[1]- '0') + 10000*(frame_data[2]- '0') + 1000*(frame_data[3]- '0') + 100*(frame_data[4]- '0') + 10*(frame_data[5]- '0') + (frame_data[6]- '0'); // write a x-coord variable
					X_buf [frame] = x_coord;
					y_coord = 100000*(frame_data[7]- '0') + 10000*(frame_data[8]- '0') + 1000*(frame_data[9]- '0') + 100*(frame_data[10]- '0') + 10*(frame_data[11]- '0') + (frame_data[12]- '0'); // write a x-coord variable
					Y_buf [frame] = y_coord;
					
					calculate_period(x_coord - x_prev,y_coord - y_prev);
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
					//send_message('4'); // program is completed
					break;
			
			default:
					send_message('5'); // error
					CNC_DeInit();
					break;
		}
		frame++;
		
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
  MX_TIM1_Init();
  MX_TIM6_Init();
  MX_TIM7_Init();
  MX_TIM8_Init();
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
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 16800;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */
  HAL_TIM_MspPostInit(&htim1);

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
  * @brief TIM8 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM8_Init(void)
{

  /* USER CODE BEGIN TIM8_Init 0 */

  /* USER CODE END TIM8_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig = {0};

  /* USER CODE BEGIN TIM8_Init 1 */

  /* USER CODE END TIM8_Init 1 */
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 0;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 16800;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 50;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM8_Init 2 */

  /* USER CODE END TIM8_Init 2 */
  HAL_TIM_MspPostInit(&htim8);

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);

  /*Configure GPIO pins : END_sense_X_Pin END_sense_Y_Pin */
  GPIO_InitStruct.Pin = END_sense_X_Pin|END_sense_Y_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : Air_Pin */
  GPIO_InitStruct.Pin = Air_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Air_GPIO_Port, &GPIO_InitStruct);

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
