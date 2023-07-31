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
		16800,17006,17212,17418,17624,17830,18036,18241,18447,18652,18856,19061,19265,19469,19672,19875,
20078,20279,20481,20682,20882,21082,21281,21479,21677,21874,22070,22265,22460,22653,22846,23038,
23229,23419,23608,23796,23983,24169,24353,24537,24719,24901,25081,25259,25437,25613,25788,25961,
26134,26304,26474,26641,26808,26973,27136,27298,27458,27616,27773,27929,28082,28234,28384,28533,
28679,28824,28967,29109,29248,29385,29521,29655,29787,29916,30044,30170,30294,30416,30535,30653,
30769,30882,30994,31103,31210,31315,31417,31518,31616,31712,31806,31898,31987,32074,32159,32241,
32321,32399,32474,32547,32618,32686,32752,32816,32877,32935,32991,33045,33097,33145,33192,33236,
33277,33316,33353,33387,33418,33447,33474,33498,33519,33538,33554,33568,33580,33589,33595,33599,
33600,33599,33595,33589,33580,33568,33554,33538,33519,33498,33474,33447,33418,33387,33353,33316,
33277,33236,33192,33145,33097,33045,32991,32935,32877,32816,32752,32686,32618,32547,32474,32399,
32321,32241,32159,32074,31987,31898,31806,31712,31616,31518,31417,31315,31210,31103,30994,30882,
30769,30653,30535,30416,30294,30170,30044,29916,29787,29655,29521,29385,29248,29109,28967,28824,
28679,28533,28384,28234,28082,27929,27773,27616,27458,27298,27136,26973,26808,26641,26474,26304,
26134,25961,25788,25613,25437,25259,25081,24901,24719,24537,24353,24169,23983,23796,23608,23419,
23229,23038,22846,22653,22460,22265,22070,21874,21677,21479,21281,21082,20882,20682,20481,20279,
20078,19875,19672,19469,19265,19061,18856,18652,18447,18241,18036,17830,17624,17418,17212,17006,
16800,16594,16388,16182,15976,15770,15564,15359,15153,14948,14744,14539,14335,14131,13928,13725,
13522,13321,13119,12918,12718,12518,12319,12121,11923,11726,11530,11335,11140,10947,10754,10562,
10371,10181,9992,9804,9617,9431,9247,9063,8881,8699,8519,8341,8163,7987,7812,7639,
7466,7296,7126,6959,6792,6627,6464,6302,6142,5984,5827,5671,5518,5366,5216,5067,
4921,4776,4633,4491,4352,4215,4079,3945,3813,3684,3556,3430,3306,3184,3065,2947,
2831,2718,2606,2497,2390,2285,2183,2082,1984,1888,1794,1702,1613,1526,1441,1359,
1279,1201,1126,1053,982,914,848,784,723,665,609,555,503,455,408,364,
323,284,247,213,182,153,126,102,81,62,46,32,20,11,5,1,
0,1,5,11,20,32,46,62,81,102,126,153,182,213,247,284,
323,364,408,455,503,555,609,665,723,784,848,914,982,1053,1126,1201,
1279,1359,1441,1526,1613,1702,1794,1888,1984,2082,2183,2285,2390,2497,2606,2718,
2831,2947,3065,3184,3306,3430,3556,3684,3813,3945,4079,4215,4352,4491,4633,4776,
4921,5067,5216,5366,5518,5671,5827,5984,6142,6302,6464,6627,6792,6959,7126,7296,
7466,7639,7812,7987,8163,8341,8519,8699,8881,9063,9247,9431,9617,9804,9992,10181,
10371,10562,10754,10947,11140,11335,11530,11726,11923,12121,12319,12518,12718,12918,13119,13321,
13522,13725,13928,14131,14335,14539,14744,14948,15153,15359,15564,15770,15976,16182,16388,16594};


	
uint8_t flag = 0; // useles
uint32_t feed_rate = 5000; // dont use
uint32_t X_period = 500;
uint32_t Y_period = 500;
uint32_t f = 1000000; // dont use
int32_t X_e;
int32_t Y_e;

uint8_t command [5000];
uint32_t X_buf [5000];
uint32_t Y_buf [5000];
uint16_t frame = 0;

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
	
		X_duty_A *= 50;
		X_duty_A /= X_period;
		X_duty_B *= 50;
		X_duty_B /= X_period;
		X_duty_C *= 50;
		X_duty_C /= X_period;
				
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
		
		Y_duty_A *= 75;
		Y_duty_A /= Y_period;
		Y_duty_B *= 75;
		Y_duty_B /= Y_period;
		Y_duty_C *= 75;
		Y_duty_C /= Y_period;
		
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
	fdt = fdt * 1000000 / 5000; // fdt = f * sqrtf(x*x + y*y) / feed_rate;
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
	
	TIM7->DIER |= TIM_DIER_UIE; // enable interrupt from tim6

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
	calculate_period(X_e, Y_e);
	
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
	
	

	flag = 1;
}


uint8_t CNC_Frame(uint32_t x, uint32_t y){
	HAL_Delay(500);
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
					
					//CNC_Frame(x_coord, y_coord);
					//CNC_Moving(x_coord, y_coord);
					//send_message('1'); // ready-message
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
	while (ProgrammIsDone == 0)
	{	
		DWT->CYCCNT = 0; // Обнуляем счетчик
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
		cycles_count = DWT->CYCCNT; // Читаем счетчик тактов
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
	X_pos = 0;
	Y_pos = 0;
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
  htim1.Init.Period = 33600;
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
  htim8.Init.Period = 33600;
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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Air_GPIO_Port, Air_Pin, GPIO_PIN_SET);

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
