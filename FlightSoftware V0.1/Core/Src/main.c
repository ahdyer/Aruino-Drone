/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"

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
I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim12;

UART_HandleTypeDef huart3;
DMA_HandleTypeDef hdma_usart3_rx;

/* USER CODE BEGIN PV */
// RC Receiver Channels
// Array to store all Channels
int16_t channel[10];
// Channel[0] = Roll
// Channel[1] = Pitch
// Channel[2] = Throttle
// Channel[3] = Yaw
// Channel[4] = Arm Switch
// Channel[5] = Mode Switch

// PID Variables
float max_rotate = 200;
//X - Pitch | Y - Roll | Z - Yaw
// Roll
float pid_p_roll = 1;
float pid_i_roll = 0;
float pid_d_roll = 10;
float pid_gy_roll = 0, pid_setpoint_roll = 0, pid_output_roll = 0, pid_prev_err_roll = 0, pid_integ_roll = 0;
float roll_al = 0; // autolevel for roll
// Pitch
// uses same PID values as roll
float pid_gy_pitch = 0, pid_setpoint_pitch = 0, pid_output_pitch = 0, pid_prev_err_pitch = 0, pid_integ_pitch = 0;
float pitch_al = 0; // autolevel for pitch
// Yaw
float pid_p_yaw = 1;
float pid_i_yaw = 0.01;
float pid_d_yaw = 0;
float pid_gy_yaw = 0, pid_setpoint_yaw = 0, pid_output_yaw = 0, pid_prev_err_yaw = 0, pid_integ_yaw = 0;

uint16_t arm = 0;

uint16_t esc_1, esc_2, esc_3, esc_4;
uint16_t motor_idle = 1100;


// HC-SR04 Variables
uint16_t echo_edge_1 = 0;
uint16_t echo_edge_2 = 0;
float height = 0;
uint8_t capture_num = 0;
// speed of sound in cm / uS divided by 2 as we get round trip distance
const float time_to_dist = 0.0343/2;

enum flight_states{
	error,
	idle,
	arming,
	rate_mode,
	auto_level,
	altitude_hold
};

enum flight_states flight_state = idle;



// Flags
// Flag to show both edges of the HC-SR04 echo have been measured
uint8_t sr_rdy_flg = 0;
// Safety Flags
uint8_t fault_flag = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_I2C2_Init(void);
static void MX_TIM1_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM12_Init(void);
static void MX_TIM2_Init(void);
/* USER CODE BEGIN PFP */
float ABS(float x);

HAL_StatusTypeDef ICM20689_Read(uint8_t address, float *acc_x, float *acc_y, float *acc_z, float *temp,
								float *gy_x, float *gy_y, float *gy_z, float gy_cal_x, float gy_cal_y,
								float gy_cal_z, float acc_cal_x, float acc_cal_y, float acc_cal_z);

HAL_StatusTypeDef ICM20689_Config(uint8_t address, float *gy_cal_x, float *gy_cal_y, float *gy_cal_z, float *acc_cal_x, float *acc_cal_y, float *acc_cal_z);

void Update_Motors();

void Calculate_PIDs();

void Calculate_SetPoints();

void HC_SR04_Read();

void uSec_Delay(uint32_t uSec);

void Check_Faults();
//void Calculate_Angles(float acc_x, float acc_y, float acc_z, float gy_x, float gy_y, float gy_z, float *roll, float *pitch, float *yaw)
//{
//	//Function variables
//	// gyroscope and accelerometer temporary angles
//
//	// initalisation counter
//	uint8_t count = 0;
//
//
//	if (count == 0){
//		count++;
//		gy_roll = 0, gy_pitch = 0, gy_yaw = 0;
//		acc_roll = 0, acc_pitch = 0, acc_yaw = 0;
//		pid_gy_roll = 0, pid_gy_pitch = 0, pid_gy_yaw = 0;
//	}
//
//
//
//
//	// transform pitch and roll depending on yaw value
//
//	//Calculate the traveled angles from the accelerometer
//
//	//Complementary filter on angles to correct gyroscope drift
//
//
//}
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
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_TIM1_Init();
  MX_USART3_UART_Init();
  MX_TIM4_Init();
  MX_TIM12_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */



  // ICM20689 variables
	const uint8_t icm20689_address = 0x68;
	float acc_x, acc_y, acc_z;
	float temp;
	float gy_x, gy_y, gy_z;
	// Calibration variables
	float gy_cal_x = 0, gy_cal_y = 0, gy_cal_z = 0;
	float acc_cal_x = 0, acc_cal_y = 0, acc_cal_z = 0;
	// Angle Variables
	//X - Pitch | Y - Roll | Z - Yaw
	float pitch = 0, roll = 0, yaw = 0;
	float pitch_t = 0, roll_t = 0, yaw_t = 0;
	float gy_roll = 0, gy_pitch = 0, gy_yaw = 0;
	float acc_roll = 0, acc_pitch = 0, acc_yaw = 0;
	float acc_tot_vect = 0;

	// MS5611
	const uint8_t ms5611_address = 0x77;

	uint16_t looptimmer = 0;
	uint8_t error_counter = 0;
	uint8_t armed = 0;

	uint32_t loopcounter = 0;

	//UART DMA variables
	uint8_t rx_buff[32] = {0};

	// Start DMA UART RX
	HAL_StatusTypeDef uart_status;
	uart_status = HAL_UART_Receive_DMA(&huart3, rx_buff, 32);
	// Check for errors in starting the UART


  HAL_Delay(1000); // Wait for the icm20689 to initialise
  while(rx_buff[0] != 0x20){
	  HAL_GPIO_TogglePin(GPIOC, r_led_Pin);
	  HAL_Delay(500);
  }
  ICM20689_Config(icm20689_address, &gy_cal_x, &gy_cal_y, &gy_cal_z, &acc_cal_x, &acc_cal_y, &acc_cal_z); // configure ICM power register and calibrate the gyroscope

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

  // Reset both status lights
  HAL_GPIO_WritePin(GPIOC, o_led_Pin, RESET);
  HAL_GPIO_WritePin(GPIOC, r_led_Pin, RESET);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  // Loop timer setup
    TIM1->CR1 |= 1<<0;
    while (!(TIM1->SR & 1<<0));
    TIM1->CNT = 0;


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  // Increment loop counter
	  loopcounter ++;

	  ICM20689_Read(icm20689_address, &acc_x, &acc_y, &acc_z, &temp, &gy_x, &gy_y, &gy_z, gy_cal_x, gy_cal_y, gy_cal_z, acc_cal_x, acc_cal_y, acc_cal_z);

	  //Complementary filter applied to gyroscope values
	  pid_gy_roll = (pid_gy_roll*0.7) + (gy_y *0.3);
	  pid_gy_pitch = (pid_gy_pitch*0.7) + (gy_x *0.3);
	  pid_gy_yaw = (pid_gy_yaw*0.7) + (gy_z *0.3);

	  //Calculate the travelled angles from the gyroscope
	  //loop every 1 / 250hz = 0.004
      roll = roll + (gy_y / 250);
	  pitch = pitch + (gy_x / 250);
	  gy_yaw = gy_yaw + (gy_z / 250);

	  //need to rotate pitch and roll by the sin of yaw
	  roll += (pitch * (sin(gy_z * 0.00006981)));
	  pitch -= roll * (sin(gy_z * 0.00006981));


	  // Calculate acceleration vectors
	  acc_tot_vect = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));
	  if (ABS(acc_y) < acc_tot_vect){
		  acc_pitch = asin(acc_y/acc_tot_vect) * 57.288;
	  }
	  if (ABS(acc_x) < acc_tot_vect){
	  		  acc_roll = asin(acc_x/acc_tot_vect) * -57.288; // negative to ensure left wing up is +roll
	  	  }

	  roll = (roll*0.9995) + (acc_roll*0.0005);
	  pitch = (pitch*0.99955) + (acc_pitch*0.0005);
	  yaw = gy_yaw;



	  roll_al = roll * 3;
	  pitch_al = pitch * 3;

	 // HC_SR04_Read();

	  Calculate_SetPoints();
	  Calculate_PIDs();

	  // function to read in values stored in the ESP32 UART DMA

	  //  Function to check fault flags
	  //Check_Faults();

	  // check to see if arm switch is disarmed
	  if (channel[4] < 1500 || channel[4] == 0){
		  flight_state = idle;
		  armed = 0;
	  }

	  // State machine for the drone switch case
	  switch (flight_state){
	  case(idle):
			// Ensure motors are not running
			TIM4->CCR1 = 0;
			TIM4->CCR2 = 0;
			TIM4->CCR3 = 0;
			TIM4->CCR4 = 0;
			// Wait for receiver connection
			if (rx_buff[0] == 0x20 & armed == 0 & (channel[4] > 1500) & (channel[2] < 1030)){
				flight_state = arming;
			}
			// need to check this is the right channel
			else if (channel[5] > 1600 & armed == 1 & channel[4] > 1500){
				 flight_state = rate_mode;
			 }
	  	  	// controller runs 250 times a second so mod 125 so the LED flashes every 500ms
	  	  	// blink Top orange LED
			if(loopcounter % 125 == 0){
				HAL_GPIO_TogglePin(GPIOC, o_led_Pin);
			}
		  break;

	  case(arming):
		  // send 1000us ESC pulses
		  if (armed == 0){
			  TIM4->CCR1 = 1000;
			  TIM4->CCR2 = 1000;
			  TIM4->CCR3 = 1000;
			  TIM4->CCR4 = 1000;
			  HAL_Delay(3000);
			  armed ++;
			  flight_state = idle;
	      }

		  break;

	  case(rate_mode):
		  Update_Motors();
		  break;

	  case(auto_level):
		  break;

	  case(altitude_hold):
		  break;

	  case(error):
		  // Alternately blink orange and red LED
		  if(loopcounter % 125 == 0){
			  HAL_GPIO_WritePin(GPIOC, o_led_Pin, RESET);
			  HAL_GPIO_WritePin(GPIOC, o_led_Pin, SET);
		  }
	      if(loopcounter % 75 == 0){
	    	  HAL_GPIO_WritePin(GPIOC, r_led_Pin, SET);
	    	  HAL_GPIO_WritePin(GPIOC, r_led_Pin, RESET);
	      }
		  // Ensure motors are not running
		  TIM4->CCR1 = 0;
		  TIM4->CCR2 = 0;
		  TIM4->CCR3 = 0;
		  TIM4->CCR4 = 0;

	      break;
	  }

	  // ensure the loop happens every 4ms
	  while(TIM1->CNT < 4000){
	  };
	  looptimmer = TIM1->CNT;
	  if (looptimmer > 4300){
		  error_counter ++;
		  if(error_counter > 3){
			  //flight_state = error;
		  }
	  }
	  TIM1->CNT = 0;

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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
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
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.ClockSpeed = 400000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 168-1;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 84-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 84-1;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 5000;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM12 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM12_Init(void)
{

  /* USER CODE BEGIN TIM12_Init 0 */

  /* USER CODE END TIM12_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM12_Init 1 */

  /* USER CODE END TIM12_Init 1 */
  htim12.Instance = TIM12;
  htim12.Init.Prescaler = 84-1;
  htim12.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim12.Init.Period = 65535;
  htim12.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim12, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_IC_Init(&htim12) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim12, &sConfigIC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM12_Init 2 */

  /* USER CODE END TIM12_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, o_led_Pin|r_led_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : o_led_Pin r_led_Pin */
  GPIO_InitStruct.Pin = o_led_Pin|r_led_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : trig_Pin */
  GPIO_InitStruct.Pin = trig_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(trig_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

float ABS(float x){
	if(x > 0){
		return x;
	}
	else{
		return (-x);
	}

}

HAL_StatusTypeDef ICM20689_Read(uint8_t address, float *acc_x, float *acc_y, float *acc_z, float *temp,
		float *gy_x, float *gy_y, float *gy_z, float gy_cal_x, float gy_cal_y, float gy_cal_z, float acc_cal_x, float acc_cal_y, float acc_cal_z)
{
	HAL_StatusTypeDef ret;
	// Bytes to store the incoming accelerometer data
	uint8_t acc_data[6];
	int16_t acc_s_data[3];

	// Bytes to store the incoming temperature data
	uint8_t temp_data[2];
	int16_t temp_s_data;

	// Bytes to store the incoming gyroscope data
	uint8_t gy_data[6];
	int16_t gy_s_data[3];

	// Reading Accelerometers registers
	ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(address<<1)| 0x01, 0x3B , 1, acc_data, 6, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(address<<1)| 0x01, 0x41 , 1, temp_data, 2, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(address<<1)| 0x01, 0x43 , 1, gy_data, 6, 50);
	if(ret != HAL_OK)
	{
		return ret;
	}

	// Shift and combine the 2 accelerometer registers per axis and store them in a signed 16 int
	acc_s_data[0] = (acc_data[0]<<8 | acc_data[1]);
	acc_s_data[1] = (acc_data[2]<<8 | acc_data[3]);
	acc_s_data[2] = (acc_data[4]<<8 | acc_data[5]);

	// Normalise the accelerations by the sensitivity/g
	*acc_x =  (acc_s_data[0] / 4096.0) - acc_cal_x;
	*acc_y = (acc_s_data[1] / 4096.0) - acc_cal_y;
	*acc_z = (acc_s_data[2] / 4096.0) - acc_cal_z ;

	// Shift and combine the 2 temperature registers and store them in a signed 16 int
	temp_s_data = (temp_data[0]<<8 | temp_data[1]);

	// Normalise the temperature values
	*temp = ((temp_s_data - 0)/326.8) + 25;

	// Shift and combine the 2 gyroscope registers per axis and store them in a signed 16 int
	gy_s_data[0] = (gy_data[0]<<8 | gy_data[1]);
	gy_s_data[1] = (gy_data[2]<<8 | gy_data[3]);
	gy_s_data[2] = (gy_data[4]<<8 | gy_data[5]);

	// Normalise the gyroscope by the sensitivity/g
	*gy_x =  ((gy_s_data[0] /32.8 ) - gy_cal_x);
	*gy_y = ((gy_s_data[1] / 32.8 ) - gy_cal_y);
	*gy_z = ((gy_s_data[2] / 32.8 ) + gy_cal_z) * -1; // Invert Z axis / yaw to match convention (Nose right = positive yaw)


	return HAL_OK;
}

HAL_StatusTypeDef ICM20689_Config(uint8_t address, float *gy_cal_x, float *gy_cal_y, float *gy_cal_z, float *acc_cal_x, float *acc_cal_y, float *acc_cal_z)
{
	HAL_StatusTypeDef ret;
	uint8_t config_pwr = 0x81;
	uint8_t config_pwr1 = 0x00;
	uint8_t whoami;
	uint8_t config = 0x10; // configure gyroscope and accelerometer to full scale
	uint8_t config_lowpass = 0x03;
	uint8_t rst = 0x01;
	uint16_t i; //calibration counter

	// temporary ICM20689 variables
	float t_acc_x, t_acc_y, t_acc_z;
	float t_temp;
	float t_gy_x, t_gy_y, t_gy_z;

	// gyroscope errors
	float gy_er_x = 0, gy_er_y = 0, gy_er_z = 0;

	// accelerometer errors
	float acc_er_x = 0, acc_er_y = 0, acc_er_z = 0;



	// Soft Reset
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x6B, 1, &config_pwr, 1, 50);
	if(ret != HAL_OK){
		return ret;
	}
	HAL_Delay(100);

	// Wake up
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x6B, 1, &config_pwr1, 1, 50);
	if(ret != HAL_OK){
		return ret;
	}
	HAL_Delay(100);

	// Check to see if icm20689 is responding correctly
	ret = HAL_I2C_Mem_Read(&hi2c2, (uint16_t)(address<<1)|  0x01 , 0x75, 1, &whoami, 1, 50);
	if(ret != HAL_OK){
		return ret;
	}
	if(whoami == 0x98){
		HAL_GPIO_TogglePin(GPIOC, r_led_Pin);
		HAL_Delay(500);
		HAL_GPIO_TogglePin(GPIOC, r_led_Pin);
	}

	// Configure the accelerometer
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x1C, 1, &config, 1, 50);
	if(ret != HAL_OK){
		return ret;
	}

	// Configure the accelerometer Filter
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x1D, 1, &config_lowpass, 1, 50);
			if(ret != HAL_OK){
				return ret;
				}

	// Configure the gyroscope
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x1B, 1, &config, 1, 50);
		if(ret != HAL_OK){
			return ret;
		}

	// Configure the gyroscope Filter
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x1A, 1, &config_lowpass, 1, 50);
	if(ret != HAL_OK){
		return ret;
		}

	// Reset gyroscope and accelerometer registers
	ret = HAL_I2C_Mem_Write(&hi2c2, (uint16_t)(address<<1), 0x6A, 1, &rst, 1, 50);
	if(ret != HAL_OK){
		return ret;
		}


	for (i=0; i < 2000; i++)
	{
		ICM20689_Read(address, &t_acc_x, &t_acc_y, &t_acc_z, &t_temp, &t_gy_x, &t_gy_y, &t_gy_z, 0, 0, 0, 0, 0, 0);
		gy_er_x += t_gy_x;
		gy_er_y += t_gy_y;
		gy_er_z += t_gy_z;
		acc_er_x += t_acc_x;
		acc_er_y += t_acc_y;
		acc_er_z += t_acc_z;
		HAL_Delay(4);
		if (i % 125 == 0){
			HAL_GPIO_TogglePin(GPIOC, o_led_Pin);
		}
	}
	// store average calibration;
	*gy_cal_x = gy_er_x / 2000;
	*gy_cal_y = gy_er_y / 2000;
	*gy_cal_z = gy_er_z / 2000;
	*acc_cal_x = acc_er_x / 2000;
	*acc_cal_y = acc_er_y / 2000;
	*acc_cal_z = (acc_er_z - 2000) / 2000;

	return HAL_OK;

}

void Update_Motors(){
	// Variable to store throttle value
	uint16_t throttle = channel[2];

	if (throttle > 1800){throttle = 1800;} //limit max throttle to allow for some room for the PIDs to still correct.

	// ESC1 | CCR1 | Front Right | CW
	// ESC2 | CCR2 | Rear Right  | CCW
	// ESC3 | CCR3 | Front Left  | CCW
	// ESC4 | CCR4 | Rear Left   | CW
	// Channel[2] = Throttle

	// test difference between ESC variables being floats and ints

	esc_1 = throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;
	esc_2 = throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;
	esc_3 = throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;
	esc_4 = throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;

	if (esc_1 > 2000){esc_1 = 2000;}
	//if (esc_1 < motor_idle){esc_1 = motor_idle;}

	if (esc_2 > 2000){esc_2 = 2000;}
	//if (esc_2 < motor_idle){esc_2 = motor_idle;}

	if (esc_3 > 2000){esc_3 = 2000;}
	//if (esc_3 < motor_idle){esc_3 = motor_idle;}

	if (esc_4 > 2000){esc_4 = 2000;}
	//if (esc_4 < motor_idle){esc_4 = motor_idle;}

	TIM4->CCR1 = esc_1;
	TIM4->CCR2 = esc_2;
	TIM4->CCR3 = esc_3;
	TIM4->CCR4 = esc_4;


}

void Calculate_PIDs(){

	//Roll - Y - Channel[0]
	float temp_err = 0;
	temp_err = pid_gy_roll - pid_setpoint_roll;
	pid_integ_roll += pid_i_roll * temp_err;
	if (pid_integ_roll > max_rotate){pid_integ_roll = max_rotate;}
	if (pid_integ_roll < (max_rotate*-1)){pid_integ_roll = (max_rotate*-1);}

	pid_output_roll = (pid_p_roll * temp_err) + (pid_integ_roll) + (pid_d_roll * (temp_err - pid_prev_err_roll));
	if (pid_output_roll > max_rotate){pid_output_roll = max_rotate;}
	if (pid_output_roll < (max_rotate*-1)){pid_output_roll = (max_rotate*-1);}

	pid_prev_err_roll = temp_err;

	//Pitch - X - Channel[1]
	temp_err = 0;
	temp_err = pid_gy_pitch - pid_setpoint_pitch;
	pid_integ_pitch += pid_i_roll * temp_err;
	if (pid_integ_pitch > max_rotate){pid_integ_pitch = max_rotate;}
	if (pid_integ_pitch < (max_rotate*-1)){pid_integ_pitch = (max_rotate*-1);}

	pid_output_pitch = (pid_p_roll * temp_err) + (pid_integ_pitch) + (pid_d_roll * (temp_err - pid_prev_err_pitch));
	if (pid_output_pitch > max_rotate){pid_output_pitch = max_rotate;}
	if (pid_output_pitch < (max_rotate*-1)){pid_output_pitch = (max_rotate*-1);}

	pid_prev_err_pitch = temp_err;

	//Yaw - Z - Channel[3]
	temp_err = 0;
	temp_err = pid_gy_yaw - pid_setpoint_yaw;
	pid_integ_yaw += pid_i_yaw * temp_err;
	if (pid_integ_yaw > max_rotate){pid_integ_yaw = max_rotate;}
	if (pid_integ_yaw < (max_rotate*-1)){pid_integ_yaw = (max_rotate*-1);}

	pid_output_yaw = (pid_p_yaw * temp_err) + (pid_integ_yaw) + (pid_d_yaw * (temp_err - pid_prev_err_yaw));
	if (pid_output_yaw > max_rotate){pid_output_yaw = max_rotate;}
	if (pid_output_yaw < (max_rotate*-1)){pid_output_yaw = (max_rotate*-1);}

	pid_prev_err_yaw = temp_err;
}

void Calculate_SetPoints(){

	//Roll - Y - Channel[0]
	if (channel[0] != 0){
		pid_setpoint_roll = 0;
		if (channel[0] > 1505){pid_setpoint_roll = (channel[0] - 1505)/3;} 	// max 495
		if (channel[0] < 1495){pid_setpoint_roll = (channel[0] - 1495)/3;} 	// max -495
		pid_setpoint_roll -= (roll_al);										// Max angular rate of 247
	}
	else {
		pid_setpoint_roll = 0;
	}

	//Pitch - X - Channel[1]
	if (channel[1] != 0){
		pid_setpoint_pitch = 0;
		if (channel[1] > 1505){pid_setpoint_pitch = (1505 - channel[1])/3;} // max 495
		if (channel[1] < 1495){pid_setpoint_pitch = (1495 - channel[1])/3;} // max -495
		pid_setpoint_pitch += pitch_al;									// Max angular rate of 247
	}
	else{
		pid_setpoint_pitch = 0;
	}
	//Yaw - Z - Channel[3]
	if (channel[3] != 0){
		if (channel[3] > 1505){pid_setpoint_yaw = (channel[3] - 1505)/3;} // max 495
		if (channel[3] < 1495){pid_setpoint_yaw = (channel[3] - 1495)/3;} // max -495
		// Max angular rate of 247
	}
	else{
		pid_setpoint_yaw = 0;
	}
}

void HC_SR04_Read(){

	// Reset the trigger pin and wait 3 micro seconds
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);
	uSec_Delay(3);

	// Take the trigger pin high for 10 micro seconds
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_SET);
	uSec_Delay(10);
	HAL_GPIO_WritePin(trig_GPIO_Port, trig_Pin, GPIO_PIN_RESET);

	// Start the echo input capture
	HAL_TIM_IC_Start_IT(&htim12, TIM_CHANNEL_1);

	//wait for the echo to be measured
	if (sr_rdy_flg == 1){
		//Reset Flag
		sr_rdy_flg = 0;
		// stop the input capture
		HAL_TIM_IC_Stop_IT(&htim12, TIM_CHANNEL_1);

		// Read in the echo pulse width
		if (echo_edge_2 > echo_edge_1){
			height = (float)(echo_edge_2 - echo_edge_1) * time_to_dist;
		}
		else{
			height = 0;
			fault_flag = 1;
		}

	}
}

void uSec_Delay(uint32_t uSec){
	if (uSec < 2){uSec = 2;}
	//Setup the timer
	TIM2->ARR = uSec - 1;
	TIM2->EGR = 1;
	TIM2->SR &= ~1;
	TIM2->CR1 |= 1;
	while((TIM2->SR & 0x0001) != 1);
	TIM2->SR &= ~(0x0001);
}

void Check_Faults(){

}

void HAL_TIM_IC_CaptureCalllback(TIM_HandleTypeDef *htim){
	if (capture_num == 0){
		echo_edge_1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		capture_num = 1;
	}
	else if (capture_num == 1){
		echo_edge_2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
		capture_num = 0;
		sr_rdy_flg = 1;
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart3){
	uint8_t *ptr = huart3->pRxBuffPtr;
	uint8_t *ptrl;
	if (*ptr == 0x20){
		ptr += 2;
		ptrl = ptr+1;
		for (int8_t i  = 0; i < 10; i++){
			channel[i] = (*(ptrl+(i*2)) << 8) | *(ptr+(i*2));
		}

	}
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

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
