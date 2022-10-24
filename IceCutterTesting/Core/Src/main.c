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
//#include <string.h>
//#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c2;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim14;

UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */

#define WIRE_INA219  		0x40
#define INPUT_INA219 		0x41

#define INA_CONFIG_REG  	0x00
#define INA_VOLT_REG		0x02
#define INA_POWER_REG 		0x03
#define INA_CURRENT_REG  	0x04
#define INA_CALIB_REG  		0x05
#define INA_CAL  			0xAEC3//0x7482//0xA6C2//0x7482
#define INA_CAL_INPUT		0x96A8
#define INA_CURRENT_LSB 	0.001220703//0.001831055
#define INA_POWER_LSB 		0.0366211 //current_LSB *20
#define INA_CONFIG_VALUE 	0x299F //sets PGA =2
#define INA_CONFIG_PGA_8	0x399F //set PGA = 8
#define INA_CONFIG_1_SAMP	0x399F //set sampling to 1
#define INA_CONFIG_4_SAMP	0x39D7//set sampling to 4 average
#define INA_CONFIG_8_SAMP	0x39DF//set sampling to 8 average
#define MAX_INRUSH_CURRENT 	59

#define TMP_TEMP_REG  		0x00
#define TMP_LSB  			0.0078125 //set from datasheet
#define ARR_MAX 			15999//0xFFF; //for 12bit resolution @ Fclk = 16MHz
#define TMP_ADD_WIRE 		0x48
#define TMP_ADD_INPUT 		0x49
#define TMP_ADD_AMBIENT 	0x4A
#define TEMP_LIMIT_AMBIENT 	0x32//50C
#define TEMP_LIMIT_WIRE 	0x55//85C
#define TEMP_LIMIT_INPUT 	0x55//85C

#define COPPER_TCR			0.00393//0.004
#define NICKEL_TCR 			0.0059//0.005671
#define R_REF_NICKEL 		0.027474
#define T_REF_NICKEL 		21.973025
#define R_REF_COPPER		0
#define T_REF_COPPER		0
#define CUTTER_TYPE			1 //1=nickel 2 = copper
#define SUPPLY_VOLTAGE 		5

#define RED_LED 		GPIO_PIN_9
#define RED_LED_state	GPIOB
#define GREEN_LED 		GPIO_PIN_15
#define GREEN_LED_state	GPIOC
#define SS_FET			GPIO_PIN_0
#define SS_FET_state	GPIOB
#define DRIVE_FET 		GPIO_PIN_4
#define DRIVE_FET_state	GPIOA
#define SW2 			GPIO_PIN_6
#define SW2_state		GPIOA
#define SW1 			GPIO_PIN_7
#define SW1_state		GPIOA

#define POWER_ON		0
#define INITIALISING	1
#define INITIALISED		2
#define SETTLING_STATE 	3
#define STEADY_STATE	4
#define CUTTING			5
#define COOLDOWN		6
#define UNKNOWN_STATE 	7


#define WARM_UP_THRESHOLD  10	//tune this
#define CUTTING_THRESHOLD -30 	//tune this
#define STEADY_STATE_THRESHOLD 5 //tune this
#define ICE_REMOVED_THRESHOLD 50 //tune this
#define ABOVE_THRESHOLD 0
#define IN_THRESHOLD 	1

#define MAX_ALLOWED_TEMP 1446.0 //0x5A6 = 1446 = Nickel Melting temp
#define POT_MAX_READING 4095.0
#define DUTY_CYCLE_MAX 1

#define FCLK			16000000
#define PSC				80//73//20
#define MAX_ARR			65535
#define ON_TIME			0.005

#define Ki_warming 0
#define Kp_warming 0
#define Kd_warming 0
float integral = 0;
float derivative = 0;
float prev_error = 0;


#define Ki_cutting 0
#define Kp_cutting 0
#define Kd_cutting 0

#define Ki_cooling 0
#define Kp_cooling 0
#define Kd_cooling 0

uint16_t duty_cycle_global;
uint8_t current_state;

float wire_temp_global;
float prev_wire_temp_global;
float input_power;
float output_power;
uint16_t pot_reading;
float temp_setpoint;
float TMP_Input;
float TMP_Output;
float TMP_Ambient;
float current_input=0;
float current_output=0;
float output_voltage;

uint8_t counter_uart = 0;
uint8_t counter_timer = 1;
uint8_t counter_temp = 0;
uint16_t volt_reg;
float wire_temp_arr[6];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_I2C2_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM14_Init(void);
static void MX_TIM3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void write_to_register(uint8_t device_address, uint8_t register_pointer, uint16_t register_value);
void write_to_reg_unspecified(uint8_t device_address, uint16_t register_value);
int16_t read_register(uint8_t device_address,uint8_t register_pointer);
int16_t readReg_unspecified(uint8_t device_address);
float get_temp_TMP117(uint8_t device_address);
float measure_current(uint8_t device_address);
float measure_power(uint8_t device_address);
int swap_endian(int p);
void ina_initialise();
float wire_temp_calc();
uint8_t temp_checks();
void SS_charge();
void pulse_charge_SS();
void startup_initialisation();
void UART_output();
void set_PWM_driveFET(float duty_cycle);
double fabs(double x);
uint16_t pot_temp();
void warm_up_controller();
void cutting_controller();
void cooling_controller();
void state_estimate();
void monitor_input_current();
uint16_t calc_ARR(float duty_cycle);
void wire_temp_average();

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
  MX_USART2_UART_Init();
  MX_I2C2_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM14_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  //calibrate ADC on start-up for better accuracy
  HAL_ADCEx_Calibration_Start(&hadc1);
  HAL_TIM_PWM_Start(&htim14, TIM_CHANNEL_1);
  HAL_TIM_Base_Start_IT(&htim14);

  //current_state = POWER_ON;
  //startup_initialisation();
  HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */


  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  HAL_ADC_Start_DMA (&hadc1, &pot_reading, 1);
	  if(temp_checks()==IN_THRESHOLD){
		  //monitor_input_current();
		  wire_temp_average();
		  UART_output();
		  //wire_temp_calc();
		  //state_estimate();
		  switch (current_state){
		  case POWER_ON:
			  startup_initialisation();
			  break;
		  case INITIALISING:
			  startup_initialisation();
			  break;
		  case INITIALISED:
			  wire_temp_calc();
			  //waiting for button press
			  HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Press SW2 to heat",sizeof("\r\n Press SW2 to heat"),10);
			  break;
		  case SETTLING_STATE:
			  warm_up_controller();
			  HAL_GPIO_TogglePin(RED_LED_state, RED_LED);
			  HAL_GPIO_TogglePin(GREEN_LED_state, GREEN_LED);
			  break;
		  case STEADY_STATE:
			  HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Wire @ temp",sizeof("\r\n Wire @ temp"),10);
			  warm_up_controller();
			  HAL_GPIO_TogglePin(RED_LED_state, RED_LED);
			  HAL_GPIO_TogglePin(GREEN_LED_state, GREEN_LED);
			  break;
		  case CUTTING:
			  cutting_controller();
			  HAL_GPIO_TogglePin(RED_LED_state, RED_LED);
			  HAL_GPIO_TogglePin(GREEN_LED_state, GREEN_LED);
			  break;
		  case COOLDOWN:
			  cooling_controller();
			  HAL_GPIO_TogglePin(RED_LED_state, RED_LED);
			  HAL_GPIO_TogglePin(GREEN_LED_state, GREEN_LED);
			  break;
		  case UNKNOWN_STATE:
			  startup_initialisation();
		  break;
		  }
		  UART_output();
		  counter_uart += counter_uart;
	  }else{
		  HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Board too hot",sizeof("\r\n Board too hot"),10);
		  set_PWM_driveFET(0);
	  }
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDiv = RCC_HSI_DIV1;
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
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

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
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  hi2c2.Init.Timing = 0x00303D5B;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
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
  htim1.Init.Prescaler = 0;
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
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 26666;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM14 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM14_Init(void)
{

  /* USER CODE BEGIN TIM14_Init 0 */

  /* USER CODE END TIM14_Init 0 */

  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM14_Init 1 */

  /* USER CODE END TIM14_Init 1 */
  htim14.Instance = TIM14;
  htim14.Init.Prescaler = 80;
  htim14.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim14.Init.Period = 65535;
  htim14.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim14) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_ENABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim14, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM14_Init 2 */

  /* USER CODE END TIM14_Init 2 */
  HAL_TIM_MspPostInit(&htim14);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9|GPIO_PIN_0, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_15, GPIO_PIN_SET);

  /*Configure GPIO pins : PB9 PB0 */
  GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PA5 */
  GPIO_InitStruct.Pin = GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /**/
  __HAL_SYSCFG_FASTMODEPLUS_ENABLE(SYSCFG_FASTMODEPLUS_PB9);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_15_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI4_15_IRQn);

}

/* USER CODE BEGIN 4 */
/**
 *@brief Swaps endian from Big to little or vice versa for given data
 * @param int p
 * @retval int p_swapped
   */
 int swap_endian(int p)
 {
 	 uint16_t p_swapped = (p>>8) | (p<<8);
 	 return p_swapped;
 }
//TODO add I2C error handling
 /**
 * @brief Write to given value to register of given device address
 * @param uint8_t device_address, uint8_t register_pointer, uint16_t register_value
 * @retval None
 */
 void write_to_register(uint8_t device_address,uint8_t register_pointer, uint16_t register_value)
 {
     HAL_StatusTypeDef status = HAL_OK;
     uint16_t register_value_swapped = swap_endian(register_value);
     HAL_I2C_Mem_Write(&hi2c2,(device_address<<1),register_pointer,1,&register_value_swapped,2,50);
     /* Check the communication status */
     if(status != HAL_OK){
    	 MX_I2C2_Init();
    	 HAL_Delay(100);
    	 HAL_I2C_Mem_Write(&hi2c2,(device_address<<1),register_pointer,1,&register_value_swapped,2,50);
    	 // Error handling, for example re-initialization of the I2C peripheral
     }
 }

 /**
 * @brief read given register of given device function
 * @param uint8_t device_address uint8_t register_pointer
 * @retval uint16_t register_value_return
 */
 int16_t read_register(uint8_t device_address,uint8_t register_pointer)
 {
     HAL_StatusTypeDef status = HAL_OK;
     uint16_t register_value_return;

     HAL_I2C_Mem_Read(&hi2c2,(device_address<<1),register_pointer,1,&register_value_return,2,500);
     /* Check the communication status */
     if(status != HAL_OK){
         	 MX_I2C2_Init();
         	 HAL_Delay(100);
         	 HAL_I2C_Mem_Read(&hi2c2,(device_address<<1),register_pointer,1,&register_value_return,2,500);
         	 // Error handling, for example re-initialization of the I2C peripheral
     }
     register_value_return = swap_endian(register_value_return);
     return register_value_return;
 }

 /**
 * @brief	Calculate and return temperature measured by TMP117
 * @param	int16_t register_value
 * @retval 	int16_t temp
 */
 float get_temp_TMP117(uint8_t device_address)
 {
	   //uint8_t value_LSB = 0.0078125;
	   int16_t device_measurement = read_register(device_address, TMP_TEMP_REG);
	   float temp = TMP_LSB * device_measurement;

	   switch(device_address){
	   case TMP_ADD_AMBIENT:
		   TMP_Ambient = temp;
		   break;
	   case TMP_ADD_INPUT:
		   TMP_Input = temp;
		   break;
	   case TMP_ADD_WIRE:
		   TMP_Output = temp;
		   break;
	   }
	   return temp;

 }

 /**
 * @brief Fetches value from current register of INA219, multiplies by current_LSB to calculate current flowing through shunt resistor
 * @param
 * @retval float variable representing current through shunt resistor
 */
 float measure_current(uint8_t device_address)
 {


	  int current_reg = read_register(device_address,INA_CURRENT_REG);
	  float current = INA_CURRENT_LSB*current_reg;
	  switch (device_address){
	  case WIRE_INA219:
		   current_output = current;
		   break;
	   case INPUT_INA219:
		   current_input = current;
		   break;
	   }
	   return current;

 }

 void monitor_input_current(){
	 //float in_current = measure_current(INPUT_INA219);

	 if (current_input >= MAX_INRUSH_CURRENT){
		 set_PWM_driveFET(0);
		 HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Input current to large",sizeof("\r\n Input current to large"),10);
		 startup_initialisation();
	 }
 }
/**
 * @brief Fetches power variable from INA219s power register, and multiplies by power_LSB to give power in watts across shunt resistor
 * @param
 * @retval float variable representing power across shunt resistor
    */
 float measure_power(uint8_t device_address)
 {
	   int temp = read_register(device_address, INA_POWER_REG);
	   float power = INA_POWER_LSB *temp;
	   switch (device_address){
	   	   case WIRE_INA219:
	   		   output_power = power;
	   		   break;
	   	   case INPUT_INA219:
	   		   input_power = power;
	   		   break;
	   	   }
	   	   return power;
 }

 float measure_voltage(){
	 uint16_t temp = read_register(WIRE_INA219, INA_VOLT_REG);
	 volt_reg = temp;
	 temp = temp>>3;
	 float voltage = temp * 0.004;
	 output_voltage = voltage;
	 return voltage;

 }

/**
 * @brief Uses TCR of Nickel wire to calculate wire temp using measued current, supply voltage is known constant
 * @param
 * @retval float variable of wire temperature
 */
 float wire_temp_calc(){
	 float temp = 0;
	 float wire_current = current_output;//measure_current(WIRE_INA219);
	 HAL_Delay(0.5);
	 //wire_current = current_output;
	 float board_resistance = 0.10726;
	 float red_lead = 0.00001;
	 float black_lead = 0.00007;
	 float board_impedance = board_resistance+red_lead+black_lead;

	 float wire_resistance = output_voltage/wire_current;//SUPPLY_VOLTAGE/wire_current;
	 wire_resistance -= board_impedance;

	 if(wire_current <=2){
		 //temp = R_REF_NICKEL;
		 //wire_temp_global = temp;

	 }else{

	 if(CUTTER_TYPE==1){
		 temp = wire_resistance/(R_REF_NICKEL*1.6*NICKEL_TCR);//todo remove x3
		 temp = temp - (1/NICKEL_TCR);
		 temp = temp + T_REF_NICKEL;
	 }else{
		 temp = wire_resistance/(R_REF_COPPER*3*COPPER_TCR);
		 temp = temp - (1/COPPER_TCR);
		 temp = temp + T_REF_COPPER;
	 }

	 if(temp <= MAX_ALLOWED_TEMP && temp >=-200){
		 return temp;
		 //prev_wire_temp_global = wire_temp_global;
		 //wire_temp_global = temp;
	 } else if(temp>MAX_ALLOWED_TEMP && prev_wire_temp_global >=TMP_Output){
		 temp = prev_wire_temp_global;
		 //wire_temp_global = temp;
	 }else{
		 temp = TMP_Output;
		 //wire_temp_global = temp;
	 }
	 }
	 //wire_temp_global = temp;

	 //counter_temp += 1;
	 return temp;
 }

 void wire_temp_average(){
	float temp = wire_temp_calc();

	if(counter_temp>=6){
		counter_temp=0;
	}

	wire_temp_arr[counter_temp] = temp;
	counter_temp+=1;
	float avg = 0;
	for(uint8_t i = 0;i<5;i++){
		avg = avg + wire_temp_arr[i];
	}
	avg = avg/6.0;
	wire_temp_global = avg;
 }


 /**
  * @brief writes calibration value to INA219s calibration register
  * @param address of INA219: uin8_t device_address
  */
 void ina_initialise(){
	   write_to_register(WIRE_INA219,INA_CALIB_REG, INA_CAL);
	   write_to_register(INPUT_INA219, INA_CALIB_REG, INA_CAL_INPUT);
	   write_to_register(WIRE_INA219, INA_CONFIG_REG, INA_CONFIG_8_SAMP);
	   write_to_register(INPUT_INA219, INA_CONFIG_REG, INA_CONFIG_8_SAMP);
 }


/**
 	 * @brief Fetches temp from all three TMP117s on board, ensures all under required temp limit.
     * @param
     * @retval 1 if all below limit 0 if at least one above limit
 */
 uint8_t temp_checks(){//TODO may need to convert to difference from ambient
	   float temp_ambient 	= get_temp_TMP117(TMP_ADD_AMBIENT);//get temp from TNMP117 in open air
	   float temp_input		= get_temp_TMP117(TMP_ADD_INPUT);//get temp from TMP117 near input
	   float temp_element 	= get_temp_TMP117(TMP_ADD_WIRE);//get temp from TMP117 near wire element

	   if((temp_ambient<=TEMP_LIMIT_AMBIENT)&&(temp_input<=TEMP_LIMIT_INPUT)&&(temp_element<=TEMP_LIMIT_WIRE)){
		   return IN_THRESHOLD;
	   }
	   else{
		   return ABOVE_THRESHOLD;
	   }
 }

 /**
  	* @brief	Allows for capacitors to charge through soft-start circuit. if inrush current exceeds maximum allowed input current initiates pulse_charge_SS
  	* slow charging time of capacitors and reduce in rush current
      * @param
      * @retval
 */
 void SS_charge(){
	 uint8_t counter = 0;
	 HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_RESET);
	 while(HAL_GPIO_ReadPin(GPIOB,SS_FET) == GPIO_PIN_RESET){
		 float input_current = current_input;//measure_current(INPUT_INA219);
		 if(input_current >= MAX_INRUSH_CURRENT && counter == 0){
			 HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_SET); //stop charging caps
			 counter = counter + 1;
			 HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_RESET); //stop charging caps
		 }else if(input_current >= MAX_INRUSH_CURRENT && counter >=1){
			 HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_SET); //stop charging caps
			 counter = counter + 1;
			 pulse_charge_SS();
		 }else if(temp_checks() == ABOVE_THRESHOLD){
			 HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_SET); //stop charging caps
			 while(temp_checks()==ABOVE_THRESHOLD){
				 HAL_Delay(100); //allow time for board to cool
			 }
			 HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_RESET);
		 }else if(current_input == 0){// measure_current(INPUT_INA219)==0  capacitors fully charged, no current flow
			 HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_SET);//stop soft start circuit charging caps
			 break;
		 }
		 UART_output();
	   }
 }

 /**
   * @brief	Uses SS_FET connection to micro-controller to increase time constant of soft-start circuit to reduce inrush current
   * @param
   * @retval
 */
 void pulse_charge_SS(){
	   for (uint8_t i = 1; i < 50; ++i)
	     {
		   HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_RESET);
		   HAL_Delay(0.05);
		   HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_SET);
	     }
	   HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_RESET);
 }

/** @brief	Use soft-start circuit to charge capacitors, green LED turned on once capacitors fully charged
      * @param
      * @retval
      */
 void startup_initialisation(){
	 	for(uint8_t i = 0;i<6;i++){
	 		wire_temp_arr[i] = 0;
	 	}
	 	TIM3->CCR1=0;

	 	current_state = INITIALISING;
	 	HAL_GPIO_WritePin(RED_LED_state, RED_LED, GPIO_PIN_SET);//turn on red led
	 	HAL_GPIO_WritePin(GREEN_LED_state, GREEN_LED, GPIO_PIN_RESET);//turn off green led
	 	set_PWM_driveFET(0);//ensure heating wire doesn't conduct
		HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_SET); //keep SS_FET high, soft start stops Caps charging
		ina_initialise();//set config registers of INA219s
	   //prompt user for reference temp input via UART
	   //once got input, shut down interrupt channel

	   uint8_t temps_in_range = ABOVE_THRESHOLD;
	   while(temps_in_range == ABOVE_THRESHOLD){
		   temps_in_range = temp_checks();
	   }
	   SS_charge();
	   HAL_GPIO_WritePin(GPIOB,SS_FET, GPIO_PIN_SET);//allow SS_FET to conduct to keep caps charged
	   HAL_GPIO_WritePin(GPIOC, GREEN_LED, GPIO_PIN_SET); // green LED2 turn on, system ready to cut
	   current_state = INITIALISED;
	   UART_output();
 }


//TODO ADD outputs from other sensors
/**
 * @brief outputs overview of all system parameters via UART interface
 */
void UART_output(){
		uint8_t output[7];
	    uint8_t output_current[7];

		HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n ------------------",sizeof("\r\n ------------------"),10);
    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Output current: ",sizeof("\r\n Output current: "),10);


    	sprintf(output_current,"%d.%03u",((int) (current_output + 0.5)));
    	HAL_UART_Transmit(&huart2,output_current,sizeof(output_current),10);


    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Output voltage: ",sizeof("\r\n Output voltage: "),10);
    	sprintf(output_current,"%d.%02u",((int) (output_voltage + 0.5)));
    	HAL_UART_Transmit(&huart2,output_current,sizeof(output_current),10);

    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Input current: ",sizeof("\r\n Input current: "),10);
    	sprintf(output_current,"%d.%02u",((int) (current_input + 0.5)));
    	HAL_UART_Transmit(&huart2,output_current,sizeof(output_current),10);

    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Input power: ",sizeof("\r\n Input power: "),10);
    	sprintf(output_current,"%d.%02u",((int) (input_power + 0.5)));
    	HAL_UART_Transmit(&huart2,output_current,sizeof(output_current),10);

    	 //gcvt(ambient_temp,6,output);
    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Ambient temp: ",sizeof("\r\n Ambient temp: "),10);
    	sprintf(output,"%d.%02u", (int) TMP_Ambient, (int) fabs(((TMP_Ambient - (int) TMP_Ambient ) * 100)));
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);


    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Input shunt temp: ",sizeof("\r\n Input shunt temp: "),10);
    	sprintf(output,"%d.%02u", (int) TMP_Input, (int) fabs(((TMP_Input - (int) TMP_Input ) * 100)));
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Output shunt temp: ",sizeof("\r\n Output shunt temp: "),10);
    	sprintf(output,"%d.%02u", (int) TMP_Output, (int) fabs(((TMP_Output - (int) TMP_Output ) * 100)));
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Potentiometer reading: ",sizeof("\r\n Potentiometer reading: "),10);
    	sprintf(output,"%d.%02u", (int) pot_reading, (int) fabs(((pot_reading - (int) pot_reading ) * 100)));
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	sprintf(output,"%d.%02u", (int) duty_cycle_global, (int) fabs(((duty_cycle_global - (int) duty_cycle_global ) * 100)));
    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Current duty cycle: ",sizeof("\r\n Current duty cycle: "),10);
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	sprintf(output,"%d.%02u", (int) temp_setpoint, (int) fabs(((temp_setpoint - (int) temp_setpoint ) * 100)));
    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Set point Temp: ",sizeof("\r\n Set point Temp: "),10);
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Current Temp: ",sizeof("\r\n Current Temp: "),10);
    	sprintf(output,"%d.%02u", (int) wire_temp_global, (int) fabs(((wire_temp_global - (int) wire_temp_global ) * 100)));
    	HAL_UART_Transmit(&huart2,output,sizeof(output),10);

    	/*if(wire_temp_global == -1000000){
    		HAL_UART_Transmit(&huart2,"Wire not conducting",sizeof("Wire not conducting"),10);
    	}else{
    		sprintf(output,"%d.%02u", (int) wire_temp_global, (int) fabs(((wire_temp_global - (int) wire_temp_global ) * 100)));
    		HAL_UART_Transmit(&huart2,output,sizeof(output),10);
    	}*/


    	char Power_on[]		= 	"Power On";
    	char Initialise[]	= 	"Initialised";
    	char Init[]			= 	"Initialising";
    	char Settle[]		= 	"Settling State";
    	char SS[]			= 	"Steady-State";
    	char cutting[]		= 	"Cutting";
    	char cooldown[]		= 	"Cooldown";
    	HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Current State: ",sizeof("\r\n Current State: "),10);
    	 switch(current_state){
    	 case POWER_ON:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)Power_on,sizeof(Power_on),10);
    		 break;
    	 case INITIALISED:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)Initialise,sizeof(Initialise),10);
    		 break;
    	 case INITIALISING:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)Init,sizeof(Init),10);
    		 break;
    	 case SETTLING_STATE:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)Settle,sizeof(Settle),10);
    		 break;
    	 case STEADY_STATE:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)SS,sizeof(SS),10);
    		 break;
    	 case CUTTING:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)cutting,sizeof(cutting),10);
    		 break;
    	 case COOLDOWN:
    		 HAL_UART_Transmit(&huart2,(uint8_t *)cooldown,sizeof(cooldown),10);
    		 break;
    	 }


    	 HAL_Delay(1000);//TODO remove this in actual application
    }

/**
 * @brief convert given duty cycle fraction into correct CRR/ARR ratio and writes to the CCR1 register of TIM14
 * limits duty cycle to 0<=duty_cycle<=DUTY_CYCLE_MAX
 */
void set_PWM_driveFET(float duty_cycle){
    	//uint16_t CRR = calculate_CRR(duty_cycle);
    	uint16_t CCR;
    	uint16_t ARR;
    	if(duty_cycle>=DUTY_CYCLE_MAX){
    		ARR = calc_ARR(DUTY_CYCLE_MAX);
    		CCR = DUTY_CYCLE_MAX*ARR;
    	}else if(duty_cycle>=1){
    		ARR = calc_ARR(1);
    		CCR = ARR;
    	}else if(duty_cycle<=0){
    		ARR = 39999;//200Hz
    		CCR=0;
    	}else{
    		ARR = calc_ARR(duty_cycle);
    		CCR = duty_cycle*ARR;
    	}

    	TIM14->CCR1 = CCR;
    	TIM14->ARR = ARR;

    	duty_cycle_global = duty_cycle*100;
    }

uint16_t calc_ARR(float duty_cycle){
	float off_time = (ON_TIME/duty_cycle)-ON_TIME;
	float period = ON_TIME+off_time;
	float Fpwm = 1/period;
	uint16_t round_Fpwm = round(Fpwm);
	uint16_t ARR = (FCLK/(round_Fpwm*(PSC+1)))-1;

	if(ARR<=65535){
		return ARR;
	}else if(ARR<0){
		ARR = 0;
		return ARR;
	}else{
		ARR = 65535;
		return ARR;
	}
}


/**
 * @brief function called when interrupt triggered on rising edge
 * @param GPIO pin on which interrupt occurred
 */
void HAL_GPIO_EXTI_Falling_Callback(uint16_t GPIO_Pin){

	switch(GPIO_Pin){
	case SW1:
		set_PWM_driveFET(0);
		HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_RESET);
		/*switch (current_state){
		case STEADY_STATE:
			current_state = CUTTING;
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Ready to cut",sizeof("\r\n Ready to cut"),10);
			HAL_Delay(10);
		case CUTTING:
			current_state = COOLDOWN;
		}
		HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n NOT",sizeof("\r\n NOT"),10);*/
		break;
	case SW2:
		set_PWM_driveFET(0.7);
		//HAL_GPIO_WritePin(GPIOB,SS_FET,GPIO_PIN_SET);

		/*switch (current_state){
		case POWER_ON:
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Please Wait",sizeof("\r\n Please Wait"),10);
			current_state = POWER_ON;
			HAL_Delay(10);
			break;
		case INITIALISING:
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Please Wait",sizeof("\r\n Please Wait"),10);
			current_state=INITIALISING;
			HAL_Delay(10);
			break;
		case INITIALISED:
			current_state = SETTLING_STATE;
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Heating to Setpoint",sizeof("\r\n Heating to Setpoint"),10);
			HAL_Delay(10);
			break;
		case SETTLING_STATE:
			current_state = INITIALISING;
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Heating turned off",sizeof("\r\n Heating turned off"),10);
		case STEADY_STATE:
			current_state = INITIALISING;
			HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n Heating turned off",sizeof("\r\n Heating turned off"),10);
		}
		HAL_UART_Transmit(&huart2,(uint8_t *)"\r\n HERE",sizeof("\r\n HERE"),10);*/
		break;
	}
}

/**
 * Function called when buffer allocated to ADC connected to potentiometer full
 */
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc){
        // Read & Update The ADC Result
    	//TIM14->CCR1 = pot_reading;
		//float duty_cycle = pot_reading/4095.0;
		//set_PWM_driveFET(duty_cycle);
    	//temp_setpoint=pot_temp();
		//temp_setpoint = pot_reading * scaling;
	}


/**
 * @brief PID controller used when system not in contact with ice, has slower response time ideal for intial heat up
 * to set point temperature
 * @param
 * @retva; duty cycle to be applied to drive_FET circuit: float duty_cycle
 */
void warm_up_controller(){
	float wire_temp = wire_temp_calc();
	float error = temp_setpoint - wire_temp;

	integral = integral + error;
	derivative = error - prev_error;

	float duty_cycle = (Kp_warming * error) + (Ki_warming*integral) + (Kd_warming*derivative);
	prev_error = error;
	set_PWM_driveFET(duty_cycle);
}
/**
 * @brief PID controller with faster response time, for when system in first contacts ice and when ice
 * removed to stop sharp temp increase
 * @param
 * @retva; duty cycle to be applied to drive_FET circuit: float duty_cycle
 */
void cutting_controller(){
	float wire_temp = wire_temp_calc();
	float error = temp_setpoint - wire_temp;

	integral = integral + error;
	derivative = error - prev_error;

	float duty_cycle = (Kp_cutting * error) + (Ki_cutting * integral) + (Kd_cutting * derivative);
	prev_error = error;
	set_PWM_driveFET(duty_cycle);
}

/**
 * @brief PID controller with faster response time, for when temp spikes after ice removed
 * removed to stop sharp temp increase
 * @param
 * @retva; duty cycle to be applied to drive_FET circuit: float duty_cycle
 */
void cooling_controller(){
	float wire_temp = wire_temp_calc();
	float error = temp_setpoint - wire_temp;

	integral = integral + error;
	derivative = error - prev_error;

	float duty_cycle = (Kp_cooling * error) + (Ki_cooling * integral) + (Kd_cooling * derivative);
	prev_error = error;
	set_PWM_driveFET(duty_cycle);
}


/**
  @brief	converts input from potentiometer to temperature set point
  * divides maximum pot reading = 4095 by maximum allowed temperature of heating element
  * @param
  * @retval uint8_t set point temperature
 */
uint16_t pot_temp(){
    	uint16_t set_temp = 0;
    	float scaling = MAX_ALLOWED_TEMP/POT_MAX_READING;

    	set_temp = scaling*pot_reading;
    	return set_temp;

    }
/**
 * @brief Estimates what state the system is in, based on the change in temperature of the nickel wire
 * State estimation used to determine which controller to apply to the system.
 * @param
 * @retval
 */
void state_estimate(){

    float diff_wire_temp = wire_temp_global - prev_wire_temp_global;

    if(wire_temp_global !=  -100000){
    switch(current_state){
    case INITIALISING:
    	break;
    case INITIALISED:
    	break;
    case SETTLING_STATE://non-steady state, includes warm-up time and oscillations prior to steady state
    	if(diff_wire_temp>WARM_UP_THRESHOLD){
    		current_state = SETTLING_STATE;
    	}else if(fabs(wire_temp_global-temp_setpoint) < WARM_UP_THRESHOLD){ //wire heated to set point
    	    current_state = STEADY_STATE;
    	}else if(diff_wire_temp < CUTTING_THRESHOLD){ //wire temp decreased  more than threshold
    	    current_state = CUTTING;
    	}
    	break;
    case STEADY_STATE:
    	if(fabs(diff_wire_temp) <= STEADY_STATE_THRESHOLD){ //wire within set steady state threshold
    		current_state = STEADY_STATE;
    	}else if(diff_wire_temp > STEADY_STATE_THRESHOLD){ //wire above set point temp no longer in steady state
    	    current_state = SETTLING_STATE;
    	}else if(diff_wire_temp <= CUTTING_THRESHOLD){ //wire temp decreased more than cutting threshold
    	    current_state = CUTTING;
    	}
    	break;
    case CUTTING://when ice applied, temp drops need aggressive controller to compensate for drop and heat transfer
    	if(diff_wire_temp > WARM_UP_THRESHOLD && diff_wire_temp < ICE_REMOVED_THRESHOLD){ //wire re-heated to set point
    		current_state = CUTTING;
    	}else if(diff_wire_temp > ICE_REMOVED_THRESHOLD){//wire temp spiked above threshold when ice removed
    		current_state = COOLDOWN;
    	}
    	break;
    case COOLDOWN://when ice removed temp spikes so need more aggressive controller
    	if(wire_temp_global < temp_setpoint && diff_wire_temp >=0){
    		//wire temp decreased below setpoint temp so not too hot
    		//wire temp beginning to increase again, so aggressive controller used to partially return system to steady state
    		current_state = SETTLING_STATE;
    	}else{
    		current_state = COOLDOWN;
    	}
    	break;
    default:
    	//system state not in known phase
    	//stop wire heating and reinitialise the system
    	current_state = UNKNOWN_STATE;
    	set_PWM_driveFET(0);
    	startup_initialisation();
    	break;
    }
    }
}

uint8_t ina_error_check(){
	uint8_t checker = 0;
	//uint16_t temp = read_register(WIRE_INA219, INA_VOLT_REG);
	//uint8_t conv = volt_reg & (1 << (2 - 1));
	//uint8_t conv = 0x02 & volt_reg;
	//uint8_t ovf = 0x01 & volt_reg;
	if(volt_reg & (1 << (2 - 1))){
		checker = 1;
	}
	return checker;
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	//HAL_UART_Transmit(&huart2, (uint8_t)"\r\n x", sizeof("\r\n x"), 10);
	if(htim == &htim3){
		//TIM3->CCR1=15999;
		if(counter_timer %2 == 0){
			measure_voltage();
			if(ina_error_check() == 1){
				measure_current(WIRE_INA219);
				measure_current(INPUT_INA219);
				counter_timer=1;
			}
		}else{
			HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_2);
			HAL_TIM_Base_Stop_IT(&htim3);
			counter_timer +=1;
		}
	}else{
		HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
		HAL_TIM_Base_Start_IT(&htim3);
	}
	//measure_power(WIRE_INA219);
	//measure_power(INPUT_INA219);
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
