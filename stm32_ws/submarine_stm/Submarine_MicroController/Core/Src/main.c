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
#include "string.h"
#include "math.h"
#include "stdio.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
//GLOBAL I2C Addresses
#define ACCEL_GYRO_ADR 0x6a
#define MAG_ADR 0x1c

//gyro
#define OUTX_L_G (0x22 | 0x01)
#define OUTX_H_G (0x23 | 0x01)
#define OUTY_L_G (0x24 | 0x01)
#define OUTY_H_G (0x25 | 0x01)
#define OUTZ_L_G (0x26 | 0x01)
#define OUTZ_H_G (0x27 | 0x01)
//accel
#define OUTX_L_XL (0x28 | 0x01)
#define OUTX_H_XL (0x29 | 0x01)
#define OUTY_L_XL (0x2A | 0x01)
#define OUTY_H_XL (0x2B | 0x01)
#define OUTZ_L_XL (0x2C | 0x01)
#define OUTZ_H_XL (0x2D | 0x01)
//magentometer
#define OUT_X_L (0x28 | 0x01)
#define OUT_X_H (0x29 | 0x01)
#define OUT_Y_L (0x2A | 0x01)
#define OUT_Y_H (0x2B | 0x01)
#define OUT_Z_L (0x2C | 0x01)
#define OUT_Z_H (0x2D | 0x01)

//stepper positions

#define full_close_steps 100


/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef hlpuart1;
UART_HandleTypeDef huart3;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

PCD_HandleTypeDef hpcd_USB_OTG_FS;

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_USB_OTG_FS_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
/* USER CODE BEGIN PFP */
uint8_t binaryToDecimal(int start_index, int bitCount);
HAL_StatusTypeDef imuRead(void);
void imuPullData(void);
uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg);


void MTR_DRV_INIT(uint8_t currentValue, uint8_t decay, uint8_t reset, uint8_t sleep);

void parseComs(void);
void transmitData(void);

void updateProps(void);
void updateServos(void);

void leftBalastOpenStep(void);
void leftBalastCloseStep(void);
void rightBalastOpenStep(void);
void rightBalastCloseStep(void);
void updateSteppers(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_received = 0;
char tx_buffer[100];
uint8_t rx_data[35];

//GLOBAL Coms Info
uint8_t depthUp = 0;
uint8_t depthDown = 0;
uint8_t captureImage = 0;
uint8_t forwardThrust = 0;
uint8_t turnThrust = 0;
uint8_t camUpDown = 0;
uint8_t camLeftRight = 0;


//GLOBAL Data info
uint16_t degreesNorth = 180;
float speedScalar = 10.4;
uint16_t depthApprox = 5;
uint16_t roll = 270;
uint16_t pitch = 270;
uint16_t yaw = 270;
float voltageBattery = 16.9;

//GLOBAL I2C pData registers
uint8_t x_ang_L;
uint8_t x_ang_H;
uint8_t y_ang_L;
uint8_t y_ang_H;
uint8_t z_ang_L;
uint8_t z_ang_H;

uint8_t x_lin_L;
uint8_t x_lin_H;
uint8_t y_lin_L;
uint8_t y_lin_H;
uint8_t z_lin_L;
uint8_t z_lin_H;

uint8_t x_mag_L;
uint8_t x_mag_H;
uint8_t y_mag_L;
uint8_t y_mag_H;
uint8_t z_mag_L;
uint8_t z_mag_H;

//GLOBAL IMU DOF 
uint16_t x_ang;
uint16_t y_ang;
uint16_t z_ang;

uint16_t x_lin;
uint16_t y_lin;
uint16_t z_lin;

uint16_t x_mag;
uint16_t y_mag;
uint16_t z_mag;


//GLOBAL PWM
uint16_t ARR_Prop = 11999;
uint32_t ARR_Servo = 2399999;

//GLOBAL Thrust
float leftThrust;
float rightThrust;
float forThrust;
float backThrust;
float leftPropThrust;
float rightPropThrust;

//GLOBAL Camera
float camVerticalDuty;
float camHorizontalDuty;

//GLOBAL Steppers
uint8_t currentStepLeft = 0;
uint8_t currentStepRight = 0;
uint8_t baseStepperPos = 0;
uint8_t leftStepperPos = 0;
uint8_t rightStepperPos = 0;






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
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_LPUART1_UART_Init();
  MX_USART3_UART_Init();
  MX_USB_OTG_FS_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  /* USER CODE BEGIN 2 */
	MTR_DRV_INIT(0x00, 0x00, 0x01, 0x01);
	HAL_UART_Receive_IT(&hlpuart1, rx_data, 35); //Init recieve global interupt for 35 bit buffer
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  	if (rx_received) //New Transmission from RPI
    {
		  
		  parseComs();
		  //imuPullData();
		  updateProps();
		  updateServos();
		  transmitData();
			rx_received = 0;

		  
		}
		  

  	
  	
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI48|RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSI48State = RCC_HSI48_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 30;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.OversamplingMode = DISABLE;
  hadc1.Init.DFSDMConfig = ADC_DFSDM_MODE_ENABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
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
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x107075B0;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 115200;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

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
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart3, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart3, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 11999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 11999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 2399999;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim5, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */
  HAL_TIM_MspPostInit(&htim5);

}

/**
  * @brief USB_OTG_FS Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_OTG_FS_PCD_Init(void)
{

  /* USER CODE BEGIN USB_OTG_FS_Init 0 */

  /* USER CODE END USB_OTG_FS_Init 0 */

  /* USER CODE BEGIN USB_OTG_FS_Init 1 */

  /* USER CODE END USB_OTG_FS_Init 1 */
  hpcd_USB_OTG_FS.Instance = USB_OTG_FS;
  hpcd_USB_OTG_FS.Init.dev_endpoints = 6;
  hpcd_USB_OTG_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_OTG_FS.Init.Sof_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.lpm_enable = DISABLE;
  hpcd_USB_OTG_FS.Init.battery_charging_enable = ENABLE;
  hpcd_USB_OTG_FS.Init.use_dedicated_ep1 = DISABLE;
  hpcd_USB_OTG_FS.Init.vbus_sensing_enable = ENABLE;
  if (HAL_PCD_Init(&hpcd_USB_OTG_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_OTG_FS_Init 2 */

  /* USER CODE END USB_OTG_FS_Init 2 */

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
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|LD3_Pin
                          |LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0|GPIO_PIN_1|USB_PowerSwitchOn_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA1 PA2 PA3 PA4
                           PA5 PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4
                          |GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PC4 PC5 */
  GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 PB2 LD3_Pin
                           LD2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|LD3_Pin
                          |LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PF12 PF13 PF14 PF15 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);

  /*Configure GPIO pins : PG0 PG1 USB_PowerSwitchOn_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|USB_PowerSwitchOn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pins : PE7 PE8 PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_7|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PE10 PE11 PE12 PE13 */
  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : USB_OverCurrent_Pin */
  GPIO_InitStruct.Pin = USB_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(USB_OverCurrent_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	UNUSED(huart); //waring suppresion
	
	//HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
	rx_received = 1;//set flag for use in while loop
	
	HAL_UART_Receive_IT(&hlpuart1, rx_data, 35); //reset UART interupt for next transmission
}


uint8_t binaryToDecimal(int start_index, int bitCount)
{
	//MSB is on the left so we start high and go low on the exp
	uint8_t result = 0;
	for (int i = bitCount - 1; i >= 0; i--)
	{
		if (rx_data[start_index + i] == 49)
		{
			result += pow(2, 7 - i);
		}
	}
	return result;
}


HAL_StatusTypeDef imuRead(void)
{
	/*
		HAL_StatusTypeDef HAL_I2C_Mem_Read	(	I2C_HandleTypeDef * 	hi2c,
																					uint16_t 	DevAddress,
																					uint16_t 	MemAddress,
																					uint16_t 	MemAddSize,
																					uint8_t * 	pData,
																					uint16_t 	Size,
																					uint32_t 	Timeout )		
	*/
	HAL_StatusTypeDef retVal = HAL_OK;
	
	//Gyro
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_L_G, 1, &x_ang_L, sizeof(x_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_H_G, 1, &x_ang_H, sizeof(x_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_L_G, 1, &y_ang_L, sizeof(y_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_H_G, 1, &y_ang_H, sizeof(y_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_L_G, 1, &z_ang_L, sizeof(z_ang_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_H_G, 1, &z_ang_H, sizeof(z_ang_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Accel
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_L_XL, 1, &x_lin_L, sizeof(x_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTX_H_XL, 1, &x_lin_H, sizeof(x_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_L_XL, 1, &y_lin_L, sizeof(y_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTY_H_XL, 1, &y_lin_H, sizeof(y_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_L_XL, 1, &z_lin_L, sizeof(z_lin_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, ACCEL_GYRO_ADR, OUTZ_H_XL, 1, &z_lin_H, sizeof(z_lin_H), 10) != HAL_OK) retVal = HAL_ERROR;
	//Magnet
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_X_L, 1, &x_mag_L, sizeof(x_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_X_H, 1, &x_mag_H, sizeof(x_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Y_L, 1, &y_mag_L, sizeof(y_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Y_H, 1, &y_mag_H, sizeof(y_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Z_L, 1, &z_mag_L, sizeof(z_mag_L), 10) != HAL_OK) retVal = HAL_ERROR;
	if (HAL_I2C_Mem_Read(&hi2c1, MAG_ADR, OUT_Z_H, 1, &z_mag_H, sizeof(z_mag_H), 10) != HAL_OK) retVal = HAL_ERROR;
	
	
	return retVal;
}

void imuPullData(void)
{
  //Begin reading IMU data
	if (imuRead() == HAL_OK) //If reading without error
	{
		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev	
		x_ang = twosComptoDec(x_ang_L, x_ang_H);//get xyz values for all 9DOF
		y_ang = twosComptoDec(y_ang_L, y_ang_H);//will need to do further calc for position, rotation, compass
		z_ang = twosComptoDec(z_ang_L, z_ang_H);

		x_lin = twosComptoDec(x_lin_L, x_lin_H);
		y_lin = twosComptoDec(y_lin_L, y_lin_H);
		z_lin = twosComptoDec(z_lin_L, z_lin_H);

		x_mag = twosComptoDec(x_mag_L, x_mag_H);
		y_mag = twosComptoDec(y_mag_L, y_mag_H);
		z_mag = twosComptoDec(z_mag_L, z_mag_H);
	}
}

uint16_t twosComptoDec(uint8_t low_reg, uint8_t high_reg)
{
	uint16_t combined = ((uint16_t)high_reg << 8) | low_reg;
	if ((combined & 0x8000) == 0) return combined; //negative if top bit is 1
	else return -(~combined + 1);
}

void MTR_DRV_INIT(uint8_t currentValue, uint8_t decay, uint8_t reset, uint8_t sleep)
{
  //Current Set
  /*
    xI1 xI0 current
    0b00 -> 0x0 -> 100%
    0b01 -> 0x1 -> 71%
    0b10 -> 0x2 -> 38%
    0b11 -> 0x3 -> 0%
    
    AI1 -> PF13
    AI0 -> PF14
    BI1 -> PF15
    BI0 -> PG0
    
  */
  //xI1
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_13, (currentValue>>1) & ~0x01);
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_15, (currentValue>>1) & ~0x01);
  //xI0
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14, (currentValue) & ~0x01);
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_0, (currentValue) & ~0x01);
  
  //Decay
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, decay);
  
  //Reset
  HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, reset);
  
  //Sleep
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_7, sleep);
  
  //Left Prop DC
  TIM3->CCR2 = 0;
  TIM3->CCR3 = 0;
  
  //Right Prop DC
  TIM4->CCR2 = 0;
  TIM4->CCR3 = 0;
  
  //Servos DC
  TIM5->CCR2 = 0.5 * ARR_Servo;
  TIM5->CCR3 = 0.5 * ARR_Servo;
  
  
}


void updateProps(void)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
  
  //Convert coms thrust vals in percentages for f,b,l,r
  if (turnThrust < 128)
  {
  	leftThrust = ((float)(-turnThrust + 128))/256.0; // 0% <-> 50% 
  	rightThrust = 0;
  }
  else
  {
  	leftThrust = 0;
  	rightThrust = ((float)(turnThrust - 128))/256.0; // 0% <-> 50% 
  }
  
  if (forwardThrust < 128)
  {
  	forThrust = 0;
		backThrust = ((float)(-forwardThrust + 128))/256.0; // 0% <-> 50%
  }
  else
  {
  	forThrust = ((float)(forwardThrust - 128))/256.0; // 0% <-> 50%
		backThrust = 0;
  }
  
  
  //mix individual thrust values into 
  rightPropThrust = leftThrust - rightThrust + forThrust - backThrust; // -100% <-> 100%
  leftPropThrust = rightThrust - leftThrust + forThrust - backThrust; // -100% <-> 100%
  
  //LEFT PROP SET DC (TIM3_CH2 = forward) (TIM3_CH3 = backward)
  if (leftPropThrust >= 0) 
  {
    TIM3->CCR2 = leftPropThrust*ARR_Prop;
    TIM3->CCR3 = 0;
  }
  else 
  {
    TIM3->CCR3 = -leftPropThrust*ARR_Prop;
    TIM3->CCR2 = 0;
  }
  
  //RIGHT PROP SET DC (TIM4_CH2 = forward) (TIM4_CH3 = backward)
  if (rightPropThrust >= 0) 
  {
    TIM4->CCR2 = rightPropThrust*ARR_Prop;
    TIM4->CCR3 = 0;
  }
  else
  {
    TIM4->CCR3 = -rightPropThrust*ARR_Prop;
    TIM4->CCR2 = 0;
  }
  
}

void updateServos(void)
{
  //HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev 
  //Convert coms thrust vals in percentages for f,b,l,r
  
  //Setup for 1ms <-> 2ms
  //camVerticalDuty = (0.000196 * (float)camUpDown) + 0.05;
  //camHorizontalDuty = (0.000196 * (float)camLeftRight) + 0.05;
  
  //Setup for 0.5ms <-> 2.5ms
  camVerticalDuty = (0.000392 * (float)camUpDown) + 0.025;
  camHorizontalDuty = (0.000392 * (float)camLeftRight) + 0.025;
  
  //Set Duty Cycles
  TIM5->CCR2 = camVerticalDuty*ARR_Servo;
  TIM5->CCR3 = camHorizontalDuty*ARR_Servo;
}

void leftBalastOpenStep(void)
{
	currentStepLeft++;
	if (currentStepLeft == 4) currentStepLeft = 0; //wrap around
	
}

void leftBalastCloseStep(void)
{
	currentStepLeft--;
	if (currentStepLeft == 255) currentStepLeft = 3; //wrap around
}

void rightBalastOpenStep(void)
{
	currentStepRight++;
	if (currentStepRight == 4) currentStepRight = 0; //wrap around
}

void rightBalastCloseStep(void)
{
	currentStepRight--;
	if (currentStepRight == 255) currentStepRight = 3; //wrap around
}

void updateSteppers(void)
{
	if (depthUp && baseStepperPos >= 10)
	{
		//Move toward full open
		leftBalastOpenStep();
		leftStepperPos--;
		rightStepperOpenStep();
		rightStepperPos--;
		baseStepperPos--;
	}
	if (depthUp && rightStepperPos <= full_close_step - 10)
	{
		rightStepperOpenStep();
		rightStepperPos--;
	}
}


void parseComs(void)
{
  //Coms parsing
  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_14); //toggle LED for dev
  if (rx_data[0] == 48)
  {
  	depthUp = 0;
  }
  else if (rx_data[0] == 49)
  {
  	depthUp = 1;
  }
  
  if (rx_data[1] == 48)
  {
  	depthDown = 0;
  }
  else if (rx_data[1] == 49)
  {
  	depthDown = 1;
  }
  
  if (rx_data[2] == 48)
  {
  	captureImage = 0;
  }
  else if (rx_data[2] == 49)
  {
  	captureImage = 1;
  }
	
	forwardThrust = binaryToDecimal(3, 8);
	turnThrust = binaryToDecimal(11, 8);
	camUpDown = binaryToDecimal(19, 8);
	camLeftRight = binaryToDecimal(27, 8);
}

void transmitData(void)
{
	//Data concat
	
	// For actual communication back to RPI (STILL USING TEST DATA)(This sprintf call works)
	sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", degreesNorth, (uint16_t)(10*speedScalar), depthApprox, roll, pitch, yaw, (uint16_t)(10*voltageBattery));
	
	// For reading the i2c values on the RPI
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u,%u,%u\n\r", x_ang, y_ang, z_ang, x_lin, y_lin, z_lin, x_mag, y_mag, z_mag );

  //sprintf(tx_buffer, "%u,%u,%u,%u,%d,%d\n\r", (uint8_t)(leftThrust*100.0), (uint8_t)(rightThrust*100.0), (uint8_t)(forThrust*100.0), (uint8_t)(backThrust*100.0), (int)(leftPropThrust*100.0), (int)(rightPropThrust*100.0));

	//Read back what was received
	//sprintf(tx_buffer, "%u,%u,%u,%u,%u,%u,%u\n\r", depthUp, depthDown, captureImage, forwardThrust, turnThrust, camUpDown, camLeftRight);
	
	//sprintf(tx_buffer, "TestMessage,,,,,,,,,,,,,\n\r");

  HAL_UART_Transmit(&hlpuart1, (uint8_t *)tx_buffer, sizeof(tx_buffer), 10);	
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
