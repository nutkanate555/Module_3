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
#include "stdio.h"
#include <stdlib.h>
#include "string.h"
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
I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_tx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */

uint8_t pidSetZeroFlag = 0;


uint64_t _micros = 0;
float EncoderVel = 0;
uint64_t Timestamp_Encoder = 0;

typedef struct _UartStructure
{
	UART_HandleTypeDef *huart;
	uint16_t TxLen, RxLen;
	uint8_t *TxBuffer;
	uint16_t TxTail, TxHead;
	uint8_t *RxBuffer;
	uint16_t RxTail; //RXHeadUseDMA

} UARTStucrture;

UARTStucrture UART2 = { 0 };

typedef struct _MotorDriveStructure
{
	uint8_t DIR;
	uint32_t PWMOut;
}MotorDriveStructure;

MotorDriveStructure DCMotorStruc = {0};

typedef enum
{
	PP_STARTandMode,
	PP_Frame2_Data_0,
	PP_Frame2_Data_1,
	PP_Frame3_Data_0,
	PP_Frame3_Data_1,
	PP_CheckSum,
} PPSTATE;

PPSTATE Munmunbot_Protocol_State = PP_STARTandMode;

typedef enum
{
	STATE_Disconnected,    /// Set_station_position
	STATE_Idle,
	STATE_PrepareDATA,
	STATE_Calculation,
	STATE_Link_Moving,
	STATE_Stabilized_Link,
	STATE_Verified,
	STATE_End_Effector_Working,
	STATE_SetHome
} Robot_STATE;

//Robot_STATE Munmunbot_State = STATE_Disconnected;
Robot_STATE Munmunbot_State = STATE_Disconnected;

typedef enum
{
	SetHomeState_0,
	SetHomeState_1,
	SetHomeState_2
} SetHome_STATE;

SetHome_STATE SethomeMode = SetHomeState_0;

typedef struct _TrajectoryGenerationStructure
{
	float BlendTimeLSPB;
	float BlendTimeTriangular;
	float LinearTimeLSPB;

	float Theta_min_for_LSPB;
	float AngularVelocityMax_Setting;
	float AngularAccerationMax_Setting;

	float AngularVelocity;
	float AngularAcceration;
	float AngularDisplacementDesire;
	float AngularVelocityDesire;

	float Theta_Stamp_0;
	float Theta_Stamp_1;
	float Theta_Stamp_2;

	double Equation_Realtime_Sec;

	uint64_t Equation_Timestamp;
	uint64_t Velocity_Timestamp;
	uint64_t Loop_Timestamp;

	uint32_t Loop_Freq;
	uint64_t Loop_Period;

	float Desire_Theta;
	float Start_Theta;
	float Delta_Theta;
	float Abs_Delta_Theta;

	float AngularVelocityFinalMin;
	float Alpha;

	uint32_t Mode;
	uint32_t Submode;
	uint32_t Subsubmode;

} TrajectoryGenerationStructure;

typedef struct _ConverterUnitSystemStructure
{
	uint32_t PPR;
	uint32_t PPRxQEI;
	float RPMp;

} ConverterUnitSystemStructure;

typedef struct _PIDStructure
{
	float Kp;
	float Ki;
	float Kd;
	float alpha;
	float offSet;
	float ControllerOutput;
	float PreviousControllerOutput;
	float OutputDesire;
	float OutputFeedback;
	float Integral_Value;
	float NowError;
	float PreviousError;
	float PreviousPreviousError;
	double SamplingTime;
} PIDStructure;


///Station Setting
uint16_t StationPos[10] = {0,1,2,3,4,36,18,54,33,67};

uint8_t Angularpos_InputArray[255] = {0};
uint16_t Angularpos_InputNumber = 0;

typedef enum
{
	LMM_Not_Set,
	LMM_Set_Pos_Directly,
	LMM_Set_Goal_1_Station,
	LMM_Set_Goal_n_Station
} LinkMovingMode;

LinkMovingMode MovingLinkMode = LMM_Not_Set;

uint8_t Current_Station = 0;
uint8_t NumberOfStationToGo = 0;
uint8_t NumberOfStationPTR = 0;
float Plant_input = 0;
uint8_t Moving_Link_Task_Flag = 0;

uint8_t sethomeTrigger = 0;

uint8_t GripperEnable = 0;
uint8_t GripperState = 0;
uint8_t GripperStatus[1] = {0};
uint64_t Timestamp_Gripper = 0;

uint8_t AcceptableError = 8;
float StabilizePosition = 0;
float StabilizeVelocity = 0;

uint64_t Verified_Timestamp = 0;

PIDStructure PositionPIDController = {0};
PIDStructure VelocityPIDController  = {0};
PIDStructure PureVelocityPIDController = {0};
PIDStructure StabilizerPIDController = {0};

TrajectoryGenerationStructure TrjStruc = {0};
ConverterUnitSystemStructure CUSSStruc = {0};

uint8_t PIDTunerMode =  0;
float EstimatedAngularAcceration = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
uint64_t micros();
void EncoderVelocityAndPosition_Update();

void ConverterUnitSystemStructureInit(ConverterUnitSystemStructure *CUSSvar);
void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar, ConverterUnitSystemStructure *CUSSvar);

void TrajectoryGenerationPrepareDATA();
void TrajectoryGenerationCalculation();
void TrajectoryGenerationProcess();

void PIDController2in1();
void StabilizerPID();

void UARTInit(UARTStucrture *uart);
void UARTResetStart(UARTStucrture *uart);
uint32_t UARTGetRxHead(UARTStucrture *uart);
int16_t UARTReadChar(UARTStucrture *uart);
void UARTTxDumpBuffer(UARTStucrture *uart);
void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len);
void Munmunbot_Protocol(int16_t dataIn,UARTStucrture *uart);
void Encoder_SetHome_Position();

void ACK1Return(UARTStucrture *uart);
void ACK2Return(UARTStucrture *uart);

void PID_Reset();

void LAMP_ON(uint8_t lampnumber);
void Emergency_switch_trigger();

void Controlling_the_LINK();
void Stabilizing_the_LINK( float Position );
void HackTheLink( float Position );

void SETHOME_StateMachine_Function();
void SETHOME_TrajectoryGenerationPrepareDATA();
void Controlling_the_LINK_Velo( float Velocity );

void UpdateMunmunBotState();
void EndEffectorWorkingState();

void StabilizerPIDLoad();
void LinkMovingPIDLoad();
void VelocityPurePIDLoad();
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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);

	// start PWM
  HAL_TIM_Base_Start(&htim3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);

  ///Init Data
  ConverterUnitSystemStructureInit(&CUSSStruc);
  TrajectoryGenerationStructureInit(&TrjStruc, &CUSSStruc);

  StabilizerPIDLoad();
  LinkMovingPIDLoad();
  VelocityPurePIDLoad();

  Encoder_SetHome_Position();

  ///UART init
  UART2.huart = &huart2;
  UART2.RxLen = 255;
  UART2.TxLen = 255;
  UARTInit(&UART2);
  UARTResetStart(&UART2);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	  int16_t inputChar = UARTReadChar(&UART2);
	  if (inputChar != -1)
	  {
		  Munmunbot_Protocol(inputChar, &UART2);

	  }

	  switch (Munmunbot_State)
	  {
	  	  case STATE_Disconnected:
	  		  if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 0)
	  		  {
	  			  LAMP_ON(0);
	  		  }
	  		  else
	  		  {
	  			  LAMP_ON(4);
	  		  }
	  		  UpdateMunmunBotState();
	  		  sethomeTrigger = 0;
	  		  Emergency_switch_trigger();
	  		  break;

	  	  case STATE_Idle:
	  		  LAMP_ON(1);
	  		  UpdateMunmunBotState();
//	  		  Stabilizing_the_LINK( StabilizePosition );
//	  		  Controlling_the_LINK_Velo( StabilizeVelocity );
	  		  if ( sethomeTrigger == 1 )
	  		  {
	  			  Encoder_SetHome_Position();
	  		  }
	  		  Emergency_switch_trigger();
		  	  break;

	  	  case STATE_PrepareDATA:
	  		  sethomeTrigger = 0;
	  		  UpdateMunmunBotState();
	  		  LAMP_ON(2);
	  		  TrajectoryGenerationPrepareDATA();
	  		  Emergency_switch_trigger();
		  	  break;

	  	  case STATE_Calculation:
	  		  LAMP_ON(2);
	  		  UpdateMunmunBotState();
	  		  TrajectoryGenerationCalculation();
	  		  Munmunbot_State = STATE_Link_Moving;
	  		  Emergency_switch_trigger();
	  		  break;

	   	  case STATE_Link_Moving:
	   		  LAMP_ON(2);
	   		  if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	   		  {
	   			  Controlling_the_LINK();


				  if (Moving_Link_Task_Flag == 1)
				  {
					  Munmunbot_State = STATE_Stabilized_Link;
					  StabilizePosition = TrjStruc.Desire_Theta;
					  PID_Reset();
				  }
	   		  }
	  		  Emergency_switch_trigger();
	  		  break;

	   	  case STATE_Stabilized_Link:
	   		  LAMP_ON(3);
//	   		  Stabilizing_the_LINK( StabilizePosition );
	   		  HackTheLink( StabilizePosition );
			  if ((PositionPIDController.OutputFeedback <= TrjStruc.Desire_Theta + AcceptableError) &&
					  (PositionPIDController.OutputFeedback >= TrjStruc.Desire_Theta - AcceptableError) &&
					  (Moving_Link_Task_Flag == 1))
			  {
				    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
					Munmunbot_State = STATE_Verified;
					Verified_Timestamp = micros();
					PID_Reset();
			  }
	   		  Emergency_switch_trigger();
	   		  break;
	   	  case STATE_Verified:
	   	  {
	   		  LAMP_ON(3);
	   		  UpdateMunmunBotState();
	   		  if ( micros()-Verified_Timestamp >= 0.5*1000000  )
	   		  {

				  if ((PositionPIDController.OutputFeedback <= TrjStruc.Desire_Theta + AcceptableError) &&
						  (PositionPIDController.OutputFeedback >= TrjStruc.Desire_Theta - AcceptableError) &&
						  (Moving_Link_Task_Flag == 1))
				  {
					  if(MovingLinkMode == LMM_Set_Pos_Directly)
					  {
						Munmunbot_State = STATE_Idle;
						MovingLinkMode = LMM_Not_Set;
						StabilizePosition = TrjStruc.Desire_Theta;
						PID_Reset();
						ACK2Return(&UART2);
					  }

					  else if ((MovingLinkMode == LMM_Set_Goal_1_Station) || (MovingLinkMode == LMM_Set_Goal_n_Station))
					  {
						Munmunbot_State = STATE_End_Effector_Working;
						GripperState = 0;
						StabilizePosition = TrjStruc.Desire_Theta;
						PID_Reset();
					  }
					 __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
					 Moving_Link_Task_Flag = 0;
				  }
				  else
				  {
					  Munmunbot_State =  STATE_Stabilized_Link;
				  }
	   		  }
	   		  Emergency_switch_trigger();
	   		  break;
	   	  }
	  	  case STATE_End_Effector_Working:
	  		  UpdateMunmunBotState();
	  		  EndEffectorWorkingState();
	  		  Emergency_switch_trigger();
	  		  break;

	  	  case STATE_SetHome:
	  		  sethomeTrigger = 0;
	  		  LAMP_ON(2);
	  		  UpdateMunmunBotState();
	  		  SETHOME_StateMachine_Function();
	  		  Emergency_switch_trigger();
	  		  break;

	  }

	  if ( pidSetZeroFlag != 0 )
	  {
		  pidSetZeroFlag = 0;
		  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		  TrjStruc.Start_Theta = PositionPIDController.OutputFeedback;  //set new start theta
		  TrjStruc.AngularVelocityDesire = 0;
		  PID_Reset();
	  }



	  UARTTxDumpBuffer(&UART2);
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
  RCC_OscInitStruct.PLL.PLLN = 100;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK)
  {
    Error_Handler();
  }
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
  hi2c1.Init.ClockSpeed = 10000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

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

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 24575;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 2;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&htim1, &sConfig) != HAL_OK)
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
  htim2.Init.Prescaler = 99;
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
  htim3.Init.Period = 10000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

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
  huart2.Init.BaudRate = 512000;
  huart2.Init.WordLength = UART_WORDLENGTH_9B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_EVEN;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
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
  /* DMA1_Stream5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);
  /* DMA1_Stream6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, Lamp1_Pin|Lamp2_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Motor_DIR_GPIO_Port, Motor_DIR_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Lamp3_GPIO_Port, Lamp3_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD2_Pin Lamp3_Pin */
  GPIO_InitStruct.Pin = LD2_Pin|Lamp3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : Lamp1_Pin Lamp2_Pin */
  GPIO_InitStruct.Pin = Lamp1_Pin|Lamp2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : Emergency_Switch_Signal_Pin */
  GPIO_InitStruct.Pin = Emergency_Switch_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  HAL_GPIO_Init(Emergency_Switch_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Index_Signal_Pin */
  GPIO_InitStruct.Pin = Index_Signal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Index_Signal_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Motor_DIR_Pin */
  GPIO_InitStruct.Pin = Motor_DIR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Motor_DIR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LimitSwitchSignal_Pin */
  GPIO_InitStruct.Pin = LimitSwitchSignal_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LimitSwitchSignal_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

#define  HTIM_ENCODER htim1
#define  MAX_SUBPOSITION_OVERFLOW 12288
#define  MAX_ENCODER_PERIOD 24576

void EncoderVelocityAndPosition_Update()
{
	//Save Last state
	static uint32_t EncoderLastPosition = 0;
	static uint64_t EncoderLastTimestamp = 0;

	static float EncoderLastVelocity = 0;
	static float Velocity_Output = 0;
	static float Acceration_Output = 0;
	//read data
	uint32_t EncoderNowPosition = HTIM_ENCODER.Instance->CNT;
	uint64_t EncoderNowTimestamp = micros();

	int32_t EncoderPositionDiff;
	float EncoderVelocityDiff;
	uint64_t EncoderTimeDiff;

	EncoderTimeDiff = EncoderNowTimestamp - EncoderLastTimestamp;
	EncoderPositionDiff = EncoderNowPosition - EncoderLastPosition;

	//Update Position and time
	EncoderLastPosition = EncoderNowPosition;
	EncoderLastTimestamp = EncoderNowTimestamp;

	//Calculate velocity and Encoder Pos
	PositionPIDController.OutputFeedback = EncoderNowPosition;
	StabilizerPIDController.OutputFeedback = EncoderNowPosition;
	// LPF
	Velocity_Output = (EncoderPositionDiff * 1000000) / (float) EncoderTimeDiff;  /// Pulse per second
	VelocityPIDController.OutputFeedback = (Velocity_Output + (VelocityPIDController.OutputFeedback*299))/300.0;
//	VelocityPIDController.OutputFeedback = (Velocity_Output + (VelocityPIDController.OutputFeedback*49))/50.0;

	float EncoderNowVelocity = VelocityPIDController.OutputFeedback;

	EncoderVelocityDiff = EncoderNowVelocity - EncoderLastVelocity;
	EncoderLastVelocity = VelocityPIDController.OutputFeedback;

	Acceration_Output = ( EncoderVelocityDiff * 1000000)  / (float) EncoderTimeDiff;
	Acceration_Output = (Acceration_Output + (EstimatedAngularAcceration*299))/300.0;
//	Acceration_Output = (Acceration_Output + (EstimatedAngularAcceration*49))/50.0;
	EstimatedAngularAcceration = ( Acceration_Output );

}


void Encoder_SetHome_Position()
{
	HTIM_ENCODER.Instance->CNT = CUSSStruc.PPRxQEI;
	StabilizePosition = CUSSStruc.PPRxQEI;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if (htim == &htim2)
	{
		_micros += 4294967295;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if ((GPIO_Pin == GPIO_PIN_13) || (GPIO_Pin == GPIO_PIN_8))  //13 -> BlueButton, 8 -> Limitswitch
	{
    	if (Munmunbot_State == STATE_SetHome)
    	{
    		if (SethomeMode == SetHomeState_1)
    		{
//    			Encoder_SetHome_Position();
    			SethomeMode = SetHomeState_2;
//    			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
//				__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
    		}
    	}
	}

}

uint64_t micros()
{
	return _micros + htim2.Instance->CNT;
}

void ConverterUnitSystemStructureInit(ConverterUnitSystemStructure *CUSSvar)
{
	CUSSvar->PPR = 2048;
	CUSSvar->PPRxQEI = CUSSvar->PPR * 4;
	CUSSvar->RPMp = 10;
}

void TrajectoryGenerationStructureInit(TrajectoryGenerationStructure *TGSvar , ConverterUnitSystemStructure *CUSSvar)
{
	TGSvar->AngularAccerationMax_Setting = (0.4*(CUSSvar->PPRxQEI))/(3.1416*2.0);
	TGSvar->AngularVelocityMax_Setting = ((CUSSvar->PPRxQEI)*10)/(60.0);  //pps
	TGSvar->Start_Theta = CUSSStruc.PPRxQEI;  /// PPRxQEI == 0 degree
	TGSvar->Mode = 0;
	TGSvar->Submode = 0;
	TGSvar->Loop_Freq = 1000;
	TGSvar->Loop_Period = 1000000/(TGSvar->Loop_Freq);
	TGSvar->BlendTimeLSPB = TGSvar->AngularVelocityMax_Setting/(TGSvar->AngularAccerationMax_Setting);
	TGSvar->Theta_min_for_LSPB = TGSvar->AngularVelocityMax_Setting*TGSvar->BlendTimeLSPB;
	TGSvar->AngularVelocityFinalMin = 0*1.8*(CUSSvar->PPRxQEI)/60.0;
	TGSvar->Alpha = 1;

}

void LinkMovingPIDLoad()
{
	PositionPIDController.Kp = 0;
	PositionPIDController.Ki = 0;
	PositionPIDController.Kd = 0;
	PositionPIDController.offSet = 0;
	PositionPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;

	VelocityPIDController.Kp = 6;
	VelocityPIDController.Ki = 10;
	VelocityPIDController.Kd = 0.00005;
	VelocityPIDController.offSet = 1500;
	VelocityPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;
}

void LinkMovingPID40to355Load()
{
	PositionPIDController.Kp = 0;
	PositionPIDController.Ki = 0;
	PositionPIDController.Kd = 0;
	PositionPIDController.offSet = 0;
	PositionPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;

	VelocityPIDController.Kp = 5;
	VelocityPIDController.Ki = 10;
	VelocityPIDController.Kd = 0.00005;
	VelocityPIDController.offSet = 1500;
	VelocityPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;
}

void LinkMovingPID20to40Load()
{
	PositionPIDController.Kp = 0;
	PositionPIDController.Ki = 0;
	PositionPIDController.Kd = 0;
	PositionPIDController.offSet = 0;
	PositionPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;

	VelocityPIDController.Kp = 5;
	VelocityPIDController.Ki = 10;
	VelocityPIDController.Kd = 0.00005;
	VelocityPIDController.offSet = 1500;
	VelocityPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;
}

void StabilizerPIDLoad()
{
//	StabilizerPIDController.Kp = 35;
//	StabilizerPIDController.Ki = 17;
	StabilizerPIDController.Kp = 0.0000000000001;
	StabilizerPIDController.Ki = 1.3;
	StabilizerPIDController.Kd = 0;
	StabilizerPIDController.offSet = 1200;
	StabilizerPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;
}

void VelocityPurePIDLoad()
{
	PureVelocityPIDController.Kp = 5;
	PureVelocityPIDController.Ki = 15;
	PureVelocityPIDController.Kd = 0.00005;
	PureVelocityPIDController.offSet = 1500;
	PureVelocityPIDController.SamplingTime = ( TrjStruc.Loop_Period )/1000000.0;
}


void TrajectoryGenerationVelocityMaxSetting(TrajectoryGenerationStructure *TGSvar , ConverterUnitSystemStructure *CUSSvar)
{
	TGSvar->AngularVelocityMax_Setting = ((CUSSvar->PPRxQEI)*(CUSSvar->RPMp))/(60.0);   ///RPM to pps
	TGSvar->BlendTimeLSPB = TGSvar->AngularVelocityMax_Setting/(TGSvar->AngularAccerationMax_Setting);
	TGSvar->Theta_min_for_LSPB = TGSvar->AngularVelocityMax_Setting*TGSvar->BlendTimeLSPB;
}

void TrajectoryGenerationPrepareDATA()
{
	// fix start position base on Encoder
	TrjStruc.Start_Theta = htim1.Instance->CNT;

	if (MovingLinkMode == LMM_Set_Pos_Directly)
	  {
		  TrjStruc.Desire_Theta = (Angularpos_InputNumber*CUSSStruc.PPRxQEI/(10000.0*2.0*3.14159));  //pulse
		  if (TrjStruc.Desire_Theta >= CUSSStruc.PPRxQEI)   ///wrap input into 1 revolute.
		  {
			 TrjStruc.Desire_Theta -= CUSSStruc.PPRxQEI;
		  }
		  TrjStruc.Desire_Theta += CUSSStruc.PPRxQEI;  /// set to middle range

		  if (TrjStruc.Desire_Theta != TrjStruc.Start_Theta)
		  {
			  TrjStruc.Delta_Theta = TrjStruc.Desire_Theta - TrjStruc.Start_Theta;
			  Munmunbot_State = STATE_Calculation;
		  }
		  else
		  {
			Munmunbot_State = STATE_Idle;
			MovingLinkMode = LMM_Not_Set;
			ACK2Return(&UART2);
		}
	  }

	else if (MovingLinkMode == LMM_Set_Goal_1_Station || MovingLinkMode == LMM_Set_Goal_n_Station )
	  {
		  if (NumberOfStationToGo == 0)
			{
				Munmunbot_State = STATE_Idle;
				NumberOfStationPTR = 0;
				NumberOfStationToGo = 0;
				MovingLinkMode = LMM_Not_Set;
				ACK2Return(&UART2);
			}
		  else
		  {
			Current_Station = Angularpos_InputArray[NumberOfStationPTR];
			if (Current_Station > 10)   //pass input
			{
				NumberOfStationPTR += 1;
				NumberOfStationToGo -= 1;
			}
			else
			{
				TrjStruc.Desire_Theta = (StationPos[Current_Station-1]*CUSSStruc.PPRxQEI/(360.0))*5.0;   ///fix this if change algorithm
				if (TrjStruc.Desire_Theta >= CUSSStruc.PPRxQEI)  ///wrap input into 1 revolute. ///shouldn't happen
				{
					TrjStruc.Desire_Theta -= CUSSStruc.PPRxQEI;
				}
				TrjStruc.Desire_Theta += CUSSStruc.PPRxQEI;  /// set to middle range
				if (TrjStruc.Desire_Theta == TrjStruc.Start_Theta)
				{
					NumberOfStationPTR += 1;
					NumberOfStationToGo -= 1;
					Munmunbot_State = STATE_End_Effector_Working;
				}
				else
				{
					TrjStruc.Delta_Theta = TrjStruc.Desire_Theta - TrjStruc.Start_Theta;
					Munmunbot_State = STATE_Calculation;
					NumberOfStationPTR += 1;
					NumberOfStationToGo -= 1;
				}
			}
		  }
	  }
	  else  ///shouldn't happen
	  {
		MovingLinkMode = LMM_Not_Set;
		Munmunbot_State = STATE_Idle;
		ACK2Return(&UART2);
	  }
}

void TrajectoryGenerationCalculation()
{
	if (TrjStruc.Delta_Theta < 0)
	  {
		 TrjStruc.AngularAcceration = TrjStruc.AngularAccerationMax_Setting * -1;
		 TrjStruc.AngularVelocity = TrjStruc.AngularVelocityMax_Setting *-1;
		 TrjStruc.Abs_Delta_Theta = TrjStruc.Delta_Theta * -1;
		 TrjStruc.Alpha = -1;
	  }
	  else if (TrjStruc.Delta_Theta > 0)
	  {
		 TrjStruc.AngularAcceration = TrjStruc.AngularAccerationMax_Setting;
		 TrjStruc.AngularVelocity = TrjStruc.AngularVelocityMax_Setting;
		 TrjStruc.Abs_Delta_Theta = TrjStruc.Delta_Theta;
		 TrjStruc.Alpha = 1;
	  }

	  if (TrjStruc.Abs_Delta_Theta < TrjStruc.Theta_min_for_LSPB)   ///Triangular mode0
	  {
		 TrjStruc.BlendTimeTriangular = sqrt(TrjStruc.Abs_Delta_Theta/TrjStruc.AngularAccerationMax_Setting);
		 TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
		 TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.BlendTimeTriangular*TrjStruc.BlendTimeTriangular))/2.0) + TrjStruc.Theta_Stamp_0;
		 TrjStruc.Mode = 0;
		 TrjStruc.Submode = 0;
		 TrjStruc.Subsubmode = 0;
	  }
	  else if (TrjStruc.Abs_Delta_Theta >= TrjStruc.Theta_min_for_LSPB)  ///LSPB mode1
	  {
		  TrjStruc.LinearTimeLSPB = (TrjStruc.Abs_Delta_Theta-TrjStruc.Theta_min_for_LSPB)/TrjStruc.AngularVelocityMax_Setting;
		  TrjStruc.Theta_Stamp_0 = TrjStruc.Start_Theta;
		  TrjStruc.Theta_Stamp_1 = ((TrjStruc.AngularAcceration*(TrjStruc.BlendTimeLSPB*TrjStruc.BlendTimeLSPB))/2.0) + TrjStruc.Theta_Stamp_0;
		  TrjStruc.Theta_Stamp_2 = (TrjStruc.AngularVelocity*TrjStruc.LinearTimeLSPB) + TrjStruc.Theta_Stamp_1;
		  TrjStruc.Mode = 1;
		  TrjStruc.Submode = 0;
		  TrjStruc.Subsubmode = 0;
	  }
	 TrjStruc.Equation_Timestamp = micros();
	 TrjStruc.Loop_Timestamp = micros();
}

void TrajectoryGenerationProcess()
{

	TrjStruc.Equation_Realtime_Sec = (micros()-TrjStruc.Equation_Timestamp)/1000000.0;

	 switch (TrjStruc.Mode)
	  {
		  case 0: ///Triangular
			  if (TrjStruc.Submode == 0)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  +TrjStruc.Theta_Stamp_0;

				  TrjStruc.AngularVelocityDesire = TrjStruc.AngularAcceration * TrjStruc.Equation_Realtime_Sec;

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 1;
				  }
			  }
			  else if (TrjStruc.Submode == 1)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*-0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  + (TrjStruc.AngularAcceration*TrjStruc.BlendTimeTriangular*(TrjStruc.Equation_Realtime_Sec))
						  + TrjStruc.Theta_Stamp_1;

//				  Moving_Link_Task_Flag = 1;

				  if ( TrjStruc.Subsubmode == 0 )
				  {
					  TrjStruc.AngularVelocityDesire = ( -1.0*TrjStruc.AngularAcceration*TrjStruc.Equation_Realtime_Sec ) +
													   (TrjStruc.AngularAcceration*TrjStruc.BlendTimeTriangular);

					  if ( TrjStruc.Alpha * TrjStruc.AngularVelocityDesire <= TrjStruc.AngularVelocityFinalMin )
					  {
						  TrjStruc.Subsubmode = 1;
						  TrjStruc.Velocity_Timestamp = micros();
					  }

				  }
				  else if ( TrjStruc.Subsubmode == 1 )
				  {
					  TrjStruc.AngularVelocityDesire = TrjStruc.Alpha * TrjStruc.AngularVelocityFinalMin;
					  if (micros()-TrjStruc.Velocity_Timestamp >= ((TrjStruc.BlendTimeTriangular*1000000)-(TrjStruc.Velocity_Timestamp - TrjStruc.Equation_Timestamp))/2.0)
					  {
						  TrjStruc.Subsubmode = 2;
					  }
				  }
				  else if ( TrjStruc.Subsubmode == 2 )
				  {
					  TrjStruc.AngularVelocityDesire = 0;
				  }

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeTriangular*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 0;
					  TrjStruc.Subsubmode = 0;
					  TrjStruc.Mode = 2; ///Final Value Mode
				  }
			  }
			  break;

		  case 1: ///LSPB
			  if (TrjStruc.Submode == 0)
			  {
				  TrjStruc.AngularDisplacementDesire =
							((TrjStruc.AngularAcceration*0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
							+TrjStruc.Theta_Stamp_0;

				  TrjStruc.AngularVelocityDesire = TrjStruc.AngularAcceration * TrjStruc.Equation_Realtime_Sec;

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 1;
				  }
			  }
			  else if (TrjStruc.Submode == 1)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  (TrjStruc.AngularVelocity*(TrjStruc.Equation_Realtime_Sec))
						  +TrjStruc.Theta_Stamp_1;

				  TrjStruc.AngularVelocityDesire = TrjStruc.AngularVelocity;

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.LinearTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 2;
				  }
			  }
			  else if (TrjStruc.Submode == 2)
			  {
				  TrjStruc.AngularDisplacementDesire =
						  ((TrjStruc.AngularAcceration*-0.5)*(TrjStruc.Equation_Realtime_Sec*TrjStruc.Equation_Realtime_Sec))
						  + (TrjStruc.AngularVelocity*(TrjStruc.Equation_Realtime_Sec))
						  + TrjStruc.Theta_Stamp_2;

				  if ( TrjStruc.Subsubmode == 0 )
				  {
					  TrjStruc.AngularVelocityDesire = ( -1*TrjStruc.AngularAcceration*TrjStruc.Equation_Realtime_Sec )
							                           + ( TrjStruc.AngularVelocity );

					  if (  TrjStruc.Alpha * TrjStruc.AngularVelocityDesire <=  TrjStruc.AngularVelocityFinalMin )
					  {
						  TrjStruc.Subsubmode = 1;
						  TrjStruc.Velocity_Timestamp = micros();
					  }
				  }
				  else if ( TrjStruc.Subsubmode == 1 )
				  {
					  TrjStruc.AngularVelocityDesire = TrjStruc.Alpha * TrjStruc.AngularVelocityFinalMin;
					  if (micros()-TrjStruc.Velocity_Timestamp >= ((TrjStruc.BlendTimeLSPB*1000000)-(TrjStruc.Velocity_Timestamp - TrjStruc.Equation_Timestamp))/2.0)
					  {
						  TrjStruc.Subsubmode = 2;
					  }
				  }
				  else if ( TrjStruc.Subsubmode == 2 )
				  {
					  TrjStruc.AngularVelocityDesire = 0;
				  }

				  if (micros()-TrjStruc.Equation_Timestamp >= TrjStruc.BlendTimeLSPB*1000000)
				  {
					  TrjStruc.Equation_Timestamp = micros();
					  TrjStruc.Submode = 0;
					  TrjStruc.Subsubmode = 0;
					  TrjStruc.Mode = 2; ///Final Value Mode
				  }
			  }
			  break;
		  case 2:
			  Moving_Link_Task_Flag = 1;
			  TrjStruc.AngularDisplacementDesire = TrjStruc.Desire_Theta;
			  TrjStruc.AngularVelocityDesire = 0;
			  break;
		  }
}

void PIDController2in1()
{
	PositionPIDController.OutputDesire = TrjStruc.AngularDisplacementDesire;
    PositionPIDController.NowError = PositionPIDController.OutputDesire - PositionPIDController.OutputFeedback;
    PositionPIDController.Integral_Value += PositionPIDController.NowError*PositionPIDController.SamplingTime;
    PositionPIDController.ControllerOutput = (PositionPIDController.Kp*PositionPIDController.NowError)
					  +(PositionPIDController.Ki * PositionPIDController.Integral_Value)
					  +(PositionPIDController.Kd * (PositionPIDController.NowError-PositionPIDController.PreviousError)/PositionPIDController.SamplingTime);
    PositionPIDController.PreviousError = PositionPIDController.NowError;

    VelocityPIDController.OutputDesire = PositionPIDController.ControllerOutput + TrjStruc.AngularVelocityDesire;
    VelocityPIDController.NowError = VelocityPIDController.OutputDesire - VelocityPIDController.OutputFeedback;
    VelocityPIDController.Integral_Value += VelocityPIDController.NowError*VelocityPIDController.SamplingTime;
    VelocityPIDController.ControllerOutput = (VelocityPIDController.Kp*VelocityPIDController.NowError)
					  +(VelocityPIDController.Ki * VelocityPIDController.Integral_Value)
					  +(VelocityPIDController.Kd * (VelocityPIDController.NowError-VelocityPIDController.PreviousError)/VelocityPIDController.SamplingTime)
					  +( TrjStruc.Alpha * VelocityPIDController.offSet );
    VelocityPIDController.PreviousError = VelocityPIDController.NowError;

}

void StabilizerPID()
{
	StabilizerPIDController.OutputDesire = TrjStruc.AngularDisplacementDesire;
	StabilizerPIDController.NowError = StabilizerPIDController.OutputDesire - StabilizerPIDController.OutputFeedback;
	if (( StabilizerPIDController.NowError <= AcceptableError ) && ( StabilizerPIDController.NowError >= -1.0*AcceptableError ))
	{
		StabilizerPIDController.NowError = 0;
	}
	else if ( StabilizerPIDController.NowError < 0 )
	{
		TrjStruc.Alpha = -1;
	}
	else if ( StabilizerPIDController.NowError >= 0 )
	{
		TrjStruc.Alpha = 1;
	}
	StabilizerPIDController.Integral_Value += StabilizerPIDController.NowError*StabilizerPIDController.SamplingTime;
	StabilizerPIDController.ControllerOutput = (StabilizerPIDController.Kp*StabilizerPIDController.NowError)
					  +(StabilizerPIDController.Ki * StabilizerPIDController.Integral_Value)
					  +(StabilizerPIDController.Kd * (StabilizerPIDController.NowError-StabilizerPIDController.PreviousError)/StabilizerPIDController.SamplingTime)
					  +( TrjStruc.Alpha * StabilizerPIDController.offSet );

	StabilizerPIDController.PreviousError = StabilizerPIDController.NowError;
}

void PureVeloPID()
{
	PureVelocityPIDController.OutputDesire = TrjStruc.AngularVelocityDesire;
	PureVelocityPIDController.NowError = PureVelocityPIDController.OutputDesire - VelocityPIDController.OutputFeedback;
	PureVelocityPIDController.Integral_Value += PureVelocityPIDController.NowError*PureVelocityPIDController.SamplingTime;
	PureVelocityPIDController.ControllerOutput = (PureVelocityPIDController.Kp*PureVelocityPIDController.NowError)
					  +(PureVelocityPIDController.Ki * PureVelocityPIDController.Integral_Value)
					  +(PureVelocityPIDController.Kd * (PureVelocityPIDController.NowError-PureVelocityPIDController.PreviousError)/PureVelocityPIDController.SamplingTime)
					  +( TrjStruc.Alpha * PureVelocityPIDController.offSet );
	PureVelocityPIDController.PreviousError = PureVelocityPIDController.NowError;
}



///UART ZONE
void UARTInit(UARTStucrture *uart)
{
	//dynamic memory allocate
	uart->RxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.RxLen);
	uart->TxBuffer = (uint8_t*) calloc(sizeof(uint8_t), UART2.TxLen);
	uart->RxTail = 0;
	uart->TxTail = 0;
	uart->TxHead = 0;
}

void UARTResetStart(UARTStucrture *uart)
{
	HAL_UART_Receive_DMA(uart->huart, uart->RxBuffer, uart->RxLen);
}

uint32_t UARTGetRxHead(UARTStucrture *uart)
{
	return uart->RxLen - __HAL_DMA_GET_COUNTER(uart->huart->hdmarx);
}

int16_t UARTReadChar(UARTStucrture *uart)
{
	int16_t Result = -1; // -1 Mean no new data

	//check Buffer Position
	if (uart->RxTail != UARTGetRxHead(uart))
	{
		//get data from buffer
		Result = uart->RxBuffer[uart->RxTail];
		uart->RxTail = (uart->RxTail + 1) % uart->RxLen;

	}
	return Result;
}

void UARTTxDumpBuffer(UARTStucrture *uart)
{
	static uint8_t MultiProcessBlocker = 0;

	if (uart->huart->gState == HAL_UART_STATE_READY && !MultiProcessBlocker)
	{
		MultiProcessBlocker = 1;

		if (uart->TxHead != uart->TxTail)
		{
			//find len of data in buffer (Circular buffer but do in one way)
			uint16_t sentingLen =
					uart->TxHead > uart->TxTail ?
							uart->TxHead - uart->TxTail :
							uart->TxLen - uart->TxTail;

			//sent data via DMA
			HAL_UART_Transmit_DMA(uart->huart, &(uart->TxBuffer[uart->TxTail]),
					sentingLen);
			//move tail to new position
			uart->TxTail = (uart->TxTail + sentingLen) % uart->TxLen;

		}
		MultiProcessBlocker = 0;
	}
}

void UARTTxWrite(UARTStucrture *uart, uint8_t *pData, uint16_t len)
{
	//check data len is more than buffur?
	uint16_t lenAddBuffer = (len <= uart->TxLen) ? len : uart->TxLen;
	// find number of data before end of ring buffer
	uint16_t numberOfdataCanCopy =
			lenAddBuffer <= uart->TxLen - uart->TxHead ?
					lenAddBuffer : uart->TxLen - uart->TxHead;
	//copy data to the buffer
	memcpy(&(uart->TxBuffer[uart->TxHead]), pData, numberOfdataCanCopy);

	//Move Head to new position

	uart->TxHead = (uart->TxHead + lenAddBuffer) % uart->TxLen;
	//Check that we copy all data That We can?
	if (lenAddBuffer != numberOfdataCanCopy)
	{
		memcpy(uart->TxBuffer, &(pData[numberOfdataCanCopy]),
				lenAddBuffer - numberOfdataCanCopy);
	}
	UARTTxDumpBuffer(uart);

}

void ACK1Return(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {0x58, 0b01110101};
	UARTTxWrite(uart, temp, 2);
	}
}

void ACK2Return(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {70, 110};
	UARTTxWrite(uart, temp, 2);
	}
}

void DEBUG_UART(UARTStucrture *uart)
{
	{
	uint8_t temp[] = {0xff, 0xff};
	UARTTxWrite(uart, temp, 2);
	}
}

void Munmunbot_Protocol(int16_t dataIn,UARTStucrture *uart)
{
	//static PPSTATE Munmunbot_Protocol_State = PP_STARTandMode;
	static uint16_t n_station = 0;
	static uint16_t n_station_mem= 0;
	static uint8_t ProtocolMode = 0;
	static uint8_t parameter[256] = {0};
	static uint8_t parameter_ptr = 0;
	static uint16_t Data_HAck = 0;
	static uint32_t CheckSum = 0;
	static uint16_t DataForReturn = 0;

	switch (Munmunbot_Protocol_State)
	{
		case PP_STARTandMode:
			if (((dataIn>>4) & 0b1111) == 0b1001)
			{
				CheckSum = dataIn;
				ProtocolMode = dataIn & 0b1111;

				if (ProtocolMode == 7)
				{
					Munmunbot_Protocol_State = PP_Frame3_Data_0; ///Frame3
				}
				else if (ProtocolMode == 1 || ProtocolMode == 4 || ProtocolMode == 5 ||ProtocolMode == 6)
				{
					Munmunbot_Protocol_State = PP_Frame2_Data_0; //Frame2
				}
				else if (ProtocolMode == 2 || ProtocolMode == 3 || ProtocolMode == 8 || ProtocolMode == 9 || ProtocolMode == 10 ||
						ProtocolMode == 11 ||ProtocolMode == 12 ||ProtocolMode == 13 || ProtocolMode == 14 )
				{
					Munmunbot_Protocol_State = PP_CheckSum;   /// Frame1
				}
			}
			break;
		case PP_Frame2_Data_0:
			 CheckSum += dataIn;
			 Data_HAck = ((dataIn&0b11111111)<<8)&0b1111111100000000;
			 parameter[0] = dataIn&0b1111;
			 parameter[1] = (dataIn>>4)&0b1111;
			 Munmunbot_Protocol_State = PP_Frame2_Data_1;

			 break;
		case PP_Frame2_Data_1:
			 CheckSum += dataIn;
			 Data_HAck = (dataIn&0b11111111) | Data_HAck;
			 parameter[2] = dataIn&0b1111;
			 parameter[3] = (dataIn>>4)&0b1111;
			 Munmunbot_Protocol_State = PP_CheckSum;
			 break;

		case PP_Frame3_Data_0:
		     CheckSum += dataIn;
		     n_station = dataIn;
		     n_station_mem = n_station;
		     Munmunbot_Protocol_State = PP_Frame3_Data_1;

		   break;

		case PP_Frame3_Data_1:
				CheckSum += dataIn;
				if (n_station >= 2)
				{
					parameter[parameter_ptr] = dataIn&0b1111;
					parameter_ptr += 1;
					parameter[parameter_ptr] = (dataIn>>4)&0b1111;
					parameter_ptr += 1;
					n_station -= 2;
				}
				else if (n_station == 1)
				{
					parameter[parameter_ptr] = dataIn&0b1111;
					n_station -= 1;
				}
				if  (n_station == 0)
				{
					Munmunbot_Protocol_State = PP_CheckSum;
				}
				break;

			case PP_CheckSum:
			{
				CheckSum = (~CheckSum) & 0xff;
				if (CheckSum == dataIn)
				{

					switch (ProtocolMode)
					{
					case 1: ///Test Command ##Complete##
						{
						uint8_t temp[] =
						{0b10010001,
						((parameter[1] & 0xff) << 4)  | (parameter[0]& 0xff),
						((parameter[3] & 0xff) << 4)  | (parameter[2]& 0xff),
						0b0 , 0x58 ,0x75 };
						temp[3] = (~(temp[0]+temp[1]+temp[2]))& 0xff;
						UARTTxWrite(uart, temp, 6);
						}
						break;
					case 2: //Connect MCU ##Complete##
						if (Munmunbot_State == STATE_Disconnected)
						{
							Munmunbot_State = STATE_Idle;
							PID_Reset();
						}
						ACK1Return(uart);
						break;
					case 3: //Disconnect MCU ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_Disconnected;
						}
						ACK1Return(uart);
						break;
					case 4: //Set Angular Velocity ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							CUSSStruc.RPMp = 0.925*(Data_HAck*10.0)/255.0;
							TrajectoryGenerationVelocityMaxSetting(&TrjStruc , &CUSSStruc);
						}
						ACK1Return(uart);
						break;
					case 5:   //Set Angular pos ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Angularpos_InputNumber = Data_HAck;
							MovingLinkMode = LMM_Set_Pos_Directly;
						}
						ACK1Return(uart);
						break;
					case 6:  /// Set 1 Station ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
//							Angularpos_InputArray[0] = parameter[0];
							Angularpos_InputArray[0] = parameter[2];    //150 0x00 0x0A Checksum
							MovingLinkMode = LMM_Set_Goal_1_Station;
							NumberOfStationToGo = 1;
						}
						ACK1Return(uart);
						break;
					case 7:  /// Set n Station ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							MovingLinkMode = LMM_Set_Goal_n_Station;
							for (int i = 0; i < n_station_mem; i++)
							{
								Angularpos_InputArray[i] = parameter[i];
							}
							NumberOfStationToGo = n_station_mem;
						}
						ACK1Return(uart);
						break;
					case 8:  /// Go go ##Complete##  ///But must implement return ACK after it's done
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_PrepareDATA;
							PID_Reset();
							ACK1Return(uart);
						}
						else
						{
							{
								uint8_t temp[] =
								{0x58 ,  0x75 ,
										70,  110};  ///ACK1 + ACK2
								UARTTxWrite(uart, temp, 4);
							}
						}
						break;

					case 9:  /// Return Current Station   ##Complete##

						{
							uint8_t temp[] =
							{0x58 ,  0x75 ,
									153,  0b0,  0b0, 0b0}; ///ACK1 + Mode 9
							uint8_t Shift = 2;
							DataForReturn = Current_Station&(0xff);
							temp[1+Shift] = (DataForReturn>>8)&(0xff);
							temp[2+Shift] = (DataForReturn)&(0xff);
							temp[3+Shift] = ~(temp[0+Shift]+temp[1+Shift]+temp[2+Shift]);
							UARTTxWrite(uart, temp, 4+Shift);
						}

						break;

					case 10: /// Return Angular Position  ##Complete##
						{
							uint8_t temp[] =
							{0x58 , 0x75 ,154, 0b0,  0b0, 0b0};
							uint8_t Shift = 2;
							DataForReturn = ((((int) htim1.Instance->CNT) % (CUSSStruc.PPRxQEI))*2*3.141*10000)/(CUSSStruc.PPRxQEI);  ///pulse to (radian*10000)
							temp[1+Shift] = (DataForReturn>>8)&(0xff);
							temp[2+Shift] = (DataForReturn)&(0xff);
							temp[3+Shift] = ~(temp[0+Shift]+temp[1+Shift]+temp[2+Shift]);
							UARTTxWrite(uart, temp, 4+Shift);
						}
						break;

					case 11: /// Return Angular Velocity Max  ##Complete##
						{
							uint8_t temp[] =
							{ 0x58, 0x75, 155, 0b0,  0b0, 0b0 };
							uint8_t Shift = 2;
							float temp3 = ( abs( VelocityPIDController.OutputFeedback )*60.0 )/( ( float ) CUSSStruc.PPRxQEI );
							DataForReturn = ( temp3 * 255.0 )/10.0;
							temp[ 1+Shift ] = ( DataForReturn>>8 )&( 0xff );
							temp[ 2+Shift ] = ( DataForReturn )&( 0xff );
							temp[ 3+Shift ] = ~( temp[ 0+Shift ]+temp[ 1+Shift ]+temp[ 2+Shift ] );
							UARTTxWrite( uart, temp, 4+Shift );
						}
						break;

					case 12:  //Enable Gripper
						if (Munmunbot_State == STATE_Idle)
						{
							GripperEnable = 1;
						}
						ACK1Return(uart);
						break;
					case 13: //Disable Gripper
						if (Munmunbot_State == STATE_Idle)
						{
							GripperEnable = 0;
						}
						ACK1Return(uart);
						break;

					case 14: /// Sethome  ##Complete##
						if (Munmunbot_State == STATE_Idle)
						{
							Munmunbot_State = STATE_SetHome;
							SethomeMode = SetHomeState_0;
							PID_Reset();
						}
						ACK1Return(uart);
						break;
				    }
			   }
			n_station = 0;
			ProtocolMode = 0;
			parameter_ptr = 0;
			Data_HAck = 0;
			CheckSum = 0;
			Munmunbot_Protocol_State = PP_STARTandMode;
			break;
			}
	}
}

void PID_Reset()
{
	PositionPIDController.PreviousError = 0;
	PositionPIDController.Integral_Value = 0;
	PositionPIDController.ControllerOutput = 0;
	PositionPIDController.NowError = 0;
	PositionPIDController.PreviousPreviousError = 0;
	PositionPIDController.PreviousControllerOutput = 0;


	VelocityPIDController.PreviousError = 0;
	VelocityPIDController.Integral_Value = 0;
	VelocityPIDController.ControllerOutput = 0;
	VelocityPIDController.NowError = 0;
	VelocityPIDController.PreviousPreviousError = 0;
	VelocityPIDController.PreviousControllerOutput = 0;

	StabilizerPIDController.PreviousError = 0;
	StabilizerPIDController.Integral_Value = 0;
	StabilizerPIDController.ControllerOutput = 0;
	StabilizerPIDController.NowError = 0;
	StabilizerPIDController.PreviousPreviousError = 0;
	StabilizerPIDController.PreviousControllerOutput = 0;

	PureVelocityPIDController.PreviousError = 0;
	PureVelocityPIDController.Integral_Value = 0;
	PureVelocityPIDController.ControllerOutput = 0;
	PureVelocityPIDController.NowError = 0;
	PureVelocityPIDController.PreviousPreviousError = 0;
	PureVelocityPIDController.PreviousControllerOutput = 0;
}

void LAMP_ON(uint8_t lampnumber)
{
	if (lampnumber == 0)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	}

	else if (lampnumber == 1)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	}
	else if (lampnumber == 2)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
	}
	else if (lampnumber == 3)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	}
	else if (lampnumber == 4)
	{
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
	}
}

void Emergency_switch_trigger()
{
	if (HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_13) == 1)
	{
		// Reset State Machine All
		Munmunbot_State = STATE_Disconnected;
		MovingLinkMode = LMM_Not_Set;
		SethomeMode = SetHomeState_0;
		TrjStruc.Mode = 0;
		TrjStruc.Submode = 0;

		// Send back ACK to User-interface
		if ((Munmunbot_State == STATE_Calculation) || (Munmunbot_State == STATE_PrepareDATA) ||
				(Munmunbot_State == STATE_Link_Moving) || (Munmunbot_State == STATE_End_Effector_Working))
		{
			ACK2Return(&UART2);
		}

		// Reset variable
		NumberOfStationToGo = 0;
		NumberOfStationPTR = 0;
		Moving_Link_Task_Flag = 0;

		// Stop the Motor
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
		TrjStruc.Start_Theta = PositionPIDController.OutputFeedback;  //set new start theta

		PID_Reset();
	}
}

void Controlling_the_LINK()
{
	  // GEN Trajectory
	  TrajectoryGenerationProcess();
	  EncoderVelocityAndPosition_Update();
	  PIDController2in1();  ///use only position
//	  Plant_input = PositionPIDController.ControllerOutput;
	  Plant_input = VelocityPIDController.ControllerOutput;

	  if (Plant_input >= 0) /// Setting DIR
	  {
		  DCMotorStruc.DIR = 1;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input);
	  }
	  else if (Plant_input < 0)
	  {
		  DCMotorStruc.DIR = 0;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input * -1.0);
	  }

	  if (DCMotorStruc.PWMOut > 10000)   /// Saturation Output
	  {
		 DCMotorStruc.PWMOut = 10000;
	  }

	  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, DCMotorStruc.DIR);

	  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DCMotorStruc.PWMOut); ///Setting PWM Pin
	  TrjStruc.Loop_Timestamp = micros();
}

void Stabilizing_the_LINK( float Position )
{
	if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	{
		TrjStruc.AngularDisplacementDesire = Position;
		EncoderVelocityAndPosition_Update();
		StabilizerPID();
		Plant_input = StabilizerPIDController.ControllerOutput;

		if (Plant_input >= 0) /// Setting DIR
		{
		  DCMotorStruc.DIR = 1;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input);
		}
		else if (Plant_input < 0)
		{
		  DCMotorStruc.DIR = 0;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input * -1.0);
		}

		if (DCMotorStruc.PWMOut > 10000)   /// Saturation Output
		{
		 DCMotorStruc.PWMOut = 10000;
		}

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, DCMotorStruc.DIR);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DCMotorStruc.PWMOut); ///Setting PWM Pin
		TrjStruc.Loop_Timestamp = micros();
	}
}

void Controlling_the_LINK_Velo( float Velocity )
{
	if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	{
		TrjStruc.AngularVelocityDesire = Velocity;
		EncoderVelocityAndPosition_Update();
		PureVeloPID();
		Plant_input = PureVelocityPIDController.ControllerOutput;

		if (Plant_input >= 0) /// Setting DIR
		{
		  DCMotorStruc.DIR = 1;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input);
		}
		else if (Plant_input < 0)
		{
		  DCMotorStruc.DIR = 0;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input * -1.0);
		}

		if (DCMotorStruc.PWMOut > 10000)   /// Saturation Output
		{
		 DCMotorStruc.PWMOut = 10000;
		}

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, DCMotorStruc.DIR);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DCMotorStruc.PWMOut); ///Setting PWM Pin
		TrjStruc.Loop_Timestamp = micros();
	}
}

void HackTheLink( float Position )
{
	if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	{
		TrjStruc.AngularDisplacementDesire = Position;
		EncoderVelocityAndPosition_Update();

		if ((PositionPIDController.OutputFeedback <= Position + AcceptableError) &&
				(PositionPIDController.OutputFeedback >= Position - AcceptableError))
		{
//			Plant_input = 1500;
		}
		else if ( Position - PositionPIDController.OutputFeedback >= 0)
		{
			TrjStruc.AngularVelocityDesire = 0.5*(8192.0/60.0);
		}
		else if ( Position - PositionPIDController.OutputFeedback < 0)
		{
			TrjStruc.AngularVelocityDesire = -0.5*(8192.0/60.0);
		}
		PureVeloPID();
		Plant_input = PureVelocityPIDController.ControllerOutput;

		if (Plant_input >= 0) /// Setting DIR
		{
		  DCMotorStruc.DIR = 1;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input);
		}
		else if (Plant_input < 0)
		{
		  DCMotorStruc.DIR = 0;
		  DCMotorStruc.PWMOut = (uint32_t) (Plant_input * -1.0);
		}

		if (DCMotorStruc.PWMOut > 10000)   /// Saturation Output
		{
		 DCMotorStruc.PWMOut = 10000;
		}

		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, DCMotorStruc.DIR);

		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, DCMotorStruc.PWMOut); ///Setting PWM Pin
		TrjStruc.Loop_Timestamp = micros();
	}
}

void SETHOME_StateMachine_Function()
{
	switch (SethomeMode)
	{
		case SetHomeState_0:
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, 0);
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2700);
			SethomeMode = SetHomeState_1;
			break;
		case SetHomeState_1:
			break;
		case SetHomeState_2:
			Munmunbot_State = STATE_Idle;
			MovingLinkMode = LMM_Not_Set;
			__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
			TrjStruc.Start_Theta = htim1.Instance->CNT;  //set new start theta
			Moving_Link_Task_Flag = 0;
			sethomeTrigger = 1;
			PID_Reset();

			break;

     }

}

void UpdateMunmunBotState()
{
	if (micros()-TrjStruc.Loop_Timestamp >=  TrjStruc.Loop_Period)
	{
		EncoderVelocityAndPosition_Update();
		TrjStruc.Loop_Timestamp = micros();
	}
}

void EndEffectorWorkingState()
{
	  if(GripperEnable == 1)
	  {
		if (GripperState == 0)
		{
			{
				uint8_t temp[1] = {0x45};
				HAL_I2C_Master_Transmit_IT(&hi2c1, (0x23 << 1) , temp, 1);
			}
			GripperState = 1;
			Timestamp_Gripper = micros();
		}

		else if (GripperState != 0)
		{
			if ((hi2c1.State == HAL_I2C_STATE_READY) && (GripperState == 1))
			{
				{
					uint8_t temp[1] = {0x23};
					HAL_I2C_Master_Transmit_IT(&hi2c1, (0x23 << 1) , temp, 1);
				}
				GripperState = 2;
			}

			else if ((hi2c1.State == HAL_I2C_STATE_READY) && ( GripperState == 2 ))
			{
				{
					HAL_I2C_Master_Receive_IT(&hi2c1, ((0x23 << 1) | 0b1), GripperStatus, 1);
				}
				GripperState = 1;
			}
			if (GripperStatus[0] == 0x12 )
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
			}
			else if (GripperStatus[0] == 0x34)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
			}
			else if (GripperStatus[0] == 0x56)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 0);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 0);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 0);
			}
			else if (GripperStatus[0] == 0x78)
			{
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_1, 1);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, 1);
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, 1);
			}

			if (micros() - Timestamp_Gripper >= 5100000)
			{
				GripperState = 0;
				Munmunbot_State = STATE_PrepareDATA;
				PID_Reset();
			}
		}


	  }
	  else if(GripperEnable == 0)
	  {
		 Munmunbot_State = STATE_PrepareDATA;
	  }
}


/// เศษซากอารยธรรม ///
//{
//	// Error term
////	PositionPIDController.OutputDesire = TrjStruc.AngularDisplacementDesire;
////	PositionPIDController.NowError = PositionPIDController.OutputDesire - PositionPIDController.OutputFeedback;
////
////	// Update integral Value
////	PositionPIDController.Integral_Value += PositionPIDController.NowError;
////
////
////	PositionPIDController.ControllerOutput = (PositionPIDController.Kp*PositionPIDController.NowError)
////					  +(PositionPIDController.Ki * PositionPIDController.Integral_Value)
////					  +(PositionPIDController.Kd * (PositionPIDController.NowError-PositionPIDController.PreviousError))
////					  +( TrjStruc.Alpha * PositionPIDController.offSet);
////	PositionPIDController.PreviousError = PositionPIDController.NowError;
//
//
//    // Error Term
////    VelocityPIDController.OutputDesire = PositionPIDController.ControllerOutput + TrjStruc.AngularVelocityDesire;
////    VelocityPIDController.NowError = VelocityPIDController.OutputDesire - VelocityPIDController.OutputFeedback;
//
////    // Calculate Output of the Controller
////    VelocityPIDController.ControllerOutput = VelocityPIDController.PreviousControllerOutput +
////    		(( VelocityPIDController.Kp + VelocityPIDController.Ki + VelocityPIDController.Kd ) * VelocityPIDController.NowError ) -
////			(( VelocityPIDController.Kp + ( 2.0 * VelocityPIDController.Kd )) * VelocityPIDController.PreviousError ) +
////			( VelocityPIDController.Kd *  VelocityPIDController.PreviousPreviousError )
////			 +( TrjStruc.Alpha * VelocityPIDController.offSet );
//
//
//    // Updating Process
////    VelocityPIDController.PreviousPreviousError = VelocityPIDController.PreviousError;
////    VelocityPIDController.PreviousError = VelocityPIDController.NowError;
////    VelocityPIDController.PreviousControllerOutput =  VelocityPIDController.ControllerOutput;
//
////	// Error term
////	VelocityPIDController.OutputDesire = PositionPIDController.ControllerOutput + TrjStruc.AngularVelocityDesire;
////	VelocityPIDController.NowError = VelocityPIDController.OutputDesire - VelocityPIDController.OutputFeedback;
////
////	// Update integral Value
////	VelocityPIDController.Integral_Value += VelocityPIDController.NowError;
////
////
////	VelocityPIDController.ControllerOutput = (VelocityPIDController.Kp*VelocityPIDController.NowError)
////					  +(VelocityPIDController.Ki * PositionPIDController.Integral_Value)
////					  +(VelocityPIDController.Kd * (VelocityPIDController.NowError-VelocityPIDController.PreviousError))
////					  +( TrjStruc.Alpha * VelocityPIDController.offSet);
////	VelocityPIDController.PreviousError = VelocityPIDController.NowError;
//
//}



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
