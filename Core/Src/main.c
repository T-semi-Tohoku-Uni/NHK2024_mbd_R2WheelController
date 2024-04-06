/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include <stdio.h>
#include <math.h>

#include "DJI_CANIDList.h"
#include "R2CANIDList.h"

#include "pid.h"
#include "filter.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define PI 3.14159265

#define WHEEL_DIAMETER 100 //[mm]
#define WHEELBASE_LEN 260 //[mm]
#define TREAD_LEN 245//[mm]

#define M2006_CURRENT_LIMIT 10000

#define CONTROL_CYCLE 1 //[ms]

#define CW -1
#define CCW 1
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

IWDG_HandleTypeDef hiwdg;

UART_HandleTypeDef hlpuart1;

TIM_HandleTypeDef htim17;

/* USER CODE BEGIN PV */
FDCAN_TxHeaderTypeDef fdcan3_TxHeader;
FDCAN_RxHeaderTypeDef fdcan3_RxHeader;
uint8_t 			  fdcan3_RxData[8];

FDCAN_TxHeaderTypeDef fdcan1_TxHeader;
FDCAN_RxHeaderTypeDef fdcan1_RxHeader;
uint8_t 			  fdcan1_RxData[8];

//The units of angle rate and angle are [rad/s] or [rad]
typedef struct{
	double actPos[3];
	double trgPos[3];
	double outPos[3];
	double actVel[3];
	double trgVel[3];
	double outVel[3];
	PID velPID[3];
} robotPosStatus;

typedef struct{
	uint16_t CANID;
	uint8_t motorID;
	int8_t rotDir;//rotation direction
	double trgVel;//target angle velocity [RPM]
	double actVel;//actual angle velocity [RPM]
	double outVel;//output
	double angle;//[rad]
	double reductionRatio; //@param reduction ratio (1:param)
	double actCurrent;//actual torque current
	PID velPID;
}motor;

typedef struct{
	double diameter;
	double trgVel;
	double actVel;
	double outVel;
	double friction;//friction coefficient
}wheel;

typedef struct{
	double treadLen;
	double wheelBaseLen;
	double mass;
	double inertia;
	wheel wheels[4];
}robotPhyParam;


robotPosStatus gRobotPos;
motor gMotors[4];
robotPhyParam gRobotPhy;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM17_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void RobotControllerInit(void);
void MotorControllerInit(void);
void RobotPosInit(void);
void RobotPhyParamInit(void);

//CAN communication functions.
void CAN_Motordrive(int32_t vel[]);

//Convert Functions
void ConvertWheel2Motor(wheel *wheels, motor *motors);
void InverseKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy);
void ForwardKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy);

//Feed Back Functions
void RobotVelFB(void);
//Sequence Functions
void Update(void);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//convert functions
void ConvertWheel2Motor(wheel *wheels, motor *motors){
	for(uint8_t i=0; i<4; i++){
		double M2W_Ratio =  motors[i].rotDir * PI * wheels[i].diameter / 60 / motors[i].reductionRatio;
		//Convert actual (angle) velocity
		wheels[i].actVel = motors[i].actVel * M2W_Ratio;

		//Convert target (angle) velocity
		motors[i].trgVel = wheels[i].trgVel / M2W_Ratio;
		motors[i].outVel = wheels[i].outVel / M2W_Ratio;
	}
}

void InverseKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy){
	//座標変換行�??
	const float wheelParam = (robotPhy->wheelBaseLen + robotPhy->treadLen) / 2;
	const float A[4][3] = {
			{-1,  1, wheelParam},
			{-1, -1, wheelParam},
			{ 1, -1, wheelParam},
			{ 1,  1, wheelParam}
	};

	for(uint8_t i=0; i<4; i++){
		wheel[i].trgVel = 0;
		for(uint8_t j=0; j<3; j++){
			wheel[i].trgVel += A[i][j] * robotPos->outVel[j];
		}
	}
}

void ForwardKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy){
	const float wheelParam = (robotPhy->wheelBaseLen + robotPhy->treadLen) / 1;
	const float gain = 0.25;
	const float A[3][4] = {
		{-1, -1, 1, 1},
		{1, -1, -1, 1},
		{1/wheelParam, 1/wheelParam, 1/wheelParam, 1/wheelParam}
	};

	for(uint8_t i=0; i<3; i++){
	robotPos->actVel[i] = 0;
		for(uint8_t j=0; j<4; j++){
			robotPos->actVel[i] += gain * A[i][j] * wheel[j].actVel;
		}
	}
}

//Call Back
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs) {
	if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
		//printf("FIFO0 callback\r\n");
		if(hfdcan == &hfdcan1){
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO0, &fdcan1_RxHeader, fdcan1_RxData) != HAL_OK) {
				Error_Handler();
			}
			if(fdcan1_RxHeader.Identifier == CANID_ROBOT_VEL){
				if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
				{
					Error_Handler();
				}

				float gain[3] = {16, 16, 0.02};
				for(uint8_t i=0; i<3; i++){
					gRobotPos.trgVel[i] = (fdcan1_RxData[i] - 127)*gain[i];
				}
			}

			//printf("RxData: %x\r\n", fdcan1_RxData[0]);
		}
	}
}

void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs) {
	//printf("FIFO1 callback\r\n");
	if ((RxFifo1ITs & FDCAN_IT_RX_FIFO1_NEW_MESSAGE) != RESET) {
		if (hfdcan == &hfdcan3) {
			if (HAL_FDCAN_GetRxMessage(hfdcan, FDCAN_RX_FIFO1, &fdcan3_RxHeader, fdcan3_RxData) != HAL_OK) {
				Error_Handler();
			}

			uint8_t motorID = fdcan3_RxHeader.Identifier - DJI_CANID_TX0 - 1;
			if(motorID<4){
				int16_t intbuff;
				//get actual angle velocity of motor from C610
				intbuff = fdcan3_RxData[2]<<8 | fdcan3_RxData[3];
				gMotors[motorID].actVel = (double)intbuff;
				//get actual torque current of motor
				intbuff = fdcan3_RxData[4]<<8 | fdcan3_RxData[5];
				gMotors[motorID].actCurrent = (double)intbuff;
			}
		}
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){

	if(htim == &htim17){
		//printf("Timer callback\r\n");
		int32_t output[4];


		ConvertWheel2Motor(gRobotPhy.wheels, gMotors);
		ForwardKinematics(&gRobotPos, gRobotPhy.wheels, &gRobotPhy);

		for(uint8_t i=0; i<3; i++){
			gRobotPos.velPID[i].setpoint = gRobotPos.trgVel[i];
			gRobotPos.outVel[i] = gRobotPos.trgVel[i] + pid_compute(&gRobotPos.velPID[i], gRobotPos.actVel[i]);
		}

		InverseKinematics(&gRobotPos, gRobotPhy.wheels, &gRobotPhy);
		ConvertWheel2Motor(gRobotPhy.wheels, gMotors);

		//motor PID
		for(uint8_t i=0; i<4; i++){
			gMotors[i].velPID.setpoint = gMotors[i].trgVel;
			output[i] = pid_compute(&gMotors[i].velPID, gMotors[i].actVel);
		}

		//transmit to C610
		CAN_Motordrive(output);
		RobotVelFB();
	}
}

void RobotVelFB(void){
	uint8_t fdcan1_TxData[3] = {};
	double gain[3] = {16, 16, 0.02};
	for(uint8_t i=0; i<3; i++){
		if(gRobotPos.actVel[i]/16 + 127 > 255){
			fdcan1_TxData[i] = 255;
			continue;
		}
		else if(gRobotPos.actVel[i]/16 + 127 < 0){
			fdcan1_TxData[i] = 0;
			continue;
		}
		int8_t buffer = gRobotPos.actVel[i]/gain[i] + 127;
		fdcan1_TxData[i] |= buffer;
	}
	fdcan1_TxHeader.DataLength = FDCAN_DLC_BYTES_3;
	fdcan1_TxHeader.Identifier = CANID_ROBOT_VEL_FB;

	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcan1_TxHeader, fdcan1_TxData) != HAL_OK){
		Error_Handler();
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
  setbuf(stdout, NULL);

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_LPUART1_UART_Init();
  MX_FDCAN3_Init();
  MX_TIM17_Init();
  MX_FDCAN1_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  RobotControllerInit();
  MotorControllerInit();
  RobotPhyParamInit();
  HAL_Delay(100);
  while(gMotors[0].actVel != 0 && gMotors[1].actVel != 0 && gMotors[2].actVel != 0 && gMotors[3].actVel != 0){
	  HAL_Delay(200);
	  int32_t vel[4] = {};
	  CAN_Motordrive(vel);
	  if (HAL_IWDG_Refresh(&hiwdg) != HAL_OK)
	  {
		Error_Handler();
	  }
  }
  printf("Initialized\r\n");
  HAL_TIM_Base_Start_IT(&htim17);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = RCC_PLLM_DIV1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief FDCAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN1_Init(void)
{

  /* USER CODE BEGIN FDCAN1_Init 0 */

  /* USER CODE END FDCAN1_Init 0 */

  /* USER CODE BEGIN FDCAN1_Init 1 */

  /* USER CODE END FDCAN1_Init 1 */
  hfdcan1.Instance = FDCAN1;
  hfdcan1.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan1.Init.FrameFormat = FDCAN_FRAME_FD_BRS;
  hfdcan1.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan1.Init.AutoRetransmission = DISABLE;
  hfdcan1.Init.TransmitPause = DISABLE;
  hfdcan1.Init.ProtocolException = DISABLE;
  hfdcan1.Init.NominalPrescaler = 4;
  hfdcan1.Init.NominalSyncJumpWidth = 1;
  hfdcan1.Init.NominalTimeSeg1 = 15;
  hfdcan1.Init.NominalTimeSeg2 = 4;
  hfdcan1.Init.DataPrescaler = 2;
  hfdcan1.Init.DataSyncJumpWidth = 1;
  hfdcan1.Init.DataTimeSeg1 = 15;
  hfdcan1.Init.DataTimeSeg2 = 4;
  hfdcan1.Init.StdFiltersNbr = 1;
  hfdcan1.Init.ExtFiltersNbr = 0;
  hfdcan1.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN1_Init 2 */
	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_MASK;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
	sFilterConfig.FilterID1 = CANID_ROBOT_VEL;
	sFilterConfig.FilterID2 = 0b11111111111;

	if (HAL_FDCAN_ConfigFilter(&hfdcan1, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan1) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

	fdcan1_TxHeader.IdType = FDCAN_STANDARD_ID;
	fdcan1_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcan1_TxHeader.DataLength = FDCAN_DLC_BYTES_3;
	fdcan1_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan1_TxHeader.BitRateSwitch = FDCAN_BRS_ON;
	fdcan1_TxHeader.FDFormat = FDCAN_FD_CAN;
	fdcan1_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan1_TxHeader.MessageMarker = 0;
	fdcan1_TxHeader.Identifier = 0x700;

  /* USER CODE END FDCAN1_Init 2 */

}

/**
  * @brief FDCAN3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_FDCAN3_Init(void)
{

  /* USER CODE BEGIN FDCAN3_Init 0 */

  /* USER CODE END FDCAN3_Init 0 */

  /* USER CODE BEGIN FDCAN3_Init 1 */

  /* USER CODE END FDCAN3_Init 1 */
  hfdcan3.Instance = FDCAN3;
  hfdcan3.Init.ClockDivider = FDCAN_CLOCK_DIV1;
  hfdcan3.Init.FrameFormat = FDCAN_FRAME_CLASSIC;
  hfdcan3.Init.Mode = FDCAN_MODE_NORMAL;
  hfdcan3.Init.AutoRetransmission = DISABLE;
  hfdcan3.Init.TransmitPause = DISABLE;
  hfdcan3.Init.ProtocolException = DISABLE;
  hfdcan3.Init.NominalPrescaler = 4;
  hfdcan3.Init.NominalSyncJumpWidth = 1;
  hfdcan3.Init.NominalTimeSeg1 = 15;
  hfdcan3.Init.NominalTimeSeg2 = 4;
  hfdcan3.Init.DataPrescaler = 1;
  hfdcan3.Init.DataSyncJumpWidth = 1;
  hfdcan3.Init.DataTimeSeg1 = 1;
  hfdcan3.Init.DataTimeSeg2 = 1;
  hfdcan3.Init.StdFiltersNbr = 1;
  hfdcan3.Init.ExtFiltersNbr = 0;
  hfdcan3.Init.TxFifoQueueMode = FDCAN_TX_FIFO_OPERATION;
  if (HAL_FDCAN_Init(&hfdcan3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN FDCAN3_Init 2 */

  	FDCAN_FilterTypeDef sFilterConfig;
	sFilterConfig.IdType = FDCAN_STANDARD_ID;
	sFilterConfig.FilterIndex = 0;
	sFilterConfig.FilterType = FDCAN_FILTER_RANGE;
	sFilterConfig.FilterConfig = FDCAN_FILTER_TO_RXFIFO1;
	sFilterConfig.FilterID1 = DJI_CANID_0;
	sFilterConfig.FilterID2 = DJI_CANID_7;

	if (HAL_FDCAN_ConfigFilter(&hfdcan3, &sFilterConfig) != HAL_OK) {
		Error_Handler();
	}
	if (HAL_FDCAN_ConfigGlobalFilter(&hfdcan3, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_Start(&hfdcan3) != HAL_OK) {
		Error_Handler();
	}

	if (HAL_FDCAN_ActivateNotification(&hfdcan3, FDCAN_IT_RX_FIFO1_NEW_MESSAGE, 0) != HAL_OK) {
		Error_Handler();
	}

  /* USER CODE END FDCAN3_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Window = 1000;
  hiwdg.Init.Reload = 1000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

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
  hlpuart1.Init.BaudRate = 1000000;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_8B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
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
  * @brief TIM17 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM17_Init(void)
{

  /* USER CODE BEGIN TIM17_Init 0 */

  /* USER CODE END TIM17_Init 0 */

  /* USER CODE BEGIN TIM17_Init 1 */

  /* USER CODE END TIM17_Init 1 */
  htim17.Instance = TIM17;
  htim17.Init.Prescaler = 79;
  htim17.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim17.Init.Period = 999;
  htim17.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim17) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM17_Init 2 */

  /* USER CODE END TIM17_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LD2_Pin */
  GPIO_InitStruct.Pin = LD2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void CAN_Motordrive(int32_t vel[])
{
	int i;
	uint8_t TxData[8];
	for(i=0; i<4; i++){

		if(vel[i]<-1*M2006_CURRENT_LIMIT)vel[i]=-1*M2006_CURRENT_LIMIT;
		else if(vel[i]>M2006_CURRENT_LIMIT)vel[i]=M2006_CURRENT_LIMIT;
		TxData[i*2]=vel[i]>>8;//上位ビ
		TxData[i*2+1]=vel[i]&0x00FF;//下位ビ
	}


	fdcan3_TxHeader.IdType = FDCAN_STANDARD_ID;
	fdcan3_TxHeader.TxFrameType = FDCAN_DATA_FRAME;
	fdcan3_TxHeader.DataLength = FDCAN_DLC_BYTES_8;
	fdcan3_TxHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
	fdcan3_TxHeader.BitRateSwitch = FDCAN_BRS_OFF;
	fdcan3_TxHeader.FDFormat = FDCAN_CLASSIC_CAN;
	fdcan3_TxHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
	fdcan3_TxHeader.MessageMarker = 0;
	fdcan3_TxHeader.Identifier = DJI_CANID_TX0;

	if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan3, &fdcan3_TxHeader, TxData) != HAL_OK) {
		/* Transmission request Error */
		Error_Handler();
	}
}

int _write(int file, char *ptr, int len)
{
    HAL_UART_Transmit(&hlpuart1,(uint8_t *)ptr,len,10);
    return len;
}

void RobotControllerInit(void){
	double velKp[3] = {0.3, 0.3, 1.6};
	double velKi[3] = {0, 0, 0.1};
	double velKd[3] = {0, 0, 0};
	double velIntegral_min[3] = {-5, -5, -3};
	double velIntegral_max[3] = {5, 5, 	3};

	for(uint8_t i=0; i<3; i++){
		gRobotPos.actPos[i] = 0;
		gRobotPos.actVel[i] = 0;
		gRobotPos.trgPos[i] = 0;
		gRobotPos.trgVel[i] = 0;
		gRobotPos.outPos[i] = 0;
		gRobotPos.outVel[i] = 0;

		pid_init(&gRobotPos.velPID[i], CONTROL_CYCLE, velKp[i], velKd[i], velKi[i], 0, velIntegral_min[i], velIntegral_max[i]);
	}
}

void MotorControllerInit(void){
	double kp[4] = {12, 12, 12, 12};
	double ki[4] = {0, 0, 0, 0};
	double kd[4] = {0, 0, 0, 0};
	double integral_min[4] = {-30, -30, -30, -30};
	double integral_max[4] = {30, 30, 30, 30};

	for(uint8_t i=0; i<4; i++){
		gMotors[i].rotDir = CW;
		gMotors[i].CANID = DJI_CANID_0;
		gMotors[i].trgVel = 0;
		gMotors[i].actVel = 0;
		gMotors[i].outVel = 0;
		gMotors[i].angle = 0;
		gMotors[i].motorID = 0;
		gMotors[i].reductionRatio = 36;
		pid_init(&gMotors[i].velPID, CONTROL_CYCLE, kp[i], kd[i], ki[i], 0, integral_min[i], integral_max[i]);
	}
}

void RobotPhyParamInit(void){
	double frictions[4] = {1, 1, 1, 1};

	gRobotPhy.inertia = 1;
	gRobotPhy.mass = 1;
	gRobotPhy.treadLen = TREAD_LEN;
	gRobotPhy.wheelBaseLen = WHEELBASE_LEN;

	for(uint8_t i=0; i<4; i++){
		gRobotPhy.wheels[i].actVel = 0;
		gRobotPhy.wheels[i].friction = frictions[i];
		gRobotPhy.wheels[i].outVel = 0;
		gRobotPhy.wheels[i].trgVel = 0;
		gRobotPhy.wheels[i].diameter = WHEEL_DIAMETER;
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
