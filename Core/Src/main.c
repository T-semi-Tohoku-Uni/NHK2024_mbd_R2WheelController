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
#include <string.h>

#include "DJI_CANIDList.h"
#include "R2CANIDList.h"

#include "bno055.h"
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

#define TRUE 1
#define FALSE 0
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
FDCAN_HandleTypeDef hfdcan1;
FDCAN_HandleTypeDef hfdcan3;

I2C_HandleTypeDef hi2c1;

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

typedef struct{
	double upward;
	double plane;
	double slope;
	double planeGrvVector[3];
	double slopeGrvVector[3];
	double slopeAngleDiff;
}field_placement;

uint8_t is_on_slope = FALSE;
uint8_t is_field_init = FALSE;

field_placement gFieldPlacement;
Low_Pass_Filter_Settings *gyroLPFsetting;
robotPosStatus gRobotPos;
motor gMotors[4];
robotPhyParam gRobotPhy;
double gGrvVector[3] = {};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_FDCAN3_Init(void);
static void MX_TIM17_Init(void);
static void MX_FDCAN1_Init(void);
static void MX_IWDG_Init(void);
static void MX_I2C1_Init(void);
/* USER CODE BEGIN PFP */
void RobotControllerInit(void);
void MotorControllerInit(void);
void RobotPosInit(void);
void RobotPhyParamInit(void);
void BNO055_Init(void);
void FieldPlacementInit(void);
void CountDown(char header[], uint8_t time);

void FieldPlacementUpdate(void);

void ReadEulerAngle(I2C_HandleTypeDef *hi2c, double euler[3], uint8_t address);
void ReadGrvVector(I2C_HandleTypeDef *hi2c, double vector[3], uint8_t address);

//CAN communication functions.
void CAN_Motordrive(int32_t vel[]);

//Convert Functions
void ConvertWheel2Motor(wheel *wheels, motor *motors);
void InverseKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy);
void ForwardKinematics(robotPosStatus *robotPos, wheel wheel[], robotPhyParam *robotPhy);
double CalcVectorAngle(double vector1[3], double vector2[3]);
double CalcRelatedHeadingAngle(double AbsHeading, double bases);

//Feed Back Functions
void RobotVelFB(void);
void SlopeStateSend(uint8_t state);

//Sequence Functions
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

		static uint8_t count = 0;

		if(count == 10 && is_field_init == TRUE){
			count = 0;
			uint8_t Rxbuffer[10] = {};

			//read euler angle from BNO
			double euler[3] = {};
			ReadEulerAngle(&hi2c1, euler, BNO055_I2C_ADDR1);

		    //北基準からフィールド上方向基準に変更
		    double a = euler[0] - gFieldPlacement.upward;
		    if(a > PI)a -= 2 * PI;
		    if(a < -PI)a += 2 * PI;

		    a *= -1;

		    gRobotPos.actPos[2] = a;
//		    printf("heading:%f \r\n",gRobotPos.actPos[2]);

		    //read gravity vector from BNO055 for detecting slope
		    ReadGrvVector(&hi2c1, gGrvVector, BNO055_I2C_ADDR1);

			//printf("Grv vector x:%f / y:%f / z:%f\r\n", gGrvVector[0], gGrvVector[1], gGrvVector[2]);
			double rawAngle = CalcVectorAngle(gFieldPlacement.planeGrvVector, gGrvVector);
			double angle = low_pass_filter_update(gyroLPFsetting, rawAngle);

			//printf("angle:%f\r\n", rawAngle);
			if (rawAngle > gFieldPlacement.slopeAngleDiff * 0.8){
				is_on_slope = TRUE;
				//printf("On slope\r\n");
			}
			else{
				is_on_slope = FALSE;
			}
			SlopeStateSend((uint8_t)(rawAngle * 100));
		}
		count++;
	}
}

void ReadEulerAngle(I2C_HandleTypeDef *hi2c, double euler[3], uint8_t address){
	uint8_t Rxbuffer[10];
	HAL_I2C_Mem_Read(hi2c, address << 1, 0x1A, I2C_MEMADD_SIZE_8BIT, Rxbuffer, 6, 100);

	for(uint8_t i=0; i<3; i++){
		euler[i] = (double)((Rxbuffer[i*2+1] << 8) | Rxbuffer[i*2])/16/180*PI;
	}
}

void ReadGrvVector(I2C_HandleTypeDef *hi2c, double vector[3], uint8_t address){
	uint8_t Rxbuffer[10];
	HAL_I2C_Mem_Read(hi2c, BNO055_I2C_ADDR1 << 1, BNO055_GRAVITY_DATA_X_LSB_ADDR, I2C_MEMADD_SIZE_8BIT, Rxbuffer, BNO055_GRAVITY_XYZ_DATA_SIZE, 100);

	for(uint8_t i=0; i<3; i++){
		int16_t b = (Rxbuffer[i*2+1] << 8) | Rxbuffer[i*2];
		vector[i] = (double)b / 100;
	}
}

void SlopeStateSend(uint8_t state){
	fdcan1_TxHeader.DataLength = FDCAN_DLC_BYTES_2;
	fdcan1_TxHeader.Identifier = CANID_SLOPE_DETECTION;

	//printf("send:%d\r\n", state);
	uint8_t fdcan1_TxData[2] = {state, (uint8_t)(gFieldPlacement.slopeAngleDiff * 100)};
	if(HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan1, &fdcan1_TxHeader, fdcan1_TxData) != HAL_OK){
		Error_Handler();
	}
}

void RobotVelFB(void){
	uint8_t fdcan1_TxData[4] = {};
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

		if(is_field_init == FALSE){
			fdcan1_TxData[3] = 255;
		}
		int8_t buffer = gRobotPos.actVel[i]/gain[i] + 127;
		fdcan1_TxData[i] |= buffer;
	}

	fdcan1_TxData[3] = (uint8_t)(40 * gRobotPos.actPos[2]);
	fdcan1_TxHeader.DataLength = FDCAN_DLC_BYTES_4;
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
  //MX_IWDG_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */
  //MX_IWDG_Init();
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
  BNO055_Init();
  HAL_TIM_Base_Start_IT(&htim17);

  FieldPlacementInit();
  FieldPlacementUpdate();
  //MX_IWDG_Init();
  printf("Initialized\r\n");


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
  hi2c1.Init.Timing = 0x00702991;
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

void BNO055_Init(void){
	HAL_Delay(700);
	uint8_t Txbuff;
	uint8_t Rxbuff;
	//char message[20];



	//Txbuff = 0x20;
	//HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x3F, I2C_MEMADD_SIZE_8BIT, &Txbuff, 1, 100); //system trigger

	Txbuff = 0x00;
	HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x3E, I2C_MEMADD_SIZE_8BIT, &Txbuff, 1, 100); //power mode

	Txbuff = 0b00010010;
	//HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, BNO055_AXIS_MAP_CONFIG_ADDR, I2C_MEMADD_SIZE_8BIT, &Txbuff, 1, 100);

	Txbuff = 0x0C;
	HAL_I2C_Mem_Write(&hi2c1, 0x28 << 1, 0x3D, I2C_MEMADD_SIZE_8BIT, &Txbuff, 1, 100);//using Nine Degree of Freedom mode




	HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x3A, I2C_MEMADD_SIZE_8BIT, &Rxbuff, 1, 100);
	printf("Error:%d\r\n", Rxbuff);

	HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x00, I2C_MEMADD_SIZE_8BIT, &Rxbuff, 1, 100);
	printf("ID:%d\r\n", Rxbuff);

	HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, 0x34, I2C_MEMADD_SIZE_8BIT, &Rxbuff, 1, 100);
	printf("Temp:%d\r\n", Rxbuff);


	double control_cycle = 0.01;
	double cutoff_freq = 0.1;

	gyroLPFsetting = low_pass_filter_init(cutoff_freq, control_cycle);

	HAL_Delay(100);

	/*
	printf("BNO Calibration\r\n");
	for(uint8_t i = 0; i < 10; i++){
		HAL_I2C_Mem_Read(&hi2c1, 0x28 << 1, BNO055_ACCEL_CALIB_STAT_REG, I2C_MEMADD_SIZE_8BIT, &Rxbuff, 1, 100);
		HAL_Delay(1000);
	}

	printf("Calibration Stats: %x\r\n", Rxbuff);
*/
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

void FieldPlacementInit(void){
	gFieldPlacement.plane = PI/2;
	gFieldPlacement.upward = 0;
	gFieldPlacement.slope = PI/2 - 0.1;
	gFieldPlacement.slopeAngleDiff = 0.1;
	for(uint8_t i = 0; i < 3; i++){
		gFieldPlacement.planeGrvVector[i] = 0;
		gFieldPlacement.slopeGrvVector[i] = 0;
	}
}

void FieldPlacementUpdate(void){
	printf("Initializing field placement...\r\n");
	printf("Put the robot facing upward of the field\r\n");

	char header[24] = "Calibration starts in";
	double buff[3] = {};

	CountDown(header, 5);
	printf("Calibrating\r\n");
	printf("DO NOT MOVE THE ROBOT\r\n");
	field_placement fieldPlacementTemp = {0, 0, 0, {}, {}, 0.05};

	for(uint8_t i = 0; i < 200; i++){
		if((gRobotPos.actVel[0] != 0 || gRobotPos.actVel[1] != 0) || gRobotPos.actVel[2] != 0){
			printf("Error: failed to get field upward\r\n");
			return ;
		}
		ReadEulerAngle(&hi2c1, buff, BNO055_I2C_ADDR1);
		fieldPlacementTemp.upward = (fieldPlacementTemp.upward * i + buff[0]) / (i+1);
		HAL_Delay(10);
		ReadGrvVector(&hi2c1, buff, BNO055_I2C_ADDR1);
		for(uint8_t j = 0; j<3; j++){
			fieldPlacementTemp.planeGrvVector[j] = (fieldPlacementTemp.planeGrvVector[j] * i + buff[j]) / (i+1);
		}
	}

	printf("Pass: get field upward:%f\r\n", fieldPlacementTemp.upward);
	printf("Pass: get plane grv vector: X:%f / Y:%f / Z:%f\r\n", fieldPlacementTemp.planeGrvVector[0], fieldPlacementTemp.planeGrvVector[1], fieldPlacementTemp.planeGrvVector[2]);

	printf("Put the robot on the slope\r\n");
	HAL_Delay(1000);
	CountDown(header, 10);
	printf("Calibrating\r\n");

	printf("DO NOT MOVE THE ROBOT\r\n");
	fieldPlacementTemp.slope = gRobotPos.actPos[3];
	for(uint8_t i = 0; i < 200; i++){
		if((gRobotPos.actVel[0] != 0 || gRobotPos.actVel[1] != 0) || gRobotPos.actVel[2] != 0){
			printf("Error: failed to get field slope\r\n");
			return ;
		}

		ReadGrvVector(&hi2c1, buff, BNO055_I2C_ADDR1);
		for(uint8_t j = 0; j<3; j++){
			fieldPlacementTemp.slopeGrvVector[j] = (fieldPlacementTemp.slopeGrvVector[j] * i + buff[j]) / (i+1);
		}
		HAL_Delay(10);
	}

	printf("Pass: get slope grv vector: X:%f / Y:%f / Z:%f\r\n", fieldPlacementTemp.slopeGrvVector[0], fieldPlacementTemp.slopeGrvVector[1], fieldPlacementTemp.slopeGrvVector[2]);

	gFieldPlacement.plane = fieldPlacementTemp.plane;
	gFieldPlacement.slope = fieldPlacementTemp.slope;
	gFieldPlacement.upward = fieldPlacementTemp.upward;
	memmove(gFieldPlacement.planeGrvVector, fieldPlacementTemp.planeGrvVector, 24);
	memmove(gFieldPlacement.slopeGrvVector, fieldPlacementTemp.slopeGrvVector, 24);

	gFieldPlacement.slopeAngleDiff = CalcVectorAngle(gFieldPlacement.planeGrvVector, gFieldPlacement.slopeGrvVector);

	printf("Field data updated\r\n");
	printf("---------RESULT----------\n upward: %6.4f / slope angle:%6.4f\r\n", gFieldPlacement.upward, gFieldPlacement.slopeAngleDiff);
	printf("Plane grv vector: X:%f / Y:%f / Z:%f\r\n", fieldPlacementTemp.planeGrvVector[0], fieldPlacementTemp.planeGrvVector[1], fieldPlacementTemp.planeGrvVector[2]);

	is_field_init = TRUE;

}

void CountDown(char header[], uint8_t time){
	if(time < 1) return;
	for(;time >=1 ; time--){
		printf("%s %d sec\r\n", header, time);
		HAL_Delay(1000);
	}
}

double CalcVectorAngle(double vector1[3], double vector2[3]){
	double dotp = 0;//dot product

	double len1 = 0;//length of vector1
	double len2 = 0;//length of vector2

	double sqlen1 = 0;//squared length of vector1
	double sqlen2 = 0;//squared length of vector2

	for(uint8_t i = 0; i < 3; i++){
		dotp += vector1[i] * vector2[i];
		sqlen1 += pow(vector1[i], 2);
		sqlen2 += pow(vector2[i], 2);
	}

	len1 = sqrt(sqlen1);
	len2 = sqrt(sqlen2);

	return acos(dotp / (len1 * len2));
}

double CalcRelatedHeadingAngle(double absHeading, double bases){
	double tempHeading = absHeading - bases;

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
