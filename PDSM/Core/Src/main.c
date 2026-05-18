/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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
#define bitRead(value, bit) (((value) >> (bit)) & 0x01)
#define SER_595_Pin    GPIO_PIN_7
#define SER_595_Port   GPIOD
#define SRCLK_595_Pin  GPIO_PIN_4
#define SRCLK_595_Port GPIOD
#define RCLK_595_Pin   GPIO_PIN_6
#define RCLK_595_Port  GPIOD


#define QH_165_Pin      GPIO_PIN_4
#define QH_165_Port     GPIOB
#define CLKINH_165_Pin  GPIO_PIN_5
#define CLKINH_165_Port GPIOB
#define SHLD_165_Pin    GPIO_PIN_6
#define SHLD_165_Port   GPIOB
#define CLK_165_Pin     GPIO_PIN_7
#define CLK_165_Port    GPIOB

//ID CAN
#define CAN_ID_VCU_REQUEST   0x01
#define CAN_ID_RSM_SEND      0x10
#define CAN_ID_PDSM_EFUSE_CTRL  0x13

//REG ID
#define REG_SENSOR           0x00
#define REG_STATUS_FUSE      0x01

#define FUSE_ON_VALUE       0
#define FUSE_OFF_VALUE      1
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
CAN_RxHeaderTypeDef RxHeader;
uint8_t RxData[8];
uint8_t RxDataN[8];

CAN_TxHeaderTypeDef TxHeader;
uint8_t TxData[8];
uint32_t TxMailbox;

volatile uint8_t active_send_sensor = 0;  // tambah ini
volatile uint8_t active_send_fuse   = 0;  // tambah ini


volatile uint16_t uart_rpmR        = 0;   // RPM Roda Kanan (dari Arduino PD2)
volatile uint16_t uart_rpmL        = 0;   // RPM Roda Kiri  (dari Arduino PD3)
volatile uint8_t  uart_rpmValid    = 0;   // 1 = frame terakhir valid
volatile uint8_t  uart_rxByte      = 0;


volatile uint8_t flag_request_sensor = 0;
volatile uint8_t flag_request_fuse   = 0;


volatile int8_t  sustraveRR = 0;
volatile int8_t  sustraveRL = 0;
volatile uint8_t rpmRL_lsb  = 0;
volatile uint8_t rpmRL_msb  = 0;
volatile uint8_t rpmRR_lsb  = 0;
volatile uint8_t rpmRR_msb  = 0;


volatile int8_t  rsmTemp    = 0;
volatile uint8_t fuseStatus = 0;
volatile uint8_t arusLV     = 0;
volatile uint8_t teganganLV = 0;

volatile uint8_t data165 = 0;
volatile uint8_t rxData_ID0_received= 0;
volatile uint8_t rxData_ID0[8];
volatile uint8_t rxData_ID03[8];
uint8_t waktursm1;
uint8_t  waktursm2;
uint32_t waktu_sensor;
uint32_t waktu_fuse;
volatile uint8_t fuseCtrl595 = 0x00;
volatile uint8_t fuseCtrlUpdateFlag = 1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_CAN_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */
void RSM_SendSensorData(void);
void RSM_SendFuseStatusData(void);
void RSM_UpdateSensorData(void);



/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void shift595_send(uint8_t data)
//{
//    HAL_GPIO_WritePin(RCLK_595_Port, RCLK_595_Pin, GPIO_PIN_RESET);
//    for (int i = 7; i >= 0; i--)
//    {
//        HAL_GPIO_WritePin(SRCLK_595_Port, SRCLK_595_Pin, GPIO_PIN_RESET);
//        if (data & (1 << i))
//            HAL_GPIO_WritePin(SER_595_Port, SER_595_Pin, GPIO_PIN_SET);
//        else
//            HAL_GPIO_WritePin(SER_595_Port, SER_595_Pin, GPIO_PIN_RESET);
//        HAL_GPIO_WritePin(SRCLK_595_Port, SRCLK_595_Pin, GPIO_PIN_SET);
//    }
//    HAL_GPIO_WritePin(RCLK_595_Port, RCLK_595_Pin, GPIO_PIN_SET);
//}
void shift595_send(uint8_t data)
{
    HAL_GPIO_WritePin(RCLK_595_Port, RCLK_595_Pin, GPIO_PIN_RESET);

    for (int i = 7; i >= 0; i--)
    {
        HAL_GPIO_WritePin(SRCLK_595_Port, SRCLK_595_Pin, GPIO_PIN_RESET);

        if (data & (1 << i))
            HAL_GPIO_WritePin(SER_595_Port, SER_595_Pin, GPIO_PIN_SET);
        else
            HAL_GPIO_WritePin(SER_595_Port, SER_595_Pin, GPIO_PIN_RESET);

        // Tambah delay kecil sebelum rising edge SRCLK
        __NOP(); __NOP(); __NOP();  // atau HAL_Delay(1) untuk debug

        HAL_GPIO_WritePin(SRCLK_595_Port, SRCLK_595_Pin, GPIO_PIN_SET);

        __NOP(); __NOP(); __NOP();
    }

    HAL_GPIO_WritePin(RCLK_595_Port, RCLK_595_Pin, GPIO_PIN_SET);
}

uint8_t shift165_read(void)
{
    uint8_t data = 0;
    HAL_GPIO_WritePin(CLKINH_165_Port, CLKINH_165_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(SHLD_165_Port, SHLD_165_Pin, GPIO_PIN_RESET);
    HAL_Delay(10);
    HAL_GPIO_WritePin(SHLD_165_Port, SHLD_165_Pin, GPIO_PIN_SET);
    for (int i = 7; i >= 0; i--)
    {
        HAL_GPIO_WritePin(CLK_165_Port, CLK_165_Pin, GPIO_PIN_RESET);
        if (HAL_GPIO_ReadPin(QH_165_Port, QH_165_Pin) == GPIO_PIN_SET)
            data |= (1 << i);
        HAL_GPIO_WritePin(CLK_165_Port, CLK_165_Pin, GPIO_PIN_SET);
    }
    return data;
}

void RSM_SendSensorData(void)
{
    TxHeader.StdId              = CAN_ID_RSM_SEND;
    TxHeader.ExtId              = 0;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.DLC                = 7;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = REG_SENSOR;
    TxData[1] = (uint8_t)sustraveRR;
    TxData[2] = (uint8_t)sustraveRL;
    TxData[3] = rpmRL_lsb;
    TxData[4] = rpmRL_msb;
    TxData[5] = rpmRR_lsb;
    TxData[6] = rpmRR_msb;

    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}

void RSM_SendFuseStatusData(void)
{
    TxHeader.StdId              = CAN_ID_RSM_SEND;
    TxHeader.ExtId              = 0;
    TxHeader.RTR                = CAN_RTR_DATA;
    TxHeader.IDE                = CAN_ID_STD;
    TxHeader.DLC                = 5;
    TxHeader.TransmitGlobalTime = DISABLE;

    TxData[0] = REG_STATUS_FUSE;
    TxData[1] = (uint8_t)rsmTemp;
    TxData[2] = fuseStatus;
    TxData[3] = arusLV;
    TxData[4] = teganganLV;

    HAL_CAN_AddTxMessage(&hcan, &TxHeader, TxData, &TxMailbox);
}
void RSM_UpdateSensorData(void)
{

    data165    = shift165_read();

    fuseStatus = data165;

    sustraveRR = (int8_t)(HAL_GetTick() % 50);
    sustraveRL = (int8_t)(HAL_GetTick() % 40);

    rpmRL_lsb = (uint8_t)(uart_rpmL & 0xFF);
    rpmRL_msb = (uint8_t)((uart_rpmL >> 8) & 0xFF);
    rpmRR_lsb = (uint8_t)(uart_rpmR & 0xFF);
    rpmRR_msb = (uint8_t)((uart_rpmR >> 8) & 0xFF);

    rsmTemp    = (int8_t)(25 + (HAL_GetTick() % 20));
    arusLV     = (uint8_t)(HAL_GetTick() % 50);
    teganganLV = (uint8_t)(12 + (HAL_GetTick() % 5));
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

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_CAN_Init();
  MX_USART3_UART_Init();

  /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig;
  sFilterConfig.FilterActivation     = ENABLE;
  sFilterConfig.FilterBank           = 0;
  sFilterConfig.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig.FilterIdHigh         = (CAN_ID_VCU_REQUEST << 5);
  sFilterConfig.FilterIdLow          = 0;
  sFilterConfig.FilterMaskIdHigh     = (0x7FF << 5);
  sFilterConfig.FilterMaskIdLow      = 0;
  sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig.SlaveStartFilterBank = 9;

  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig);

  CAN_FilterTypeDef sFilterConfig2;
  sFilterConfig2.FilterActivation     = ENABLE;
  sFilterConfig2.FilterBank           = 1;
  sFilterConfig2.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig2.FilterIdHigh         = (0x00 << 5);  // ID 0x00
  sFilterConfig2.FilterIdLow          = 0;
  sFilterConfig2.FilterMaskIdHigh     = (0x7FF << 5);
  sFilterConfig2.FilterMaskIdLow      = 0;
  sFilterConfig2.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.SlaveStartFilterBank = 9;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig2);

  CAN_FilterTypeDef sFilterConfig3;
  sFilterConfig3.FilterActivation     = ENABLE;
  sFilterConfig3.FilterBank           = 2;                  // bank berbeda
  sFilterConfig3.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig3.FilterIdHigh         = (0x03 << 5);        // ID 0x03
  sFilterConfig3.FilterIdLow          = 0;
  sFilterConfig3.FilterMaskIdHigh     = (0x7FF << 5);
  sFilterConfig3.FilterMaskIdLow      = 0;
  sFilterConfig3.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig3.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig3.SlaveStartFilterBank = 9;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig3);

  CAN_FilterTypeDef sFilterConfig4;
  sFilterConfig4.FilterActivation     = ENABLE;
  sFilterConfig4.FilterBank           = 3;
  sFilterConfig4.FilterFIFOAssignment = CAN_FILTER_FIFO1;
  sFilterConfig4.FilterIdHigh         = (0X13 << 5);
  sFilterConfig4.FilterIdLow          = 0;
  sFilterConfig4.FilterMaskIdHigh     = (0x7FF << 5);
  sFilterConfig4.FilterMaskIdLow      = 0;
  sFilterConfig4.FilterMode           = CAN_FILTERMODE_IDMASK;
  sFilterConfig4.FilterScale          = CAN_FILTERSCALE_32BIT;
  sFilterConfig4.SlaveStartFilterBank = 9;
  HAL_CAN_ConfigFilter(&hcan, &sFilterConfig4);

  HAL_CAN_Start(&hcan);
  HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO1_MSG_PENDING);
  HAL_UART_Receive_IT(&huart3, &uart_rxByte, 1);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
//  HAL_Delay(250);
//  shift595_send(0b00000001);
//  HAL_Delay(250);
//  shift595_send(0b00000011);
//  HAL_Delay(250);
//  shift595_send(0b0001011);
//  HAL_Delay(250);
//  shift595_send(0b00011011);
//  HAL_Delay(250);
//  shift595_send(0b00111011);
//  HAL_Delay(250);
//  shift595_send(0b01111011);
//  HAL_Delay(250);
//  shift595_send(0b11111011);
//  HAL_Delay(250);
//  shift595_send(0b01111011);
//  HAL_Delay(250);
//  shift595_send(0b00111011);
//  HAL_Delay(250);
//  shift595_send(0b00011011);
//  HAL_Delay(250);
//  shift595_send(0b00001011);
//  HAL_Delay(250);
//  shift595_send(0b00000011);
//  HAL_Delay(250);
//  shift595_send(0b00000001);
//  HAL_Delay(250);
//  shift595_send(0b00000000);




//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//  shift595_send(0b00000011);
//  HAL_Delay(100);
//
//  shift595_send(0b00001011);
//  HAL_Delay(100);
//
//  shift595_send(0b00011011);
//  HAL_Delay(100);
//
//  shift595_send(0b00111011);
//  HAL_Delay(100);
//
//  shift595_send(0b01111011);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//
//  // Pola balik dari kiri ke kanan
//  shift595_send(0b01111011);
//  HAL_Delay(100);
//
//  shift595_send(0b00111011);
//  HAL_Delay(100);
//
//  shift595_send(0b00011011);
//  HAL_Delay(100);
//
//  shift595_send(0b00001011);
//  HAL_Delay(100);
//
//  shift595_send(0b00000011);
//  HAL_Delay(100);
//
//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//
//  // Pola nyala satu-satu, lompat bit ke-3
//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//  shift595_send(0b00000010);
//  HAL_Delay(100);
//
//  shift595_send(0b00001000);
//  HAL_Delay(100);
//
//  shift595_send(0b00010000);
//  HAL_Delay(100);
//
//  shift595_send(0b00100000);
//  HAL_Delay(100);
//
//  shift595_send(0b01000000);
//  HAL_Delay(100);
//
//  shift595_send(0b10000000);
//  HAL_Delay(100);
//
//
//  // Pola satu-satu balik
//  shift595_send(0b10000000);
//  HAL_Delay(100);
//
//  shift595_send(0b01000000);
//  HAL_Delay(100);
//
//  shift595_send(0b00100000);
//  HAL_Delay(100);
//
//  shift595_send(0b00010000);
//  HAL_Delay(100);
//
//  shift595_send(0b00001000);
//  HAL_Delay(100);
//
//  shift595_send(0b00000010);
//  HAL_Delay(100);
//
//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//
//  // Pola pinggir ke tengah
//  shift595_send(0b10000001);
//  HAL_Delay(100);
//
//  shift595_send(0b11000011);
//  HAL_Delay(100);
//
//  shift595_send(0b11100011);
//  HAL_Delay(100);
//
//  shift595_send(0b11110011);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//
//  // Pola tengah ke pinggir
//  shift595_send(0b00011000);
//  HAL_Delay(100);
//
//  shift595_send(0b00111001);
//  HAL_Delay(100);
//
//  shift595_send(0b01111011);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//
//  // Pola kedip semua
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//  shift595_send(0b00000000);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//  shift595_send(0b00000000);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//  shift595_send(0b00000000);
//  HAL_Delay(100);
//
//
//  // Pola selang-seling
//  shift595_send(0b10101011);
//  HAL_Delay(100);
//
//  shift595_send(0b01010001);
//  HAL_Delay(100);
//
//  shift595_send(0b10101011);
//  HAL_Delay(100);
//
//  shift595_send(0b01010001);
//  HAL_Delay(100);
//
//
//  // Pola gelombang 1
//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//  shift595_send(0b00001011);
//  HAL_Delay(100);
//
//  shift595_send(0b00101011);
//  HAL_Delay(100);
//
//  shift595_send(0b10101011);
//  HAL_Delay(100);
//
//  shift595_send(0b11101011);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//
//  // Pola gelombang 2
//  shift595_send(0b10000000);
//  HAL_Delay(100);
//
//  shift595_send(0b10100000);
//  HAL_Delay(100);
//
//  shift595_send(0b10101000);
//  HAL_Delay(100);
//
//  shift595_send(0b10101010);
//  HAL_Delay(100);
//
//  shift595_send(0b10101011);
//  HAL_Delay(100);
//
//  shift595_send(0b11101011);
//  HAL_Delay(100);
//
//  shift595_send(0b11111011);
//  HAL_Delay(100);
//
//
//  // Pola isi setengah kanan
//  shift595_send(0b00000001);
//  HAL_Delay(100);
//
//  shift595_send(0b00000011);
//  HAL_Delay(100);
//
//  shift595_send(0b00001011);
//  HAL_Delay(100);
//
//  shift595_send(0b00011011);
//  HAL_Delay(100);
//
//
//  // Pola isi setengah kiri
//  shift595_send(0b10000000);
//  HAL_Delay(100);
//
//  shift595_send(0b11000000);
//  HAL_Delay(100);
//
//  shift595_send(0b11100000);
//  HAL_Delay(100);
//
//  shift595_send(0b11110000);
//  HAL_Delay(100);
//
//
//  // Pola pecah tengah
//  shift595_send(0b00011000);
//  HAL_Delay(100);
//
//  shift595_send(0b00101001);
//  HAL_Delay(100);
//
//  shift595_send(0b01001010);
//  HAL_Delay(100);
//
//  shift595_send(0b10000011);
//  HAL_Delay(100);
//
//
//  // Pola gabung lagi
//  shift595_send(0b10000011);
//  HAL_Delay(100);
//
//  shift595_send(0b01001010);
//  HAL_Delay(100);
//
//  shift595_send(0b00101001);
//  HAL_Delay(100);
//
//  shift595_send(0b00011000);
//  HAL_Delay(100);


  // Pola naik turun cepat
  shift595_send(0b00000001);
  HAL_Delay(100);

  shift595_send(0b00001000);
  HAL_Delay(100);

  shift595_send(0b00100000);
  HAL_Delay(100);

  shift595_send(0b10000000);
  HAL_Delay(100);

  shift595_send(0b00100000);
  HAL_Delay(100);

  shift595_send(0b00001000);
  HAL_Delay(100);

  shift595_send(0b00000001);
  HAL_Delay(100);
//
//
//  // Pola random tapi tetap aman bit ke-3
  shift595_send(0b01000011);
  HAL_Delay(100);

  shift595_send(0b10010001);
  HAL_Delay(100);

  shift595_send(0b00110011);
  HAL_Delay(100);

  shift595_send(0b11000001);
  HAL_Delay(100);

  shift595_send(0b01110011);
  HAL_Delay(100);

  shift595_send(0b10110001);
  HAL_Delay(100);

  shift595_send(0b11110011);
  HAL_Delay(100);

  shift595_send(0b11110011);
  HAL_Delay(100);
  shift595_send(0b00000000);
  // Akhiri mati semua
  HAL_Delay(100);

//  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {




	      RSM_UpdateSensorData();

	      if (active_send_sensor==1){
	    	  if(HAL_GetTick() - waktu_sensor >= waktursm1){
	    		  RSM_SendSensorData();
	    		  waktu_sensor = HAL_GetTick();
	    	  }
	      }

	      if (active_send_fuse==1){
	    	  if(HAL_GetTick() - waktu_fuse >= waktursm2){
	    		  RSM_SendFuseStatusData();
	    		  waktu_fuse = HAL_GetTick();
	    	  }
	      }
	      if (fuseCtrlUpdateFlag)
	      {
	          fuseCtrlUpdateFlag = 0;
	          shift595_send(fuseCtrl595);
	      }


	      if (bitRead(rxData_ID0[0], 0) == 1)
	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
	      else
	          HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);

	      if (bitRead(rxData_ID0[0], 1) == 1)
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
	      else
	          HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

	      HAL_Delay(10);

//	  RSM_UpdateSensorData();
//
//	        if (flag_request_sensor)
//	        {
//	            flag_request_sensor = 0;
//	            RSM_SendSensorData();
//	        }
//
//	        if (flag_request_fuse)
//	        {
//	            flag_request_fuse = 0;
//	            RSM_SendFuseStatusData();
//	        }

//	        HAL_Delay(10);

//	        if (rxData_ID0_received)
//	            {
//	                rxData_ID0_received = 0;
//
//	                if (rxData_ID0[0] & (1 << 0))
//	                {
//	                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//	                }
//	                else
//	                {
//	                    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//	                }
//	            }



//	        	if(RxData[0]==2){
//	        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//	        		HAL_Delay(1);
//	        	}else if(RxData[0]==0){
//	        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//	        	}
//        	if(bitRead(rxData_ID0[0],0) ==1){
//        		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_SET);
//        		HAL_Delay(1);
//        	}else if(bitRead(rxData_ID0[0],0) ==0){
//        		HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13, GPIO_PIN_RESET);
//        	}
//
//	        	if(bitRead(rxData_ID0[0],1) ==1){
//	        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_SET);
//	        		HAL_Delay(1);
//	        	}else if(bitRead(rxData_ID0[0],1) ==0){
//	        		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);
//	        	}
//	        }

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = DISABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_2, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PE2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : PA2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PD13 PD4 PD6 PD7 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_4|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB4 */
  GPIO_InitStruct.Pin = GPIO_PIN_4;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB5 PB6 PB7 */
  GPIO_InitStruct.Pin = GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
//{
//    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
//    {
//        if (RxHeader.StdId == CAN_ID_VCU_REQUEST)
//        {
//            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
//
//            switch (RxData[0])
//            {
//                case REG_SENSOR:
//                    flag_request_sensor = 1;
//                    break;
//
//                case REG_STATUS_FUSE:
//                    flag_request_fuse = 1;
//                    break;
//
//                default:
//                    break;
//            }
//        }
//    }
//}

//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
//{
//    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
//    {
//        if (RxHeader.StdId == CAN_ID_VCU_REQUEST)  // ID 0x01 — pakai reg ID
//        {
////            HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_2);
//
//            switch (RxData[0])
//            {
//                case REG_SENSOR:
//                    flag_request_sensor = 1;
//
//                    break;
//                case REG_STATUS_FUSE:
//                    flag_request_fuse = 1;
//                    break;
//                default:
//                    break;
//            }
//        }
//        else if (RxHeader.StdId == 0x00)  // ID 0x00 — langsung baca semua byte
//        {
//            rxData_ID0[0] = RxData[0];
//            rxData_ID0[1] = RxData[1];
//            rxData_ID0[2] = RxData[2];
//            rxData_ID0[3] = RxData[3];
//            rxData_ID0[4] = RxData[4];
//            rxData_ID0[5] = RxData[5];
//            rxData_ID0[6] = RxData[6];
//            rxData_ID0[7] = RxData[7];
//            rxData_ID0_received = 1;
//        }
//
//    }
//}
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan_ptr)
{
    if (HAL_CAN_GetRxMessage(hcan_ptr, CAN_RX_FIFO1, &RxHeader, RxData) == HAL_OK)
    {
        if (RxHeader.StdId == CAN_ID_VCU_REQUEST)
        {
            switch (RxData[0])
            {
                case REG_SENSOR:
                    active_send_sensor = 1;  // latch, tidak akan reset
                    waktursm1 = RxData[1] * 15;
                    break;

                case REG_STATUS_FUSE:
                    active_send_fuse = 1;
                    waktursm2 = RxData[1]* 10;
                    break;

                default:
                    break;
            }
        }
        	//REG ID
           else if (RxHeader.StdId == 0x00)
           	   {
                    rxData_ID0[0] = RxData[0];
                    rxData_ID0[1] = RxData[1];
                    rxData_ID0[2] = RxData[2];
                    rxData_ID0[3] = RxData[3];
                    rxData_ID0[4] = RxData[4];
                    rxData_ID0[5] = RxData[5];
                    rxData_ID0[6] = RxData[6];
                    rxData_ID0[7] = RxData[7];
                    rxData_ID0_received = 1;
                }
           else if (RxHeader.StdId == 0x03)
           	   {
                    rxData_ID03[0] = RxData[0];
                    rxData_ID03[1] = RxData[1];
                    rxData_ID03[2] = RxData[2];
                    rxData_ID03[3] = RxData[3];
                    rxData_ID03[4] = RxData[4];
                    rxData_ID03[5] = RxData[5];
                    rxData_ID03[6] = RxData[6];
                    rxData_ID03[7] = RxData[7];
//                    rxData_ID03_received = 1;
                }
           else if (RxHeader.StdId == CAN_ID_PDSM_EFUSE_CTRL)
           {
               if (RxHeader.DLC >= 1)
               {
                   fuseCtrl595 = RxData[0];        // Byte 0 langsung isi 8-bit eFuse
                   fuseCtrlUpdateFlag = 1;         // nanti dikirim ke shift register di while
               }
           }

    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3)
    {
        static uint8_t frame[7];
        static uint8_t idx = 0;

        // Tunggu header '$' (0x24) sebelum mulai kumpul frame
        if (idx == 0 && uart_rxByte != 0x24)
        {
            // Buang byte, tetap idx=0
        }
        else
        {
            frame[idx++] = uart_rxByte;

            if (idx >= 7)
            {
                idx = 0;

                // Validasi header, footer, dan XOR checksum
                if (frame[0] == 0x24 && frame[6] == 0x0A)
                {
                    uint8_t chk = frame[1] ^ frame[2] ^ frame[3] ^ frame[4];
                    if (chk == frame[5])
                    {
                        uart_rpmR     = ((uint16_t)frame[1] << 8) | frame[2];
                        uart_rpmL     = ((uint16_t)frame[3] << 8) | frame[4];
                        uart_rpmValid = 1;
                    }
                }
            }
        }

        // Re-arm interrupt untuk byte berikutnya
        HAL_UART_Receive_IT(&huart3, &uart_rxByte, 1);
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
#ifdef USE_FULL_ASSERT
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
