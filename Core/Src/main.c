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
 * This software component is licensed by ST under Ultimate Liberty license
 * SLA0044, the "License"; You may not use this file except in compliance with
 * the License. You may obtain a copy of the License at:
 *                             www.st.com/SLA0044
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_fatfs.h"
#include "i2c.h"
#include "app_lorawan.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Adafruit_BME680.h"
#include "Ublox.h"
#include "sys_debug.h"
#include <sys_app.h>
#include "gpsCalls.h"
#include "radio_driver.h"


#include "fatfs_sd.h"
#include "string.h"


/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

//static UTIL_TIMER_Object_t TxTimer;
//extern UTIL_TIMER_Object_t TxTimer;


#define PI 3.14159265358979323846
#define EARTH_R 6371000.0				//approximation of earth radius in m
#define MIN_DIST 30						//minimum distance between measurement point

float degtoRad(long degrees){

    float deg = (float)degrees/10000000;
	float radians = (deg / 180.0)* PI ;
	return radians;
}


float haversineDistance(float (*f)(long), long longitude1, long latitude1, long longitude2, long latitude2 ){

	float delta_lat = degtoRad(latitude2 - latitude1);
	float delta_long = degtoRad(longitude2 - longitude1);

	float a =  pow(   sin(  delta_lat/2.0 ), 2 ) + cos(degtoRad(latitude1))* cos(degtoRad(latitude2))*pow(sin(delta_long/2.0), 2);
    float c = 2*atan2(sqrt(a), sqrt(1-a));

    float distance = EARTH_R * c;
	return distance;
}

//static uint8_t AppDataBufferMeasure[242];
//
//static LmHandlerAppData_t measureMessage = {0, 0, AppDataBufferMeasure};
//
//void sendData(long latitude, long longitude, int8_t rssi, int8_t snr){
//
//	uint32_t i = 0;
//
//	LmHandlerErrorStatus_t sendErrorsMsg;
//	LmHandlerMsgTypes_t feedbackConfirmData = LORAMAC_HANDLER_UNCONFIRMED_MSG;
//
//	measureMessage.Port = 1;
//
//
//
//	measureMessage.Buffer[i++] = (uint8_t) ((latitude >> 24) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) ((latitude >> 16) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) ((latitude >> 8) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) (latitude & 0xFF);
//
//	measureMessage.Buffer[i++] = (uint8_t) ((longitude >> 24) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) ((longitude >> 16) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) ((longitude >> 8) & 0xFF);
//	measureMessage.Buffer[i++] = (uint8_t) (longitude & 0xFF);
//
//	measureMessage.Buffer[i++] = rssi;
//	measureMessage.Buffer[i++] = snr;
//
//	measureMessage.BufferSize = i;
//
//	LmHandlerSend(&measureMessage, feedbackConfirmData, 0, false);
//
////	measureMessage.BufferSize = 0;
//
//}
//
//void gaugeRssi(void){
//	static uint8_t AppDataBuffer[242];
//
//	LmHandlerErrorStatus_t sendErrorsFeedback;
//	LmHandlerAppData_t triggerRssi = {0, 0, AppDataBuffer};
//	LmHandlerMsgTypes_t feedbackConfirm = LORAMAC_HANDLER_CONFIRMED_MSG;
//
////	triggerRssi.Buffer[0] = (uint8_t) 0x02;
//	triggerRssi.BufferSize = 0;
//
//	LmHandlerSend(&triggerRssi, feedbackConfirm, 0, false);
//}
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME680_ADDRESS 0X67

//SPI_HandleTypeDef hspi1;
FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;
char buffer[100];

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int counter = 0;
int counterExtra = 0;
int waitrxTimeout = 0;
Adafruit_BME680 bme; // I2C
//I2C_HandleTypeDef hi2c1;

#define doGPS 1

//SFE_UBLOX_GPS myGPS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */


#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

bool mLockout = false;
bool isJoined = false;
bool rxReceived = false;				//set externally in case of received MAC rx in LoRaMac.c (replace in case CUBEMX overwrites)

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
  MX_LoRaWAN_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_SPI1_Init();
//  testFunktion();
  MX_FATFS_Init();

  /* USER CODE BEGIN 2 */

//while(1){
//int test = 4;
//test =  HAL_I2C_IsDeviceReady(&hi2c1, (0x42<<1) , 2, 50);
//
//HAL_Delay(100);
//APP_LOG(TS_ON, VLEVEL_M, "test: %d\r\n", test);
//}

//  /* Waiting for the Micro SD module to initialize */
//		HAL_Delay(500);
//
//		fres = f_mount(&fs, "", 0);
//		if (fres == FR_OK) {
////			transmit_uart("Micro SD card is mounted successfully!\n");
//			  APP_LOG(TS_ON, VLEVEL_M, "Micro SD card is mounted successfully! \r\n");
//
//		} else if (fres != FR_OK) {
////			transmit_uart("Micro SD card's mount error!\n");
//			APP_LOG(TS_ON, VLEVEL_M, "Micro SD card's mount error! \r\n");
//		}
//
//		// FA_OPEN_APPEND opens file if it exists and if not then creates it,
//		// the pointer is set at the end of the file for appending
//		fres = f_open(&fil, "log-file.txt", FA_OPEN_APPEND | FA_WRITE | FA_READ);
//		if (fres == FR_OK) {
////			transmit_uart("File opened for reading and checking the free space.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "File opened for reading and checking the free space. \r\n");
//		} else if (fres != FR_OK) {
////			transmit_uart("File was not opened for reading and checking the free space!\n");
//			APP_LOG(TS_ON, VLEVEL_M, "File was not opened for reading and checking the free space! \r\n");
//		}
//
//		fres = f_getfree("", &fre_clust, &pfs);
//		totalSpace = (uint32_t) ((pfs->n_fatent - 2) * pfs->csize * 0.5);
//		freeSpace = (uint32_t) (fre_clust * pfs->csize * 0.5);
//		char mSz[12];
//		sprintf(mSz, "%lu", freeSpace);
//		if (fres == FR_OK) {
////			transmit_uart("The free space is: ");
////			transmit_uart(mSz);
////			transmit_uart("\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The free space is: %d \r\n", mSz);
//		} else if (fres != FR_OK) {
////			transmit_uart("The free space could not be determined!\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The free space could not be determined! \r\n");
//		}
//
//		for (uint8_t i = 0; i < 10; i++) {
//			f_puts("This text is written in the file.\n", &fil);
//		}
//
//		fres = f_close(&fil);
//		if (fres == FR_OK) {
////			transmit_uart("The file is closed.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The file is closed. \r\n");
//		} else if (fres != FR_OK) {
////			transmit_uart("The file was not closed.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The file was not closed. \r\n");
//		}
//
//		/* Open file to read */
//		fres = f_open(&fil, "log-file.txt", FA_READ);
//		if (fres == FR_OK) {
////			transmit_uart("File opened for reading.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "File opened for reading. \r\n");
//		} else if (fres != FR_OK) {
////			transmit_uart("File was not opened for reading!\n");
//			APP_LOG(TS_ON, VLEVEL_M, "File not opened for reading! \r\n");
//		}
//
//		while (f_gets(buffer, sizeof(buffer), &fil)) {
//			char mRd[100];
//			sprintf(mRd, "%s", buffer);
////			transmit_uart(mRd);
//			APP_LOG(TS_ON, VLEVEL_M, "mRd: %d \r\n", mRd);
//
//		}
//
//		/* Close file */
//		fres = f_close(&fil);
//		if (fres == FR_OK) {
////			transmit_uart("The file is closed.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The file is closed. \r\n");
//		} else if (fres != FR_OK) {
////			transmit_uart("The file was not closed.\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The file was not closed. \r\n");
//		}
//
//		f_mount(NULL, "", 1);
//		if (fres == FR_OK) {
////			transmit_uart("The Micro SD card is unmounted!\n");
//			APP_LOG(TS_ON, VLEVEL_M, "The Micro SD card is unmounted! \r\n");
//		} else if (fres != FR_OK) {
////			transmit_uart("The Micro SD was not unmounted!");
//			APP_LOG(TS_ON, VLEVEL_M, "The Micro SD was not unmounted! \r\n");
//		}


#if doGPS == 1
	APP_LOG(TS_ON, VLEVEL_M, "Enters SETUP GPS \r\n");
	setupGPS();
	APP_LOG(TS_ON, VLEVEL_M, "EXIT SETUP GPS \r\n");
#else
	HAL_Delay(100);
#endif

	long latitude, longitude;

//	long arrlatitude[10];
//	long arrlong[10];
	float distance = 0;
//	int arrInd = 0;
	bool firstrun = true;
	long arrLastPoint[2]; 				//lati in element 0, longitude in element 1

	PacketStatus_t pktStatus;




  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	    //HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	    //myMX_GPIO_Init();
		//printf("... vor ... MX_LoRaWAN_Process() ... counter: %d \n", counter++);
//	if ( (counter++ > 1000) && (mLockout == true) ) {
//	  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
//	  APP_LOG(TS_ON, VLEVEL_M, "counter: %d\r\n", counter);
//	  counter = 0;
//	  mLockout = false;
////	  rxReceived = false;
//	}
//	if(mLockout ) {
//		MX_LoRaWAN_Process();							// mainly runs UTIL_SEQ_Run() defined in stm32_seq.c
//    	APP_LOG(TS_ON, VLEVEL_M, "Main LoRaprocess \r\n");
//	}
//
//
//    if(!isJoined && !mLockout){
//    	mLockout = true;
//    	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
//    	HAL_Delay(50);
//    }
//    if((counter%10)==0) {
//    	APP_LOG(TS_ON, VLEVEL_M, "While running \r\n");
//    }
//    if(!mLockout && isJoined){
//    	APP_LOG(TS_ON, VLEVEL_M, "Main if \r\n");
//		loopGPS(&latitude, &longitude);
//
////		long groundSpeed = loopsGroundSpeed();
//
//	//		if((arrInd==0) && !firstrun ){
//	//			distance = haversineDistance(degtoRad, arrlong[9], arrlatitude[9], arrlong[arrInd], arrlatitude[arrInd]);
//	//			APP_LOG(TS_ON, VLEVEL_M, "Distance: if 1 %d \r\n", distance);
//	//		}else if(arrInd>0){
//	//			distance = haversineDistance(degtoRad, arrlong[arrInd-1], arrlatitude[arrInd-1], arrlong[arrInd], arrlatitude[arrInd]);
//	//			APP_LOG(TS_ON, VLEVEL_M, "Distance: if 2 %d \r\n", distance);
//	//		}
//
//		if(firstrun){
//			// insert save of current rssi and maybe initiate send of message
//
////			gaugeRssi();
//
//			SUBGRF_GetPacketStatus(&pktStatus);
//			arrLastPoint[0] = latitude;
//			arrLastPoint[1] = longitude;
//
////			sendData(arrLastPoint[0], arrLastPoint[1]);
//
//			firstrun = false;
//		} else if ( !firstrun ) {
//			distance = haversineDistance(degtoRad, arrLastPoint[1], arrLastPoint[0], longitude, latitude);
//		}
//
//		if((distance>MIN_DIST) || (counterExtra++ > 40)){
//	    	APP_LOG(TS_ON, VLEVEL_M, "Gauge Rssi \r\n");
//			gaugeRssi();
//			while(!rxReceived){
////				gaugeRssi();
//				MX_LoRaWAN_Process();
//		    	APP_LOG(TS_ON, VLEVEL_M, "Waiting for MAC rxDone \r\n");
//		    	if(waitrxTimeout++ > 3000){
//		    		rxReceived = true;
//		    		waitrxTimeout = 0;
//		    	}
//
//			}
//			rxReceived = true;
//			SUBGRF_GetPacketStatus(&pktStatus);
//			APP_LOG(TS_ON, VLEVEL_M, "Reset rxReceived \r\n");
//			counterExtra = 0;
//		}
//
//
//
//		if(rxReceived){
//			APP_LOG(TS_ON, VLEVEL_M, "Send if \r\n");
//
//			APP_LOG(TS_ON, VLEVEL_M, "RSSI: %d, SNR: %d \r\n", pktStatus.Params.LoRa.RssiPkt, pktStatus.Params.LoRa.SnrPkt);
//			arrLastPoint[0] = latitude;
//			arrLastPoint[1] = longitude;
//			mLockout = true;
////			rxReceived = false;
////			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
//			sendData(arrLastPoint[0], arrLastPoint[1], pktStatus.Params.LoRa.RssiPkt, pktStatus.Params.LoRa.SnrPkt);
////			counterExtra = 0;
//
//		}
//    }
//
////	arrInd++;
////	firstrun = false;
////	if(arrInd>9){
////		arrInd = 0;
////		firstrun = false;
////	}

  }
    /* USER CODE END WHILE */
    MX_LoRaWAN_Process();

    /* USER CODE BEGIN 3 */

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

  /** Configure LSE Drive Capability
  */
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = RCC_MSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the SYSCLKSource, HCLK, PCLK1 and PCLK2 clocks dividers
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK3|RCC_CLOCKTYPE_HCLK
                              |RCC_CLOCKTYPE_SYSCLK|RCC_CLOCKTYPE_PCLK1
                              |RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_MSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.AHBCLK3Divider = RCC_SYSCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
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
	while (1) {
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
