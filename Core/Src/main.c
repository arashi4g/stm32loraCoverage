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
#include "i2c.h"
#include "app_lorawan.h"
#include "tim.h"
#include <math.h>
#include "gpio.h"

#include "stm32_timer.h"
#include "stm32_seq.h"
#include "utilities_def.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Adafruit_BME680.h"
#include "Ublox.h"
#include "sys_debug.h"
#include <sys_app.h>
#include "gpsCalls.h"
#include "radio_driver.h"

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
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define BME680_ADDRESS 0X67
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int counter = 0;
int counterExtra = 0;
Adafruit_BME680 bme; // I2C
I2C_HandleTypeDef hi2c1;

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
  MX_LoRaWAN_Init();					// mainly runs LoRaWAN_Init() function defined in lora_app.c
  MX_I2C1_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

//while(1){
//int test = 4;
//test =  HAL_I2C_IsDeviceReady(&hi2c1, (0x42<<1) , 2, 50);
//
//HAL_Delay(100);
//APP_LOG(TS_ON, VLEVEL_M, "test: %d\r\n", test);
//}


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

	MX_LoRaWAN_Process();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	    //HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	    //myMX_GPIO_Init();
		//printf("... vor ... MX_LoRaWAN_Process() ... counter: %d \n", counter++);
	if ( (counter++ > 1000) && (mLockout == true) ) {
	  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	  APP_LOG(TS_ON, VLEVEL_M, "counter: %d\r\n", counter);
	  counter = 0;
	  mLockout = false;
	}
	if(mLockout) {
		MX_LoRaWAN_Process();							// mainly runs UTIL_SEQ_Run() defined in stm32_seq.c
	}
//    UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);

//    UTIL_TIMER_Start(&TxTimer);
//    HAL_Delay(100);

    if(!isJoined && !mLockout){
    	mLockout = true;
    	UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
    	HAL_Delay(50);
    }
    if((counter%10)==0) {
    	APP_LOG(TS_ON, VLEVEL_M, "While running \r\n");
    }
    if(!mLockout && isJoined){
    	APP_LOG(TS_ON, VLEVEL_M, "Main if \r\n");
		loopGPS(&latitude, &longitude);
//		APP_LOG(TS_ON, VLEVEL_M, "lat: %d, long: %d \r\n", latitude, longitude);
	//	HAL_Delay(100);
	//
//		long groundSpeed = loopsGroundSpeed();

	//		if((arrInd==0) && !firstrun ){
	//			distance = haversineDistance(degtoRad, arrlong[9], arrlatitude[9], arrlong[arrInd], arrlatitude[arrInd]);
	//			APP_LOG(TS_ON, VLEVEL_M, "Distance: if 1 %d \r\n", distance);
	//		}else if(arrInd>0){
	//			distance = haversineDistance(degtoRad, arrlong[arrInd-1], arrlatitude[arrInd-1], arrlong[arrInd], arrlatitude[arrInd]);
	//			APP_LOG(TS_ON, VLEVEL_M, "Distance: if 2 %d \r\n", distance);
	//		}

//		int rssi = SUBGRF_GetRssiInst();

//		APP_LOG(TS_ON, VLEVEL_M, "Rssi: %d \r\n", rssi);
	//
		if(firstrun){
	//		// insert save of current rssi and maybe initiate send of message
			arrLastPoint[0] = latitude;
			arrLastPoint[1] = longitude;
			firstrun = false;
		} else if ( !firstrun ) {
			distance = haversineDistance(degtoRad, arrLastPoint[1], arrLastPoint[0], longitude, latitude);
		}

		if((distance>MIN_DIST) || (counterExtra++ > 5)){
			APP_LOG(TS_ON, VLEVEL_M, "Send if \r\n");

			arrLastPoint[0] = latitude;
			arrLastPoint[1] = longitude;
			mLockout = true;
			UTIL_SEQ_SetTask((1 << CFG_SEQ_Task_LoRaSendOnTxTimerOrButtonEvent), CFG_SEQ_Prio_0);
			counterExtra = 0;

		}
    }

//	arrInd++;
	firstrun = false;
//	if(arrInd>9){
//		arrInd = 0;
//		firstrun = false;
//	}

  }
    /* USER CODE END WHILE */

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
