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
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include "Adafruit_BME680.h"
#include "Ublox.h"
#include "sys_debug.h"
#include <sys_app.h>
#include "gpsCalls.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

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
Adafruit_BME680 bme; // I2C
//char myString[256] = "";
I2C_HandleTypeDef hi2c1;
//TIM_HandleTypeDef htim1;

//int i = 0;
//long lastTime = 0; //Simple local timer. Limits amount if I2C traffic to Ublox module.
//byte i2cDataXX[] = { 0x00, 0x00 };
//uint16_t Hour = 0;
//uint16_t Minute = 0;
//uint16_t Second = 0;

#define doGPS 1

//SFE_UBLOX_GPS myGPS;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void setupGPS();
//void loopGPS();

//void setupGPS() {
//
//  //Wire.begin();
//
//	HAL_I2C_Master_Transmit( &hi2c1, ( 0x15 << 1 ), i2cDataXX, 1, 10 );
//  while (myGPS.begin(0x42) == false) //Connect to the Ublox module using Wire port
//  {
//    //Serial.println(F("Ublox GPS not detected at default I2C address. Please check wiring. Freezing."));
//    HAL_Delay(1);
//	APP_LOG(TS_ON, VLEVEL_M, "Enters SETUP GPS in function \r\n");
//		//HAL_I2C_Master_Transmit( &hi2c2, ( 0x16 << 1 ), i2cDataXX, 1, 10 );
//    //while (1);
//  }
//	//HAL_I2C_Master_Transmit( &hi2c2, ( 0x17 << 1 ), i2cDataXX, 1, 10 );
//
//  myGPS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output UBX only (turn off NMEA noise)
//	//HAL_I2C_Master_Transmit( &hi2c2, ( 0x18 << 1 ), i2cDataXX, 1, 10 );
//  myGPS.saveConfiguration(); //Save the current settings to flash and BBR
//	//HAL_I2C_Master_Transmit( &hi2c2, ( 0x19 << 1 ), i2cDataXX, 1, 10 );
//}

//long longitude;
//long altitude;

//void loopGPS() {
//  bool timeValid = false, dateValid = false;
//  //Query module only every second. Doing it more often will just cause I2C traffic.
//  //The module only responds when a new position is available
//  if ( HAL_GetTick() - lastTime > 1000 ) {
//    lastTime = HAL_GetTick(); //Update the timer
//    long latitude = myGPS.getLatitude();
//		longitude = myGPS.getLongitude();
//    altitude = myGPS.getAltitude();
//    long accuracy = myGPS.getPositionAccuracy();
//
//		//printf( "latitude: %ld longitude: %ld \n", latitude, longitude );
//
//    	APP_LOG(TS_ON, VLEVEL_M, "Lati: %d, Long: %d \r\n", latitude, longitude);
//
//    byte SIV = myGPS.getSIV();
//
//    uint16_t Year = myGPS.getYear();
//    uint16_t Month = myGPS.getMonth();
//    uint16_t Day = myGPS.getDay();
//    Hour = myGPS.getHour();
//    Minute = myGPS.getMinute();
//    Second = myGPS.getSecond();
//
////		printf( "Hour: %2d Minute: %2d Second: %2d\n", Hour, Minute, Second );
//
////  timeValid = myGPS.getTimeValid();
////  dateValid = myGPS.getDateValid();
//  }
//}


void myMX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
}

/*!
 * \brief Initialize the SWO trace port for debug message printing
 * \param portBits Port bit mask to be configured
 * \param cpuCoreFreqHz CPU core clock frequency in Hz
 */
void SWO_Init(uint32_t portBits, uint32_t cpuCoreFreqHz) {
  uint32_t SWOSpeed = 64000; /* default 64k baud rate */
  uint32_t SWOPrescaler = (cpuCoreFreqHz / SWOSpeed) - 1; /* SWOSpeed in Hz, note that cpuCoreFreqHz is expected to be match the CPU core clock */

  CoreDebug->DEMCR = CoreDebug_DEMCR_TRCENA_Msk; /* enable trace in core debug */
  *((volatile unsigned *)(ITM_BASE + 0x400F0)) = 0x00000002; /* "Selected PIN Protocol Register": Select which protocol to use for trace output (2: SWO NRZ, 1: SWO Manchester encoding) */
  *((volatile unsigned *)(ITM_BASE + 0x40010)) = SWOPrescaler; /* "Async Clock Prescaler Register". Scale the baud rate of the asynchronous output */
  *((volatile unsigned *)(ITM_BASE + 0x00FB0)) = 0xC5ACCE55; /* ITM Lock Access Register, C5ACCE55 enables more write access to Control Register 0xE00 :: 0xFFC */
  ITM->TCR = ITM_TCR_TraceBusID_Msk | ITM_TCR_SWOENA_Msk | ITM_TCR_SYNCENA_Msk | ITM_TCR_ITMENA_Msk; /* ITM Trace Control Register */
  ITM->TPR = ITM_TPR_PRIVMASK_Msk; /* ITM Trace Privilege Register */
  ITM->TER = portBits; /* ITM Trace Enable Register. Enabled tracing on stimulus ports. One bit per stimulus port. */
  *((volatile unsigned *)(ITM_BASE + 0x01000)) = 0x400003FE; /* DWT_CTRL */
  *((volatile unsigned *)(ITM_BASE + 0x40304)) = 0x00000100; /* Formatter and Flush Control Register */
}

/*!
 * \brief Sends a character over the SWO channel
 * \param c Character to be sent
 * \param portNo SWO channel number, value in the range of 0 to 31
 */
void SWO_PrintChar(char c, uint8_t portNo) {
  volatile int timeout;

  /* Check if Trace Control Register (ITM->TCR at 0xE0000E80) is set */
  if ((ITM->TCR&ITM_TCR_ITMENA_Msk) == 0) { /* check Trace Control Register if ITM trace is enabled*/
    return; /* not enabled? */
  }
  /* Check if the requested channel stimulus port (ITM->TER at 0xE0000E00) is enabled */
  if ((ITM->TER & (1ul<<portNo))==0) { /* check Trace Enable Register if requested port is enabled */
    return; /* requested port not enabled? */
  }
  timeout = 5000; /* arbitrary timeout value */
  while (ITM->PORT[0].u32 == 0) {
    /* Wait until STIMx is ready, then send data */
    timeout--;
    if (timeout==0) {
      return; /* not able to send */
    }
  }
  ITM->PORT[0].u16 = 0x08 | (c<<8);
  //HAL_GPIO_TogglePin(LED1_GPIO_PORT, LED1_PIN);
}

/*!
 * \brief Sends a string over SWO to the host
 * \param s String to send
 * \param portNumber Port number, 0-31, use 0 for normal debug strings
 */
void SWO_PrintString(const char *s, uint8_t portNumber) {
  while (*s!='\0') {
    SWO_PrintChar(*s++, portNumber);
  }
}

void ITM_PrintString(const char *s) {
  while (*s!='\0') {
    ITM_SendChar(*s++);
  }
}

#include  <errno.h>
#include  <sys/unistd.h> // STDOUT_FILENO, STDERR_FILENO

extern UART_HandleTypeDef huart1; // access huart1 instance
int _write(int file, char *data, int len)
{
   if ((file != STDOUT_FILENO) && (file != STDERR_FILENO))
   {
      errno = EBADF;
      return -1;
   }

   // arbitrary timeout 1000
   HAL_StatusTypeDef status = // HAL_OK;
      HAL_UART_Transmit(&huart1, (uint8_t*)data, len, 1000);
   //ITM_PrintString(data);

   // return # of bytes written - as best we can tell
   return (status == HAL_OK ? len : 0);
}

//#include <sys_app.h>

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
  /* USER CODE BEGIN 2 */


//while(1){
//int test = 4;
//test =  HAL_I2C_IsDeviceReady(&hi2c1, (0x42<<1) , 2, 50);
//
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


HAL_Delay(10);
HAL_Delay(100);


  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1) {
	    //HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	    //myMX_GPIO_Init();
		//printf("... vor ... MX_LoRaWAN_Process() ... counter: %d \n", counter++);
	if ( counter++ > 1000 ) {
	  HAL_GPIO_TogglePin(GPIOB, LED1_Pin);
	  APP_LOG(TS_ON, VLEVEL_M, "counter: %d\r\n", counter);
	  counter = 0;
	}
    /* USER CODE END WHILE */
//    MX_LoRaWAN_Process();
	loopGPS();
    /* USER CODE BEGIN 3 */
		//HAL_Delay(100);
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
