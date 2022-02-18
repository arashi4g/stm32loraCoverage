/**
  ******************************************************************************
  * @file    app_lorawan.c
  * @author  MCD Application Team
  * @brief   Application of the LRWAN Middleware
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "app_lorawan.h"
#include "lora_app.h"
#include "sys_app.h"
#include "stm32_seq.h"
/* USER CODE BEGIN Includes */
#include <stdio.h>
extern int counter;
extern char *myString;
extern void ITM_PrintString(const char *s);
/*void ITM_PrintString(const char *s) {
  while (*s!='\0') {
    ITM_SendChar(*s++);
  }
}*/

/* USER CODE END Includes */

/* External variables ---------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

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

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Exported functions --------------------------------------------------------*/

void MX_LoRaWAN_Init(void)
{
  /* USER CODE BEGIN MX_LoRaWAN_Init_1 */
  //sprintf( myString, "... vor ... SystemApp_Init() ... counter: %d \n", counter++ ); ITM_PrintString(myString);
  //ITM_SendChar('x');
  printf( "... MX_LoRaWAN_init() ... \n" );
  /* USER CODE END MX_LoRaWAN_Init_1 */
  SystemApp_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_2 */
  //sprintf( myString, "... nach ... SystemApp_Init() ... counter: %d \n", counter++ ); ITM_PrintString(myString);
  /* USER CODE END MX_LoRaWAN_Init_2 */
  LoRaWAN_Init();
  /* USER CODE BEGIN MX_LoRaWAN_Init_3 */
  //sprintf( myString, "... nach ... LoRaWAN_Init() ... counter: %d \n", counter++ ); ITM_PrintString(myString);
  /* USER CODE END MX_LoRaWAN_Init_3 */
}

void MX_LoRaWAN_Process(void)
{
  /* USER CODE BEGIN MX_LoRaWAN_Process_1 */
  //sprintf( myString, "... nach ... SystemClock_Config() ... counter: %d \n", counter++ ); ITM_PrintString(myString);
  printf( "... MX_LoRaWAN_Process() ... \n" );

  /* USER CODE END MX_LoRaWAN_Process_1 */
  UTIL_SEQ_Run(UTIL_SEQ_DEFAULT);
  /* USER CODE BEGIN MX_LoRaWAN_Process_2 */

  /* USER CODE END MX_LoRaWAN_Process_2 */
}

/* USER CODE BEGIN EF */

/* USER CODE END EF */

/* Private Functions Definition -----------------------------------------------*/
/* USER CODE BEGIN PrFD */

/* USER CODE END PrFD */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
