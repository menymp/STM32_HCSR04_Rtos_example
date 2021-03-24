/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2019 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include "tim.h"
#include "usart.h"
#include "gpio.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     

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
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
osThreadId TaskUltrasonicHandle;
osThreadId TaskSerialHandle;
osMutexId myMutex01Handle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
   
/* USER CODE END FunctionPrototypes */

void Task_ultrasonic(void const * argument);
void Task_Serial(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of myMutex01 */
  osMutexDef(myMutex01);
  myMutex01Handle = osMutexCreate(osMutex(myMutex01));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of TaskUltrasonic */
  osThreadDef(TaskUltrasonic, Task_ultrasonic, osPriorityNormal, 0, 128);
  TaskUltrasonicHandle = osThreadCreate(osThread(TaskUltrasonic), NULL);

  /* definition and creation of TaskSerial */
  osThreadDef(TaskSerial, Task_Serial, osPriorityIdle, 0, 128);
  TaskSerialHandle = osThreadCreate(osThread(TaskSerial), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_Task_ultrasonic */
/**
  * @brief  Function implementing the TaskUltrasonic thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_Task_ultrasonic */
extern state State_measure;
extern UART_HandleTypeDef huart2;
extern volatile float Medicion;
extern volatile int count;
extern volatile int Medicion_d;

union u_type            //Setup a Union
{
  unsigned int IntVar;
  unsigned char Bytes[4];
}Range;

void Task_ultrasonic(void const * argument)
{

  /* USER CODE BEGIN Task_ultrasonic */
  /* Infinite loop */
  for(;;)
  {
	//semaphore get
	xSemaphoreTake(myMutex01Handle,portMAX_DELAY);
	State_measure = Ready;
	HAL_TIM_Base_Start_IT(&htim15);
	xSemaphoreGive(myMutex01Handle);
	//semaphore free
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5); //heartbeat
    osDelay(30);
  }
  /* USER CODE END Task_ultrasonic */
}

/* USER CODE BEGIN Header_Task_Serial */
/**
* @brief Function implementing the TaskSerial thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Task_Serial */
void Task_Serial(void const * argument)
{
  /* USER CODE BEGIN Task_Serial */
  /* Infinite loop */
  uint8_t str[100];
  int len = 0;

  for(;;)
  {
	//semaphore get
	xSemaphoreTake(myMutex01Handle,portMAX_DELAY);
	Range.IntVar = Medicion_d;
	len = sprintf(str,"Range (cm) = %d , %d , %d\n",Medicion_d,Range.Bytes[0],Range.Bytes[1]);//variable compartida
	//semaphore free
	xSemaphoreGive(myMutex01Handle);
	HAL_UART_Transmit(&huart2,str,len , 15);

    osDelay(200);
  }
  /* USER CODE END Task_Serial */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
