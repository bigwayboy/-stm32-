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
  * Copyright (c) 2022 STMicroelectronics International N.V. 
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

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include "main.h"
#include "stm32f4xx_hal.h"
#include "mpu6050.h"
#include "mytask.h"
#include "inv_mpu_user.h"


/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId Task200hzHandle;
osThreadId Task100hzHandle;
osThreadId Task20hzHandle;
osThreadId Task5hzHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartTask200hz(void const * argument);
void StartTask100hz(void const * argument);
void StartTask20hz(void const * argument);
void StartTask5hz(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
       
  /* USER CODE END Init */

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
  /* definition and creation of Task200hz */
  osThreadDef(Task200hz, StartTask200hz, osPriorityNormal, 0, 128);
  Task200hzHandle = osThreadCreate(osThread(Task200hz), NULL);

  /* definition and creation of Task100hz */
  osThreadDef(Task100hz, StartTask100hz, osPriorityIdle, 0, 128);
  Task100hzHandle = osThreadCreate(osThread(Task100hz), NULL);

  /* definition and creation of Task20hz */
  osThreadDef(Task20hz, StartTask20hz, osPriorityIdle, 0, 128);
  Task20hzHandle = osThreadCreate(osThread(Task20hz), NULL);

  /* definition and creation of Task5hz */
  osThreadDef(Task5hz, StartTask5hz, osPriorityIdle, 0, 128);
  Task5hzHandle = osThreadCreate(osThread(Task5hz), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartTask200hz function */
void StartTask200hz(void const * argument)
{

  /* USER CODE BEGIN StartTask200hz */
	
	printf("环境数据采集进程系统启动\n");
	MPU_Init();//初始化MPU6050
	while(mpu_dmp_init())
//	{
//	}

	printf("初始化成功\n");
  /* Infinite loop */
  for(;;)
  {
		task_200hz();
    osDelay(5);
  }
  /* USER CODE END StartTask200hz */
}

/* StartTask100hz function */
void StartTask100hz(void const * argument)
{
  /* USER CODE BEGIN StartTask100hz */
  /* Infinite loop */
  for(;;)
  {
		task_100hz();
    osDelay(10);
  }
  /* USER CODE END StartTask100hz */
}

/* StartTask20hz function */
void StartTask20hz(void const * argument)
{
  /* USER CODE BEGIN StartTask20hz */
  /* Infinite loop */
  for(;;)
  {
		task_20hz();
    osDelay(50);
  }
  /* USER CODE END StartTask20hz */
}

/* StartTask5hz function */
void StartTask5hz(void const * argument)
{
  /* USER CODE BEGIN StartTask5hz */
  /* Infinite loop */
  for(;;)
  {
		task_5hz();
    osDelay(1000);
  }
  /* USER CODE END StartTask5hz */
}

/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
