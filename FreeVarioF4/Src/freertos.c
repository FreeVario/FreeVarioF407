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
  * Copyright (c) 2018 STMicroelectronics International N.V. 
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include <displaytask.h>
#include <globaldata.h>
#include <readsensors.h>
#include "fatfs.h"
#include "audio.h"
#include <sd_hal_mpu6050.h>
#include "gps.h"
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SENDBUFFER 256
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
QueueHandle_t sysQueueHandle;
QueueHandle_t uartQueueHandle;

/* FV CCM memory allocation-----------------------------------------------------*/
unsigned char * frame_buffer[EPD_WIDTH * EPD_HEIGHT / 8] __attribute__((section(".ccmram")));
SensorData sensors __attribute__((section(".ccmram")));
gps_t hgps __attribute__((section(".ccmram")));
/*-----------------------------------------------------------------------------*/
TaskHandle_t xTaskToNotify = NULL;
TaskHandle_t xSendDataNotify = NULL;
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern char SDPath[4];   /* SD logical drive path */
extern FATFS SDFatFS;    /* File system object for SD logical drive */

/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId sensorsTaskHandle;
osThreadId gpsTaskHandle;
osThreadId sendDataTaskHandle;
osThreadId audioTaskHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{

	//memcpy(&transferBuffer, &receiveBuffer, GPSDMAHALFBUFFER);
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );

	xTaskToNotify = NULL;
    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}


void HAL_UART_RxIdleCallback(UART_HandleTypeDef *UartHandle) {
	 __HAL_UART_DISABLE_IT(UartHandle, UART_IT_IDLE);

	//	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	//	vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
	//	xTaskToNotify = NULL;
	//    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {


		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		vTaskNotifyGiveFromISR( xSendDataNotify, &xHigherPriorityTaskWoken );
		xSendDataNotify = NULL;
	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );


}

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartSensorsTask(void const * argument);
void StartGPSTask(void const * argument);
void StartSendDataTask(void const * argument);
void StartAudioTask(void const * argument);

extern void MX_USB_HOST_Init(void);
extern void MX_FATFS_Init(void);
void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
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
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 1024);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of displayTask */
  osThreadDef(displayTask, StartDisplayTask, osPriorityBelowNormal, 0, 1024);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of sensorsTask */
  osThreadDef(sensorsTask, StartSensorsTask, osPriorityNormal, 0, 1024);
  sensorsTaskHandle = osThreadCreate(osThread(sensorsTask), NULL);

  /* definition and creation of gpsTask */
  osThreadDef(gpsTask, StartGPSTask, osPriorityNormal, 0, 1024);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

  /* definition and creation of sendDataTask */
  osThreadDef(sendDataTask, StartSendDataTask, osPriorityAboveNormal, 0, 1024);
  sendDataTaskHandle = osThreadCreate(osThread(sendDataTask), NULL);

  /* definition and creation of audioTask */
  osThreadDef(audioTask, StartAudioTask, osPriorityNormal, 0, 1024);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */


  uartQueueHandle = xQueueCreate( 4, SENDBUFFER );
  /* USER CODE END RTOS_QUEUES */
}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* init code for USB_HOST */
  MX_USB_HOST_Init();

  /* init code for FATFS */
  MX_FATFS_Init();

  /* USER CODE BEGIN StartDefaultTask */
  //BSP_SD_Init();
  setupConfig();
  FIL MyFile;

  if (f_mount(&SDFatFS, SDPath, 0)== FR_OK){

	  uint8_t wtext[] = "This is STM32 working with FatFs\r\n";
	  FRESULT res;
	  uint32_t byteswritten;
	  if (f_open(&MyFile, "STM32.TXT", FA_CREATE_ALWAYS | FA_WRITE)
				!= FR_OK) {
			/* 'STM32.TXT' file Open for write Error */
			//Error_Handler();
		} else {
			/* Write data to the text file */
			res = f_write(&MyFile, wtext, sizeof(wtext),
					(void *) &byteswritten);

			if ((byteswritten == 0) || (res != FR_OK)) {
				/* 'STM32.TXT' file Write or EOF Error */
				//Error_Handler();
			} else {
				/* Close the open text file */
				f_close(&MyFile);
				f_mount(0, "0:", 1); //unmount
			}

		}
	//  f_mount(0, "0:", 1);
  }



  /* Infinite loop */
  for(;;)
  {

    osDelay(500);
  }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the displayTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDisplayTask */
void StartDisplayTask(void const * argument)
{
  /* USER CODE BEGIN StartDisplayTask */
	//unsigned char * frame_buffer = (unsigned char*)malloc(EPD_WIDTH * EPD_HEIGHT / 8);
	memset(frame_buffer , 0, EPD_WIDTH * EPD_HEIGHT / 8);
	Paint paint;
	EPD epd;
	displayTaskSetup(&paint,&epd, &frame_buffer);

  /* Infinite loop */
  for(;;)
  {
	  displayTaskUpdate(&paint,&epd,&frame_buffer);
	  osDelay(500);
  }
  /* USER CODE END StartDisplayTask */
}

/* USER CODE BEGIN Header_StartSensorsTask */
/**
* @brief Function implementing the sensorsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorsTask */
void StartSensorsTask(void const * argument)
{
  /* USER CODE BEGIN StartSensorsTask */

	TickType_t times;
	const TickType_t xDelay = 20; //50hz
	uint8_t timetosend=1;
	BMP280_HandleTypedef bmp280;
	SD_MPU6050 mpu1;

	sensors.humidity = 0;
	sensors.pressure = 0;
	sensors.temperature = 0;

	setupReadSensorsBMP280(&bmp280);
	setupReadSensorsMPU6050(&mpu1);

  /* Infinite loop */
  for(;;)
  {
	  times = xTaskGetTickCount();
	  timetosend++;
	  readSensorsBMP280(&bmp280);
	  readSensorsMPU6050(&mpu1);

	  if (timetosend >= 5) { //every 100 ticks
		  calculateVario100ms();
	  }

	  if (timetosend >= 10) { //every 200 ticks
		  timetosend=1;
		  uint8_t notice[SENDBUFFER];
		  memset(notice, 0, SENDBUFFER);
		  sprintf(notice,"----------ACCL_X:%d--------------------\r\n", sensors.accel_x);
		  xQueueSendToBack(uartQueueHandle, notice, 10);

	  }

	  vTaskDelayUntil( &times, xDelay );
  }
  /* USER CODE END StartSensorsTask */
}

/* USER CODE BEGIN Header_StartGPSTask */
/**
* @brief Function implementing the gpsTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartGPSTask */
void StartGPSTask(void const * argument)
{
  /* USER CODE BEGIN StartGPSTask */
	gps_init(&hgps);
	uint8_t buffer[SENDBUFFER];// __attribute__((section(".ccmram")));
	uint8_t rcvdCount;
	configASSERT(xTaskToNotify == NULL);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);


	/* Infinite loop */
	for (;;) {
		memset(buffer, 0, sizeof(buffer));
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		__HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

		if (HAL_UART_Receive_DMA(&huart3, buffer, sizeof(buffer)) != HAL_OK) {
			// error
		}
		xTaskToNotify = xTaskGetCurrentTaskHandle();
		ulTaskNotifyTake( pdTRUE, portMAX_DELAY);

		rcvdCount = sizeof(buffer) - huart3.hdmarx->Instance->NDTR;
		HAL_UART_DMAStop(&huart3);


		gps_process(&hgps, buffer, rcvdCount);
		xQueueSendToBack(uartQueueHandle, buffer, 10);

		uint8_t notice[SENDBUFFER];
		memset(notice, 0, SENDBUFFER);
		sprintf(notice, "----------GPS:%d --------------------\r\n", hgps.fix_mode);
		xQueueSendToBack(uartQueueHandle, notice, 10);
	}
  /* USER CODE END StartGPSTask */
}

/* USER CODE BEGIN Header_StartSendDataTask */
/**
* @brief Function implementing the sendDataTask thread.
* @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSendDataTask */
void StartSendDataTask(void const * argument)
{
  /* USER CODE BEGIN StartSendDataTask */

	configASSERT(xSendDataNotify == NULL);
	const TickType_t xMaxBlockTime = pdMS_TO_TICKS(500);
	/* Infinite loop */
	for (;;) {
		uint8_t receiveqBuffer[SENDBUFFER];

		xQueueReceive ( uartQueueHandle,&receiveqBuffer,portMAX_DELAY );
		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);

		xSendDataNotify = xTaskGetCurrentTaskHandle();
		HAL_UART_Transmit_DMA(&huart1, (uint8_t *) &receiveqBuffer, SENDBUFFER); //Usart global interupt must be enabled for this to work

		ulTaskNotifyTake( pdTRUE, xMaxBlockTime);

		osDelay(1);
	}
  /* USER CODE END StartSendDataTask */
}

/* USER CODE BEGIN Header_StartAudioTask */
/**
* @brief Function implementing the audioTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartAudioTask */
void StartAudioTask(void const * argument)
{
  /* USER CODE BEGIN StartAudioTask */
	setupAudio();
  /* Infinite loop */
  for(;;)
  {
		tone(1000);
	    osDelay(250);
	   // noTone();
	    osDelay(250);
  }
  /* USER CODE END StartAudioTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
     
/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
