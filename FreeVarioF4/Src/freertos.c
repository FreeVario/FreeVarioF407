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
#include "nmea.h"
#include "usbd_cdc_if.h"
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


/* FV CCM memory allocation-----------------------------------------------------*/
unsigned char * frame_buffer[EPD_WIDTH * EPD_HEIGHT / 8] __attribute__((section(".ccmram")));
Paint  paint __attribute__((section(".ccmram")));
SensorData sensors __attribute__((section(".ccmram")));
ActivityData activity __attribute__((section(".ccmram")));
gps_t  hgps __attribute__((section(".ccmram")));
settings_t conf __attribute__((section(".ccmram")));
uint8_t nmeasendbuffer[SENDBUFFER] __attribute__((section(".ccmram")));
/*-----------------------------------------------------------------------------*/

/* FV Task allocation----------------------------------------------------------*/

TaskHandle_t xTaskToNotify = NULL;
TaskHandle_t xSendDataNotify = NULL;
TaskHandle_t xDisplayNotify = NULL;


/* FV Queues  -----------------------------------------------------------------*/

QueueHandle_t uartQueueHandle;

/* FV Semaphores and Mutexes---------------------------------------------------*/



/*-----------------------------------------------------------------------------*/

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart3;
extern char SDPath[4]; /* SD logical drive path */
extern FATFS SDFatFS; /* File system object for SD logical drive */
__IO uint8_t UserPowerButton = 0;
uint8_t SDcardMounted = 0;


/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId displayTaskHandle;
osThreadId sensorsTaskHandle;
osThreadId gpsTaskHandle;
osThreadId sendDataTaskHandle;
osThreadId audioTaskHandle;
osThreadId loggerTaskHandle;
osMutexId confMutexHandle;
osMutexId sdCardMutexHandle;
osMutexId activityMutexHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) {


	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(xTaskToNotify, &xHigherPriorityTaskWoken);

	xTaskToNotify = NULL;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}

void HAL_UART_RxIdleCallback(UART_HandleTypeDef *UartHandle) {
	__HAL_UART_DISABLE_IT(UartHandle, UART_IT_IDLE);
//
//		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
//		vTaskNotifyGiveFromISR( xTaskToNotify, &xHigherPriorityTaskWoken );
//		xTaskToNotify = NULL;
//	    portYIELD_FROM_ISR( xHigherPriorityTaskWoken );

}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *UartHandle) {

	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	vTaskNotifyGiveFromISR(xSendDataNotify, &xHigherPriorityTaskWoken);
	xSendDataNotify = NULL;
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);


}

/**
 * @brief EXTI line detection callbacks
 * @param GPIO_Pin: Specifies the pins connected EXTI line
 * @retval None
 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == PWRBUTTON_Pin) {
		/* Set the variable: button pressed */
		if (HAL_GetTick() > 5000) {
			UserPowerButton = 1;
		}
	}else if (GPIO_Pin == OPTBUTTON_Pin) {

		if(activity.takeOff){
			activity.landed = 1;

		}else{

		   activity.takeOff = 1;
		}

	}

}


/**
  * @brief  This function configures the system to enter Standby mode for
  *         current consumption measurement purpose.
  *         STANDBY Mode
  *         ============
  *           - Backup SRAM and RTC OFF
  *           - IWDG and LSI OFF
  *           - Wakeup using WakeUp Pin (PA.00)
  * @param  None
  * @retval None
  */
void StandbyMode(void)
{
  /* Enable Power Clock*/
  __HAL_RCC_PWR_CLK_ENABLE();

  /* Allow access to Backup */
  HAL_PWR_EnableBkUpAccess();

  /* Reset RTC Domain */
  __HAL_RCC_BACKUPRESET_FORCE();
  __HAL_RCC_BACKUPRESET_RELEASE();

  /* Disable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_DisableWakeUpPin(PWR_WAKEUP_PIN1);

  /* Clear all related wakeup flags */
  __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);

  /* Re-enable all used wakeup sources: Pin1(PA.0) */
  HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1);

  /* Request to enter STANDBY mode  */
  HAL_PWR_EnterSTANDBYMode();
}



/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartDisplayTask(void const * argument);
void StartSensorsTask(void const * argument);
void StartGPSTask(void const * argument);
void StartSendDataTask(void const * argument);
void StartAudioTask(void const * argument);
void StartLoggerTask(void const * argument);

extern void MX_FATFS_Init(void);
extern void MX_USB_DEVICE_Init(void);
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
  /* definition and creation of confMutex */
  osMutexDef(confMutex);
  confMutexHandle = osMutexCreate(osMutex(confMutex));

  /* definition and creation of sdCardMutex */
  osMutexDef(sdCardMutex);
  sdCardMutexHandle = osMutexCreate(osMutex(sdCardMutex));

  /* definition and creation of activityMutex */
  osMutexDef(activityMutex);
  activityMutexHandle = osMutexCreate(osMutex(activityMutex));

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
  osThreadDef(displayTask, StartDisplayTask, osPriorityBelowNormal, 0, 2048);
  displayTaskHandle = osThreadCreate(osThread(displayTask), NULL);

  /* definition and creation of sensorsTask */
  osThreadDef(sensorsTask, StartSensorsTask, osPriorityNormal, 0, 1024);
  sensorsTaskHandle = osThreadCreate(osThread(sensorsTask), NULL);

  /* definition and creation of gpsTask */
  osThreadDef(gpsTask, StartGPSTask, osPriorityNormal, 0, 2048);
  gpsTaskHandle = osThreadCreate(osThread(gpsTask), NULL);

  /* definition and creation of sendDataTask */
  osThreadDef(sendDataTask, StartSendDataTask, osPriorityAboveNormal, 0, 2048);
  sendDataTaskHandle = osThreadCreate(osThread(sendDataTask), NULL);

  /* definition and creation of audioTask */
  osThreadDef(audioTask, StartAudioTask, osPriorityNormal, 0, 1024);
  audioTaskHandle = osThreadCreate(osThread(audioTask), NULL);

  /* definition and creation of loggerTask */
  osThreadDef(loggerTask, StartLoggerTask, osPriorityNormal, 0, 1024);
  loggerTaskHandle = osThreadCreate(osThread(loggerTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */

	uartQueueHandle = xQueueCreate(2, SENDBUFFER);

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
  /* init code for FATFS */
  MX_FATFS_Init();

  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();

  /* USER CODE BEGIN StartDefaultTask */
	//BSP_SD_Init();
  xSemaphoreGive(confMutexHandle);
  xSemaphoreGive(sdCardMutexHandle);
  xSemaphoreGive(activityMutexHandle);

  memset(&activity, 0, sizeof(activity));


  if ( xSemaphoreTake( sdCardMutexHandle, ( TickType_t ) 500 ) == pdTRUE) {
		if ( xSemaphoreTake( confMutexHandle, ( TickType_t ) 100 ) == pdTRUE) {
			if (f_mount(&SDFatFS, SDPath, 0) == FR_OK) { //Mount SD card
				SDcardMounted =1;
			}
			loadConfigFromSD();
			xSemaphoreGive(confMutexHandle);
		}
		xSemaphoreGive(sdCardMutexHandle);
  }

	/* Infinite loop */
	for (;;) {

		if  (UserPowerButton) {
			UserPowerButton = 0;
			xTaskNotify(xDisplayNotify,0x01,eSetValueWithOverwrite);
			while(HAL_GPIO_ReadPin(PWRBUTTON_GPIO_Port,PWRBUTTON_Pin) == GPIO_PIN_SET) { //wait for button to be released
			}
			f_mount(0, "0:", 1); //unmount SDCARD

			osDelay(4000);
			StandbyMode();
		}


		//Flight Operations

		if(activity.takeOff && !activity.isFlying)  { //took off
			activity.currentLogID = conf.lastLogNumber + 1;
			activity.takeoffLocationLAT = hgps.latitude;
			activity.takeoffLocationLON=hgps.longitude;
			activity.takeoffTemp = sensors.temperature;
			activity.takeoffTime = hgps.date;

			activity.currentLogDataSet = 0;
			activity.isFlying = 1;
		}

		if(activity.isFlying){ //flying
			if(sensors.AltitudeMeters > activity.MaxAltitudeMeters) activity.MaxAltitudeMeters = sensors.AltitudeMeters;
			if(sensors.VarioMs > activity.MaxVarioMs) activity.MaxVarioMs = sensors.VarioMs;
			if(sensors.VarioMs < activity.MaxVarioSinkMs) activity.MaxVarioSinkMs = sensors.VarioMs;

		}

		if (activity.landed) {
			activity.landingAltitude = sensors.AltitudeMeters;
			activity.landingLocationLAT = hgps.latitude;
			activity.landingLocationLON = hgps.longitude;
			activity.MaxAltitudeGainedMeters = activity.MaxAltitudeMeters  - activity.takeoffAltitude;
			activity.currentLogDataSet = 1;
			activity.landed =0;
			activity.takeOff =0;
			activity.isFlying = 0;




			conf.lastLogNumber = activity.currentLogID;

		}

		osDelay(100);
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
	uint32_t ulNotifiedValue;
	BaseType_t xResult;
	 TickType_t xMaxBlockTime;
	 TickType_t times;
	 activity.currentLogID = 1;
		EPD epd;
		displayTaskSetup(&paint,&epd, frame_buffer);


	/* Infinite loop */
	for (;;) {
		times = xTaskGetTickCount();
		 displayTaskUpdate(&paint,&epd,frame_buffer);

		 xDisplayNotify = xTaskGetCurrentTaskHandle();
		 TickType_t waittime = abs( 500 - (xTaskGetTickCount() - times) ) ;
		 if (waittime > 500) waittime = 500;
		 xMaxBlockTime = pdMS_TO_TICKS(waittime);

		 xResult = xTaskNotifyWait( pdFALSE,    /* Don't clear bits on entry. */
		                            pdTRUE,        /* Clear all bits on exit. */
		                            &ulNotifiedValue, /* Stores the notified value. */
		                            xMaxBlockTime );
		 if( xResult == pdPASS )
		      {
		         /* A notification was received.  See which bits were set. */
		         if( ( ulNotifiedValue & 0x01 ) != 0 )
		         {
		        	 displayMessageShutdown(&paint,&epd,frame_buffer);
		        	 osDelay(4000); //just sleep till shutdown
		         }

		         if( ( ulNotifiedValue & 0x02 ) != 0 )
		         {

		         }
		      }

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
	TickType_t startTime = xTaskGetTickCount();
	const TickType_t xDelay = 50; //20hz
	uint8_t timetosend = 1;
	BMP280_HandleTypedef bmp280;
	SD_MPU6050 mpu1;
	 memset(&sensors, 0, sizeof(sensors));

	sensors.humidity = 0;
	sensors.pressure = 0;
	sensors.temperature = 0;
	sensors.barotakeoff = 0;

	setupReadSensorsBMP280(&bmp280);
	setupReadSensorsMPU6050(&mpu1);
	osDelay(100);

	/* Infinite loop */
	for (;;) {
		times = xTaskGetTickCount();
		timetosend++;
		readSensorsBMP280(&bmp280);
		readSensorsMPU6050(&mpu1);

		if ((timetosend >= 2)) { //every 100 ticks
			calculateVario100ms();
		}

		if ((timetosend >= 4)
				& ((xTaskGetTickCount() - startTime) > STARTDELAY)) { //every 200 ticks
			timetosend = 1;

			memset(nmeasendbuffer, 0, SENDBUFFER);
			NMEA_getPTAS1(nmeasendbuffer, sensors.VarioMs, sensors.VarioMs,
					sensors.AltitudeMeters);
			NMEA_getnmeaShortLXWP0(nmeasendbuffer, sensors.AltitudeMeters,
					sensors.VarioMs);
			NMEA_getNmeaLK8EX1(nmeasendbuffer, sensors.pressure, sensors.AltitudeMeters,
					sensors.VarioMs, sensors.temperature, 8000);
			NMEA_getNmeaPcProbe(nmeasendbuffer, sensors.accel_x, sensors.accel_y,
					sensors.accel_z, sensors.temperature, sensors.humidity, 100,
					0);
			xQueueSendToBack(uartQueueHandle, nmeasendbuffer, 10);

		}

#if defined(TAKEOFFVARIO) && !defined(TESTBUZZER)
		if ((int) xTaskGetTickCount() > (STARTDELAY + 4000)
				&& !sensors.barotakeoff) {
			if (abs(sensors.VarioMs) > TAKEOFFVARIO) {
				sensors.barotakeoff = true;

			}

		}
#else
		sensors.barotakeoff = true;
#endif

		vTaskDelayUntil(&times, xDelay);
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
	uint8_t buffer[SENDBUFFER]; //DMA buffer can't use ccm
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

		gps_process(&hgps, &buffer, sizeof(buffer));
		xQueueSendToBack(uartQueueHandle, buffer, 10);

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

		xQueueReceive(uartQueueHandle, &receiveqBuffer, portMAX_DELAY);
		uint16_t buffsize = strlen((char *) receiveqBuffer);

		xSendDataNotify = xTaskGetCurrentTaskHandle();

		HAL_UART_Transmit_DMA(&huart1, (uint8_t *) &receiveqBuffer, buffsize); //Usart global interupt must be enabled for this to work
		CDC_Transmit_FS((uint8_t *) &receiveqBuffer, buffsize);
		//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
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
	//testvarss
//    int step=0;
//	int t_vario=-5000;
//	uint32_t times=0;
	//
	audio_t audiorun;
	audiorun.multiplier = 2000000;
	uint8_t running = 0;

	setupAudio(&audiorun);
	/* Infinite loop */
	for (;;) {

		if (xTaskGetTickCount() > STARTDELAY) {
			running = 1;
		}

		// uint32_t i =xTaskGetTickCount() - times;

//	  if (i > 1000) {
//	    times = xTaskGetTickCount();
//	    if(t_vario <= -5000) step = 100;
//		if(t_vario >= 9000) step = -100;
//		HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
//		t_vario += step;
//	  }
		if (running) {
		if (sensors.barotakeoff) {
				makeVarioAudio(&audiorun, sensors.VarioMs); //flying
			}
		}

		osDelay(10);
	}
  /* USER CODE END StartAudioTask */
}

/* USER CODE BEGIN Header_StartLoggerTask */
/**
* @brief Function implementing the loggerTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartLoggerTask */
void StartLoggerTask(void const * argument) {
	/* USER CODE BEGIN StartLoggerTask */

	TickType_t times;
	const TickType_t xDelay = 1000;

	osDelay(5000); //wait for setup of environment
	/* Infinite loop */
	for (;;) {
		times = xTaskGetTickCount();

		if (!SDcardMounted) { //can't continue without a SD card
			vTaskSuspend( NULL);
		}

		if (activity.takeOff && !activity.isFlying) { //just log the start of the flight
			activity.isLogging = 1;
			if ( xSemaphoreTake(sdCardMutexHandle,
					(TickType_t ) 600) == pdTRUE) {
				writeFlightLogSummaryFile();
				xSemaphoreGive(sdCardMutexHandle);
			}

		}

		if (activity.isFlying) { //track logger

			if ( xSemaphoreTake(sdCardMutexHandle,
					(TickType_t ) 600) == pdTRUE) {
				HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
				//TODO:writelog
				xSemaphoreGive(sdCardMutexHandle);
			}

		}

		if (activity.currentLogDataSet) { //all activity data is set
			activity.isLogging = 0;
			if ( xSemaphoreTake(sdCardMutexHandle,
					(TickType_t ) 600) == pdTRUE) {
				writeFlightLogSummaryFile();
				xSemaphoreGive(sdCardMutexHandle);
			}

		}

		//TODO: try fsync

		vTaskDelayUntil(&times, xDelay);
	}
	/* USER CODE END StartLoggerTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
