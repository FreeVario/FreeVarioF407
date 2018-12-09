/*
 * datalog.c
 *
 *  Created on: Dec 9, 2018
 *      Author: marco
 */


#include "datalog.h"


/**
 * @brief Rewrites the Flight log summary file
 * @param None
 * @retval None
 */
void writeFlightLogSummaryFile(){
	//Make sure the mutex is set
	//filename will be <lognumber>.log
	//and <lognumber>.igc

	FIL logSumFile;
	FRESULT res;
	uint32_t byteswritten;
	uint8_t wtext[256];
	char filename[32];
	sprintf(filename,"%d.log",activity.currentLogID);





	if (f_open(&logSumFile, filename,
						FA_OPEN_APPEND | FA_WRITE) != FR_OK) {
					/* 'STM32.TXT' file Open for write Error */
					//Error_Handler();
				} else {
					/* Write data to the text file */
					res = f_write(&logSumFile, wtext, sizeof(wtext),
							(void *) &byteswritten);
					//HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
					if ((byteswritten == 0) || (res != FR_OK)) {
						/* 'STM32.TXT' file Write or EOF Error */
						//Error_Handler();
					} else {
						/* Close the open text file */
						f_close(&logSumFile);

					}

				}


}
