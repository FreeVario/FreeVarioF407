/*
 * displaytask.h
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */

#ifndef DISPLAYTASK_H_
#define DISPLAYTASK_H_

#define COLORED      0
#define UNCOLORED    1

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "epd1in54.h"
#include "epdif.h"
#include "epdpaint.h"
#include "imagedata.h"
#include <stdlib.h>
#include <globaldata.h>

extern SensorData sensors;

//TODO: Fix compiler warning
void intTocharFloat(char *buffer, int value, uint16_t div);
void displayTaskSetup(Paint *paint,EPD *epd, unsigned char *frame_buffer);
void displayTaskUpdate( Paint *paint,EPD *epd,unsigned char *frame_buffer);


#endif /* DISPLAYTASK_H_ */