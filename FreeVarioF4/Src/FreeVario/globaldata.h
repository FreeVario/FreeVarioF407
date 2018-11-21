/*
 * messageq.h
 *
 *  Created on: Oct 27, 2018
 *      Author: marco
 */

#ifndef GLOBALDATA_H_
#define GLOBALDATA_H_
#include "../fvconfig.h"

 typedef struct {
	int16_t temperature;  // C x100
	uint32_t pressure;    //Pa x100
	int16_t humidity;     //% x100
	int16_t accel_x;   //x1000
	int16_t accel_y;   //x1000
	int16_t accel_z;   //x1000
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
	int32_t AltitudeMeters; //x1000
	int32_t VarioMs; //x1000
	Queue_t  QAltitudeMeters;

}SensorData;

extern settings_t conf; //declared at fvconf.h. So yes, you must include both files

#endif /* GLOBALDATA_H_ */
