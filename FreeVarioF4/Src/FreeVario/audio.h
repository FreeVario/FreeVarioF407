/*
 * audio.h
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */

#ifndef AUDIO_H_
#define AUDIO_H_

#include "FreeRTOS.h"
#include "cmsis_os.h"
#include <stdlib.h>
#include <bmp280.h>
#include <globaldata.h>

void setupAudio();
void tone(float freq);
void noTone();

#endif /* AUDIO_H_ */
