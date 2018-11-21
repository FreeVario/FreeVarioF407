/*
 * audio.c
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */


#include "audio.h"


extern DAC_HandleTypeDef hdac;
extern TIM_HandleTypeDef htim6;

#define FV_DACTMR htim6
#define FV_DACHALTMR TIM6
#define FV_DAC hdac
#define FV_DAC_CHANNEL DAC_CHANNEL_1
#define DACTMRMULTIPLIER 1000000

const uint16_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,
        156, 39,  0,  39,  156,  345,
        600, 910, 1264, 1648, 2048, 2447,
        2831, 3185, 3495, 3750, 3939, 4056,
        4095, 4056, 3939, 3750, 3495, 3185,
        2831, 2447};

void setupAudio() {
	 HAL_TIM_Base_Start(&FV_DACTMR);
	 HAL_DAC_Start(&FV_DAC,FV_DAC_CHANNEL);
	 HAL_DAC_Start_DMA(&FV_DAC, FV_DAC_CHANNEL, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);
	 FV_DACHALTMR->PSC = SystemCoreClock/100000000;
}

void tone(float freq) {

	FV_DACHALTMR->ARR = 1/(float)freq * DACTMRMULTIPLIER;
	FV_DACHALTMR->CR1 |= TIM_CR1_CEN;

}


void noTone() {

	FV_DACHALTMR->CR1 &= ~TIM_CR1_CEN;

}
