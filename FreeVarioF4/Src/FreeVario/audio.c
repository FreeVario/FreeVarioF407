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


const uint16_t sine_wave_array[32] = {2047, 1648, 1264, 910, 600,  345,
        156, 39,  0,  39,  156,  345,
        600, 910, 1264, 1648, 2048, 2447,
        2831, 3185, 3495, 3750, 3939, 4056,
        4095, 4056, 3939, 3750, 3495, 3185,
        2831, 2447};

void setupAudio(audio_t * audio) {
	 HAL_TIM_Base_Start(&FV_DACTMR);
	 HAL_DAC_Start(&FV_DAC,FV_DAC_CHANNEL);
	 HAL_DAC_Start_DMA(&FV_DAC, FV_DAC_CHANNEL, (uint32_t*)sine_wave_array, 32, DAC_ALIGN_12B_R);
	 FV_DACHALTMR->PSC = SystemCoreClock/100000000;
}




void noTone() {

	FV_DACHALTMR->CR1 &= ~TIM_CR1_CEN;

}
#define BASEPULSE 200
#define TOPPULSE  4000

 void tone(audio_t * audio, float freq, int period) {

    audio->notonetimer = period + millis();
	FV_DACHALTMR->ARR = 1/(float)freq * audio->multiplier;
	FV_DACHALTMR->CR1 |= TIM_CR1_CEN;
}

void toneconstant(audio_t * audio, float freq) {
	FV_DACHALTMR->ARR = 1/(float)freq * audio->multiplier;
	FV_DACHALTMR->CR1 |= TIM_CR1_CEN;
}

//changes tone while beeping
 void dynaTone(audio_t * audio, float freq) {
	FV_DACHALTMR->ARR = 1/(float)freq * audio->multiplier;
}


 int millis() {

	//return HAL_GetTick();
	return xTaskGetTickCount();
}



 void noToneTimer(audio_t * audio) {
   if(millis() >= audio->notonetimer && audio->notonetimer > 0) {
     noTone();
     audio->notonetimer = 0;
   }

}


// Non-Blocking beep blob beep
 void playTwoToneInterval(audio_t * audio, int freq,int freq2, int period, int intervald) {


  if (audio->toneOn) {
    int wait = period + audio->tm;


    if ( wait < millis()) {
    	audio->toneOn = false;
      //noTone();
      toneconstant(audio, freq2);
      audio->rm = millis();
    }

  } else {
    int ndwait = intervald + audio->rm;

    if(ndwait < millis()) {

    toneconstant(audio, freq);
    audio->toneOn = true;
    audio->tm = millis();
    }
  }

}


// Non-Blocking beep beep beep
 void playToneInterval(audio_t * audio, int freq, int period, int tinterval) {

  if (audio->toneOn) {
    int wait = period + tinterval + audio->tm;

    if ( wait < millis()) {
    	audio->toneOn = false;
      noTone();
      audio->tcount++; // count the amount of beeps for playTonePause
      if (audio->tcount > 1000) { // prevent overflow
    	  audio->tcount = 0;
      }
    }

  } else {
	  tone(audio, freq, period);
	  audio->toneOn = true;
	  audio->tm = millis();
  }

}


// Plays nbeeps then pause

 void playTonePause(audio_t * audio, int freq, int nbeeps, int tpause) {

   if (audio->pause < millis()) {

      if (audio->tcount < nbeeps) {
        playToneInterval(audio,freq, 500, 200);

      }else{
    	  audio->pause = millis() + tpause;
    	  audio->tcount=0;

      }


   }


}

//main task to poll
void makeVarioAudio(audio_t * audio, float vario) {
  int pulse;
  float varioorg = vario;
   noToneTimer(audio);


  if (vario > 20) {
    vario = 20;

  }

  if (vario < -20) {
    vario = -20;

  }

#if defined(SOARDETECTION) && !defined(TESTBUZZER)
  float variofr;

  if (varioorg > -0.2 && varioorg < 0.2) { //TODO: add to conf
    int diffe = millis() - audio->stime;
    if (diffe >  (int)(conf.SoarDeadBandTime)) {
    	audio->muted = true;
    }
  } else {
	  audio->stime = millis();
	  audio->muted = false;
  }

#endif

  variofr = ((float)(fabs(vario + 1)) * 50 ) + FV_TONEBASE;

  audio->variof = (AUDIOSMOOTH * audio->variof + variofr )/(AUDIOSMOOTH + 1);

    if (vario <= BUZZERCLIMBING && vario >= BUZZERZEROCLIMB) { // prethermal audio bip bip bip

      if (!audio->muted) {
         playToneInterval(audio, audio->variof, 100, 400);
      }

    }

   if (vario <= (double)(conf.sinkAlarmLevel)/1000 ) { //sink alarm
      if (!audio->muted) {
         playTwoToneInterval(audio,1400, 1800, 70, 70);
      }

   }

#if defined(BUZZSINKALERT) //sink alert beh beh beh (-3)
    if (vario <=  BUZZSINKALERT && vario > (double)(conf.sinkAlarmLevel)/1000 ) {
       playTonePause(audio, 300, fabs(vario), BUZZSINKALERTPAUSE);
    }

#endif


  if (vario > BUZZERCLIMBING) {
	  pulse = TOPPULSE / (vario * 10) + 100;

    if (!audio->muted) {
      dynaTone(audio, audio->variof);
      playToneInterval(audio, audio->variof, pulse, pulse / 2);
    }
   // climbing = true;
  }



  //else {
  //  if (climbing ) { //dropped out of the thermal
  //    tone( 100, OUTOFTHERMALBUZZT);
  //    climbing = false;
  //  }
  //}

}

void testtone(audio_t * audio, float freq) {

	FV_DACHALTMR->ARR = 1/(float)freq * audio->multiplier;
	FV_DACHALTMR->CR1 |= TIM_CR1_CEN;

}
