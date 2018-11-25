/*
 * displaytask.c
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */
#include <displaytask.h>


void displayTaskSetup(Paint *paint, EPD *epd, unsigned char * frame_buffer) {

	  if (EPD_Init(epd, lut_full_update) != 0) {
	    return;
	  }


	  Paint_Init(paint, frame_buffer, epd->width, epd->height);
	  Paint_Clear(paint, UNCOLORED);
//
//	  /* For simplicity, the arguments are explicit numerical coordinates */
//	  /* Write strings to the buffer */
//	  Paint_DrawFilledRectangle(paint, 0, 10, 128, 34, COLORED);
//	  Paint_DrawStringAt(paint, 0, 14, "Hello world!", &Font16, UNCOLORED);
//	  Paint_DrawStringAt(paint, 0, 34, "e-Paper Demo", &Font16, COLORED);
//
//	  /* Draw something to the frame buffer */
//	  Paint_DrawRectangle(paint, 16, 60, 56, 110, COLORED);
//	  Paint_DrawLine(paint, 16, 60, 56, 110, COLORED);
//	  Paint_DrawLine(paint, 56, 60, 16, 110, COLORED);
//	  Paint_DrawCircle(paint, 120, 90, 30, COLORED);
//	  Paint_DrawFilledRectangle(paint, 16, 130, 56, 180, COLORED);
//	  Paint_DrawFilledCircle(paint, 120, 160, 30, COLORED);
//
//	  /* Display the frame_buffer */
//	  EPD_SetFrameMemory(epd, frame_buffer, 0, 0, Paint_GetWidth(paint), Paint_GetHeight(paint));
//	  EPD_DisplayFrame(epd);
//	  EPD_DelayMs(epd, 2000);
//
//	  /**
//	   *  there are 2 memory areas embedded in the e-paper display
//	   *  and once the display is refreshed, the memory area will be auto-toggled,
//	   *  i.e. the next action of SetFrameMemory will set the other memory area
//	   *  therefore you have to set the frame memory and refresh the display twice.
//	   */
//	  EPD_ClearFrameMemory(epd, 0xFF);
//	  EPD_DisplayFrame(epd);
//	  EPD_ClearFrameMemory(epd, 0xFF);
//	  EPD_DisplayFrame(epd);
//
//	  /* EPD_or partial update */
//	  if (EPD_Init(epd, lut_partial_update) != 0) {
//	    return;
//	  }
//
//	  /**
//	   *  there are 2 memory areas embedded in the e-paper display
//	   *  and once the display is refreshed, the memory area will be auto-toggled,
//	   *  i.e. the next action of SetFrameMemory will set the other memory area
//	   *  therefore you have to set the frame memory and refresh the display twice.
//	   */
//	  EPD_SetFrameMemory(epd, IMAGE_DATA, 0, 0, epd->width, epd->height);
//	  EPD_DisplayFrame(epd);
//	  EPD_SetFrameMemory(epd, IMAGE_DATA, 0, 0, epd->width, epd->height);
//	  EPD_DisplayFrame(epd);
//	   osDelay(1000);

		  EPD_ClearFrameMemory(epd, 0xFF);
		  EPD_DisplayFrame(epd);
		  EPD_ClearFrameMemory(epd, 0xFF);
		  EPD_DisplayFrame(epd);

		  /* EPD_or partial update */
		  if (EPD_Init(epd, lut_partial_update) != 0) {
		    return;
		  }


}

void displayTaskUpdate(Paint *paint,EPD *epd, unsigned char * frame_buffer) {
	char BmpPressure[11];
	char BmpHumid[9];
	char BmpVario[9];
	char BmpTemp[6];

	    Paint_SetWidth(paint, 124);
	    Paint_SetHeight(paint, 24);


//    fpart = sensors.temperature/100;
//    bpart = sensors.temperature % 100;
//    sprintf(BmpTemp,"%d.%02d",fpart,bpart);

	intTocharFloat(BmpVario, sensors.VarioMs,1000,10);

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpVario, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 3, 10, Paint_GetWidth(paint), Paint_GetHeight(paint));



    intTocharFloat(BmpPressure, sensors.pressure/100,100,10);

    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpPressure, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 3, 80, Paint_GetWidth(paint), Paint_GetHeight(paint));


	intTocharFloat(BmpHumid, sensors.AltitudeMeters,1000,100);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpHumid, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 3, 150, Paint_GetWidth(paint), Paint_GetHeight(paint));

    intTocharFloat(BmpTemp, sensors.temperature,100,1);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpTemp, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 3, 220, Paint_GetWidth(paint), Paint_GetHeight(paint));


   EPD_DisplayFrame(epd);


}

//char array, the value, divide by amount (1000), reduce after 0
void intTocharFloat(char *buffer, int value, uint16_t div, uint16_t dif){

	int fpart;
	int16_t bpart;



    char *tmpSign = (value < 0) ? "-" : " ";
    fpart = abs(value)/div;
    bpart = abs(value) % div;
    if (dif >0) bpart = bpart / dif;

    if ((div/dif) >= 100) {
    	sprintf(buffer,"%s%d.%02d",tmpSign,fpart,bpart);
    }else{
    	sprintf(buffer,"%s%d.%1d",tmpSign,fpart,bpart);
    }


}
