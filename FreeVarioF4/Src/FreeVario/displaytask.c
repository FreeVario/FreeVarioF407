/*
 * displaytask.c
 *
 *  Created on: Nov 18, 2018
 *      Author: marco
 */
#include <displaytask.h>


void displayTaskSetup(Paint *paint,EPD *epd,unsigned char * frame_buffer) {




	   if (EPD_Init(epd, lut_full_update) != 0) {
	     //printf("e-Paper init failed\n");
	     //return -1;
	   }




	   Paint_Init(paint, frame_buffer, epd->width, epd->height);
	   Paint_Clear(paint, UNCOLORED);

	   /* For simplicity, the arguments are explicit numerical coordinates */
	   /* Write strings to the buffer */
	   Paint_DrawFilledRectangle(paint, 0, 6, 200, 26, COLORED);
	   Paint_DrawStringAt(paint, 28, 10, "Hello world!", &Font16, UNCOLORED);
	   Paint_DrawStringAt(paint, 30, 30, "e-Paper Demo", &Font16, COLORED);

	   /* Draw something to the frame buffer */
	   Paint_DrawRectangle(paint, 10, 60, 50, 110, COLORED);
	   Paint_DrawLine(paint, 10, 60, 50, 110, COLORED);
	   Paint_DrawLine(paint, 50, 60, 10, 110, COLORED);
	   Paint_DrawCircle(paint, 120, 80, 30, COLORED);
	   Paint_DrawFilledRectangle(paint, 10, 130, 50, 180, COLORED);
	   Paint_DrawFilledCircle(paint, 120, 150, 30, COLORED);

	   /* Display the frame_buffer */
	   EPD_SetFrameMemory(epd, frame_buffer, 0, 0, Paint_GetWidth(paint), Paint_GetHeight(paint));

	   EPD_DisplayFrame(epd);
	   ;

	   if (EPD_Init(epd, lut_partial_update) != 0) {
	   //  printf("e-Paper init failed\n");
	    // return -1;
	   }

	   osDelay(1000);
	   /**
	     *  there are 2 memory areas embedded in the e-paper display
	     *  and once the display is refreshed, the memory area will be auto-toggled,
	     *  i.e. the next action of SetFrameMemory will set the other memory area
	     *  therefore you have to set the frame memory and refresh the display twice.
	     */
//	   EPD_SetFrameMemory(&epd, IMAGE_DATA, 0, 0, epd.width, epd.height);
//	   EPD_DisplayFrame(&epd);
//	   EPD_SetFrameMemory(&epd, IMAGE_DATA, 0, 0, epd.width, epd.height);
//	   EPD_DisplayFrame(&epd);

	   Paint_Clear(paint, UNCOLORED);
	   EPD_SetFrameMemory(epd, frame_buffer, 0, 0, Paint_GetWidth(paint), Paint_GetHeight(paint));

	   EPD_DisplayFrame(epd);

	   EPD_SetFrameMemory(epd, frame_buffer, 0, 0, Paint_GetWidth(paint), Paint_GetHeight(paint));

	   EPD_DisplayFrame(epd);






}

//char array, the value, divide by amount (1000)
void intTocharFloat(char *buffer, int value, uint16_t div){

	int fpart;
	uint16_t bpart;

    char *tmpSign = (value < 0) ? "-" : " ";
    fpart = abs(value)/div;
    bpart = abs(value) % div;
    sprintf(buffer,"%s%d.%02u",tmpSign,fpart,bpart);

}

void displayTaskUpdate(Paint *paint,EPD *epd, unsigned char * frame_buffer) {
	char BmpPressure[11];
	char BmpHumid[9];
	char BmpTemp[9];

	int fpart;
	uint16_t bpart;

	Paint_SetWidth(paint, 32);
	Paint_SetHeight(paint, 170);
	Paint_SetRotate(paint, ROTATE_270);

//    fpart = sensors.temperature/100;
//    bpart = sensors.temperature % 100;
//    sprintf(BmpTemp,"%d.%02d",fpart,bpart);

	intTocharFloat(BmpTemp, sensors.VarioMs,1000);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpTemp, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 10, 10, Paint_GetWidth(paint), Paint_GetHeight(paint));



    intTocharFloat(BmpPressure, sensors.pressure,100);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpPressure, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 80, 10, Paint_GetWidth(paint), Paint_GetHeight(paint));


	intTocharFloat(BmpHumid, sensors.AltitudeMeters,1000);
    Paint_Clear(paint, UNCOLORED);
    Paint_DrawStringAt(paint, 0, 4, BmpHumid, &Font24, COLORED);
    EPD_SetFrameMemory(epd, frame_buffer, 150, 10, Paint_GetWidth(paint), Paint_GetHeight(paint));


    EPD_DisplayFrame(epd);


}
