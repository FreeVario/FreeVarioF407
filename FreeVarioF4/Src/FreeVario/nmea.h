/*
 FreeVario http://FreeVario.org

  Copyright (c), PrimalCode (http://www.primalcode.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#ifndef NMEA_H_
#define NMEA_H_
#include "../fvconfig.h"

void NMEA_setPTAS1(double vario, double varioAv, double altitude);
void NMEA_setnmeaShortLXWP0(float varioAlt, float varioMts);
void NMEA_setNmeaLK8EX1(int rawPressure, int varioAlt, float climbRate, int temperature, int pbat);
void NMEA_setNmeaPcProbe(float aax, float aay, float aaz, float temperature, float humidity, uint8_t batPers, uint8_t charging);
void getCRC(char *buff);
char *dtostrf (float val, signed char width, unsigned char prec, char *sout);

char nmeaVarioLXWP0[60];
char nmeaVarioLK8EX1[44];
char nmeaPTAS1[44];
char nmeaPcProbe[60];

#endif /* NMEA_H_ */
