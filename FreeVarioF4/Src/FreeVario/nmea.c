/*
 FreeVario http://FreeVario.org

  Copyright (c), PrimalCode (http://www.primalcode.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/

#include "nmea.h"
#include <string.h>
#include <stdio.h>

const char *hex = "0123456789ABCDEF";
char t_check[3];
char t_newline[3]="\r\n";

//NOTE: add -u _printf_float to C linker for Sprintf to work with floats
// See: http://www.openstm32.org/forumthread954


void NMEA_setPTAS1(double vario, double varioAv, double altitude) {
	/*
	   * $PTAS1,xxx,yyy,zzzzz,aaa*CS<CR><LF>
	   *
	   * xxx
	   * CV or current vario. =vario*10+200 range 0-400(display +/-20.0 knots)
	   *
	   * yyy
	   * AV or average vario. =vario*10+200 range 0-400(display +/-20.0 knots)
	   *
	   * zzzzz
	   * Barometric altitude in feet +2000
	   *
	   * aaa
	   * TAS knots 0-200
	   */

  vario = (vario * 3.28084 * 10) + 200;
  varioAv = varioAv * 3.28084 * 10 + 200;
  altitude = altitude * 3.28084 + 2000;

  sprintf(nmeaPTAS1,"$PTAS1,%d,%d,%d,*",(int)vario,(int)varioAv,(int)altitude);

  getCRC(nmeaPTAS1);

  strcat(nmeaPTAS1, t_check);
  strncat(nmeaPTAS1, t_newline, sizeof(t_newline));

}

void NMEA_setnmeaShortLXWP0(float varioAlt, float varioMts) { //in m/s
  // short version, for high speed updates. only v1[0] given
  // $LXWP0,logger_stored, airspeed, airaltitude,
  //   v1[0],v1[1],v1[2],v1[3],v1[4],v1[5], hdg, windspeed*CS<CR><LF>
  //
  // 0 loger_stored : [Y|N] (not used in LX1600)
  // 1 IAS [km/h] ----> Condor uses TAS!
  // 2 baroaltitude [m]
  // 3-8 vario values [m/s] (last 6 measurements in last second)
  // 9 heading of plane (not used in LX1600)
  // 10 windcourse [deg] (not used in LX1600)
  // 11 windspeed [km/h] (not used in LX1600)
  //
  // e.g.:
  // $LXWP0,Y,222.3,1665.5,1.71,,,,,,239,174,10.1


  sprintf(nmeaVarioLXWP0,"$LXWP0,N,,%5.2f,%2.2f,,,,,,,,,*",varioAlt,varioMts);
  getCRC(nmeaVarioLXWP0);

  strcat(nmeaVarioLXWP0, t_check);
  strncat(nmeaVarioLXWP0, t_newline, sizeof(t_newline));

}



void NMEA_setNmeaLK8EX1(int rawPressure, int varioAlt, float climbRate, int temperature, int pbat) {
  //https://github.com/LK8000/LK8000/blob/master/Docs/LK8EX1.txt
  //LK8EX1,pressure,altitude,vario,temperature,battery,*checksum
	/*
	 * Field 0, raw pressure in hPascal:hPA*100
	 *
	 * Field 1, altitude in meters, relative to QNH 1013.25
	 * If raw pressure is available, this value will be IGNORED (you can set it to 99999)
	 *
	 * Field 2, vario in cm/s
	 *	If vario not available, send 9999  (4 times 9)
	 *
	 *	Field 3, temperature in C , can be also negative
	 *	If not available, send 99
     *
	 *	Field 4, battery charge percentage
	 *	Cannot be negative
	 *	If not available, send 999 (3 times 9)
	 *
	 */


  climbRate = climbRate * 100;

  sprintf(nmeaVarioLK8EX1,"$LK8EX1,%d,%d,%d,%d,%d,*",rawPressure,varioAlt,(int)climbRate,temperature,pbat);

  getCRC(nmeaVarioLK8EX1);
  strcat(nmeaVarioLK8EX1, t_check);
  strncat(nmeaVarioLK8EX1, t_newline, sizeof(t_newline));


}


//Using the C-probe sentence
//temp and

void NMEA_setNmeaPcProbe(float aax, float aay, float aaz, float temperature, float humidity, uint8_t batPers, uint8_t charging) {

  // $PCPROBE,T,Q0,Q1,Q2,Q3,ax,ay,az,temp,rh,batt,delta_press,abs_press,C,
  // - "T" after "$PCPROBE" indicates that the string contains data. Data are represented as signed,
  //  16-bit hexadecimal integers. The only exception is abs_press which is in signed 24-bits hex
  //  format.
  // - Q0, Q1, Q2, Q3: 3D orientation of the C-Probe in quaternion format. Heading, pitch, and roll can
  // - temp: temperature in units of 0.1°C.
  // - rh: relative humidity in units of 0.1%.
  // - batt: battery level from 0 to 100%.
  // - delta_press: differential pressure (dynamic - static) in units of 0.1 Pa.
  // - abs_press: absolute pressure in units of 1/400 Pa
  // - C: is transmitted only if the C-Probe is being charged. In this case, heat produced by the charging
  //    process is likely to affect the readings of the temperature and humidity sensors.

	uint8_t ch;

	  if (charging) {
		  ch = 12;
	  } else {
		  ch = 0;
	  }


  sprintf(nmeaPcProbe,"$PCPROBE,T,,,,,%02x,%02x,%02x,%02x,%02x,%02x,,,%1X*",(int)aax * 100,(int)aay * 100,(int)aaz * 100, (int)temperature * 10,(int)humidity * 10,(int)batPers,ch);

  getCRC(nmeaPcProbe);
  strcat(nmeaPcProbe, t_check);
  strncat(nmeaPcProbe, t_newline, sizeof(t_newline));


}


void getCRC(char *buff) {
  // NMEA CRC: XOR each byte with previous for all chars between '$' and '*'
  char c;
  uint8_t i;
  uint8_t start_with = 0;
  uint8_t end_with = 0;
  char crca = 0;

  for (i = 0; i < 128; i++) {
    c = buff[i];
    if (c == 0x24) {
      start_with = i;
    }
    if (c == 0x2a) {
      end_with = i;
      break;
    }
  }
  if (end_with > start_with) {
    for (i = start_with + 1; i < end_with; i++) { // XOR every character between '$' and '*'
      crca = crca ^ buff[i] ;  // compute crc

    }

  }

  //Single threaded, so this is allowed
  t_check[0] = hex[(crca >> 4) & 0xF];
  t_check[1] = hex[(crca) & 0xF];
  t_check[2] = '\0';

  //based on code by Elimeléc López - July-19th-2013
}

char *dtostrf (float val, signed char width, unsigned char prec, char *sout) {
  char fmt[20];
  sprintf(fmt, "%%%d.%df", width, prec);
  sprintf(sout, fmt, val);
  return sout;
}

