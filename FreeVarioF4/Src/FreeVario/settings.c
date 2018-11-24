/*
 FreeVario http://FreeVario.org

  Copyright (c), PrimalCode (http://www.primalcode.org)
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  any later version. see <http://www.gnu.org/licenses/>
*/



#include "settings.h"

void setupConfig(){
#if defined(CONFIGOPT)

#else
  getDefaultConfig();
#endif
}




void getDefaultConfig() {
  //QNH value to calculate vario Altitude
  conf.qnePressure = 101325;

  // X 1000 Level to sound sink alarm
  conf.sinkAlarmLevel = -4000;

  //send data via serial port
  conf.SerialOut = true;

  //send data via bluetooth. BT uses alot of power if not linked. Better to disable if not used.
  //BT will be available during startup, if the config menu is used, it will not be disabled.
  conf.SerialOutBT = true;

  //send data via attached ESP
  conf.SerialOutESP = false;

  // send data via USB for OTG
  conf.SerialOutUSB = true;

  // send ptas1 nmea, uses the gps channel (once every 100ms)
  conf.ptas1 = true;

  //send vario lxnav sentence
  conf.lxnav = false;

  //use the c-probe nmea sentence
  conf.pcprobe = true;

  //use custom nmea sentence
  conf.xcs = false;

  //low pass filter, the higher the number the slower the raw vario reading changes.
  conf.variosmooth = 5;

  // turn vario audio on or off
  conf.buzzer = true;

  //X 1000 Time Delay when speaker is muted during soar detection
  conf.SoarDeadBandTime = 30000;

  //X 1000 Value to detect vario movement (abs value needed)
  conf.advTriggerLevel = 200;

  // if vario level goes lower than advLowTrigger in this time, it will cause a trigger and increase conf.variosmooth.
  conf.advTriggerTime = 1000;

  // if no trigger occurs in this time frame, conf.variosmooth is reduced by 1,
  conf.advRelaxTime = 20000;

  // lowest level for conf.variosmooth
  conf.advMinSmooth = 8;

  // highest level for conf.variosmooth
  conf.advMaxSmooth = 30;

}





