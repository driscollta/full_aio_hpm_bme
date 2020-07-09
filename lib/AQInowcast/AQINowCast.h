/*
  AQINowCast.h - Library for calculating Air Quality Index
  Created by Tom Driscoll, September, 2019.
  Released into the public domain.
  based on the technical information at https://forum.airnowtech.org/t/the-aqi-equation/169
*/
#ifndef AQINowCast_h
#define AQINowCast_h

#include "Arduino.h"

class AQINowCast
{
  public:
    AQINowCast();
    int calcNowCastAQI(byte AQIType, float conc);
    const byte AQI_TYPE_OZONE = 2;
    const byte AQI_TYPE_PM2_5 = 1;
    const byte AQI_TYPE_PM10 = 0;
  private:
    byte AQIType;
    float conc;
};

#endif
