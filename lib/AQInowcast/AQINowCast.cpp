#include "Arduino.h"
#include "AQINowCast.h"
AQINowCast::AQINowCast()
{

}
int AQINowCast::calcNowCastAQI(byte AQIType, float conc) {
  const int AQI_LO[] = {0, 51, 101, 151, 201, 301};
  const int AQI_HI[] = {50, 100, 150, 200, 300, 500};
  // concentration for each Ozone AQI level, in ppm
  float OZONE_CONC_LO[] = {0.0, .055, .071, .086, .106, .201};
  float OZONE_CONC_HI[] = {.054, .070, .085, .105, .200, .300};
  // concentration for each pm2.5 AQI level, in ugm/m^3
  float PM2_5_CONC_LO[] = {0.0, 12.1, 35.5, 55.5, 150.5, 250.5};
  float PM2_5_CONC_HI[] = {12.0, 35.4, 55.4, 150.4, 250.4, 500.4};
  // concentration for each pm10 AQI level, in ugm/m^3
  float PM10_CONC_LO[] = {0.0, 55.0, 155.0, 255.0, 355.0, 425.0};
  float PM10_CONC_HI[] = {54.0, 154.0, 254.0, 354.0, 424.0, 604.0};

  byte concIndex = 0;
  float conc_hi = 604.0; 
  float conc_lo = 0.0; 

  if (AQIType == AQI_TYPE_OZONE){
    for (byte i = 0; i < 6; i++){
      if (conc > OZONE_CONC_LO[i] && conc <= OZONE_CONC_HI[i]){
        concIndex  = i;
      }
    }
    conc_hi = OZONE_CONC_HI[concIndex];
    conc_lo = OZONE_CONC_LO[concIndex];
  } else if (AQIType == AQI_TYPE_PM2_5){
    for (byte i = 0; i < 6; i++){
      if (conc > PM2_5_CONC_LO[i] && conc <= PM2_5_CONC_HI[i]){
        concIndex  = i;
      }
    }
    conc_hi = PM2_5_CONC_HI[concIndex];
    conc_lo = PM2_5_CONC_LO[concIndex];
  } else if (AQIType == AQI_TYPE_PM10){
    for (byte i = 0; i < 6; i++){
      if (conc > PM10_CONC_LO[i] && conc <= PM10_CONC_HI[i]){
        concIndex  = i;
      }
    }
    conc_hi = PM10_CONC_HI[concIndex];
    conc_lo = PM10_CONC_LO[concIndex];
  } 
  int AQI_hi = AQI_HI[concIndex];
  int AQI_lo = AQI_LO[concIndex];
  return round(((conc - conc_lo) / (conc_hi - conc_lo)) * (AQI_hi - AQI_lo) + AQI_lo);
}
