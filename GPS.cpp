#include "GPS.h"
#include "LED.h"
#include "Streaming_.h"
#include "Enums.h"
#include "FlightControl.h"
#include "Comm.h"
#include "Radio.h"

boolean newGPSData,GPSDetected;
int32_t homeLat,homeLon;

boolean LLHFlag,VELFlag;
boolean gpsFailSafe;
uint8_t GPSState;
float deltaLon;
float deltaLat;
volatile uint32_t GPSFailSafeCounter;

GPS_Union_t GPSData;

uint16_t msgLength;
uint8_t index,msgLengthLSB,msgLengthMSB,msgType,inBuffer[50],localSumA,localSumB;

float gpsAlt;

float floatLat, floatLon;
float homeLatFloat,homeLonFloat;
float velN, velE, velD;
float hAcc,sAcc;

uint8_t gpsStartState = 0;

void GPSInit(){
  GPSSerialBegin(38400);
  GPSState=0;
  newGPSData = false;
}

void DistBearing(int32_t *lat1, int32_t *lon1, int32_t *lat2, int32_t *lon2,float *distX,float *distY,float *distDirect,float *bearing){
  //using euqirectangular projection since the distances are << than the radius of the earth
  deltaLat = ToRad( (*lat2 - * lat1) *0.0000001 );
  //the below line is as such to get the signs between the accelerometer and GPS position to the same sign convention
  //this will work for the north west heimsphere
  deltaLon = (ToRad( (*lon2 - * lon1) *0.0000001 ) ) * cos( ToRad((*lat2 * 0.0000001)) );
  *distX = deltaLat * RADIUS_EARTH;
  *distY = deltaLon * RADIUS_EARTH;
  *distDirect = sqrt(*distX * *distX + *distY * *distY);
  *bearing = ToDeg(atan2(*distY,*distX));
  if (*bearing < 360){
    *bearing += 360;
  }
  if (*bearing > 360){
    *bearing -= 360;
  }
}

void GPSStart() {

  uint32_t gpsStartTimer = 0;

  initProgress = 3; 
  boolean gpsStartComplete = false;

  LEDPatternSet(0,1,2,0);
  GPSInit();

  gpsStartTimer = millis();

  while ((millis() - gpsStartTimer < 1000) && (newGPSData== false)) {
    Radio();
    TuningTransmitter(); 
    GPSMonitor();
    if (newGPSData == true) {
      GPSDetected = true;
    }
  }
  //to do add feed back with leds
  if (GPSDetected == true) {

    while(gpsStartComplete == false){
      switch (gpsStartState){
      case GPS_START_FIX:
        LEDPatternSet(0,1,3,0);
        while (GPSData.vars.gpsFix != 0x3) {
          Radio();
          TuningTransmitter(); 
          GPSMonitor();
        }
        gpsStartState = GPS_START_H_ACC;
        break;
      case GPS_START_H_ACC:
        LEDPatternSet(0,1,4,0);
        while (GPSData.vars.hAcc * 0.001 > HACC_MAX ) {
          GPSMonitor();
          Radio();
          TuningTransmitter(); 
        }
        gpsStartState = GPS_START_S_ACC;
        break;
      case GPS_START_S_ACC:
        LEDPatternSet(0,1,5,0);
        while (GPSData.vars.sAcc * 0.001 > SACC_MAX  ) {
          GPSMonitor();
          Radio();
          TuningTransmitter(); 
        }
        gpsStartState = GPS_START_WAIT;
        break;
      case GPS_START_WAIT:
        LEDPatternSet(0,1,6,0);
        gpsStartTimer = millis();
        while(millis() - gpsStartTimer < 30000){
          GPSMonitor();
          Radio();
          TuningTransmitter(); 
          if (GPSData.vars.sAcc * 0.001 > SACC_MAX || GPSData.vars.hAcc * 0.001 > HACC_MAX || GPSData.vars.gpsFix != 0x3){
            gpsStartState = GPS_START_FIX;
            break;
          }
        }
        gpsStartComplete = true;
        gpsFailSafe = false;
        break;
      }
    }
    newGPSData = false;
    while (newGPSData == false) {
      GPSMonitor();
      Radio();
      TuningTransmitter(); 
    }
    homeLat = GPSData.vars.lat;
    homeLon = GPSData.vars.lon;
    homeLatFloat = homeLat * 0.0000001;
    homeLonFloat = homeLon * 0.0000001;
  }

}


void GPSMonitor(){
  uint8_t inByte;
  while (GPSSerialAvailable() > 0){
    switch (GPSState){
    case 0:
      inByte = GPSSerialRead();
      if (inByte == 0xB5){
        GPSState = 1;
      }
      break;
    case 1:
      inByte = GPSSerialRead();
      if (inByte == 0x62){
        GPSState = 2;
      }
      else{
        GPSState = 0;
      }
      break;
    case 2:
      inByte = GPSSerialRead();
      localSumB = localSumA = inByte;
      if (inByte == 0x01){
        GPSState = 3;
      }
      else{
        GPSState = 0;
      }
      break;
    case 3://get message type
      inByte = GPSSerialRead();
      localSumB += (localSumA += inByte);
      msgType = inByte;
      GPSState = 4;
      break;
    case 4://get number of bytes in message LSB
      inByte = GPSSerialRead();
      localSumB += (localSumA += inByte);
      msgLengthLSB = inByte;
      index = 0;
      GPSState = 5;
      break;
    case 5://get number of bytes in message MSB
      inByte = GPSSerialRead();
      localSumB += (localSumA += inByte);
      msgLengthMSB = inByte;
      msgLength = (msgLengthMSB << 8) | msgLengthLSB;
      if (msgLength > 40){
        GPSState = 0;
      }
      index = 0;
      GPSState = 6;
      break;
    case 6://buffer in data
      inByte = GPSSerialRead();
      localSumB += (localSumA += inByte);
      inBuffer[index++] = inByte;
      if (index >=49){
        GPSState = 0;
      }
      if(index == msgLength){
        GPSState = 7;
      }
      break;
    case 7://get first sum and check
      inByte = GPSSerialRead();
      if (inByte == localSumA){
        GPSState = 8;
      }
      else{
        GPSState = 0;
      }
      break;
    case 8://get second sum and check
      inByte = GPSSerialRead();
      if (inByte == localSumB){
        GPSState = 9;
      }
      else{
        GPSState = 0;
      }
      break;
    case 9://copy the bytes to the union
      switch(msgType){
      case 0x02:
        memcpy(&GPSData.buffer[0],&inBuffer[0],msgLength);//LLH
        LLHFlag = true;
        break;
      case 0x12:
        memcpy(&GPSData.buffer[28],&inBuffer[4],msgLength-4);//VELNED
        VELFlag = true;
        break;
      case 0x03:
        GPSData.vars.gpsFix = inBuffer[4];//status
        break;
      default:
        GPSState = 0;
        break;
      }
      GPSState = 0;
      break;

    }
  }
  if (LLHFlag == true && VELFlag == true){
    LLHFlag = false;
    VELFlag = false;
    newGPSData = true;
    floatLat = GPSData.vars.lat * 0.0000001;
    floatLon = GPSData.vars.lon * 0.0000001;
    gpsAlt = GPSData.vars.height * 0.001;
    velN = GPSData.vars.velN * 0.01;
    velE = GPSData.vars.velE * 0.01;
    velD = GPSData.vars.velD * 0.01;
    hAcc = GPSData.vars.hAcc * 0.001;
    sAcc = GPSData.vars.sAcc * 0.001;
    if (GPSData.vars.gpsFix != 3 || hAcc > HACC_MAX || sAcc > SACC_MAX) {//5
      gpsFailSafe = true;
    }
    GPSFailSafeCounter = 0;
  }
}






















