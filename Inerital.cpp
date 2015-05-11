#include "Inertial.h"
#include "Calibration.h"
#include "Attitude.h"
#include "GPS.h"
#include "Sensors.h"
#include <Streaming.h>

float inertialX,inertialY,inertialZ;
float velX,velY,velZ,velZUp;
float XEst,YEst,ZEst,ZEstUp;
float gpsX,gpsY,baroZ,baroVel;
float gpsVelX,gpsVelY;
float accelBiasX,accelBiasY,accelBiasZ;
float distToCraft,headingToCraft;

//-------------------
float inertialZGrav;
int16_t currentEstIndex,lagIndex,currentEstIndex_z,lagIndex_z;  
float XEstHist[LAG_SIZE],YEstHist[LAG_SIZE],ZEstHist[LAG_SIZE_BARO];
float XVelHist[LAG_SIZE],YVelHist[LAG_SIZE],ZVelHist[LAG_SIZE_BARO];



void GetInertial(){

  inertialX = ((R11 * (filtAccX)) + (R21 * (filtAccY))) + (R31 * (filtAccZ));
  inertialY = R12 * filtAccX + R22 * filtAccY + R32 * filtAccZ;
  inertialZGrav = R13 * filtAccX + R23 * filtAccY + R33 * filtAccZ;
  inertialZ = inertialZGrav + initialAccMagnitude;

}

void InertialInit(){

  memset(XEstHist,0,sizeof(XEstHist));
  memset(YEstHist,0,sizeof(YEstHist));
  memset(ZEstHist,0,sizeof(ZEstHist));

  memset(XVelHist,0,sizeof(XVelHist));
  memset(YVelHist,0,sizeof(YVelHist));
  memset(ZVelHist,0,sizeof(ZVelHist));

  velX = 0;
  velY = 0;
  velZ = 0;


  XEst = 0;
  YEst = 0;
  ZEst = 0;

}

void Predict(float dt){

  float biasedX,biasedY,biasedZ;
  float accelBiasX,accelBiasY,accelBiasZ;
  float inertialXBiased,inertialYBiased,inertialZBiased;

  biasedX = (scaledAccX - accelBiasX);
  biasedY = (scaledAccY - accelBiasY);
  biasedZ = (scaledAccZ - accelBiasZ);

  inertialXBiased = R11 * biasedX + R21 * biasedY + R31 * biasedZ;
  inertialYBiased = R12 * biasedX + R22 * biasedY + R32 * biasedZ;
  inertialZBiased = R13 * biasedX + R23 * biasedY + R33 * biasedZ + initialAccMagnitude;



  velX = velX + inertialXBiased * dt;
  velY = velY + inertialYBiased * dt;
  velZ = velZ + inertialZBiased * dt;


  XEst = XEst + velX * dt;
  YEst = YEst + velY * dt;
  ZEst = ZEst + velZ * dt;



  XEstHist[currentEstIndex] = XEst;
  YEstHist[currentEstIndex] = YEst;


  XVelHist[currentEstIndex] = velX;
  YVelHist[currentEstIndex] = velY;

  ZEstHist[currentEstIndex_z] = ZEst;
  ZVelHist[currentEstIndex_z] = velZ;

  ZEstUp = -1.0 * ZEst;
  velZUp = -1.0 * velZ;


}

void GetGPSXY(){
  DistBearing(&homeLat, &homeLon, &GPSData.vars.lat, &GPSData.vars.lon, &gpsX, &gpsY, &distToCraft, &headingToCraft);

  gpsVelX = velN;
  gpsVelY = velE;

}


void GetBaroZ(){
  static uint32_t baroTimer = 0;
  static float prevBaro = 0;

  float baroDT;
  float baroAlt,baroRate;

  baroDT = (millis() - baroTimer) * 0.001;
  baroTimer = millis();

  if (baroDT >= 0.1 || baroDT < 0) {
    baroDT = 0.1;
  }
  GetAltitude(&pressure, &initialPressure, &baroAlt);
  LPF(&baroZ,&baroAlt,&baroDT,RC_CONST_BARO);
  baroRate = (baroZ - prevBaro) / baroDT;
  LPF(&baroVel,&baroRate,&baroDT,RC_CONST_BARO);
  prevBaro = baroZ;


}

void CorrectXY(){
  float xPosError,yPosError,xVelError,yVelError;
  float accelBiasXEF,accelBiasYEF,accelBiasZEF;

  GetGPSXY();

  xPosError = XEstHist[lagIndex] - gpsX;
  yPosError = YEstHist[lagIndex] - gpsY;

  xVelError = XVelHist[lagIndex] - gpsVelX;
  yVelError = YVelHist[lagIndex] - gpsVelY;

  XEst = XEst - K_P_GPS * xPosError;
  YEst = YEst - K_P_GPS * yPosError;

  velX = velX - K_V_GPS * xVelError;
  velY = velY - K_V_GPS * yVelError;

  accelBiasXEF = R11 * accelBiasX + R21 * accelBiasY + R31 * accelBiasZ;
  accelBiasYEF = R12 * accelBiasX + R22 * accelBiasY + R32 * accelBiasZ;
  accelBiasZEF = R13 * accelBiasX + R23 * accelBiasY + R33 * accelBiasZ;


  accelBiasXEF = accelBiasXEF + K_B_GPS * xVelError;
  accelBiasYEF = accelBiasYEF + K_B_GPS * yVelError;

  accelBiasX = R11*accelBiasXEF + R12*accelBiasYEF + R13*accelBiasZEF;
  accelBiasY = R21*accelBiasXEF + R22*accelBiasYEF + R23*accelBiasZEF;
  accelBiasZ = R31*accelBiasXEF + R32*accelBiasYEF + R33*accelBiasZEF;

}

void CorrectZ(){
  GetBaroZ();
  float zPosError,zVelError;
  float accelBiasXEF,accelBiasYEF,accelBiasZEF;
  
  zPosError = ZEstHist[lagIndex_z] + baroZ;
  zVelError = ZVelHist[lagIndex_z] + baroVel;

  ZEst = ZEst - K_P_BARO * zPosError;
  velZ = velZ - K_V_BARO * zVelError;

  accelBiasXEF = R11*accelBiasX + R21*accelBiasY + R31*accelBiasZ;
  accelBiasYEF = R12*accelBiasX + R22*accelBiasY + R32*accelBiasZ;
  accelBiasZEF = R13*accelBiasX + R23*accelBiasY + R33*accelBiasZ;


  accelBiasZEF = accelBiasZEF + K_B_BARO * zVelError;

  accelBiasX = R11*accelBiasXEF + R12*accelBiasYEF + R13*accelBiasZEF;
  accelBiasY = R21*accelBiasXEF + R22*accelBiasYEF + R23*accelBiasZEF;
  accelBiasZ = R31*accelBiasXEF + R32*accelBiasYEF + R33*accelBiasZEF;

  ZEstUp = -1.0 * ZEst;
  velZUp = -1.0 * velZ;
}


void UpdateLagIndex(){


  currentEstIndex++;
  if (currentEstIndex >= (LAG_SIZE) || currentEstIndex < 0){
    currentEstIndex = 0;
  }

  lagIndex = currentEstIndex - (LAG_SIZE - 1);


  if (lagIndex < 0){
    lagIndex = LAG_SIZE + lagIndex;
  }


  //0.3sec lag
  currentEstIndex_z++;
  if (currentEstIndex_z >= (LAG_SIZE_BARO) || currentEstIndex_z < 0){
    currentEstIndex_z = 0;
  }
  lagIndex_z = currentEstIndex_z - (LAG_SIZE_BARO- 1);

  if (lagIndex_z < 0){
    lagIndex_z = LAG_SIZE_BARO + lagIndex_z;
  }
}










