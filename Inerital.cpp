#include "Inertial.h"
#include "Calibration.h"
#include "Attitude.h"
#include "GPS.h"
#include "Sensors.h"
#include "Streaming_.h"
#include "FlightControl.h"

float inertialX,inertialY,inertialZ;
float velX,velY,velZ,velZUp;
float XEst,YEst,ZEst,ZEstUp;
float gpsX,gpsY,baroZ,baroVel;
float gpsVelX,gpsVelY;
float accelBiasX,accelBiasY,accelBiasZ;
float distToCraft,headingToCraft;
float prevBaro;
float inertialXBiased,inertialYBiased,inertialZBiased;

float xPosError,yPosError,xVelError,yVelError;
float baroAlt,baroRate;
//-------------------
float inertialZGrav;
int16_t currentEstIndex,lagIndex,currentEstIndex_z,lagIndex_z;  
float XEstHist[LAG_SIZE],YEstHist[LAG_SIZE],ZEstHist[LAG_SIZE_BARO];
float XVelHist[LAG_SIZE],YVelHist[LAG_SIZE],ZVelHist[LAG_SIZE_BARO];

float kPosGPS,kVelGPS,kBiasGPS,kPosBaro,kVelBaro,kBiasBaro;


void GetInertial(){

  inertialX = ((R11_ * (filtAccX)) + (R21_ * (filtAccY))) + (R31_ * (filtAccZ));
  inertialY = R12_ * filtAccX + R22_ * filtAccY + R32_ * filtAccZ;
  inertialZGrav = R13_ * filtAccX + R23_ * filtAccY + R33_ * filtAccZ;
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
  accelBiasX = 0;
  accelBiasY = 0;
  accelBiasZ = 0;


}

void Predict(float dt){

  float biasedX,biasedY,biasedZ;
  //float accelBiasX,accelBiasY,accelBiasZ;
  //float inertialXBiased,inertialYBiased,inertialZBiased;

  biasedX = (filtAccX - accelBiasX);
  biasedY = (filtAccY - accelBiasY);
  biasedZ = (filtAccZ - accelBiasZ);

  inertialXBiased = R11_ * biasedX + R21_ * biasedY + R31_ * biasedZ;
  inertialYBiased = R12_ * biasedX + R22_ * biasedY + R32_ * biasedZ;
  inertialZBiased = R13_ * biasedX + R23_ * biasedY + R33_ * biasedZ + initialAccMagnitude;



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
  
  float baroDT;
  //float baroAlt,baroRate;

  baroDT = (millis() - baroTimer) * 0.001;
  baroTimer = millis();

  if (baroDT >= 0.1 || baroDT <= 0) {
    baroDT = 0.1;
  }
  GetAltitude(&pressure, &initialPressure, &baroAlt);
  LPF(&baroZ,&baroAlt,&baroDT,RC_CONST_BARO);
  baroRate = (baroZ - prevBaro) / baroDT;
  LPF(&baroVel,&baroRate,&baroDT,RC_CONST_BARO);
  prevBaro = baroZ;


}

void CorrectXY(){
  //float xPosError,yPosError,xVelError,yVelError;
  float accelBiasXEF,accelBiasYEF,accelBiasZEF;

  GetGPSXY();

  xPosError = XEstHist[lagIndex] - gpsX;
  yPosError = YEstHist[lagIndex] - gpsY;

  xVelError = XVelHist[lagIndex] - gpsVelX;
  yVelError = YVelHist[lagIndex] - gpsVelY;

  XEst = XEst - kPosGPS * xPosError;
  YEst = YEst - kPosGPS * yPosError;

  velX = velX - kVelGPS * xVelError;
  velY = velY - kVelGPS * yVelError;

  accelBiasXEF = R11_ * accelBiasX + R21_ * accelBiasY + R31_ * accelBiasZ;
  accelBiasYEF = R12_ * accelBiasX + R22_ * accelBiasY + R32_ * accelBiasZ;
  accelBiasZEF = R13_ * accelBiasX + R23_ * accelBiasY + R33_ * accelBiasZ;


  accelBiasXEF = accelBiasXEF + kBiasGPS * xVelError;
  accelBiasYEF = accelBiasYEF + kBiasGPS * yVelError;

  accelBiasX = R11_*accelBiasXEF + R12_*accelBiasYEF + R13_*accelBiasZEF;
  accelBiasY = R21_*accelBiasXEF + R22_*accelBiasYEF + R23_*accelBiasZEF;
  accelBiasZ = R31_*accelBiasXEF + R32_*accelBiasYEF + R33_*accelBiasZEF;

}

void CorrectZ(){
  
  float zPosError,zVelError;
  float accelBiasXEF,accelBiasYEF,accelBiasZEF;
   
  GetBaroZ();
 
  zPosError = ZEstHist[lagIndex_z] + baroZ;
  zVelError = ZVelHist[lagIndex_z] + baroVel;

  ZEst = ZEst - kPosBaro * zPosError;
  velZ = velZ - kVelBaro * zVelError;

  accelBiasXEF = R11_*accelBiasX + R21_*accelBiasY + R31_*accelBiasZ;
  accelBiasYEF = R12_*accelBiasX + R22_*accelBiasY + R32_*accelBiasZ;
  accelBiasZEF = R13_*accelBiasX + R23_*accelBiasY + R33_*accelBiasZ;


  accelBiasZEF = accelBiasZEF + kBiasBaro * zVelError;

  accelBiasX = R11_*accelBiasXEF + R12_*accelBiasYEF + R13_*accelBiasZEF;
  accelBiasY = R21_*accelBiasXEF + R22_*accelBiasYEF + R23_*accelBiasZEF;
  accelBiasZ = R31_*accelBiasXEF + R32_*accelBiasYEF + R33_*accelBiasZEF;

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










