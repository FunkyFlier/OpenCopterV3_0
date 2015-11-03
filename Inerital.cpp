#include "Inertial.h"
#include "Calibration.h"
#include "Attitude.h"
#include "GPS.h"
#include "Sensors.h"
#include "Streaming_.h"
#include "FlightControl.h"
#include "Motors.h"
#include "Definitions.h"
#include "Enums.h"
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
float kPosBiasBaro,kPosVelBaro;
float zPosError,zVelError;
float xPosOutput,yPosOutput,zPosOutput,xVelOutput,yVelOutput,zVelOutput;

boolean baroGlitchHandling;
uint32_t takeOffBaroGlitchTimer;
float baroDT = 0.01;

void HandleBaroFeedBack();

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

  xPosOutput = XEstHist[lagIndex];
  yPosOutput = YEstHist[lagIndex];
  xVelOutput = XVelHist[lagIndex];
  yVelOutput = YVelHist[lagIndex];

  ZEstHist[currentEstIndex_z] = ZEst;
  ZVelHist[currentEstIndex_z] = velZ;

  zPosOutput = -1.0 * ZEstHist[lagIndex_z];
  zVelOutput = -1.0 * ZVelHist[lagIndex_z];

  ZEstUp = -1.0 * ZEst;
  velZUp = -1.0 * velZ;


}

void GetGPSXY(){
  DistBearing(&homeLat, &homeLon, &GPSData.vars.lat, &GPSData.vars.lon, &gpsX, &gpsY, &distToCraft, &headingToCraft);

  gpsVelX = velN;
  gpsVelY = velE;

}


void CorrectXY(){
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
  if (accelBiasX > BIAS_MAX){
    accelBiasX = BIAS_MAX;
  }
  if (accelBiasX < BIAS_MIN){
    accelBiasX = BIAS_MIN;
  }
  if (accelBiasY > BIAS_MAX){
    accelBiasY = BIAS_MAX;
  }
  if (accelBiasY < BIAS_MIN){
    accelBiasY = BIAS_MIN;
  }
  if (accelBiasZ > BIAS_MAX){
    accelBiasZ = BIAS_MAX;
  }
  if (accelBiasZ < BIAS_MIN){
    accelBiasZ = BIAS_MIN;
  }
}

void GetBaroZ(){
  static uint32_t baroTimer = 0;
  static float filteredPressure = 0;
  

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

void CorrectZ(){
  static float pressurePrevious;
  static uint8_t errorCorrectCount;

  GetBaroZ();

  zPosError = ZEstHist[lagIndex_z] + baroZ;
  zVelError = ZVelHist[lagIndex_z] + baroVel;

  if (baroGlitchHandling == true || motorState == LAND){
    if (millis() - takeOffBaroGlitchTimer > BARO_GLITCH_TIME){
      baroGlitchHandling = false;
    }
    if (fabs(zPosError) < BARO_ERR_LIMIT  || errorCorrectCount > BARO_ERR_COUNTS_OFF){
      if(errorCorrectCount >= BARO_ERR_COUNTS_OFF){
        errorCorrectCount++;
        if (errorCorrectCount > BARO_ERR_COUNTS_ON){
          errorCorrectCount = 0;
          initialPressure = takeOffPressure;
          GetAltitude(&pressure, &initialPressure, &baroAlt);
          baroZ = baroAlt;
          prevBaro = baroZ;
          ZEstUp = baroZ;
          ZEst = -1.0 * ZEstUp;
          return;
        }
      }
      if (errorCorrectCount < BARO_ERR_COUNTS_OFF && errorCorrectCount > 0){
        errorCorrectCount = 0;
        initialPressure = takeOffPressure;
        GetAltitude(&pressure, &initialPressure, &baroAlt);
        baroZ = baroAlt;
        prevBaro = baroZ;
        ZEstUp = baroZ;
        ZEst = -1.0 * ZEstUp;

        return;
      }
      HandleBaroFeedBack();
    }
    else{
      errorCorrectCount++;
      initialPressure += pressure - pressurePrevious;
      GetAltitude(&pressure, &initialPressure, &baroAlt);
      baroZ = baroAlt;
      prevBaro = baroZ;
      ZEstUp = baroZ;
      ZEst = -1.0 * ZEstUp;
    }
  }
  else{
    initialPressure = takeOffPressure;
    HandleBaroFeedBack();
  }

  pressurePrevious = pressure;
}
void HandleBaroFeedBack(){
  float accelBiasXEF = 0,accelBiasYEF = 0,accelBiasZEF = 0;
  accelBiasXEF = R11_*accelBiasX + R21_*accelBiasY + R31_*accelBiasZ;
  accelBiasYEF = R12_*accelBiasX + R22_*accelBiasY + R32_*accelBiasZ;
  accelBiasZEF = R13_*accelBiasX + R23_*accelBiasY + R33_*accelBiasZ;

  ZEst = ZEst - kPosBaro * zPosError;
  velZ = velZ - kVelBaro * zVelError - kPosVelBaro * zPosError;

  accelBiasZEF = accelBiasZEF + kBiasBaro * zVelError + kPosBiasBaro * zPosError;
  
/*#ifdef NEW_BARO_FEEDBACK
  ZEst = ZEst - kPosBaro * zPosError;
  velZ = velZ - kVelBaro * zVelError - kPosVelBaro * zPosError;

  accelBiasZEF = accelBiasZEF + kBiasBaro * zVelError + kPosBiasBaro * zPosError;
#else
  ZEst = ZEst - kPosBaro * zPosError;
  velZ = velZ - kVelBaro * zVelError;

  accelBiasZEF = accelBiasZEF + kBiasBaro * zVelError;
#endif*/


  accelBiasX = R11_*accelBiasXEF + R12_*accelBiasYEF + R13_*accelBiasZEF;
  accelBiasY = R21_*accelBiasXEF + R22_*accelBiasYEF + R23_*accelBiasZEF;
  accelBiasZ = R31_*accelBiasXEF + R32_*accelBiasYEF + R33_*accelBiasZEF;
  if (accelBiasX > BIAS_MAX){
    accelBiasX = BIAS_MAX;
  }
  if (accelBiasX < BIAS_MIN){
    accelBiasX = BIAS_MIN;
  }
  if (accelBiasY > BIAS_MAX){
    accelBiasY = BIAS_MAX;
  }
  if (accelBiasY < BIAS_MIN){
    accelBiasY = BIAS_MIN;
  }
  if (accelBiasZ > BIAS_MAX){
    accelBiasZ = BIAS_MAX;
  }
  if (accelBiasZ < BIAS_MIN){
    accelBiasZ = BIAS_MIN;
  }
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


  currentEstIndex_z++;
  if (currentEstIndex_z >= (LAG_SIZE_BARO) || currentEstIndex_z < 0){
    currentEstIndex_z = 0;
  }
  lagIndex_z = currentEstIndex_z - (LAG_SIZE_BARO- 1);

  if (lagIndex_z < 0){
    lagIndex_z = LAG_SIZE_BARO + lagIndex_z;
  }
}


































