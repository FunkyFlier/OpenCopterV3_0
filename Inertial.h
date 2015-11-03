#ifndef INERTIAL_H
#define INERTIAL_H

#include <Arduino.h>
#include "Definitions.h"
#define LAG_SIZE 32
#ifdef RAW_PRES_FILT
#define LAG_SIZE_BARO 14  
#else
#define LAG_SIZE_BARO 14
#endif  




void GetInertial();
void Predict(float);
void InertialInit();
void CorrectXY();
void UpdateLagIndex();
void GetBaroZ();
void CorrectZ();

extern float inertialX,inertialY,inertialZ;
extern float velX,velY,velZ,velZUp,XEst,YEst,ZEst,ZEstUp;
extern float accelBiasX,accelBiasY,accelBiasZ;
extern float inertialXBiased,inertialYBiased,inertialZBiased;
extern float distToCraft,headingToCraft;
extern float gpsX,gpsY,baroZ,baroVel;
extern float prevBaro;
extern float xPosError,yPosError,xVelError,yVelError;
extern float baroAlt,baroRate;

extern float kPosGPS,kVelGPS,kBiasGPS,kPosBaro,kVelBaro,kBiasBaro;
extern float zPosError,zVelError;
extern float xPosOutput,yPosOutput,zPosOutput,xVelOutput,yVelOutput,zVelOutput;
extern float kPosBiasBaro,kPosVelBaro;

extern boolean baroGlitchHandling;
extern uint32_t takeOffBaroGlitchTimer;
extern float baroDT;
#endif

