#ifndef INERTIAL_H
#define INERTIAL_H

#include <Arduino.h>

#define LAG_SIZE 43
#define LAG_SIZE_BARO 20


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
#endif

