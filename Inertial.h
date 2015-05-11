#ifndef INERTIAL_H
#define INERTIAL_H

#define LAG_SIZE 56
#define LAG_SIZE_BARO 27

#define FC_BARO 3.0
#define RC_CONST_BARO 1/(2.0 * 3.14 * FC_BARO)

#define K_P_GPS 0.1
#define K_V_GPS 0.2
#define K_B_GPS 0.03
#define K_P_BARO 0.07
#define K_V_BARO 0.1
#define K_B_BARO 0.01

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
extern float distToCraft,headingToCraft;
extern float gpsX,gpsY,baroZ,baroVel;

#endif
