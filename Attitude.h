#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "Calibration.h"
#include "Types.h"
#include "Definitions.h"
#include "Math.h"
#include "Sensors.h"
#include <Arduino.h> 


#define SAMPLES_FOR_ACC_MAGNITUDE 200

void AHRSupdate(float);
void SetInitialQuaternion();
void GenerateRotationMatrix();

void GetEuler();

extern float declination;
extern float cosDec,sinDec;
extern float rawPitch,rawRoll,pitchOffset,rollOffset;
extern float yawInDegrees,pitchInDegrees,rollInDegrees;
extern float yawInRadians,pitchInRadians,rollInRadians;
extern float R11,R12,R13,R21,R22,R23,R31,R32,R33;
extern float R11_,R12_,R13_,R21_,R22_,R23_,R31_,R32_,R33_;
extern float initialAccMagnitude;
extern float q0,q1,q2,q3;
extern float kpAcc,kiAcc,kpMag,kiMag,feedbackLimit;
extern float exa,eya,eza;
extern boolean startReset,resetTriggered;
#endif 
