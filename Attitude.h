#ifndef ATTITUDE_H
#define ATTITUDE_H

#define KP_ACC 1.0
#define KI_ACC 0.0
#define KP_MAG 1.0
#define KI_MAG 0.0
#define FEEDBACK_LIMIT 0.1
#define SAMPLES_FOR_ACC_MAGNITUDE 200

void SetInitialQuaternion();
void LoadAttValuesFromRom();

void AHRSupdate(float);
void GenerateRotationMatrix();

void GetEuler();
void GetPitch();
void GetRoll();
void GetYaw();

extern float declination;
extern float yawInDegrees,pitchInDegrees,rollInDegrees;
extern float yawInRadians,pitchInRadians,rollInRadians;
extern float R11,R12,R13,R21,R22,R23,R31,R32,R33;
extern float initialAccMagnitude;


#endif 
