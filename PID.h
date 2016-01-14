#ifndef PID_h
#define PID_h


#include <Arduino.h>
#include "Math.h"

class PID{
public:
  PID(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
private:


};
class PID_2{
public:
  PID_2(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
private:


};

class YAW{
public:
  YAW(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
private:
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevError;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
  float PIDAngle;
  float errorDiff;

};
class YAW_2{
public:
  YAW_2(float*, float*, float*,boolean*,float*, float*, float*, float*,float*,float,float);
  void calculate();
  void reset();
private:
  float integralLimitHigh;
  float integralLimitLow;
  float outputLimitHigh;
  float outputLimitLow;
  float *dt;
  float *setPoint;
  float *actual;
  float *adjustment;
  boolean *integrate;
  float *kp;
  float *ki;
  float *kd;
  float *fc;
  float prevActual;
  float error;
  float iError;
  float dError;
  float dErrorPrev;
  float PIDAngle;
  float errorDiff;

};


#endif


