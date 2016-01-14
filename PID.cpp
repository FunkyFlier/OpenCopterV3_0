#include "PID.h"

PID::PID(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  dErrorPrev = 0;

}

void PID::calculate(){
  error = *setPoint - *actual;

  if (*integrate == true){
    iError += *ki * *dt * error;
  }
  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);
  *adjustment = *kp * error  + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }

  prevError = error;
  dErrorPrev = dError;
}

void PID::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevError =0;
  dErrorPrev = 0;
}
//PID_2 prevents deravitive kick on changing set point

PID_2::PID_2(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  dErrorPrev = 0;

}

void PID_2::calculate(){
  error = *setPoint - *actual;


  if (*integrate == true){
    iError += *ki * *dt * error;
  }
  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev - *kd * *fc * (*actual - prevActual);
  *adjustment = *kp * error  + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }

  dErrorPrev = dError;
  prevActual = *actual;
}

void PID_2::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  dErrorPrev = 0;
}


YAW::YAW(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevError =0;
  dErrorPrev = 0;

}

void YAW::calculate(){
  PIDAngle = *setPoint;

  error = PIDAngle - *actual;

  if (error < -180.0){
    error += 360.0;
  }
  if (error > 180.0){
    error -= 360.0;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (error - prevError);

  if (*integrate == true){
    iError += *ki * *dt * error;
  }

  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  *adjustment = *kp * error + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }
  prevError = error;
  dErrorPrev = dError;

}

void YAW::reset(){
  error = 0;
  prevError=0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  dErrorPrev = 0;
}

YAW_2::YAW_2(float *set,float *act, float *adj,boolean *intToggle,float *p, float *i, float *d,float *n,float *delta,float iLim,float lim){
  setPoint = set;
  actual = act;
  adjustment = adj;
  integrate = intToggle;
  kp = p;
  ki = i;
  kd = d;
  fc = n;
  integralLimitHigh = iLim;
  integralLimitLow = -1*iLim;
  outputLimitHigh = lim;
  outputLimitLow = -1*lim;
  dt = delta;
  prevActual = 0;
  dErrorPrev = 0;

}

void YAW_2::calculate(){
  PIDAngle = *setPoint;


  error = PIDAngle - *actual;

  if (error < -180.0){
    error += 360.0;
  }
  if (error > 180.0){
    error -= 360.0;
  }

  actualDiff = *actual - prevActual;
  
  if (actualDiff > 180.0){
    actualDiff -= 360.0;
  }
  if (actualDiff < -180.0){
    actualDiff += 360.0;
  }

  dError = dErrorPrev - *fc * *dt * dErrorPrev + *kd * *fc * (actualDiff);

  if (*integrate == true){
    iError += *ki * *dt * error;
  }

  if (iError > integralLimitHigh){
    iError = integralLimitHigh;
  }
  if (iError < integralLimitLow){
    iError = integralLimitLow;
  }

  *adjustment = *kp * error + iError +  dError;

  if (*adjustment > outputLimitHigh){
    *adjustment  = outputLimitHigh;
  }
  if (*adjustment < outputLimitLow){
    *adjustment = outputLimitLow;
  }
  prevActual = *actual;
  dErrorPrev = dError;
}

void YAW_2::reset(){
  error = 0;
  iError = 0;
  dError = 0;
  *adjustment = 0;
  prevActual = *actual;
  dErrorPrev = 0;
}






