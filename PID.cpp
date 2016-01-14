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

  if (*setPoint >= 0.0 && *setPoint < 180.0){
    //quadrant one and two handling
    if (*actual < 360.0 && *actual >= (PIDAngle + 180.0)){
      PIDAngle += 360;
    }
  }
  if (*setPoint >= 180.0 && *setPoint < 360.0){
    //quadrant three and four handling
    if (*actual >= 0.0 && *actual < (PIDAngle - 180.0)){
      PIDAngle -= 360.0;
    }
  }

  error = PIDAngle - *actual;

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

  if (*setPoint >= 0.0 && *setPoint < 180.0){
    //quadrant one and two handling
    if (*actual < 360.0 && *actual >= (PIDAngle + 180.0)){
      PIDAngle += 360;
    }
  }
  if (*setPoint >= 180.0 && *setPoint < 360.0){
    //quadrant three and four handling
    if (*actual >= 0.0 && *actual < (PIDAngle - 180.0)){
      PIDAngle -= 360.0;
    }
  }
  error = PIDAngle - *actual;
  
  //assume abs(delta) << 180
  actualDiff = *actual - prevActual;
  if (actualDiff > 180.0){
    actualDiff = *actual - ( prevActual - 360.0);
  }
  if (actualDiff < -180.0){
    actualDiff = *actual - ( prevActual + 360.0);
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




