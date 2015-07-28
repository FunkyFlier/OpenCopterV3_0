#include "Math.h"

void LPF(float *output, float *input ,float *dt,float TIME_CONST){
  float alpha,beta;
  alpha = *dt/(*dt + TIME_CONST);
  beta = 1.0 - alpha;
  *output = *output * beta + *input * alpha;
}
void LPF(float *output, uint16_t *input ,float *dt,float TIME_CONST){
  float alpha,beta;
  alpha = *dt/(*dt + TIME_CONST);
  beta = 1.0 - alpha;
  *output = *output * beta + *input * alpha;
}



void MapVar (volatile int16_t *x, float *y, float in_min, float in_max, float out_min, float out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MapVar ( float *x, float *y, int16_t in_min, int16_t in_max, int16_t out_min, int16_t out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void MapVar ( uint16_t *x, float *y, uint16_t in_min, uint16_t in_max, uint16_t out_min, uint16_t out_max){
  *y = (*x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
