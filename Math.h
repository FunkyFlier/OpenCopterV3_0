#ifndef MATH_h
#define MATH_h

#include <Arduino.h>
//#include <Streaming.h>

#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi
#define PI_FLOAT     3.14159265f
#define PIBY2_FLOAT  1.5707963f



float FastAtan2( float, float);

float InvSqrt(float);

void MapVar (volatile int16_t*, float*, float, float, float, float);
void MapVar ( float* , float* , int16_t,int16_t, int16_t , int16_t );

void LPF(float*, float*, float*, float);

#endif 





