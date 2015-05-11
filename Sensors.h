#ifndef Sensors_h
#define Sensors_h

#include "Definitions.h"
#include "Types.h"
#include <Streaming.h>
#include "Comm.h"
#include <SPI.h>
#include <I2c.h>

extern int16_u gyroX, gyroY, gyroZ;
extern int16_u  accX, accY, accZ;
extern int16_u magX,magY,magZ;
extern float initialPressure, pressure, alti;
extern boolean newBaro,magDetected;

#ifdef V1
extern short temperature;
#endif
#ifdef V2
extern float temperature;
#endif

void GetGro();
void GyroInit();

void GetAcc();
void AccInit();

void GetMag();
void MagInit();

void BaroInit();
void PollPressure();
boolean GetAltitude(float*, float*, float*);


#endif
