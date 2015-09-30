#ifndef Sensors_h
#define Sensors_h

#include "Definitions.h"
#include "Types.h"
#include "Streaming_.h"
#include "Comm.h"
#include <SPI.h>
#include "I2C_.h"

void ReadBatteryInfo(float*);

void VerifyMag();

void GetGro();
void GyroInit();

void GetAcc();
void AccInit();

void GetMag();
void MagInit();

void BaroInit();
void PollPressure();
boolean GetAltitude(float*, float*, float*);

void GetInitialPressure();

void BatteryInfoInit();

extern int16_u gyroX, gyroY, gyroZ;
extern int16_u  accX, accY, accZ;
extern int16_u magX,magY,magZ;
extern float initialPressure, pressure, alti,takeOffPressure;
;
extern boolean newBaro,magDetected;
extern uint32_t baroPollTimer;
extern boolean rotateSensor45Deg;
extern boolean batteryFailSafe;

extern float batteryPercent,cellVoltage,mAh;
#ifdef V1
extern short temperature;
#endif
#ifdef V2
extern float temperature;
#endif




#endif

