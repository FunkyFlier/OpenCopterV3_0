#ifndef CALIBRATION_H
#define CALIBRATION_H

#include "Definitions.h"
#include "Types.h"
#include "Sensors.h"
#include "Math.h"

#include <Arduino.h>


#define NUMBER_GYRO_SAMPLES_FOR_AVG 200
void LoadCalibValuesFromRom();
void ACCScale();
void MAGScale();
void GROScale();
void SetGyroOffsets();
boolean StationaryGyro();
void PollAcc();
void PollMag();
void PollGro();

extern float scaledMagX, scaledMagY, scaledMagZ, scaledAccX, scaledAccY, scaledAccZ;
extern float degreeGyroX,degreeGyroY,degreeGyroZ,radianGyroX,radianGyroY,radianGyroZ;
extern float filtAccX,filtAccY,filtAccZ;


extern float magOffSetX, magOffSetY, magOffSetZ,
magWInv00,  magWInv01,  magWInv02,
magWInv10,  magWInv11,  magWInv12, 
magWInv20,  magWInv21,  magWInv22;

extern float accXScale, accYScale, accZScale,
accXOffset,accYOffset,accZOffset;
#endif
