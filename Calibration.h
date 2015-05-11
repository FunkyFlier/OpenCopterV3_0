#ifndef CALIBRATION_H
#define CALIBRATION_H
#include <Arduino.h>


#define NUMBER_GYRO_SAMPLES_FOR_AVG 200
void LoadCalibValuesFromRom();
void ACCScale();
void MAGScale();
void GROScale();
void SetGyroOffsets();
boolean StationaryGyro();

extern float scaledMagX, scaledMagY, scaledMagZ, scaledAccX, scaledAccY, scaledAccZ;
extern float degreeGyroX,degreeGyroY,degreeGyroZ,radianGyroX,radianGyroY,radianGyroZ;
extern float filtAccX,filtAccY,filtAccZ;
#endif
