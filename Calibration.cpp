#include "Calibration.h"
#include "Rom.h"


float scaledMagX, scaledMagY, scaledMagZ, scaledAccX, scaledAccY, scaledAccZ;
float degreeGyroX,degreeGyroY,degreeGyroZ,radianGyroX,radianGyroY,radianGyroZ;
float filtAccX,filtAccY,filtAccZ;

int16_t gyroOffsetX,gyroOffsetY,gyroOffsetZ;

float magOffSetX, magOffSetY, magOffSetZ,
magWInv00,  magWInv01,  magWInv02,
magWInv10,  magWInv11,  magWInv12, 
magWInv20,  magWInv21,  magWInv22;

float accXScale, accYScale, accZScale,
accXOffset,accYOffset,accZOffset;



void PollAcc(){
  GetAcc();
  ACCScale();
}
void PollMag(){
  GetMag();
  MAGScale();
}
void PollGro(){
  GetGro();
  GROScale();
}
boolean StationaryGyro(){
  static int16_t gyroPrevX=0,gyroPrevY=0,gyroPrevZ=0;
  boolean stationary;
  if ( abs(gyroPrevX - gyroX.val) > 25 || abs(gyroPrevY - gyroY.val) > 25 || abs(gyroPrevZ - gyroZ.val) > 25 ) {
      stationary = false;
    }else{
      stationary = true;
    }
    gyroPrevX = gyroX.val;
    gyroPrevY = gyroY.val;
    gyroPrevZ = gyroZ.val;
    return stationary;
    
}
void SetGyroOffsets(){
  int32_t gyroSumX=0,gyroSumY=0,gyroSumZ=0;

  for (uint16_t i = 0; i < NUMBER_GYRO_SAMPLES_FOR_AVG; i ++) {
    GetGro();
    gyroSumX += gyroX.val;
    gyroSumY += gyroY.val;
    gyroSumZ += gyroZ.val;
    
    if (StationaryGyro() == false){
      gyroSumX = gyroX.val;
      gyroSumY = gyroY.val;
      gyroSumZ = gyroZ.val;
      i = 1;
    }


    delay(3);
  }
  gyroOffsetX = gyroSumX / NUMBER_GYRO_SAMPLES_FOR_AVG;
  gyroOffsetY = gyroSumY / NUMBER_GYRO_SAMPLES_FOR_AVG;
  gyroOffsetZ = gyroSumZ / NUMBER_GYRO_SAMPLES_FOR_AVG;  
}

void ACCScale(){
  static uint32_t previousTime;
  float dt;
  float shiftedAccX,shiftedAccY,shiftedAccZ;
  
  shiftedAccX  = accX.val - accXOffset;
  shiftedAccY  = accY.val - accYOffset;
  shiftedAccZ  = accZ.val - accZOffset;

  scaledAccX = shiftedAccX * accXScale;
  scaledAccY = shiftedAccY * accYScale;
  scaledAccZ = shiftedAccZ * accZScale;
  
  dt = (micros() - previousTime) * 0.000001;
  previousTime = micros();
  if (dt > 0.1 || dt <= 0){
    filtAccX = scaledAccX ;
    filtAccY = scaledAccY;
    filtAccZ = scaledAccZ;
  }else{
    LPF(&filtAccX,&scaledAccX,&dt,RC_CONST_ACC);
    LPF(&filtAccY,&scaledAccY,&dt,RC_CONST_ACC);
    LPF(&filtAccZ,&scaledAccZ,&dt,RC_CONST_ACC);
  }
  
}
void MAGScale(){
  float shiftedMagX,shiftedMagY,shiftedMagZ;
  shiftedMagX  = magX.val - magOffSetX;
  shiftedMagY  = magY.val - magOffSetY;
  shiftedMagZ  = magZ.val - magOffSetZ;

  scaledMagX = magWInv00 * shiftedMagX + magWInv01 * shiftedMagY + magWInv02 * shiftedMagZ;
  scaledMagY = magWInv10 * shiftedMagX + magWInv11 * shiftedMagY + magWInv12 * shiftedMagZ;
  scaledMagZ = magWInv20 * shiftedMagX + magWInv21 * shiftedMagY + magWInv22 * shiftedMagZ;
}

void GROScale(){
  degreeGyroX = (gyroX.val - gyroOffsetX) * 0.07;
  degreeGyroY = (gyroY.val - gyroOffsetY) * 0.07;
  degreeGyroZ = (gyroZ.val - gyroOffsetZ) * 0.07;

  radianGyroX = ToRad(degreeGyroX);
  radianGyroY = ToRad(degreeGyroY);
  radianGyroZ = ToRad(degreeGyroZ);
}







