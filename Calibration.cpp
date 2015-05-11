#include "Calibration.h"
#include "Definitions.h"
#include "Types.h"
#include "Sensors.h"
#include "Math.h"


void LoadACC();
void LoadMAG();
//void LPF(float*, float*, float*, float);

//rename?
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



void LoadCalibValuesFromRom(){
  LoadACC();
  LoadMAG();
  SetGyroOffsets();
  MAGScale();
  ACCScale();
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
  //int16_t gyroPrevX=0,gyroPrevY=0,gyroPrevZ=0;

  /*GetGro();
  gyroPrevX = gyroX.val;
  gyroPrevY = gyroY.val;
  gyroPrevZ = gyroZ.val;*/

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
    /*if ( abs(gyroPrevX - gyroX.val) > 25 || abs(gyroPrevY - gyroY.val) > 25 || abs(gyroPrevZ - gyroZ.val) > 25 ) {
      gyroSumX = gyroX.val;
      gyroSumY = gyroY.val;
      gyroSumZ = gyroZ.val;
      i = 0;
    }

    gyroPrevX = gyroX.val;
    gyroPrevY = gyroY.val;
    gyroPrevZ = gyroZ.val;*/

    delay(3);
  }
  gyroOffsetX = gyroSumX / NUMBER_GYRO_SAMPLES_FOR_AVG;
  gyroOffsetY = gyroSumY / NUMBER_GYRO_SAMPLES_FOR_AVG;
  gyroOffsetZ = gyroSumZ / NUMBER_GYRO_SAMPLES_FOR_AVG;  
}

void LoadACC() {
  float_u outFloat;
  uint8_t outFloatIndex = 0;
  for (uint16_t i = ACC_CALIB_START; i <= ACC_CALIB_END; i++) { //load acc values
    outFloat.buffer[outFloatIndex] = EEPROMRead(i);
    outFloatIndex++;
    switch (i) {
    case ACC_S_X_INDEX:
      accXScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_S_Y_INDEX:
      accYScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_S_Z_INDEX:
      accZScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_X_INDEX:
      accXOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_Y_INDEX:
      accYOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_Z_INDEX:
      accZOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    default:
      break;
    }
  }
}

void LoadMAG() {
  float_u outFloat;
  uint8_t outFloatIndex = 0;
  for (uint16_t i = MAG_CALIB_START; i <= MAG_CALIB_END; i++) { //load the compass values
    outFloat.buffer[outFloatIndex] = EEPROMRead(i);
    outFloatIndex++;
    switch (i) {
    case MAG_OFF_X_INDEX:
      magOffSetX = outFloat.val;
      outFloatIndex = 0;
      break;
    case MAG_OFF_Y_INDEX:
      magOffSetY = outFloat.val;
      outFloatIndex = 0;
      break;
    case MAG_OFF_Z_INDEX:
      magOffSetZ = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_00_INDEX:
      magWInv00 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_01_INDEX:
      magWInv01 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_02_INDEX:
      magWInv02 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_10_INDEX:
      magWInv10 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_11_INDEX:
      magWInv11 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_12_INDEX:
      magWInv12 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_20_INDEX:
      magWInv20 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_21_INDEX:
      magWInv21 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_22_INDEX:
      magWInv22 = outFloat.val;
      outFloatIndex = 0;
      break;
    default:
      break;
    }
  }
}
//rename can't think of anything better right now
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
  if (dt > 0.1){
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







