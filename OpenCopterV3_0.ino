#include <SPI.h>
#include "I2C_.h"
#include <EEPROM.h>
#include "Streaming_.h"
#include "GPS.h"
#include "Types.h"
#include "Definitions.h"
#include "Enums.h"
#include "Sensors.h"
#include "Comm.h"
#include "Calibration.h"
#include "Attitude.h"
#include "Inertial.h"
#include "LED.h"
#include "RCSignals.h"
#include "PID.h"
#include "Motors.h"
#include "FlightControl.h"
#include "Rom.h"
#include "Radio.h"
#include "ISR.h"

uint32_t loopTime;


void setup() {

  RadioSerialBegin();

  AssignPointerArray();
  SetPinModes();


  DetectRC();

  _200HzISRConfig();
  SPIInit(MSBFIRST,SPI_CLOCK_DIV2,SPI_MODE0);
  I2CInit();
  ROMFlagsCheck();
  CheckESCFlag();
  if (rcDetected == true){
    SetRCControlESCCalFlag();
  }
  MotorInit();

  if (handShake == false) {
    TryHandShake();
  }

  AccInit();
  MagInit();
  BaroInit();
  GyroInit();

  if (calibrationMode == true){
    CalibrateSensors();
  }

  if (gsCTRL == false && (rcDetected == false || RCFailSafe == true)){
    LEDPatternSet(0,1,0,2);
  }

  StartCalibration();

  if (RCValue[GEAR] > 1700 && RCValue[AUX1] > 1700 && RCValue[AUX2] > 1700 && RCValue[AUX3] > 1700){
    GPSDetected = false;
  }
  else{
    GPSStart();
  }

  SetInitialQuaternion();
  InertialInit();
  CheckTXPositions();
  LEDPatternSet(0,1,1,0);
  SetGyroOffsets();
  GetInitialPressure();
  watchDogStartCount = true;
  _400HzTimer = micros();
  _100HzTimer = _400HzTimer;

}


void loop() {

  _400HzTask();
  loopTime = micros();
  _100HzTask(loopTime);
  Telemetry();
  watchDogFailSafeCounter = 0;

}

void Telemetry(){
  if (handShake == true) {
    Radio();
    if (tuningTrasnmitOK == true) {
      TuningTransmitter(); 
      tuningTrasnmitOK = false;
    }

  }

}

void SetPinModes(){
  GyroSSOutput();
  AccSSOutput();
  BaroSSOutput();
  MagSSOutput();
  FlashSSOutput();
  GyroSSHigh();
  AccSSHigh();
  BaroSSHigh();
  MagSSHigh();
  FlashSSHigh();

  D22Output();
  D23Output();
  D24Output();
  D25Output();
  D26Output();
  D27Output();
  D28Output();
  D29Output();

  LEDInit();

}










