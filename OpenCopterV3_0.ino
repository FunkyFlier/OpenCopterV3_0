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
  ControlLED(0x0F);  

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
    NoControlIndicatior();
  }

  Arm();
  GPSStart();

  SetInitialQuaternion();
  InertialInit();

  CheckTXPositions();

  ControlLED(0x00);  
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

void NoControlIndicatior(){
  uint8_t LEDIndex = 0;
  uint8_t LEDArray[8] = {
    0x00,0x01,0x03,0x02,0x06,0x04,0x0C,0x08        };
  while(1){
    ControlLED(LEDArray[LEDIndex++]);
    if(LEDIndex == 8){
      LEDIndex = 0;
    }
    delay(250);
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






