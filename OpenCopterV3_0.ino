#include <SPI.h>
#include <I2C.h>
#include <EEPROM.h>
#include <Streaming.h>
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

uint32_t loopTime,printTimer;



void setup() {

  RadioSerialBegin();
  AssignPointerArray();
  SetPinModes();
  ControlLED(0x0F);  
  //CheckDefines();

  DetectRC();
  _200HzISRConfig();

  SPIInit(MSBFIRST,SPI_CLOCK_DIV2,SPI_MODE0);
  I2CInit();

  ROMFlagsCheck();
  if (rcDetected == true){
    CheckESCFlag();
    CalibrateESC();
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
  SetGyroOffsets();
  SetInitialQuaternion();
  InertialInit();
  CheckTXPositions();

  ControlLED(0x00);  
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
  if (millis() - printTimer > 250){
    Serial<<throCommand<<","<<cmdAile<<","<<cmdElev<<","<<cmdRudd<<"\r\n";
  }

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
    0x00,0x01,0x03,0x02,0x06,0x04,0x0C,0x08      };
  while(1){
    ControlLED(LEDArray[LEDIndex++]);
    if(LEDIndex == 8){
      LEDIndex = 0;
    }
    delay(250);
  }
}
void CheckDefines(){

#ifdef V1
  Serial<<"V1\r\n";
#endif
#ifdef V2
  Serial<<"V2\r\n";
#endif
#ifdef ROT_45
  Serial<<"ROT_45\r\n";
#endif
#ifdef QUAD_CAMP
  Serial<<"QUAD_CAMP\r\n";
#endif
#ifdef QUAD
  Serial<<"QUAD\r\n";
#endif
#ifdef HEX_FRAME
  Serial<<"HEX_FRAME\r\n";
#endif
#ifdef X_8
  Serial<<"X_8\r\n";
#endif
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





