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

uint32_t printTimer,watchDogFailSafeCounter;
uint32_t loopTime;



void setup() {
  Serial.begin(115200);

  SetPinModes();
  ControlLED(0x0F);  
  CheckDefines();

  DetectRC();
  _200HzISRConfig();

  SPIInit(MSBFIRST,SPI_CLOCK_DIV2,SPI_MODE0);
  I2CInit();

  //GPSInit();
  GPSStart();
  BaroInit();
  GyroInit();
  AccInit();
  MagInit();

  LoadCalibValuesFromRom();
  LoadAttValuesFromRom();
  LoadRCValuesFromRom();

  SetInitialQuaternion();
  InertialInit();

  ControlLED(0x00);  


  Serial<<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";

}



void loop() {
  watchDogFailSafeCounter = 0;
  _400HzTask();
  loopTime = micros();
  _100HzTask(loopTime);
  if (millis() - printTimer > 100) {
    printTimer = millis();
    //Serial <<yawInDegrees<<","<<rollInDegrees<<","<<pitchInDegrees<<"\r\n";
  }

}
//to do move these functions
void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
  RCFailSafeCounter++;
  watchDogFailSafeCounter++;
  if (rcType != RC){
    FeedLine();
  }
}



//end move these functions



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
#ifdef V2
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


