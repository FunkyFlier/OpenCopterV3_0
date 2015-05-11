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

uint32_t printTimer;
uint32_t loopTime;
volatile uint32_t RCFailSafeCounter;

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
  _400HzTask();
  loopTime = micros();
  _100HzTask();
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
  if (rcType != RC){
    FeedLine();
  }
}



//end move these functions


void _400HzTask() {
  uint32_t _400HzTime;
  static uint32_t _400HzTimer;
  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    //what to do with DT calculation
    D22High();
    _400HzTimer = _400HzTime;
    PollAcc();
    D22Low();
  }
}

void _100HzTask(){
  static uint8_t _100HzState = 0;
  static uint32_t _100HzTimer = 0;
  float _100HzDt;

  if (loopTime - _100HzTimer >= 10000){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    D23High();
    while(_100HzState < LAST_100HZ_TASK){
      switch (_100HzState){
      case GET_GYRO:
        PollGro();
        if(magDetected == true){
          _100HzState = GET_MAG;
        }
        else{
          _100HzState = ATT_UPDATE;
        }
        break;
      case GET_MAG:
        PollMag();  
        _100HzState = ATT_UPDATE;
        break;
      case ATT_UPDATE:
        AHRSupdate(_100HzDt);
        _100HzState = ROT_MATRIX;
        break;
      case ROT_MATRIX:
        GenerateRotationMatrix();
        _100HzState = GET_EULER;
        break;
      case GET_EULER:
        GetEuler();
        _100HzState = GET_INERTIAL;
        break;
      case GET_INERTIAL:
        GetInertial();
        _100HzState = POS_VEL_PREDICTION;
        break;
      case POS_VEL_PREDICTION:
        Predict(_100HzDt);
        _100HzState = POLL_GPS;
        break;
      case UPDATE_LAG_INDEX:
        UpdateLagIndex();
        _100HzState = POLL_GPS;
        break;
      case POLL_GPS:
        GPSMonitor();
        if (newGPSData == true) {
          newGPSData = false;
          CorrectXY();
        }
        _100HzState = POLL_BARO;
        break;
      case POLL_BARO:
        PollPressure();
        if (newBaro == true) {
          newBaro = false;
          CorrectZ();
        }
        _100HzState = PROCESS_CONTROL_SIGNALS;
        break;
      case PROCESS_CONTROL_SIGNALS:
        if (newRC == true) {//9
          newRC = false;
          ProcessChannels();
          RCFailSafeCounter = 0;
        }
        if (RCFailSafeCounter > 200){
          RCFailSafe = true;
        }
        _100HzState = LAST_100HZ_TASK;
        break;
      default:
        _100HzState = GET_GYRO;
        break;
      }
      _400HzTask();

    }
    D23Low();
    _100HzState = GET_GYRO;
  }




}
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

































