#include "FlightControl.h"


void PollAcc();
void PollMag();
void PollGro();

uint32_t _100HzTimer,_400HzTimer;
volatile uint32_t RCFailSafeCounter;
float initialYaw;
boolean integrate;
uint8_t HHState;
float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
uint8_t XYLoiterState, ZLoiterState,flightMode;

float homeBaseXOffset,homeBaseYOffset;
float xTarget,yTarget,zTarget;
boolean enterState;

void _400HzTask() {
  uint32_t _400HzTime;
  
  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    //what to do with DT calculation
    D22High();
    _400HzTimer = _400HzTime;
    PollAcc();
    D22Low();
  }
}

void _100HzTask(uint32_t loopTime){
  static uint8_t _100HzState = 0;
  
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
