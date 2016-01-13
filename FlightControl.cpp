#include "FlightControl.h"
#include "LED.h"
#include "Definitions.h"
#include "Enums.h"
#include "Sensors.h"
#include "Attitude.h"
#include "Inertial.h"
#include "GPS.h"
#include "RCSignals.h"
#include "PID.h"
#include "Motors.h"
#include "Radio.h"
#include "Flash.h"


void FlightSM();
void TrimCheck();
void InitLoiter();
void RTBStateMachine();
void LoiterCalculations();
void FailSafeHandler();

void LoiterSM();
void Rotate2dVector(float*, float*, float*, float*, float*, float*);
void HeadingHold();

void ProcessModes();
void WayPointTasks();
void UpdateWPTarget();
void UpdateLookAtHeading();

float zero = 0;
boolean batteryFSOverride = false;


uint8_t flightMode = 0;
uint8_t flightModeControl=0;
uint32_t _100HzTimer,_400HzTimer,lowRateTimer;
uint8_t lowRateCounter;
boolean lowRateTasks = false;

volatile uint32_t RCFailSafeCounter=0,watchDogFailSafeCounter=0,groundFSCount=0,baroFSCount=0,telemFSCount=0;
float initialYaw;
boolean integrate;
uint8_t HHState;
float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
uint8_t XYLoiterState, ZLoiterState,RTBState,txLossRTB,previousFlightMode;
boolean GSCTRLFailSafe,txFailSafe,tuningTrasnmitOK,baroFS,telemFS;

float homeBaseXOffset,homeBaseYOffset;
float xTarget,yTarget,zTarget;
float velSetPointX,velSetPointY,velSetPointZ;
float tiltAngleX,tiltAngleY;
float distToWayPoint;
float controlBearing;
boolean enterState;
boolean setTrim,trimComplete,calcYaw;
float yawInput;
float _100HzDt = 0.01;
float highRateDT = 0.0025;

float lowRateDT = 0.04;

float bodyVelX,bodyVelY;

uint8_t initProgress = 0;

uint8_t modeArray[9] = {
  RATE,L1,ATT,ATT,ATT,ATT_TRIM,RATE,RATE,L0};


uint8_t wayPointState;
boolean lookAtFlag = false, newWayPointFlag = false;
int32_t wpLat,wpLon;
float wpYaw,wpX,wpY,wpZ;
//float yawSetPointWP;
int32_t lookAtLat,lookAtLon;
float lookAtYaw;

float kp_pitch_rate;
float ki_pitch_rate;
float kd_pitch_rate;
float fc_pitch_rate;

float kp_roll_rate;
float ki_roll_rate;
float kd_roll_rate;
float fc_roll_rate;

float kp_yaw_rate;
float ki_yaw_rate;
float kd_yaw_rate;
float fc_yaw_rate;

float kp_pitch_attitude;
float ki_pitch_attitude;
float kd_pitch_attitude;
float fc_pitch_attitude;

float kp_roll_attitude;
float ki_roll_attitude;
float kd_roll_attitude;
float fc_roll_attitude;

float kp_yaw_attitude;
float ki_yaw_attitude;
float kd_yaw_attitude;
float fc_yaw_attitude;

float kp_altitude_position;
float ki_altitude_position;
float kd_altitude_position;
float fc_altitude_position;

float kp_altitude_velocity;
float ki_altitude_velocity;
float kd_altitude_velocity;
float fc_altitude_velocity;

float kp_loiter_pos_x;
float ki_loiter_pos_x;
float kd_loiter_pos_x;
float fc_loiter_pos_x;

float kp_loiter_velocity_x;
float ki_loiter_velocity_x;
float kd_loiter_velocity_x;
float fc_loiter_velocity_x;

float kp_loiter_pos_y;
float ki_loiter_pos_y;
float kd_loiter_pos_y;
float fc_loiter_pos_y;

float kp_loiter_velocity_y;
float ki_loiter_velocity_y;
float kd_loiter_velocity_y;
float fc_loiter_velocity_y;

float kp_waypoint_position;
float ki_waypoint_position;
float kd_waypoint_position;
float fc_waypoint_position;

float kp_waypoint_velocity;
float ki_waypoint_velocity;
float kd_waypoint_velocity;
float fc_waypoint_velocity;

float kp_cross_track;
float ki_cross_track;
float kd_cross_track;
float fc_cross_track;

float pitchSetPoint;
float rollSetPoint;
float yawSetPoint;
float rateSetPointX;
float rateSetPointY;
float rateSetPointZ;

float wpVelSetPoint,wpPathVelocity,wpCrossTrackVelocity,wpTilX,wpTiltY,headingToWayPoint;
float xFromTO,yFromTO;
float alhpaForPressure;

int16_t floorLimit,ceilingLimit;

boolean executeStep = false,stepStart = false;
float debugVariable;

float baroErrorLim,countsOff,countsOn,landErrorLim;

PID PitchRate(&rateSetPointY, &degreeGyroY, &adjustmentY, &integrate, &kp_pitch_rate, &ki_pitch_rate, &kd_pitch_rate, &fc_pitch_rate, &highRateDT, 400, 400);
PID RollRate(&rateSetPointX, &degreeGyroX, &adjustmentX, &integrate, &kp_roll_rate, &ki_roll_rate, &kd_roll_rate, &fc_roll_rate, &highRateDT, 400, 400);
PID_2 YawRate(&rateSetPointZ, &degreeGyroZ, &adjustmentZ, &integrate, &kp_yaw_rate, &ki_yaw_rate, &kd_yaw_rate, &fc_yaw_rate, &highRateDT, 400, 400);

PID_2 PitchAngle(&pitchSetPoint, &pitchInDegrees, &rateSetPointY, &integrate, &kp_pitch_attitude, &ki_pitch_attitude, &kd_pitch_attitude, &fc_pitch_attitude, &_100HzDt, 360, 360);
PID_2 RollAngle(&rollSetPoint, &rollInDegrees, &rateSetPointX, &integrate, &kp_roll_attitude, &ki_roll_attitude, &kd_roll_attitude, &fc_roll_attitude, &_100HzDt, 360, 360);
YAW_2 YawAngle(&yawSetPoint, &yawInDegrees, &rateSetPointZ, &integrate, &kp_yaw_attitude, &ki_yaw_attitude, &kd_yaw_attitude, &fc_yaw_attitude, &_100HzDt, 360, 360);

PID_2 LoiterXPosition(&xTarget, &XEst, &velSetPointX, &integrate, &kp_loiter_pos_x, &ki_loiter_pos_x, &kd_loiter_pos_x, &fc_loiter_pos_x, &lowRateDT, 2, 2);
PID_2 LoiterXVelocity(&velSetPointX, &velX, &tiltAngleX, &integrate, &kp_loiter_velocity_x, &ki_loiter_velocity_x, &kd_loiter_velocity_x, &fc_loiter_velocity_x, &lowRateDT, 30, 30);

PID_2 LoiterYPosition(&yTarget, &YEst, &velSetPointY, &integrate, &kp_loiter_pos_y, &ki_loiter_pos_y, &kd_loiter_pos_y, &fc_loiter_pos_y, &lowRateDT, 2, 2);
PID_2 LoiterYVelocity(&velSetPointY, &velY, &tiltAngleY, &integrate, &kp_loiter_velocity_y, &ki_loiter_velocity_y, &kd_loiter_velocity_y, &fc_loiter_velocity_y, &lowRateDT, 30, 30);

PID_2 AltHoldPosition(&zTarget, &ZEstUp, &velSetPointZ, &integrate, &kp_altitude_position, &ki_altitude_position, &kd_altitude_position, &fc_altitude_position, &lowRateDT, 1.5, 1.5);
PID_2 AltHoldVelocity(&velSetPointZ, &velZUp, &throttleAdjustment, &integrate, &kp_altitude_velocity, &ki_altitude_velocity, &kd_altitude_velocity, &fc_altitude_velocity, &lowRateDT, 1000, 1000);

PID_2 WPPosition(&zero, &distToWayPoint, &wpVelSetPoint, &integrate, &kp_waypoint_position, &ki_waypoint_velocity, &kd_waypoint_velocity, &fc_waypoint_velocity, &lowRateDT, 4,4);
PID_2 WPVelocity(&wpVelSetPoint, &wpPathVelocity,&wpTilX, &integrate, &kp_waypoint_velocity, &ki_waypoint_velocity, &kd_waypoint_velocity, &fc_waypoint_velocity, &lowRateDT, 15,15);

PID_2 WPCrossTrack(&zero, &wpCrossTrackVelocity, &wpTiltY, &integrate, &kp_cross_track ,&ki_cross_track, &kd_cross_track,&fc_cross_track , &lowRateDT, 15,15);

void highRateTasks() {
  uint32_t _400HzTime;

  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    highRateDT =  (_400HzTime - _400HzTimer) * 0.000001; 
    _400HzTimer = _400HzTime;
    PollAcc();
    PollGro();
    if (flightMode > L2){
      if (rateSetPointZ > Z_RATE_LIMIT){
        rateSetPointZ = Z_RATE_LIMIT;
      }
      if (rateSetPointZ < (-1.0* Z_RATE_LIMIT) ){
        rateSetPointZ = (-1.0* Z_RATE_LIMIT);
      }
    }
    PitchRate.calculate();
    RollRate.calculate();
    YawRate.calculate();
  }
}

void _100HzTask(uint32_t loopTime){
  static uint8_t _100HzState = 0;
  if (loopTime - _100HzTimer >= 13300){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    while(_100HzState < LAST_100HZ_TASK){
      switch (_100HzState){
      case GET_GYRO:
        lowRateCounter++;
        if (lowRateCounter >= LOW_RATE_DIVIDER){
          lowRateDT = (millis() - lowRateTimer) * 0.001;
          lowRateTimer = millis();
          if (lowRateDT > 0.1){
            lowRateDT = 0.1;
          }
          lowRateTasks = true;
          lowRateCounter = 0;
        }
        else{
          lowRateTasks = false;
        }     
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
        _100HzState = UPDATE_LAG_INDEX;
        break;
      case UPDATE_LAG_INDEX:
        UpdateLagIndex();
        _100HzState = POLL_GPS;
        break;
      case POLL_GPS:
        if (GPSDetected == true) {
          GPSMonitor();
          if (newGPSData == true) {
            newGPSData = false;
            CorrectXY();
          }
        }
        if (GPSFailSafeCounter > 200) {
          gpsFailSafe = true;
        }
        _100HzState = POLL_BARO;
        break;
      case POLL_BARO:
        PollPressure();
        if (baroFSCount > 200){
          baroFS = true;
        }
        if (newBaro == true && baroFS == false) {
          baroFSCount = 0;
          newBaro = false;
          CorrectZ();
        }
        _100HzState = FLIGHT_STATE_MACHINE;
        break;
      case FLIGHT_STATE_MACHINE:
        FlightSM();
        _100HzState = PROCESS_RC_CONTROL_SIGNALS;
        break;
      case PROCESS_RC_CONTROL_SIGNALS:
        if (newRC == true) {
          newRC = false;
          ProcessChannels();
          RCFailSafeCounter = 0;
        }
        if (RCFailSafeCounter > 200){
          RCFailSafe = true;
        }
        txFailSafe = RCFailSafe;
        _100HzState = PROCESS_GS_CONTROL_SIGNALS;
        break;
      case PROCESS_GS_CONTROL_SIGNALS:
        if (newGSRC == true) {
          groundFSCount = 0;
          newGSRC = false;
          GSCTRLFailSafe = false;
          if (gsCTRL == true){
            ProcessModes();
          }
        }
        if (telemFSCount >= 6000) {
          telemFS = true;
        }
        if (groundFSCount >= 200) {
          GSCTRLFailSafe = true;
        }
        _100HzState = HANDLE_FAILSAFES;
        break;
      case HANDLE_FAILSAFES:
        FailSafeHandler();
        if (flightMode > 0){
          _100HzState = ATTITUDE_PID_LOOPS;
        }
        else{
          _100HzState = READ_BATTERY;
        }

        break;
      case ATTITUDE_PID_LOOPS:
        PitchAngle.calculate();
        RollAngle.calculate();
        if (calcYaw == true) {
          YawAngle.calculate();
        }
        if (flightMode > L2){
          if (rateSetPointZ > Z_RATE_LIMIT){
            rateSetPointZ = Z_RATE_LIMIT;
          }
          if (rateSetPointZ < (-1.0* Z_RATE_LIMIT) ){
            rateSetPointZ = (-1.0* Z_RATE_LIMIT);
          }
        }
        _100HzState = READ_BATTERY;
        break;

      case READ_BATTERY:
        ReadBatteryInfo(&_100HzDt);
        _100HzState = LED_HANDLE;
        break;
      case LED_HANDLE:
        LEDPatternHandler(loopTime);
        _100HzState = FLASH_LOGGING;
        break;
      case FLASH_LOGGING:
        LoggingStateMachine();
        LogHandler();
        _100HzState = MOTOR_HANDLER;
        break;
      case MOTOR_HANDLER:
        MotorHandler();
        tuningTrasnmitOK = true;
        _100HzState = LAST_100HZ_TASK;
        break;
      default:
        _100HzState = GET_GYRO;
        break;
      }
      highRateTasks();

    }
    _100HzState = GET_GYRO;
  }

}


void WayPointStateMachine(){
  //switches between traveling and loitering 
  static float initialWPVel,commandedWPVel;
  float xDist,yDist;
  static boolean startSlowing = false;
  if (motorState == HOLD || motorState == TO) {
    return;
  }
  if (lowRateTasks == true){
    HeadingHold();
    switch (wayPointState){
    case WP_TAKE_OFF:
      //loiter at 0,0,2 until 2 meters reached
      if (motorState == LAND){
        motorState = FLIGHT;
      }
      LoiterCalculations();
      Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      //HeadingHold();
      if (fabs(ZEstUp - zTarget) < 0.25){
        wayPointState = WP_LOITER;
      }
      break;
    case WP_TRAVEL:
      if (motorState == LAND){
        motorState = FLIGHT;
      }
      WayPointTasks();
      if (wayPointState != WP_TRAVEL){
        break;
      }
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      //HeadingHold();
      //within min radius switch to lioter 
      xDist = XEst - xTarget;
      yDist = YEst - yTarget;
      distToWayPoint = sqrt(xDist * xDist + yDist * yDist);
      if (distToWayPoint < MIN_RTB_DIST){
        wayPointState = WP_LOITER;
        startSlowing = false;
        break;
      }
      //calculate distance and heading to origin

      headingToWayPoint = ToDeg(atan2(-1.0 * yDist , -1.0 * xDist));
      if (headingToWayPoint < 0.0){
        headingToWayPoint += 360.0;
      }
      //rotate EF velocity to path frame
      Rotate2dVector(&zero,&headingToWayPoint,&velX,&velY,&wpPathVelocity,&wpCrossTrackVelocity);
      //wp pos PID
      WPPosition.calculate(); 
      wpVelSetPoint *= -1.0;
      //wp vel PID
      if (distToWayPoint < LOW_SPEED_RADIUS && startSlowing == false){
        startSlowing = true;
        initialWPVel = wpVelSetPoint;
        commandedWPVel = wpVelSetPoint;
      }
      if (startSlowing == true){
        commandedWPVel = commandedWPVel * RAMP_DOWN_ALPHA + (1 - RAMP_DOWN_ALPHA) * RAMP_DOWN_VEL_RTB;
        wpVelSetPoint = commandedWPVel;
      }
      WPVelocity.calculate(); 
      wpTilX *= -1.0;
      //cross track vel PID
      WPCrossTrack.calculate(); 
      //rotate wp tilx and tilty to body pitch and roll
      Rotate2dVector(&yawInDegrees,&headingToWayPoint,&wpTilX,&wpTiltY,&pitchSetPoint,&rollSetPoint);
      break;
    case WP_LOITER:
      if (motorState == LAND){
        motorState = FLIGHT;
      }
      WayPointTasks();
      if (wayPointState != WP_LOITER){
        break;
      }
      LoiterCalculations();
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      //HeadingHold();
      Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      break;
    case WP_LAND:
      if (motorState == FLIGHT){
        motorState = LANDING;
      }

      WayPointTasks();
      LoiterCalculations();
      Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      AltHoldVelocity.calculate();
      //HeadingHold();
      break;
    case WP_RTB:
      RTBStateMachine();
      break;
    }
  }

}
void WayPointTasks(){
  if (lookAtFlag == true){
    UpdateLookAtHeading();
  }
  if (newWayPointFlag == true){
    newWayPointFlag = false;
    UpdateWPTarget();
  }
  HeadingHold();
}
void UpdateWPTarget(){
  float xDist,yDist;

  xTarget = wpX;
  yTarget = wpY;

  zTarget = wpZ;
  if (lookAtFlag == false){
    yawSetPoint = wpYaw;
  }

  xDist = XEst - xTarget;
  yDist = YEst - yTarget;
  if (sqrt(xDist * xDist +  yDist * yDist) <  MIN_RTB_DIST){
    wayPointState = WP_LOITER;
  }
  else{
    wayPointState = WP_TRAVEL;
  }
}

void UpdateLookAtHeading(){
  static uint32_t lookAtTimer = 0;
  float tempX,tempY,tempDist;
  if (millis() - lookAtTimer >= 1000){
    lookAtTimer = millis();
    DistBearing(&GPSData.vars.lat,&GPSData.vars.lon,&lookAtLat,&lookAtLon,&tempX,&tempY,&tempDist,&yawSetPoint);
    if (sqrt(tempX * tempX + tempY * tempY) < MIN_RTB_DIST){
      yawSetPoint = wpYaw;
    } 
  }
}
void WayPointUpdate(float lat, float lon, float alt, float yaw){
  //called by radio.cpp
  //updates the wp from the ground station
  //sets the correct state
  if (wayPointState != WP_TAKE_OFF){
    float tempX,tempY,tempDist,tempYaw;
    wpLat = (int32_t)(lat * 10000000);
    wpLon = (int32_t)(lon * 10000000);
    wpZ = alt;
    wpYaw = yaw;
    DistBearing(&homeLat,&homeLon,&wpLat,&wpLon,&wpX,&wpY,&tempDist,&tempYaw);
    if (lookAtFlag == true){
      DistBearing(&GPSData.vars.lat,&GPSData.vars.lon,&lookAtLat,&lookAtLon,&tempX,&tempY,&tempDist,&yawSetPoint);
      if (sqrt(tempX * tempX + tempY * tempY) < MIN_RTB_DIST){
        yawSetPoint = wpYaw;
      }
    }
    else{
      yawSetPoint = wpYaw;
    }
    newWayPointFlag = true;
  }
} 
void WayPointLookAt(float lat, float lon, boolean lookAt ){
  //called by radio.cpp
  //if lookAt false or too close the craft use yaw from last wp
  float tempX,tempY,tempDist;
  //float yawSetPointWP;
  if (wayPointState != WP_TAKE_OFF){
    lookAtLat = (int32_t)(lat * 10000000);
    lookAtLon = (int32_t)(lon * 10000000);
    lookAtFlag = lookAt;
    if (lookAtFlag == true){
      DistBearing(&GPSData.vars.lat,&GPSData.vars.lon,&lookAtLat,&lookAtLon,&tempX,&tempY,&tempDist,&yawSetPoint);
      if (sqrt(tempX * tempX + tempY * tempY) < MIN_RTB_DIST){
        yawSetPoint = wpYaw;
      } 
    }
    else{
      yawSetPoint = wpYaw;
    }
  }

}
void WayPointTakeOff(){
  //called by motors when operator takes throttle to mid stick
  wayPointState = WP_TAKE_OFF;
  xTarget = XEst;
  yTarget = YEst;

  zTarget = TAKE_OFF_ALT;
  if (zTarget < floorLimit) {
    zTarget = floorLimit;
  }
  if (zTarget > ceilingLimit) {
    zTarget = ceilingLimit;
  }
  wpX = xTarget;
  wpY = yTarget;
  wpZ = zTarget;
  wpYaw = yawInDegrees;
  yawSetPoint = wpYaw;
  newWayPointFlag = false;
}
void WayPointStop(){
  //called by radio.cpp
  //called from ground station
  //sets loiter to current position
  if (wayPointState != WP_TAKE_OFF){
    wayPointState = WP_LOITER;
    xTarget = XEst;
    yTarget = YEst;

    zTarget = ZEstUp;
    newWayPointFlag = false;
  }

}

void WayPointLandGS(){
  //called by radio.cpp
  if (wayPointState != WP_TAKE_OFF){
    motorState = LANDING;
    wayPointState = WP_LAND;
    xTarget = XEst;
    yTarget = YEst;

    zTarget = ZEstUp;

    velSetPointZ = LAND_VEL;
    newWayPointFlag = false;
  }

}
void WayPointLandTX(){
  //called by motors.cpp when stick is lowered
  //should switch flight mode to loiter
  flightMode = L0;

  if (zTarget < floorLimit) {
    zTarget = floorLimit;
  }
  if (zTarget > ceilingLimit) {
    zTarget = ceilingLimit;
  }

  xTarget = XEst;
  yTarget = YEst;
  throttleCheckFlag = false;
  yawSetPoint = yawInDegrees;
  LEDPatternSet(1,0,1,0);

  velSetPointZ = LAND_VEL;
  motorState = LANDING;
  ZLoiterState = LAND;
}
void WayPointReturnToBase(){
  //called by radio.cpp

  if (wayPointState != WP_TAKE_OFF){
    LEDPatternSet(1,0,7,0);
    enterState = false;
    //InitLoiter();
    xTarget = XEst;
    yTarget = YEst;
    zTarget = ZEstUp + 1;
    xFromTO = XEst - homeBaseXOffset;
    yFromTO = YEst - homeBaseYOffset;
    if (sqrt(xFromTO * xFromTO + yFromTO * yFromTO) < MIN_RTB_DIST || gpsFailSafe == true){
      yawSetPoint = initialYaw;
    }
    else{
      yawSetPoint = ToDeg(atan2(yFromTO , xFromTO));
      if (yawSetPoint < 0.0){
        yawSetPoint += 360.0;
      }
    }
    if (zTarget > ceilingLimit) {
      zTarget = ceilingLimit;
    }
    if (zTarget < floorLimit) {
      zTarget = floorLimit;
    }
    RTBState = RTB_CLIMB;
    wayPointState = WP_RTB;
  }

}

void MotorShutDown(){
  TIMSK5 = (0 << OCIE5A);
  Motor1WriteMicros(0);
  Motor2WriteMicros(0);
  Motor3WriteMicros(0);
  Motor4WriteMicros(0);
  Motor5WriteMicros(0);
  Motor6WriteMicros(0);
  Motor7WriteMicros(0);
  Motor8WriteMicros(0);


  while (1) {
    LEDPatternHandler(millis());
  }

}
void FailSafeHandler(){

  if (magDetected == false){
    GPSDetected = false;
    gpsFailSafe = true;
    batteryFSOverride = true;
    if (flightMode > L0){
      flightMode = L0;
      if (flightMode != previousFlightMode) {
        enterState = true;
        previousFlightMode = flightMode;
      }
    }
  }
  if (baroFS == true){
    GPSDetected = false;
    gpsFailSafe = true;
    batteryFSOverride = true;
    if (flightMode > ATT){
      flightMode = ATT;
      if (flightMode != previousFlightMode) {
        enterState = true;
        previousFlightMode = flightMode;
      }
    }
  }



  if (gsCTRL == true){
    if (GSCTRLFailSafe == true){
      gsCTRL = false;
      if (rcDetected == false || RCFailSafe == true){
        LEDPatternSet(0,3,0,1);
        if (txLossRTB == 0){
          LEDPatternSet(0,3,0,1);
          MotorShutDown();
        }
        else{
          if (magDetected == false){
            LEDPatternSet(0,6,0,1);
            MotorShutDown();
          }
          if (baroFS == true){
            LEDPatternSet(0,7,0,1);
            MotorShutDown();
          }
          if (flightMode != RTB) {
            enterState = true;
            flightMode = RTB;
          }
        }

      }


    }

  }
  else{
    if (RCFailSafe == true){
      LEDPatternSet(0,2,0,1);
      if (txLossRTB == 0) {
        LEDPatternSet(0,2,0,1);
        MotorShutDown();
      }
      else{
        if (magDetected == false){
          LEDPatternSet(0,4,0,1);
          MotorShutDown();
        }
        if (baroFS == true){
          LEDPatternSet(0,5,0,1);
          MotorShutDown();
        }
        if (flightMode != RTB) {
          enterState = true;
          flightMode = RTB;
        }
      }
    }
  }

  if (batteryFailSafe == true && batteryFSOverride == false){
    if (txLossRTB == 1){
      if (flightMode != RTB) {
        enterState = true;
        flightMode = RTB;
      }
    }
  }


}
void FlightSM() {
  switch (flightMode) {
  case RATE:
    if (enterState == true) {
      enterState = false;
      if (previousFlightMode != RATE && previousFlightMode != ATT) {
        throttleCheckFlag = true;
      }
      LEDPatternSet(1,6,0,6);
    }
    TrimCheck();
    break;
  case ATT:
    if (enterState == true) {
      if (previousFlightMode != RATE && previousFlightMode != ATT) {
        throttleCheckFlag = true;
      }
      yawSetPoint = yawInDegrees;
      enterState = false;
      LEDPatternSet(1,2,0,0);
    }
    HeadingHold();
    TrimCheck();
    break;
  case L0:
    if (enterState == true) {
      enterState = false;
      InitLoiter();
      yawSetPoint = yawInDegrees;
      LEDPatternSet(1,0,1,0);
    }
    HeadingHold();
    controlBearing = yawInDegrees;
    LoiterSM();
    break;
  case L1:
    if (enterState == true) {
      enterState = false;
      InitLoiter();
      yawSetPoint = yawInDegrees;
      LEDPatternSet(1,0,2,0);
      controlBearing = initialYaw;
    }
    HeadingHold();
    LoiterSM();
    break;
  case L2:
    if (enterState == true) {
      InitLoiter();
      yawSetPoint = yawInDegrees;
      LEDPatternSet(1,0,3,0);
      enterState = false;
    }
    HeadingHold();
    controlBearing = headingToCraft;
    LoiterSM();
    break;
  case FOLLOW:
    /*if (enterState == true) {
     LEDPatternSet(1,0,4,0);
     enterState = false;
     yawSetPoint = yawInDegrees;
     }
     break;*/
  case WP:

    if (enterState == true) {
      enterState = false;
      yawSetPoint = yawInDegrees;
      LEDPatternSet(1,0,5,0);
    }
    WayPointStateMachine();
    break;
  case RTB:
    if (enterState == true) {
      LEDPatternSet(1,0,7,0);
      enterState = false;
      InitLoiter();
      xTarget = XEst;
      yTarget = YEst;
      zTarget = ZEstUp + 1;
      xFromTO = XEst - homeBaseXOffset;
      yFromTO = YEst - homeBaseYOffset;
      if (sqrt(xFromTO * xFromTO + yFromTO * yFromTO) < MIN_RTB_DIST || gpsFailSafe == true){
        yawSetPoint = initialYaw;
      }
      else{
        yawSetPoint = ToDeg(atan2(yFromTO , xFromTO));
        if (yawSetPoint < 0.0){
          yawSetPoint += 360.0;
        }
      }
      if (zTarget > ceilingLimit) {
        zTarget = ceilingLimit;
      }
      if (zTarget < floorLimit) {
        zTarget = floorLimit;
      }
      RTBState = RTB_CLIMB;
    }
    HeadingHold();
    RTBStateMachine();
    break;
  }
}

void TrimCheck() {
  uint8_t j;
  float_u outFloat;
  if (setTrim == true) {
    if (trimComplete == false) {
      trimComplete = true;
      pitchOffset = rawPitch;
      rollOffset = rawRoll;
      j = 0;
      outFloat.val = pitchOffset;
      for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
        EEPROMWrite(i, outFloat.buffer[j++]);
      }
      j = 0;
      outFloat.val = rollOffset;
      for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
        EEPROMWrite(i, outFloat.buffer[j++]);
      }
      EEPROMWrite(PR_FLAG, 0xAA);
    }
  }
}

void InitLoiter() {
  if (previousFlightMode != L1 && previousFlightMode != L2 && previousFlightMode != L0) {
    if (motorState == LANDING) {
      velSetPointZ = LAND_VEL;
    }
    else{
      zTarget = ZEstUp;
    }
    if (zTarget < floorLimit) {
      zTarget = floorLimit;
    }
    if (zTarget > ceilingLimit) {
      zTarget = ceilingLimit;
    }
    throttleCheckFlag = true;

    xTarget = XEst;
    yTarget = YEst;

    LoiterXPosition.reset();
    LoiterXVelocity.reset();
    LoiterYPosition.reset();
    LoiterYVelocity.reset();
    AltHoldPosition.reset();
    AltHoldVelocity.reset();
    if (motorState >= FLIGHT){
      AltHoldVelocity.iError = throCommand - propIdleCommand;
    }
  }

}

void RTBStateMachine() {
  static boolean startSlowing = false;
  static float initialRTBVel,commandedRTBVel;
  if (motorState == HOLD || motorState == TO) {
    return;
  }
  if (lowRateTasks == true){
    switch (RTBState) {
    case RTB_CLIMB:
      if(motorState == LANDING){
        motorState = FLIGHT;
      }
      LoiterCalculations();
      Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      if (ZEstUp >= (zTarget - 0.1) ) {
        RTBState = RTB_TRAVEL;
        startSlowing = false;
      }
      if (gpsFailSafe == true) {
        velSetPointZ = LAND_VEL;
        RTBState = RTB_LAND;
        motorState = LANDING;
      }

      break;

    case RTB_TRAVEL:
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      //within min radius switch to lioter 
      xFromTO = XEst - homeBaseXOffset;
      yFromTO = YEst - homeBaseYOffset;
      distToWayPoint = sqrt(xFromTO * xFromTO + yFromTO * yFromTO);
      if (distToWayPoint < MIN_RTB_DIST){
        RTBState = RTB_LOITER;
        xTarget = homeBaseXOffset;
        yTarget = homeBaseYOffset;
        startSlowing = false;
        break;
      }


      //calculate distance and heading to origin

      headingToWayPoint = ToDeg(atan2(-1.0 * yFromTO , -1.0 * xFromTO));
      if (headingToWayPoint < 0.0){
        headingToWayPoint += 360.0;
      }
      yawSetPoint = headingToWayPoint - 180.0;
      if(yawSetPoint < 0){
        yawSetPoint +=360;
      }
      //rotate EF velocity to path frame
      Rotate2dVector(&zero,&headingToWayPoint,&velX,&velY,&wpPathVelocity,&wpCrossTrackVelocity);
      //wp pos PID
      WPPosition.calculate(); 
      wpVelSetPoint *= -1.0;
      //wp vel PID
      if (distToWayPoint < LOW_SPEED_RADIUS && startSlowing == false){
        startSlowing = true;
        initialRTBVel = wpVelSetPoint;
        commandedRTBVel = wpVelSetPoint;
      }
      if (startSlowing == true){
        commandedRTBVel = commandedRTBVel * RAMP_DOWN_ALPHA + (1 - RAMP_DOWN_ALPHA) * RAMP_DOWN_VEL_RTB;
        wpVelSetPoint = commandedRTBVel;
      }
      WPVelocity.calculate(); 
      wpTilX *= -1.0;
      //cross track vel PID
      WPCrossTrack.calculate(); 
      //rotate wp tilx and tilty to body pitch and roll
      Rotate2dVector(&yawInDegrees,&headingToWayPoint,&wpTilX,&wpTiltY,&pitchSetPoint,&rollSetPoint);
      if (gpsFailSafe == true) {
        velSetPointZ = LAND_VEL;
        RTBState = RTB_LAND;
        motorState = LANDING;
        startSlowing = false;
      }
      break;
    case RTB_LOITER:

      LoiterCalculations();
      Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      if ( (fabs(XEst - xTarget) < 1.0 && fabs(YEst - yTarget) < 1.0) || gpsFailSafe == true) {
        velSetPointZ = LAND_VEL;
        RTBState = RTB_LAND;
        motorState = LANDING;
      }

      break;
    case RTB_LAND:
      if (gpsFailSafe == true) {
        pitchSetPoint = pitchSetPointTX;
        rollSetPoint = rollSetPointTX;

        if (RCFailSafe == true) {
          pitchSetPoint = 0;
          rollSetPoint = 0;
        }
      }
      else {
        LoiterCalculations();
        Rotate2dVector(&yawInDegrees, &zero, &tiltAngleX, &tiltAngleY, &pitchSetPoint, &rollSetPoint);
      }
      if (motorState == FLIGHT){
        motorState = LANDING;
      }
      AltHoldVelocity.calculate();
      break;
    }
  }

}

void LoiterCalculations() {
  static boolean resetPositionFlag = false;
  LoiterXPosition.calculate();
  LoiterYPosition.calculate();
#ifdef AUX3_VEL
  if (RCValue[AUX3] > 1750) {
    velSetPointY = STEP_VEL;
    resetPositionFlag = true;
  }
  if (RCValue[AUX3] > 1400 && RCValue[AUX3] < 1600) {
    velSetPointY = 0;
    resetPositionFlag = true;
  }
  if (resetPositionFlag == true){
    resetPositionFlag = false;
    yTarget = YEst;
  }
#endif  
  LoiterXVelocity.calculate();
  tiltAngleX *= -1.0;
  LoiterYVelocity.calculate();
}

//move the rudder to the right to start calibration
void StartCalibration(){
  //calibration procedure
  LEDPatternSet(0,7,0,0);
  newRC = false;
  newGSRC = false;
  if(gsCTRL == false){
    while (newRC == false){
    }
  }
  else{
    while (newGSRC == false){
      Radio();
    }
  }
  newRC = false;

  if (gsCTRL == true){
    while (GSRCValue[RUDD] < 1750){
      Radio();
    } 
  }
  else{
    while (RCValue[RUDD] < 1750){
      if (newRC == true){
        ProcessChannels();
        newRC = false;
      }
    } 
  }


}

void LoiterSM(){
  float absVelX,absVelY;
  float limitedPitch,limitedRoll;
  float maxPitch,maxRoll;
  int16_t rcDifference;
  static boolean limitX = false,limitY = false;
  if (lowRateTasks == true){
    switch(ZLoiterState){
    case LOITERING:
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      if (abs(throCommand - 1500) > 200 && throttleCheckFlag == false){
        ZLoiterState = RCINPUT;
      }
      if (motorState == LANDING){
        zTarget = ZEstUp;
        motorState = FLIGHT;
      }
      if (zTarget < floorLimit) {
        zTarget = floorLimit;
      }
      if (zTarget > ceilingLimit) {
        zTarget = ceilingLimit;
      }
      if (throCommand < 1050 && motorState == FLIGHT){
        throttleCheckFlag = false;
        ZLoiterState = LAND;
        motorState = LANDING;
        velSetPointZ = LAND_VEL;
      }
      break;
    case RCINPUT:
      if (throttleCheckFlag == true){
        ZLoiterState = LOITERING;
        break;
      }
      rcDifference = throCommand - 1500;
      if (abs(rcDifference) < 200){
        ZLoiterState = LOITERING;
        zTarget = ZEstUp;
        if (zTarget <= floorLimit){
          zTarget = floorLimit;
        } 
        if (zTarget >= ceilingLimit){
          zTarget = ceilingLimit;
        }
        AltHoldPosition.calculate();
        AltHoldVelocity.calculate();
        break;
      }
      velSetPointZ = rcDifference * 0.0034;
      if (velSetPointZ > MAX_Z_RATE){
        velSetPointZ = MAX_Z_RATE;
      }
      if (velSetPointZ < MIN_Z_RATE){
        velSetPointZ = MIN_Z_RATE;
      }
      if (throCommand < 1050 && motorState == FLIGHT){
        throttleCheckFlag = false;
        ZLoiterState = LAND;
        motorState = LANDING;
        velSetPointZ = LAND_VEL;
        break;
      }


      if (ZEstUp >= ceilingLimit && velSetPointZ > 0){
        throttleCheckFlag = true;
        zTarget = ceilingLimit;
        AltHoldPosition.calculate();
        AltHoldVelocity.calculate();
        break;
      }
      if (ZEstUp <= floorLimit && velSetPointZ < 0){
        throttleCheckFlag = true;
        zTarget = floorLimit;
        AltHoldPosition.calculate();
        AltHoldVelocity.calculate();
        break;
      }

      AltHoldVelocity.calculate();
      break;


    case LAND:
      if (motorState == FLIGHT){
        motorState = LANDING;
      }
      AltHoldVelocity.calculate();

      if (throCommand > 1200 && motorState == LANDING){
        ZLoiterState = LOITERING;
        motorState = FLIGHT;
        zTarget = ZEstUp;
        if (zTarget <= floorLimit){
          zTarget = floorLimit;
        } 
        if (zTarget >= ceilingLimit){
          zTarget = ceilingLimit;
        }
        throttleCheckFlag = true;

      }
      break;
    }
    if (gpsFailSafe == false && GPSDetected == true){
      switch(XYLoiterState){
      case LOITERING:
        LoiterCalculations();
        Rotate2dVector(&yawInDegrees,&zero,&tiltAngleX,&tiltAngleY,&pitchSetPoint,&rollSetPoint);
        if (fabs(rollSetPointTX) > 0.5 || fabs(pitchSetPointTX) > 0.5){
          XYLoiterState = RCINPUT;
        }
        break;
      case RCINPUT:
        //              end angle      start angle      x in            y in            x out            y out
        Rotate2dVector(&yawInDegrees,&controlBearing,&pitchSetPointTX,&rollSetPointTX,&pitchSetPoint,&rollSetPoint);
        Rotate2dVector(&zero,&yawInDegrees,&velX,&velY,&bodyVelX,&bodyVelY);
        if (pitchSetPoint < 0.0){
          maxPitch = -1.0 * (4.0 - bodyVelX) * 7.0;
          if (maxPitch > 0){
            maxPitch = 0;
          }
          if (pitchSetPoint < maxPitch){
            pitchSetPoint = maxPitch;
          }
        }
        if (pitchSetPoint > 0.0){
          maxPitch = -1.0 * (-4.0 - bodyVelX) * 7.0;
          if (maxPitch < 0){
            maxPitch = 0.0;
          }
          if (pitchSetPoint > maxPitch){
            pitchSetPoint = maxPitch;
          }
        }
        if (rollSetPoint > 0.0){
          maxRoll = 7.0 * (4.0 - bodyVelY);
          if (maxRoll < 0){
            maxRoll = 0.0;
          }
          if (rollSetPoint > maxRoll){
            rollSetPoint = maxRoll;
          }
        }
        if (rollSetPoint < 0.0){
          maxRoll = 7.0 * (-4.0 - bodyVelY);
          if (maxRoll > 0){
            maxRoll = 0.0;
          }
          if (rollSetPoint < maxRoll){
            rollSetPoint = maxRoll;
          }
        }
        if (fabs(rollSetPointTX) < 0.5 && fabs(pitchSetPointTX) < 0.5){
          XYLoiterState = WAIT;
          velSetPointX = velX;
          velSetPointY = velY;
        }

        break;
      case WAIT:
        if (fabs(rollSetPointTX) > 0.5 || fabs(pitchSetPointTX) > 0.5){
          XYLoiterState = RCINPUT;
          break;
        }
        velSetPointX *= RAMP_DOWN_ALPHA;
        velSetPointY *= RAMP_DOWN_ALPHA;
        LoiterXVelocity.calculate();
        tiltAngleX *= -1.0;
        LoiterYVelocity.calculate();
        Rotate2dVector(&yawInDegrees,&zero,&tiltAngleX,&tiltAngleY,&pitchSetPoint,&rollSetPoint);
        if (fabs(velSetPointX) < LOIT_RAMP_MIN && fabs(velSetPointY) < LOIT_RAMP_MIN){
          XYLoiterState = LOITERING;
          xTarget = XEst;
          yTarget = YEst;
        }
        break;

      }  
    }
    else{
      if (flightMode == L2){//check
        controlBearing = initialYaw;
      }
      Rotate2dVector(&yawInDegrees,&controlBearing,&pitchSetPointTX,&rollSetPointTX,&pitchSetPoint,&rollSetPoint);
    }
  }
}

void Rotate2dVector(float *currentBearing, float *initialBearing, float *xStart, float *yStart, float *xEnd, float *yEnd){
  //uses counter clockwise rotation
  //that is why for rotating the velocity for way points the start and end bearings must be switched
  float angleDifference;
  float sinDiff;
  float cosDiff;
  angleDifference = *currentBearing - *initialBearing;
  sinDiff = sin(ToRad(angleDifference));
  cosDiff = cos(ToRad(angleDifference));
  *xEnd = *xStart * cosDiff + -1.0 * *yStart * sinDiff;
  *yEnd = *xStart * sinDiff + *yStart * cosDiff;  
}



void HeadingHold(){
  if (magDetected == true){
    switch (HHState){
    case HH_ON:
      calcYaw = true;
      if (fabs(yawInput) > 1){
        HHState = HH_OFF;
      }
      break;
    case HH_OFF:
      calcYaw = false;
      rateSetPointZ = yawInput;
      if (fabs(yawInput) < 1){
        yawSetPoint = yawInDegrees;
        HHState = HH_ON;
      }
      break;
    default:
      HHState = HH_OFF;

      break;
    }


  }
  else{
    rateSetPointZ = yawInput;
    calcYaw = false;
  }
}

void ProcessChannels() {
  static uint8_t clearTXRTB;
  for (uint8_t i = 0; i < 8; i++)  {
    switch (rcData[i].chan) {
    case THRO:
      if (rcData[i].reverse == 0) {
        RCValue[THRO] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[THRO] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }
      break;
    case AILE:
      if (rcData[i].reverse == 0) {
        RCValue[AILE] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[AILE] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case ELEV:
      if (rcData[i].reverse == 0) {
        RCValue[ELEV] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[ELEV] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }

      break;
    case RUDD:
      if (rcData[i].reverse == 0) {
        RCValue[RUDD] = (rcData[i].rcvd - rcData[i].mid) * rcData[i].scale + 1500;
      }
      else {
        RCValue[RUDD] = (rcData[i].rcvd - rcData[i].mid) * - rcData[i].scale + 1500;
      }
      break;
    case GEAR:
      if (rcData[i].reverse == 0) {
        RCValue[GEAR] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[GEAR] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX1:
      if (rcData[i].reverse == 0) {
        RCValue[AUX1] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX1] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX2:
      if (rcData[i].reverse == 0) {
        RCValue[AUX2] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX2] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;
    case AUX3:
      if (rcData[i].reverse == 0) {
        RCValue[AUX3] = (rcData[i].rcvd - rcData[i].min) * rcData[i].scale + 1000;
      }
      else {
        RCValue[AUX3] = (rcData[i].rcvd - rcData[i].min) * - rcData[i].scale + 2000;
      }

      break;


    }
  }

#ifdef AUX3_FS_TESTS
  static boolean functionCallFlag = false;
  static uint8_t FSDebugState = 0;
  if (RCValue[AUX3] > 1750) {
    if (functionCallFlag == false){
      functionCallFlag = true;
      switch (FSDebugState){
      case 0:
        magDetected = false;
        //batteryFSOverride = false;
        //batteryFailSafe = true;
        //RCFailSafe = true;
        FSDebugState = 1;
        break;
      case 1:
        //gpsFailSafe = true;
        baroFS = true;
        //gpsFailSafe = true;

        FSDebugState = 2;
        break;
      case 2:
        //gpsFailSafe = true;
        //baroFS = true;
        FSDebugState = 3;
        break;
      case 3:

        FSDebugState = 4;
        break;
      case 4:
        FSDebugState = 0;
        break;
      }
    }
  }
  else{
    functionCallFlag = false;
  }
#endif

  if (RCFailSafe == true) {

    switch (clearTXRTB) {

    case 0:
      if (RCValue[GEAR] > 1850) {
        clearTXRTB = 1;
      }
      return;
      break;

    case 1:
      if (RCValue[GEAR] < 1150) {
        RCFailSafe = false;
        RCFailSafeCounter = 0;
        clearTXRTB = 0;
        break;
      }
      return;
      break;
    default:
      clearTXRTB = 0;
      return;
      break;

    }


  }
  else {
    GetSwitchPositions();
    if (gsCTRL == false){
      ProcessModes();
    }
  }




}

void ProcessModes() {
  static uint8_t clearBATTRTB=0;
  //uint8_t flightModeControl=0;
  previousFlightMode = flightMode;

  if (RCValue[AUX2] > 1750) {
    if (batteryFailSafe == true){
      batteryFSOverride = true;
    }
    gsCTRL = false;
    setTrim = false;
    trimComplete = false;
    flightMode = RATE;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdElev, &rateSetPointY, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ, 1000, 2000, -400, 400);
    if (rateSetPointY < 5 && rateSetPointY > -5) {
      rateSetPointY = 0;
    }
    if (rateSetPointX < 5 && rateSetPointX > -5) {
      rateSetPointX = 0;
    }
    if (rateSetPointZ < 5 && rateSetPointZ > -5) {
      rateSetPointZ = 0;
    }


    if (flightMode != previousFlightMode) {
      enterState = true;
    }

    return;

  }
  if (RCValue[AUX2] > 1400 && RCValue[AUX2] < 1600) {
    if (batteryFailSafe == true){
      batteryFSOverride = true;
    }
    gsCTRL = false;
    setTrim = false;
    trimComplete = false;
    flightMode = ATT;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO]; 

    MapVar(&cmdElev, &pitchSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint < 1 && rollSetPoint > -1) {
      rollSetPoint = 0;
    }
    if (pitchSetPoint < 1 && pitchSetPoint > -1) {
      pitchSetPoint = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }


    if (flightMode != previousFlightMode) {
      enterState = true;
    }

    return;

  }

  /*if (RCValue[AUX3] > 1750) {
   if (batteryFailSafe == true){
   batteryFSOverride = true;
   }
   gsCTRL = false;
   setTrim = false;
   trimComplete = false;
   flightMode = RTB;
   cmdElev = RCValue[ELEV];
   cmdAile = RCValue[AILE];
   cmdRudd = RCValue[RUDD];
   throCommand = RCValue[THRO];
   MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -60, 60);
   MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -60, 60);
   MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
   if (rollSetPointTX < 1 && rollSetPointTX > -1) {
   rollSetPointTX = 0;
   }
   if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
   pitchSetPointTX = 0;
   }
   if (yawInput < 5 && yawInput > -5) {
   yawInput = 0;
   }
   if (flightMode != previousFlightMode) {
   enterState = true;
   }
   return;
   
   }*/

#ifdef AUX3_WP_DEBUG
  static boolean functionCallFlag = false;
  static uint8_t WPDebugState = 0;
  if (RCValue[AUX3] > 1750) {
    if (functionCallFlag == false){
      functionCallFlag = true;
      //WayPointStop();
      //WayPointLandGS();
      //WayPointReturnToBase();


      /*switch (WPDebugState){
       case 0:
       //WayPointUpdate(32.935153,-96.774441,  50, 0);
       WayPointLandGS();
       Serial<<"WPDebugState = 0\r\n";
       WPDebugState = 1;
       break;
       
       case 1:
       WayPointUpdate(32.935153,-96.774441,  50, 0);
       //WayPointLookAt(32.934577,-96.773636,true);
       
       Serial<<"WPDebugState = 1\r\n";
       WPDebugState = 2;
       break;
       case 2:
       //WayPointLookAt(32.934577,-96.773636,false);
       WayPointStop();
       Serial<<"WPDebugState = 2\r\n";
       WPDebugState = 3;
       break;
       case 3:
       //WayPointLookAt(32.934577,-96.773636,false);
       WayPointReturnToBase();
       Serial<<"WPDebugState = 3\r\n";
       WPDebugState = 4;
       break;
       case 4:
       //WayPointLookAt(32.934577,-96.773636,false);
       WayPointStop();
       Serial<<"WPDebugState = 4\r\n";
       WPDebugState = 0;
       break;
       }*/
      /*void WayPointUpdate(float lat, float lon, float alt, float yaw);
       void WayPointLookAt(float lat, float lon, boolean lookAt );
       void WayPointStateMachine();
       
       void WayPointStop();
       void WayPointLandGS();
       void WayPointReturnToBase();
       */
    }
  }
  else{
    functionCallFlag = false;
  }
#endif
  static uint32_t stepTimer;
  static boolean resetBias = false;
#ifdef AUX3_RESET_BIAS  
  if (RCValue[AUX3] > 1750) {
    if (resetBias == false){
      resetBias = true;
      accelBiasX = 0;
      accelBiasY = 0;
      accelBiasZ = 0;
    }
  }
  else{
    resetBias = false;
  }
#endif
#ifdef AUX3_FORCE_ATT_RESET  
  if (RCValue[AUX3] > 1750) {
    if (resetTriggered == false){
      startReset = true;
      resetTriggered = true;
    }
  }
  else{
    startReset = false;
    resetTriggered = false;
  } 
#endif
#ifdef AUX3_RTB
  if (RCValue[AUX3] > 1750) {
    if (batteryFailSafe == true){
      batteryFSOverride = true;
    }
    gsCTRL = false;
    setTrim = false;
    trimComplete = false;
    flightMode = RTB;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX < 1 && rollSetPointTX > -1) {
      rollSetPointTX = 0;
    }
    if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
      pitchSetPointTX = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    return;

  }
#endif
#ifdef AUX3_RATEX
  if (RCValue[AUX3] > 1750) {
    if (stepStart == false){
      executeStep = true;
      stepTimer = millis();
    }
    flightMode = RATE;
    setTrim = false;
    trimComplete = false;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdElev, &rateSetPointY, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ, 1000, 2000, -400, 400);

    if (rateSetPointY < 5 && rateSetPointY > -5) {
      rateSetPointY = 0;
    }

    if (executeStep == true){
      stepStart = true;
      rateSetPointX = STEP_RATE;
      if (millis() - stepTimer > STEP_DURATION){
        executeStep = false;
      }
    }
    else{
      if (rateSetPointX < 5 && rateSetPointX > -5) {
        rateSetPointX = 0;
      }
    }
    if (rateSetPointZ < 5 && rateSetPointZ > -5) {
      rateSetPointZ = 0;
    }

    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    return;

  }
  executeStep = false;
  stepStart = false;
#endif
#ifdef AUX3_ROLL
  if (RCValue[AUX3] > 1750) {
    if (stepStart == false){
      executeStep = true;
      stepTimer = millis();
    }
    flightMode = ATT;
    setTrim = false;
    trimComplete = false;
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    MapVar(&cmdElev, &pitchSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    rollSetPoint = STEP_ATT;
    if (pitchSetPoint < 1 && pitchSetPoint > -1) {
      pitchSetPoint = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }

    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    return;

  }
  executeStep = false;
  stepStart = false;
#endif
#ifdef AUX3_POS
  if (RCValue[AUX3] > 1750) {
    if (stepStart == true){
      stepStart = false;
      yTarget += STEP_DIST;
    }
  }
  else{
    stepStart = true;
  }

#endif
  if (txLossRTB == 1 && batteryFailSafe == true && batteryFSOverride == false){
    if (gsCTRL == false){
      switch (clearBATTRTB) {

      case 0:
        if (RCValue[GEAR] > 1850) {
          clearBATTRTB = 1;
        }
        return;
        break;

      case 1:
        if (RCValue[GEAR] < 1150) {
          batteryFSOverride = true;
          clearBATTRTB = 0;
          break;
        }
        return;
        break;
      default:
        clearBATTRTB = 0;
        return;
        break;

      }
    }
    else{
      if (GSRCValue[AUX1] > 0){
        batteryFSOverride = true;
      }
    }
    return;
  }



  if (gsCTRL == false){
    cmdElev = RCValue[ELEV];
    cmdAile = RCValue[AILE];
    cmdRudd = RCValue[RUDD];
    throCommand = RCValue[THRO];
    flightModeControl = modeArray[switchPositions];
  }
  else{
    cmdElev = GSRCValue[ELEV];
    cmdAile = GSRCValue[AILE];
    cmdRudd = GSRCValue[RUDD];
    throCommand = GSRCValue[THRO];
    flightModeControl = GSRCValue[GEAR];
    if (flightModeControl < 0){
      flightModeControl = 0;
    }
    if (flightModeControl > 9){
      flightModeControl = 2;
    }
  }  

  if (telemFS == true && (flightModeControl == FOLLOW || flightModeControl == WP )){
    flightModeControl = L0;
  }

  if (gsCTRL == true && flightModeControl > L2){
    flightModeControl = L0;
  }
  switch(flightModeControl){
  case RATE:
    flightMode = RATE;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &rateSetPointY, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ, 1000, 2000, -400, 400);
    if (rateSetPointY < 5 && rateSetPointY > -5) {
      rateSetPointY = 0;
    }
    if (rateSetPointX < 5 && rateSetPointX > -5) {
      rateSetPointX = 0;
    }
    if (rateSetPointZ < 5 && rateSetPointZ > -5) {
      rateSetPointZ = 0;
    }
    break;
  case RATE_TRIM:
    setTrim = true;
    flightMode = RATE;
    MapVar(&cmdElev, &rateSetPointY, 1000, 2000, -400, 400);
    MapVar(&cmdAile, &rateSetPointX, 1000, 2000, -400, 400);
    MapVar(&cmdRudd, &rateSetPointZ, 1000, 2000, -400, 400);
    if (rateSetPointY < 5 && rateSetPointY > -5) {
      rateSetPointY = 0;
    }
    if (rateSetPointX < 5 && rateSetPointX > -5) {
      rateSetPointX = 0;
    }
    if (rateSetPointZ < 5 && rateSetPointZ > -5) {
      rateSetPointZ = 0;
    }
    break;
  case ATT:
    flightMode = ATT;
    setTrim = false;
    trimComplete = false;
    MapVar(&cmdElev, &pitchSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint < 1 && rollSetPoint > -1) {
      rollSetPoint = 0;
    }
    if (pitchSetPoint < 1 && pitchSetPoint > -1) {
      pitchSetPoint = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case ATT_TRIM:
    flightMode = ATT;
    setTrim = true;
    MapVar(&cmdElev, &pitchSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdAile, &rollSetPoint, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPoint < 1 && rollSetPoint > -1) {
      rollSetPoint = 0;
    }
    if (pitchSetPoint < 1 && pitchSetPoint > -1) {
      pitchSetPoint = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L0:
    flightMode = L0;
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, LOIT_YAW_MIN, LOIT_YAW_MAX);
    if (rollSetPointTX < 1 && rollSetPointTX > -1) {
      rollSetPointTX = 0;
    }
    if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
      pitchSetPointTX = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L1:
    flightMode = L1;
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, LOIT_YAW_MIN, LOIT_YAW_MAX);
    if (rollSetPointTX < 1 && rollSetPointTX > -1) {
      rollSetPointTX = 0;
    }
    if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
      pitchSetPointTX = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    break;
  case L2:
    flightMode = L2;
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, LOIT_YAW_MIN, LOIT_YAW_MAX);
    if (rollSetPointTX < 1 && rollSetPointTX > -1) {
      rollSetPointTX = 0;
    }
    if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
      pitchSetPointTX = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    if (gpsFailSafe == true) {
      flightMode = L1;

    }
    break;
  case FOLLOW: //TBD impliment FOLLOW and WP modes currently operate as ATT
  case WP:
    if (gpsFailSafe == true) {
      flightMode = L0;
      MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
      MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
      MapVar(&cmdRudd, &yawInput, 1000, 2000, LOIT_YAW_MIN, LOIT_YAW_MAX);
      if (rollSetPointTX < 1 && rollSetPointTX > -1) {
        rollSetPointTX = 0;
      }
      if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
        pitchSetPointTX = 0;
      }
      if (yawInput < 5 && yawInput > -5) {
        yawInput = 0;
      }
      break;
    }
    if (motorState == HOLD){
      flightMode = WP;
      //setTrim = false;
      //trimComplete = false;
      MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -60, 60);
      MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -60, 60);
      MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
      if (rollSetPointTX < 1 && rollSetPointTX > -1) {
        rollSetPointTX = 0;
      }
      if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
        pitchSetPointTX = 0;
      }
      if (yawInput < 5 && yawInput > -5) {
        yawInput = 0;
      }
    }
    else{
      if (flightMode == WP){
        //setTrim = false;
        //trimComplete = false;
        MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -60, 60);
        MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -60, 60);
        MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
        if (rollSetPointTX < 1 && rollSetPointTX > -1) {
          rollSetPointTX = 0;
        }
        if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
          pitchSetPointTX = 0;
        }
        if (yawInput < 5 && yawInput > -5) {
          yawInput = 0;
        }
      }
      else{
        flightMode = L0;
        MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
        MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, LOIT_TILT_MIN, LOIT_TILT_MAX);
        MapVar(&cmdRudd, &yawInput, 1000, 2000, LOIT_YAW_MIN, LOIT_YAW_MAX);
        if (rollSetPointTX < 1 && rollSetPointTX > -1) {
          rollSetPointTX = 0;
        }
        if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
          pitchSetPointTX = 0;
        }
        if (yawInput < 5 && yawInput > -5) {
          yawInput = 0;
        }
      }
    }

    break;
  case RTB:
    flightMode = RTB;
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -60, 60);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -60, 60);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -300, 300);
    if (rollSetPointTX < 1 && rollSetPointTX > -1) {
      rollSetPointTX = 0;
    }
    if (pitchSetPointTX < 1 && pitchSetPointTX > -1) {
      pitchSetPointTX = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
    /*if (flightMode != previousFlightMode) {
     enterState = true;
     }*/
    break;

  }

  if (flightMode != previousFlightMode) {
    enterState = true;
  }
}















































