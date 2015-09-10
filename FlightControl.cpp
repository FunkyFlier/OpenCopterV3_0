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


float zero = 0;



uint8_t flightMode = 0;
uint32_t _100HzTimer,_400HzTimer;
volatile uint32_t RCFailSafeCounter=0,watchDogFailSafeCounter=0,groundFSCount=0;
float initialYaw;
boolean integrate;
uint8_t HHState;
float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
uint8_t XYLoiterState, ZLoiterState,RTBState,txLossRTB,previousFlightMode;
boolean telemFailSafe,txFailSafe,tuningTrasnmitOK;

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

uint8_t modeArray[9] = {
  RATE,L1,ATT,ATT,ATT,ATT_TRIM,RATE,RATE,L0};

float kp_pitch_rate;//52
float ki_pitch_rate;//56
float kd_pitch_rate;//60
float fc_pitch_rate;//64

float kp_roll_rate;//68
float ki_roll_rate;//72
float kd_roll_rate;//76
float fc_roll_rate;//80

float kp_yaw_rate;//84
float ki_yaw_rate;//88
float kd_yaw_rate;//92
float fc_yaw_rate;//96

float kp_pitch_attitude;//100
float ki_pitch_attitude;//104
float kd_pitch_attitude;//108
float fc_pitch_attitude;//112

float kp_roll_attitude;//116
float ki_roll_attitude;//120
float kd_roll_attitude;//124
float fc_roll_attitude;//128

float kp_yaw_attitude;//132
float ki_yaw_attitude;//136
float kd_yaw_attitude;//140
float fc_yaw_attitude;//144

float kp_altitude_position;//148
float ki_altitude_position;//152
float kd_altitude_position;//156
float fc_altitude_position;//160

float kp_altitude_velocity;//164
float ki_altitude_velocity;//168
float kd_altitude_velocity;//172
float fc_altitude_velocity;///176
//float mul_altitude_velocity;

float kp_loiter_pos_x;//180
float ki_loiter_pos_x;//184
float kd_loiter_pos_x;//188
float fc_loiter_pos_x;//192

float kp_loiter_velocity_x;//196
float ki_loiter_velocity_x;//200
float kd_loiter_velocity_x;//204
float fc_loiter_velocity_x;//208

float kp_loiter_pos_y;//212
float ki_loiter_pos_y;//216
float kd_loiter_pos_y;//220
float fc_loiter_pos_y;//224

float kp_loiter_velocity_y;//228
float ki_loiter_velocity_y;//232
float kd_loiter_velocity_y;//236
float fc_loiter_velocity_y;//240

float kp_waypoint_position;//244
float ki_waypoint_position;//248
float kd_waypoint_position;//252
float fc_waypoint_position;//256

float kp_waypoint_velocity;//260
float ki_waypoint_velocity;//264
float kd_waypoint_velocity;//268
float fc_waypoint_velocity;//272

float kp_cross_track;//276
float ki_cross_track;//280
float kd_cross_track;//284
float fc_cross_track;//288

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

PID PitchRate(&rateSetPointY, &degreeGyroY, &adjustmentY, &integrate, &kp_pitch_rate, &ki_pitch_rate, &kd_pitch_rate, &fc_pitch_rate, &_100HzDt, 400, 400);
PID RollRate(&rateSetPointX, &degreeGyroX, &adjustmentX, &integrate, &kp_roll_rate, &ki_roll_rate, &kd_roll_rate, &fc_roll_rate, &_100HzDt, 400, 400);
PID YawRate(&rateSetPointZ, &degreeGyroZ, &adjustmentZ, &integrate, &kp_yaw_rate, &ki_yaw_rate, &kd_yaw_rate, &fc_yaw_rate, &_100HzDt, 400, 400);

PID_2 PitchAngle(&pitchSetPoint, &pitchInDegrees, &rateSetPointY, &integrate, &kp_pitch_attitude, &ki_pitch_attitude, &kd_pitch_attitude, &fc_pitch_attitude, &_100HzDt, 360, 360);
PID_2 RollAngle(&rollSetPoint, &rollInDegrees, &rateSetPointX, &integrate, &kp_roll_attitude, &ki_roll_attitude, &kd_roll_attitude, &fc_roll_attitude, &_100HzDt, 360, 360);
YAW_2 YawAngle(&yawSetPoint, &yawInDegrees, &rateSetPointZ, &integrate, &kp_yaw_attitude, &ki_yaw_attitude, &kd_yaw_attitude, &fc_yaw_attitude, &_100HzDt, 360, 360);

PID_2 LoiterXPosition(&xTarget, &XEst, &velSetPointX, &integrate, &kp_loiter_pos_x, &ki_loiter_pos_x, &kd_loiter_pos_x, &fc_loiter_pos_x, &_100HzDt, 1, 1);
PID_2 LoiterXVelocity(&velSetPointX, &velX, &tiltAngleX, &integrate, &kp_loiter_velocity_x, &ki_loiter_velocity_x, &kd_loiter_velocity_x, &fc_loiter_velocity_x, &_100HzDt, 15, 15);

PID_2 LoiterYPosition(&yTarget, &YEst, &velSetPointY, &integrate, &kp_loiter_pos_y, &ki_loiter_pos_y, &kd_loiter_pos_y, &fc_loiter_pos_y, &_100HzDt, 1, 1);
PID_2 LoiterYVelocity(&velSetPointY, &velY, &tiltAngleY, &integrate, &kp_loiter_velocity_y, &ki_loiter_velocity_y, &kd_loiter_velocity_y, &fc_loiter_velocity_y, &_100HzDt, 15, 15);

PID_2 AltHoldPosition(&zTarget, &ZEstUp, &velSetPointZ, &integrate, &kp_altitude_position, &ki_altitude_position, &kd_altitude_position, &fc_altitude_position, &_100HzDt, 1.5, 1.5);
PID_2 AltHoldVelocity(&velSetPointZ, &velZUp, &throttleAdjustment, &integrate, &kp_altitude_velocity, &ki_altitude_velocity, &kd_altitude_velocity, &fc_altitude_velocity, &_100HzDt, 1000, 1000);

PID_2 WPPosition(&zero, &distToWayPoint, &wpVelSetPoint, &integrate, &kp_waypoint_position, &ki_waypoint_velocity, &kd_waypoint_velocity, &fc_waypoint_velocity, &_100HzDt, 5,5);
PID_2 WPVelocity(&wpVelSetPoint, &wpPathVelocity,&wpTilX, &integrate, &kp_waypoint_velocity, &ki_waypoint_velocity, &kd_waypoint_velocity, &fc_waypoint_velocity, &_100HzDt, 12,12);

PID_2 WPCrossTrack(&zero, &wpCrossTrackVelocity, &wpTiltY, &integrate, &kp_cross_track ,&ki_cross_track, &kd_cross_track,&fc_cross_track , &_100HzDt, 15,15);

void _400HzTask() {
  uint32_t _400HzTime;

  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    _400HzTimer = _400HzTime;
    PollAcc();
  }
}

void _100HzTask(uint32_t loopTime){
  static uint8_t _100HzState = 0;

  if (loopTime - _100HzTimer >= 10000){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    while(_100HzState < LAST_100HZ_TASK){

      switch (_100HzState){
      case GET_GYRO:
        errorLimit = kp_waypoint_position;
        offlineMax = ki_waypoint_position;
        onlineReq = kd_waypoint_position;
        alhpaForPressure = fc_waypoint_position;
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
        if (newBaro == true) {
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
          telemFailSafe = false;
          if (gsCTRL == true){
            ProcessModes();
          }
        }
        if (groundFSCount >= 200) {
          telemFailSafe = true;
        }
        _100HzState = HANDLE_FAILSAFES;
        break;
      case HANDLE_FAILSAFES:
        FailSafeHandler();
        if (flightMode > 0){
          _100HzState = ATTITUDE_PID_LOOPS;
        }
        else{
          _100HzState = RATE_PID_LOOPS;
        }

        break;
      case ATTITUDE_PID_LOOPS:
        PitchAngle.calculate();
        RollAngle.calculate();
        if (calcYaw == true) {
          YawAngle.calculate();
        }
        _100HzState = RATE_PID_LOOPS;
        break;
      case RATE_PID_LOOPS:
        if (flightMode == RTB){
          if (rateSetPointZ > 100.0){
            rateSetPointZ = 100.0;
          }
          if (rateSetPointZ < -100.0){
            rateSetPointZ = -100.0;
          }
        }
        PitchRate.calculate();
        RollRate.calculate();
        YawRate.calculate();
        _100HzState = READ_BATTERY;
        break;
      case READ_BATTERY:
        ReadBatteryInfo(&_100HzDt);
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
      _400HzTask();

    }
    _100HzState = GET_GYRO;
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
  ControlLED(0x00); 
  BlueLEDHigh();
  while (1) {

    YellowLEDHigh();
    if (groundFSCount >= 200 ) {
      GreenLEDLow();
    }
    delay(500);
    YellowLEDLow();
    if (groundFSCount >= 200 ) {
      GreenLEDHigh();
    }
    delay(500);
  }

}
void FailSafeHandler(){

  if (magDetected == false){
    GPSDetected = false;
    gpsFailSafe = false;
  }
  if (gsCTRL == true){
    if (telemFailSafe == true){
      gsCTRL = false;
      if (rcDetected == false || RCFailSafe == true){
        if (txLossRTB == 0){
          MotorShutDown();
        }
        else{
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
      if (txLossRTB == 0) {
        MotorShutDown();
      }
      else{
        if (flightMode != RTB) {
          enterState = true;
          flightMode = RTB;
        }
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
      ControlLED(0x0F); 
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
      ControlLED(flightMode);
    }
    HeadingHold();
    TrimCheck();
    break;
  case L0:
    if (enterState == true) {
      enterState = false;
      InitLoiter();
      yawSetPoint = yawInDegrees;
      ControlLED(flightMode);
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
      ControlLED(flightMode);
      controlBearing = initialYaw;
    }
    HeadingHold();
    LoiterSM();
    break;
  case L2:
    if (enterState == true) {
      InitLoiter();
      yawSetPoint = yawInDegrees;
      ControlLED(flightMode);
      enterState = false;
    }
    HeadingHold();
    controlBearing = headingToCraft;
    LoiterSM();
    break;
  case FOLLOW:
    ControlLED(flightMode);
    if (enterState == true) {
      enterState = false;
      yawSetPoint = yawInDegrees;
    }
    break;
  case WP:
    ControlLED(flightMode);
    if (enterState == true) {
      enterState = false;
      yawSetPoint = yawInDegrees;
    }
    break;
  case RTB:
    ControlLED(0x0B);
    if (enterState == true) {
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
  if (motorState == HOLD || motorState == TO) {
    return;
  }
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
    if (sqrt(xFromTO * xFromTO + yFromTO * yFromTO) < MIN_RTB_DIST){
      RTBState = RTB_LOITER;
      xTarget = homeBaseXOffset;
      yTarget = homeBaseYOffset;
      break;
    }


    //calculate distance and heading to origin
    distToWayPoint = sqrt(xFromTO * xFromTO + yFromTO * yFromTO);
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
    if (distToWayPoint < LOW_SPEED_RADIUS){
      if (wpVelSetPoint > 1.0){
        wpVelSetPoint = 1.0;
      }
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
    }
    break;
  case RTB_LOITER:

    LoiterXPosition.calculate();
    LoiterYPosition.calculate();
    LoiterXVelocity.calculate();
    tiltAngleX *= -1.0;
    LoiterYVelocity.calculate();
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
      motorState = LAND;
    }
    AltHoldVelocity.calculate();
    break;
  }
}

void LoiterCalculations() {
  LoiterXPosition.calculate();
  LoiterYPosition.calculate();
  LoiterXVelocity.calculate();
  tiltAngleX *= -1.0;
  LoiterYVelocity.calculate();
}

//move the rudder to the right to start calibration
void Arm(){
  //arming procedure
  ControlLED(0x00);
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
  ControlLED(0x02);
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
  ControlLED(0x09);

}

void LoiterSM(){
  static uint32_t waitTimer;
  int16_t rcDifference;

  switch(ZLoiterState){
  case LOITERING:
    AltHoldPosition.calculate();
    AltHoldVelocity.calculate();
    if (abs(throCommand - 1500) > 200 && throttleCheckFlag == false){
      ZLoiterState = RCINPUT;
    }
    if (motorState == LANDING){
      zTarget = ZEstUp;
      /*if (zTarget <= floorLimit){
       zTarget = floorLimit;
       } 
       if (zTarget >= ceilingLimit){
       zTarget = ceilingLimit;
       }*/
      motorState = FLIGHT;
    }
    if (zTarget < floorLimit) {
      zTarget = floorLimit;
    }
    if (zTarget > ceilingLimit) {
      zTarget = ceilingLimit;
    }
    if (throCommand < (int16_t)propIdleCommand && motorState == FLIGHT){
      ZLoiterState = LAND;
      motorState = LANDING;
      velSetPointZ = LAND_VEL;
      //break;
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
      ZLoiterState = LAND;
      motorState = LANDING;
      velSetPointZ = LAND_VEL;
      break;
    }


    if (ZEstUp >= ceilingLimit && velSetPointZ > 0){
      zTarget = ceilingLimit;
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      break;
    }
    if (ZEstUp <= floorLimit && velSetPointZ < 0){
      zTarget = floorLimit;
      AltHoldPosition.calculate();
      AltHoldVelocity.calculate();
      break;
    }

    AltHoldVelocity.calculate();
    break;


  case LAND:
    if (motorState == FLIGHT){
      motorState = LAND;
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
      Rotate2dVector(&yawInDegrees,&controlBearing,&pitchSetPointTX,&rollSetPointTX,&pitchSetPoint,&rollSetPoint);
      if (fabs(rollSetPointTX) < 0.5 && fabs(pitchSetPointTX) < 0.5){
        XYLoiterState = WAIT;
        waitTimer = millis();
      }
      break;
    case WAIT:
      if (fabs(rollSetPointTX) > 0.5 || fabs(pitchSetPointTX) > 0.5){
        XYLoiterState = RCINPUT;
        break;
      }
      if (millis() - waitTimer > 250){
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
  uint8_t flightModeControl=0;
  previousFlightMode = flightMode;
  if (RCValue[AUX2] > 1750) {
    gsCTRL = false;
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


  if (RCValue[AUX3] > 1750) {
    gsCTRL = false;
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
    if (flightModeControl < 2){
      flightModeControl = 2;
    }
    if (flightModeControl > 9){
      flightModeControl = 2;
    }
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
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -100, 100);
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
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -100, 100);
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
    MapVar(&cmdAile, &rollSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdElev, &pitchSetPointTX, 1000, 2000, -35, 35);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -100, 100);
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
  case FOLLOW://TBD impliment FOLLOW and WP modes currently operate as ATT
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
  case WP:
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
    if (flightMode != previousFlightMode) {
      enterState = true;
    }
    break;

  }
  if (flightMode > L0 && magDetected == false) {
    flightMode = L0;
    MapVar(&cmdElev, &pitchSetPoint, 1000, 2000, -35, 35);
    MapVar(&cmdAile, &rollSetPoint, 1000, 2000, -35, 35);
    MapVar(&cmdRudd, &yawInput, 1000, 2000, -100, 100);
    if (rollSetPoint < 1 && rollSetPoint > -1) {
      rollSetPoint = 0;
    }
    if (pitchSetPoint < 1 && pitchSetPoint > -1) {
      pitchSetPoint = 0;
    }
    if (yawInput < 5 && yawInput > -5) {
      yawInput = 0;
    }
  }
  if (flightMode != previousFlightMode) {
    enterState = true;
  }
}









































