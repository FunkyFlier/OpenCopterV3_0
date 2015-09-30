#include "Motors.h"

#include "Types.h"
#include "Definitions.h"
#include "Comm.h"
#include "RCSignals.h"
#include "LED.h"
#include "Enums.h"
#include "Attitude.h"
#include "FlightControl.h"
#include "Sensors.h"
#include "Inertial.h"
#include "Calibration.h"
#include "Rom.h"
#include "Radio.h"
#include <EEPROM.h>
#include "GPS.h"

void SaveGains();
void ResetPIDs();
void CompleteESCCalibration();
void CalculateMotorMixing();
void WriteMotorPWM();
void SaveEstiamtorGains();
void CommandAllMotors(float);

uint32_t romWriteDelayTimer;
float motorCommand1, motorCommand2, motorCommand3, motorCommand4,motorCommand5, motorCommand6, motorCommand7, motorCommand8;
int16_t pwmHigh, pwmLow;
int16_t throttleCommand;
uint8_t motorState;
uint16_t propIdleCommand, hoverCommand;

float m1X,m1Y,m1Z,m2X,m2Y,m2Z,m3X,m3Y,m3Z,m4X,m4Y,m4Z,m5X,m5Y,m5Z,m6X,m6Y,m6Z,m7X,m7Y,m7Z,m8X,m8Y,m8Z;
float landRampValue;

boolean saveGainsFlag = false;
boolean saveEstimatorGainsFlag = false;
boolean throttleCheckFlag = false;
boolean calibrateESCs = false;
boolean calibrationModeESCs = false;

void CheckESCFlag(){
  int16_u outInt16;
  uint32_t timeOutTimer = 0;
  uint8_t LEDPatternArray[4];
  if (EEPROMRead(PWM_FLAG) != 0xAA){
    pwmHigh = PWM_HIGH_MAX;
    pwmLow = PWM_LOW_MIN;

    outInt16.val = pwmHigh;
    EEPROMWrite(PWM_LIM_HIGH_START,outInt16.buffer[0]);
    EEPROMWrite(PWM_LIM_HIGH_END,outInt16.buffer[1]);

    outInt16.val = pwmLow;
    EEPROMWrite(PWM_LIM_LOW_START,outInt16.buffer[0]);
    EEPROMWrite(PWM_LIM_LOW_END,outInt16.buffer[1]);
    EEPROMWrite(PWM_FLAG,0xAA);
  }

  outInt16.buffer[0] = EEPROMRead(PWM_LIM_HIGH_START);
  outInt16.buffer[1] = EEPROMRead(PWM_LIM_HIGH_END);
  pwmHigh = outInt16.val;
  if (pwmHigh > PWM_HIGH_MAX){
    pwmHigh = PWM_HIGH_MAX;
  }
  if (pwmHigh < PWM_HIGH_MIN){
    pwmHigh = PWM_HIGH_MIN;
  }
  outInt16.buffer[0] = EEPROMRead(PWM_LIM_LOW_START);
  outInt16.buffer[1] = EEPROMRead(PWM_LIM_LOW_END);
  pwmLow = outInt16.val;
  if (pwmLow < PWM_LOW_MIN){
    pwmLow = PWM_LOW_MIN;
  }
  if (pwmLow > PWM_LOW_MAX){
    pwmLow = PWM_LOW_MAX;
  }
  timeOutTimer = millis();
  if (EEPROMRead(ESC_CAL_FLAG) == 0xAA){
    calibrationMode = false;
    if (rcDetected == false){
      LEDPatternSet(1,1,0,1);
      while(1){
      }
    }
    else{
      newRC = false;
      while(newRC == false){
      }
      ProcessChannels();
      newRC = false;
    }
    while(RCValue[THRO] > 1100 || RCValue[AILE] > 1100 || RCValue[ELEV] > 1100 || RCValue[RUDD] > 1100){

      if (millis() - timeOutTimer > 60000){
        EEPROMWrite(ESC_CAL_FLAG,0xFF);
        LEDPatternSet(3,3,3,1);
        while(1){
        }
      }

      if (newRC == true){
        newRC = false;
        ProcessChannels();
        if (RCValue[THRO] > 1100 ){
          LEDPatternArray[0] = 5;
        }
        else{
          LEDPatternArray[0] = 1;
        }
        if (RCValue[AILE] > 1100 ){
          LEDPatternArray[1] = 5;
        }
        else{
          LEDPatternArray[1] = 1;
        }
        if (RCValue[ELEV] > 1100 ){
          LEDPatternArray[2] = 5;
        }
        else{
          LEDPatternArray[2] = 1;
        }
        if (RCValue[RUDD] > 1100 ){
          LEDPatternArray[3] = 5;
        }
        else{
          LEDPatternArray[3] = 1;
        }
        LEDPatternSet(LEDPatternArray[0],LEDPatternArray[1],LEDPatternArray[2],LEDPatternArray[3]);
      } 
    }
    CompleteESCCalibration();
  }
  if (EEPROMRead(ESC_CAL_FLAG) == 0xBB){
    calibrationMode = false;
    calibrateESCs = false;
    TryHandShake();
    if (handShake == false || calibrationModeESCs == false){
      EEPROMWrite(ESC_CAL_FLAG,0xFF);
      LEDPatternSet(4,4,4,1);
      while(1){
      }
    }
    while(calibrateESCs == false){
      LEDPatternSet(6,1,1,1);
      Radio();
    }
    if (calibrateESCs == true){
      CompleteESCCalibration();
    }
  }
}

void CompleteESCCalibration(){


  DDRE |= B00111000;
  DDRH |= B00111000;
  DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PERIOD; 

  Motor1WriteMicros(pwmHigh);//set the output compare value
  Motor2WriteMicros(pwmHigh);
  Motor3WriteMicros(pwmHigh);
  Motor4WriteMicros(pwmHigh);
  Motor5WriteMicros(pwmHigh);
  Motor6WriteMicros(pwmHigh);
  Motor7WriteMicros(pwmHigh);
  Motor8WriteMicros(pwmHigh);

  delay(ESC_CALIBRATION_DELAY);
  Motor1WriteMicros(pwmLow);//set the output compare value
  Motor2WriteMicros(pwmLow);
  Motor3WriteMicros(pwmLow);
  Motor4WriteMicros(pwmLow);
  Motor5WriteMicros(pwmLow);
  Motor6WriteMicros(pwmLow);
  Motor7WriteMicros(pwmLow);
  Motor8WriteMicros(pwmLow);

  EEPROMWrite(ESC_CAL_FLAG,0xFF);
  LEDPatternSet(1,2,2,0);
  while(1){

  }
}

void SetRCControlESCCalFlag(){


  delay(100);//wait for new frame
  newRC = false;
  while(newRC == false){

  }
  ProcessChannels();
  if (RCValue[THRO] > 1900){
    EEPROMWrite(HS_FLAG,0xFF);//clear the handshake flag
    EEPROMWrite(ESC_CAL_FLAG,0xAA);
    LEDPatternSet(6,6,0,1);
    while(1){
    }

  }

}


void MotorInit(){
  DDRE |= B00111000;
  DDRH |= B00111000;
  DDRB |= B01100000;


  TCCR3A = (1<<WGM31)|(1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);  
  TCCR3B = (1<<WGM33)|(1<<WGM32)|(1<<CS31);               
  ICR3 = PERIOD;   

  TCCR4A = (1<<WGM41)|(1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1);
  TCCR4B = (1<<WGM43)|(1<<WGM42)|(1<<CS41);
  ICR4 = PERIOD;  

  TCCR1A = (1<<WGM11)|(1<<COM1A1)|(1<<COM1B1);
  TCCR1B = (1<<WGM13)|(1<<WGM12)|(1<<CS11);
  ICR1 = PERIOD;


  Motor1WriteMicros(pwmLow);//set the output compare value
  Motor2WriteMicros(pwmLow);
  Motor3WriteMicros(pwmLow);
  Motor4WriteMicros(pwmLow);
  Motor5WriteMicros(pwmLow);
  Motor6WriteMicros(pwmLow);
  Motor7WriteMicros(pwmLow);
  Motor8WriteMicros(pwmLow);

}



void SaveGains(){
  uint8_t calibrationFlags;
  uint16_t j = GAINS_START;
  float_u outFloat;

  j = GAINS_START;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++) {
    outFloat.val = *floatPointerArray[i];
    EEPROMWrite(j++, outFloat.buffer[0]);
    EEPROMWrite(j++, outFloat.buffer[1]);
    EEPROMWrite(j++, outFloat.buffer[2]);
    EEPROMWrite(j++, outFloat.buffer[3]);
  }
  j = DEC_START;
  outFloat.val = *floatPointerArray[MAG_DEC_];
  EEPROMWrite(j++, outFloat.buffer[0]);
  EEPROMWrite(j++, outFloat.buffer[1]);
  EEPROMWrite(j++, outFloat.buffer[2]);
  EEPROMWrite(j++, outFloat.buffer[3]);
  cosDec = cos(declination);
  sinDec = sin(declination);

  calibrationFlags = EEPROMRead(CAL_FLAGS);
  calibrationFlags &= ~(1<<GAINS_FLAG);
  EEPROMWrite(CAL_FLAGS,calibrationFlags);
  RCFailSafeCounter = 0;
  groundFSCount = 0;
  GPSFailSafeCounter = 0;
  baroFSCount = 0;
  watchDogFailSafeCounter = 0;
}
void SaveEstiamtorGains(){
  float_u outFloat;
  uint16_t j = EST_GAIN_START;
  for (uint16_t i = KP_ACC; i <= K_B_BARO; i++) {
    outFloat.val = *floatPointerArray[i];
    EEPROMWrite(j++, outFloat.buffer[0]);
    EEPROMWrite(j++, outFloat.buffer[1]);
    EEPROMWrite(j++, outFloat.buffer[2]);
    EEPROMWrite(j++, outFloat.buffer[3]);
  } 
  RCFailSafeCounter = 0;
  groundFSCount = 0;
  GPSFailSafeCounter = 0;
  baroFSCount = 0;
  watchDogFailSafeCounter = 0; 
}
void ResetPIDs(){
  PitchAngle.reset();
  RollAngle.reset();
  YawAngle.reset();

  PitchRate.reset();
  RollRate.reset();
  YawRate.reset();

  AltHoldPosition.reset();
  AltHoldVelocity.reset();

  LoiterXPosition.reset();
  LoiterXVelocity.reset();

  LoiterYPosition.reset();
  LoiterYVelocity.reset();

  WPPosition.reset(); 
  WPVelocity.reset(); 
  WPCrossTrack.reset(); 
}
void MotorHandler(){
  static boolean rudderFlag = false,landDetected = false;

  switch(motorState){
  case HOLD:
    landDetected = false;
    if (saveGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
      SaveGains();

      saveGainsFlag = false;
      _100HzTimer = micros();
      baroPollTimer = millis();
      _400HzTimer = _100HzTimer;
    }
    if (saveEstimatorGainsFlag == true && (millis() - romWriteDelayTimer) > 2000){
      SaveEstiamtorGains();

      saveEstimatorGainsFlag = false;
      _100HzTimer = micros();
      baroPollTimer = millis();
      _400HzTimer = _100HzTimer;
    }
    initialYaw = yawInDegrees;
    integrate = false;
    HHState = 0;
    throttleAdjustment = 0;
    ZLoiterState = LOITERING;
    XYLoiterState = LOITERING;
    if (throCommand > 1100){
      CommandAllMotors((float)pwmLow);
      break;
    }
    if (flightMode == RTB){
      CommandAllMotors((float)pwmLow);
      break;
    }

    if (cmdRudd < 1300){
      rudderFlag = true;

    }
    if (rudderFlag == true){

      if (abs(cmdRudd - 1500) < 50){
        rudderFlag = false;
        motorState = TO;
        takeOffPressure = pressure;
        initialPressure = pressure;

        homeBaseXOffset = XEst;
        homeBaseYOffset = YEst;
        if (magDetected == true){
          VerifyMag();
        }
        SetGyroOffsets();
      }
    }
    throttleAdjustment = 0;
    CommandAllMotors((float)pwmLow);
    throttleCheckFlag = false;
    break;
  case TO:
    landDetected = false;
    CommandAllMotors((float)propIdleCommand);
    throttleCheckFlag = false;
    initialPressure = pressure;

    ZEst = -baroZ;
    ZEstUp = baroZ;
    velZ = 0;
    velZUp = 0;

    initialYaw = yawInDegrees;

    if (cmdRudd > 1700){
      motorState = HOLD;
    }
    if (flightMode == RTB){
      motorState = HOLD;
    }

    if (flightMode == RATE || flightMode == ATT){
      if (throCommand > 1150 && throCommand < 1350){
        motorState = FLIGHT;
        ResetPIDs();
        integrate = true;
      }
    }
    if (flightMode <= L2 && flightMode >= L0){
      if (throCommand <= 1600 && throCommand >= 1450){
        motorState = FLIGHT;
        ResetPIDs();
        zTarget = TAKE_OFF_ALT;
        enterState = true;
        throttleAdjustment = 0;
        xTarget = XEst;
        yTarget = YEst;
        LoiterXPosition.reset();
        LoiterXVelocity.reset();
        LoiterYPosition.reset();
        LoiterYVelocity.reset();
        AltHoldPosition.reset();
        AltHoldVelocity.reset();
        integrate = true;
      }
    }
    if (flightMode == WP || flightMode == FOLLOW){

    }

    break;
  case FLIGHT:
    landDetected = false;
    if (flightMode == RATE || flightMode == ATT){
      throttleAdjustment = 0;
      throttleCommand = throCommand;

      if (throttleCommand > 1900){
        throttleCommand = 1900;
      }
      if (throttleCommand < (int16_t)propIdleCommand){
        throttleCommand = propIdleCommand;
        if (cmdRudd > 1700){
          motorState = HOLD;
        }
        motorState = TO;

      }
    }
    if (flightMode >= L0){
      throttleCommand = hoverCommand;
    }
    if (throttleCheckFlag == true){
      if (throCommand <= 1600 && throCommand >= 1450){
        throttleCheckFlag = false;
        throttleCommand = hoverCommand;
      }
    }

    CalculateMotorMixing();

    break;
  case LANDING:
    if (flightMode == RATE || flightMode == ATT){
      landDetected = false;
      motorState = FLIGHT;
    }
    if (throttleCheckFlag == true){
      if (throCommand <= 1600 && throCommand >= 1450){
        throttleCheckFlag = false;
      }
    }
    throttleCommand = hoverCommand;
    if ( (throttleAdjustment + throttleCommand) < (propIdleCommand) &&  landDetected == false){
      CommandAllMotors((float)pwmLow);
      landDetected = false;
      motorState = HOLD;
      break;
    }
    if (cmdRudd > 1700){
      CommandAllMotors((float)pwmLow);
      landDetected = false;
      motorState = HOLD;
      break;
    }

    CalculateMotorMixing();
    if (zPosError < -0.75 && landDetected == false){
      landDetected = true;
      landRampValue = throttleAdjustment;// - 75;//AltHoldVelocity.iError;//throttleAdjustment - 100;// + throttleCommand;// - 100;
    }
    if (landDetected == true){
      landRampValue = landRampValue - 4.5;
      //landRampValue = landRampValue - 0.01;
      //landRampValue = landRampValue * 0.9 + propIdleCommand * 0.1;
      //throttleAdjustment = landRampValue;
      /*CommandAllMotors((float)(landRampValue--));*/
      throttleAdjustment = landRampValue;
      if (landRampValue <= 0){
        //if (landRampValue +  throttleCommand < propIdleCommand + 50){
        CommandAllMotors((float)pwmLow);
        landDetected = false;
        motorState = HOLD;
      } 
    }
    CalculateMotorMixing();
    break;
  }

  WriteMotorPWM();
}

void CommandAllMotors(float command){
  motorCommand1 = command;
  motorCommand2 = command;
  motorCommand3 = command;
  motorCommand4 = command;
  motorCommand5 = command;
  motorCommand6 = command;
  motorCommand7 = command;
  motorCommand8 = command;
}
void CalculateMotorMixing(){
  float throToMotors;
  throToMotors = throttleAdjustment + throttleCommand;
  if (throToMotors > pwmHigh - 150){
    throToMotors = pwmHigh - 150;
  }
  if (throToMotors < propIdleCommand){
    throToMotors = propIdleCommand;
  }
  motorCommand1 = constrain((throToMotors + m1X * adjustmentX + m1Y * adjustmentY + m1Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand2 = constrain((throToMotors + m2X * adjustmentX + m2Y * adjustmentY + m2Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand3 = constrain((throToMotors + m3X * adjustmentX + m3Y * adjustmentY + m3Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand4 = constrain((throToMotors + m4X * adjustmentX + m4Y * adjustmentY + m4Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand5 = constrain((throToMotors + m5X * adjustmentX + m5Y * adjustmentY + m5Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand6 = constrain((throToMotors + m6X * adjustmentX + m6Y * adjustmentY + m6Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand7 = constrain((throToMotors + m7X * adjustmentX + m7Y * adjustmentY + m7Z * adjustmentZ),propIdleCommand,pwmHigh);
  motorCommand8 = constrain((throToMotors + m8X * adjustmentX + m8Y * adjustmentY + m8Z * adjustmentZ),propIdleCommand,pwmHigh);

}

void WriteMotorPWM(){
  Motor1WriteMicros(motorCommand1);
  Motor2WriteMicros(motorCommand2);
  Motor3WriteMicros(motorCommand3);
  Motor4WriteMicros(motorCommand4);

  Motor5WriteMicros(motorCommand5);
  Motor6WriteMicros(motorCommand6);
  Motor7WriteMicros(motorCommand7);
  Motor8WriteMicros(motorCommand8);

}















