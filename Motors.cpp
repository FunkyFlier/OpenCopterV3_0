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

void SaveGains();

float motorCommand1, motorCommand2, motorCommand3, motorCommand4,motorCommand5, motorCommand6, motorCommand7, motorCommand8;
int16_t pwmHigh, pwmLow;
int16_t throttleCommand;
float landingThroAdjustment,adjustmentX,adjustmentY,adjustmentZ;
uint8_t motorState;

//todo move
uint16_t propIdleCommand, hoverCommand;


boolean saveGainsFlag;
boolean throttleCheckFlag = false;

void CheckESCFlag(){
  int16_u outInt16;
  uint32_t LEDTimer;
  uint8_t LEDControlNumber;
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
  newRC = false;
  while(newRC == false){
    ProcessChannels();
  }
  newRC = false;

  LEDTimer = millis();
  if (EEPROMRead(ESC_CAL_FLAG) == 0xAA){

    while(RCValue[THRO] > 1100 || RCValue[AILE] > 1100 || RCValue[ELEV] > 1100 || RCValue[RUDD] > 1100){
      if (millis() - LEDTimer > 250){
        LEDTimer = millis();
        ControlLED(LEDControlNumber++);
      }
      if (newRC == true){
        newRC = false;
        ProcessChannels();
      } 
    }
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
    LEDTimer = millis();
    while(1){
      if (millis() - LEDTimer > 250){
        LEDTimer = millis();
        ControlLED(LEDControlNumber--);
      }
    }
  }
}


void CalibrateESC(){
  uint32_t LEDTimer;
  uint8_t LEDControlNumber;
  delay(100);//wait for new frame
  newRC = false;
  while(newRC == false){

  }
  ProcessChannels();
  if (RCValue[THRO] > 1900){
    EEPROMWrite(HS_FLAG,0xFF);//clear the handshake flag
    EEPROMWrite(ESC_CAL_FLAG,0xAA);
    LEDTimer = millis();
    while(1){
      if (millis() - LEDTimer > 250){
        LEDTimer = millis();
        ControlLED(LEDControlNumber--);
      }
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
  uint16_t j_ = GAINS_START;
  for(uint16_t i = KP_PITCH_RATE_; i <= MAG_DEC_; i++){
    /*EEPROMWrite(j_++,(*floatPointerArray[i]).buffer[0]); 
     EEPROMWrite(j_++,(*floatPointerArray[i]).buffer[1]); 
     EEPROMWrite(j_++,(*floatPointerArray[i]).buffer[2]); 
     EEPROMWrite(j_++,(*floatPointerArray[i]).buffer[3]); */
    //watchDogFailSafeCounter = 0;

  }
  cosDec = cos(declination);
  sinDec = sin(declination);

  calibrationFlags = EEPROMRead(CAL_FLAGS);
  calibrationFlags &= ~(1<<GAINS_FLAG);
  EEPROMWrite(CAL_FLAGS,calibrationFlags);
}
void MotorHandler(){
  boolean rudderFlag = false;
  switch(motorState){
  case HOLD:

    if (saveGainsFlag == true){// && (millis() - romWriteDelayTimer) > 2000){
      SaveGains();

      saveGainsFlag = false;
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
      motorCommand1 = pwmLow;
      motorCommand2 = pwmLow;
      motorCommand3 = pwmLow;
      motorCommand4 = pwmLow;
      motorCommand5 = pwmLow;
      motorCommand6 = pwmLow;
      motorCommand7 = pwmLow;
      motorCommand8 = pwmLow;
      break;
    }
    if (flightMode == RTB){
      motorCommand1 = pwmLow;
      motorCommand2 = pwmLow;
      motorCommand3 = pwmLow;
      motorCommand4 = pwmLow;
      motorCommand5 = pwmLow;
      motorCommand6 = pwmLow;
      motorCommand7 = pwmLow;
      motorCommand8 = pwmLow;
      break;
    }

    if (cmdRudd < 1300){
      rudderFlag = true;

    }
    if (rudderFlag == true){

      if (abs(cmdRudd - 1500) < 50){
        rudderFlag = false;
        motorState = TO;
        /*
        PitchAngle.reset();
         RollAngle.reset();
         YawAngle.reset();
         
         PitchRate.reset();
         RollRate.reset();
         YawRate.reset();
         
         AltHoldPosition.reset();
         AltHoldVelocity.reset();
         
         WayPointPosition.reset();
         WayPointRate.reset();
         
         LoiterXPosition.reset();
         LoiterXVelocity.reset();
         
         LoiterYPosition.reset();
         LoiterYVelocity.reset();*/
        homeBaseXOffset = XEst;
        homeBaseYOffset = YEst;
        if (magDetected == true){
          VerifyMag();
        }
        SetGyroOffsets();
      }
    }
    throttleAdjustment = 0;
    motorCommand1 = pwmLow;
    motorCommand2 = pwmLow;
    motorCommand3 = pwmLow;
    motorCommand4 = pwmLow;
    motorCommand5 = pwmLow;
    motorCommand6 = pwmLow;
    motorCommand7 = pwmLow;
    motorCommand8 = pwmLow;
    throttleCheckFlag = false;
    break;
  case TO:
    motorCommand1 = propIdleCommand;
    motorCommand2 = propIdleCommand;
    motorCommand3 = propIdleCommand;
    motorCommand4 = propIdleCommand;
    motorCommand5 = propIdleCommand;
    motorCommand6 = propIdleCommand;
    motorCommand7 = propIdleCommand;
    motorCommand8 = propIdleCommand;
    throttleCheckFlag = false;
    initialPressure = pressure;
    ZEst = 0;
    ZEstUp = 0;
    velZ = 0;
    velZUp = 0;
    prevBaro = 0;
    baroZ = 0;
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
        integrate = true;
      }
    }
    if (flightMode <= L2 && flightMode >= L0){
      if (throCommand <= 1600 && throCommand >= 1450){
        motorState = FLIGHT;
        zTarget = TAKE_OFF_ALT;
        enterState = true;
        throttleAdjustment = 0;
        xTarget = XEst;
        yTarget = YEst;
        /*LoiterXPosition.reset();
         LoiterXVelocity.reset();
         LoiterYPosition.reset();
         LoiterYVelocity.reset();
         AltHoldPosition.reset();
         AltHoldVelocity.reset();*/
        integrate = true;
      }
    }
    if (flightMode == WP || flightMode == FOLLOW){
      /*if (throCommand <= 1600 && throCommand >= 1450){
       autoMaticReady = true;
       }*/
    }

    break;
  case FLIGHT:
    if (flightMode == RATE || flightMode == ATT){
      throttleAdjustment = 0;
      throttleCommand = throCommand;

      if (throttleCommand > 1900){
        throttleCommand = 1900;
      }
      if (throttleCommand < 1050){
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

    landingThroAdjustment = 0.997 * landingThroAdjustment + 0.003 * throttleAdjustment;
#ifdef QUAD
    motorCommand1 = constrain((throttleCommand + throttleAdjustment + adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + throttleAdjustment - adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + throttleAdjustment - adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + throttleAdjustment + adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand5 = pwmLow;
    motorCommand6 = pwmLow;
    motorCommand7 = pwmLow;
    motorCommand8 = pwmLow;
#endif
#ifdef X_8
    motorCommand1 = constrain((throttleCommand + throttleAdjustment + adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + throttleAdjustment - adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + throttleAdjustment - adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + throttleAdjustment + adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);

    motorCommand5 = constrain((throttleCommand + throttleAdjustment + adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand6 = constrain((throttleCommand + throttleAdjustment - adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand7 = constrain((throttleCommand + throttleAdjustment - adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand8 = constrain((throttleCommand + throttleAdjustment + adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
#endif
#ifdef HEX_FRAME
    motorCommand1 = constrain((throttleCommand + throttleAdjustment + 0.5 * adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + throttleAdjustment - 0.5 * adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + throttleAdjustment -       adjustmentX               - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + throttleAdjustment - 0.5 * adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand5 = constrain((throttleCommand + throttleAdjustment + 0.5 * adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand6 = constrain((throttleCommand + throttleAdjustment +       adjustmentX               + adjustmentZ),pwmLow,pwmHigh);
    motorCommand7 = pwmLow;
    motorCommand8 = pwmLow;
#endif

    break;
  case LANDING:
    if (flightMode == RATE || flightMode == ATT){
      motorState = FLIGHT;
    }
    if (throttleCheckFlag == true){
      if (throCommand <= 1600 && throCommand >= 1450){
        throttleCheckFlag = false;
      }
    }
    throttleCommand = hoverCommand;
    if ( (hoverCommand + throttleAdjustment) < 1350){
      motorCommand1 = pwmLow;
      motorCommand2 = pwmLow;
      motorCommand3 = pwmLow;
      motorCommand4 = pwmLow;

      motorCommand5 = pwmLow;
      motorCommand6 = pwmLow;
      motorCommand7 = pwmLow;
      motorCommand8 = pwmLow;
      motorState = HOLD;
      break;
    }
    if (cmdRudd > 1700){
      motorCommand1 = pwmLow;
      motorCommand2 = pwmLow;
      motorCommand3 = pwmLow;
      motorCommand4 = pwmLow;
      motorCommand5 = pwmLow;
      motorCommand6 = pwmLow;
      motorCommand7 = pwmLow;
      motorCommand8 = pwmLow;
      motorState = HOLD;
      break;
    }
    if (fabs(inertialZ) > 5.0){
      motorCommand1 = pwmLow;
      motorCommand2 = pwmLow;
      motorCommand3 = pwmLow;
      motorCommand4 = pwmLow;
      motorCommand5 = pwmLow;
      motorCommand6 = pwmLow;
      motorCommand7 = pwmLow;
      motorCommand8 = pwmLow;
      motorState = HOLD;
      break;
    }
    if (throttleAdjustment > 0){
      throttleAdjustment = 0;
    }

    landingThroAdjustment = 0.997 * landingThroAdjustment + 0.003 * throttleAdjustment;

#ifdef QUAD
    motorCommand1 = constrain((throttleCommand + landingThroAdjustment + adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + landingThroAdjustment - adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + landingThroAdjustment - adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + landingThroAdjustment + adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand5 = pwmLow;
    motorCommand6 = pwmLow;
    motorCommand7 = pwmLow;
    motorCommand8 = pwmLow;
#endif 
#ifdef X_8
    motorCommand1 = constrain((throttleCommand + landingThroAdjustment + adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + landingThroAdjustment - adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + landingThroAdjustment - adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + landingThroAdjustment + adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand5 = constrain((throttleCommand + landingThroAdjustment + adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand6 = constrain((throttleCommand + landingThroAdjustment - adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand7 = constrain((throttleCommand + landingThroAdjustment - adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand8 = constrain((throttleCommand + landingThroAdjustment + adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
#endif
#ifdef HEX_FRAME
    motorCommand1 = constrain((throttleCommand + landingThroAdjustment + 0.5 * adjustmentX + adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand2 = constrain((throttleCommand + landingThroAdjustment - 0.5 * adjustmentX + adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand3 = constrain((throttleCommand + landingThroAdjustment -       adjustmentX               - adjustmentZ),pwmLow,pwmHigh);
    motorCommand4 = constrain((throttleCommand + landingThroAdjustment - 0.5 * adjustmentX - adjustmentY + adjustmentZ),pwmLow,pwmHigh);
    motorCommand5 = constrain((throttleCommand + landingThroAdjustment + 0.5 * adjustmentX - adjustmentY - adjustmentZ),pwmLow,pwmHigh);
    motorCommand6 = constrain((throttleCommand + landingThroAdjustment +       adjustmentX               + adjustmentZ),pwmLow,pwmHigh);
    motorCommand7 = pwmLow;
    motorCommand8 = pwmLow;
#endif
    break;
  }

  Motor1WriteMicros(motorCommand1);
  Motor2WriteMicros(motorCommand2);
  Motor3WriteMicros(motorCommand3);
  Motor4WriteMicros(motorCommand4);

  Motor5WriteMicros(motorCommand5);
  Motor6WriteMicros(motorCommand6);
  Motor7WriteMicros(motorCommand7);
  Motor8WriteMicros(motorCommand8);

}


