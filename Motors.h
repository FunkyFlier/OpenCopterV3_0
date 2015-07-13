#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void CheckESCFlag();
void SetRCControlESCCalFlag();
void MotorInit();
void MotorHandler();

extern uint32_t romWriteDelayTimer;
extern float motorCommand1, motorCommand2, motorCommand3, motorCommand4,motorCommand5, motorCommand6, motorCommand7, motorCommand8;
extern int16_t pwmHigh, pwmLow;
extern int16_t throttleCommand;
extern uint8_t motorState;
extern uint16_t propIdleCommand, hoverCommand;

extern float m1X,m1Y,m1Z,m2X,m2Y,m2Z,m3X,m3Y,m3Z,m4X,m4Y,m4Z,m5X,m5Y,m5Z,m6X,m6Y,m6Z,m7X,m7Y,m7Z,m8X,m8Y,m8Z;

extern boolean saveGainsFlag;
extern boolean throttleCheckFlag;
extern boolean calibrateESCs;
extern boolean calibrationModeESCs;

#endif
