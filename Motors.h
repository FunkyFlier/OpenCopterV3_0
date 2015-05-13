#ifndef MOTORS_H
#define MOTORS_H

#include <Arduino.h>

void CheckESCFlag();
void CalibrateESC();
void MotorInit();
void MotorHandler();

extern uint32_t romWriteDelayTimer;
extern float motorCommand1, motorCommand2, motorCommand3, motorCommand4,motorCommand5, motorCommand6, motorCommand7, motorCommand8;
extern int16_t pwmHigh, pwmLow;
extern int16_t throttleCommand;
extern uint8_t motorState;
extern uint16_t propIdleCommand, hoverCommand;

extern boolean saveGainsFlag;
extern boolean throttleCheckFlag;
#endif
