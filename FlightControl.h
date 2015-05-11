#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include "Definitions.h"
#include "Enums.h"
#include "Sensors.h"
#include "Attitude.h"
#include "Inertial.h"
#include "GPS.h"
#include "RCSignals.h"
#include <Arduino.h>

void _400HzTask();
void _100HzTask(uint32_t);

extern uint32_t _100HzTimer,_400HzTimer;
extern volatile uint32_t RCFailSafeCounter;
extern float initialYaw;
extern boolean integrate;
extern uint8_t HHState;
extern float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
extern uint8_t XYLoiterState, ZLoiterState,flightMode;
extern float homeBaseXOffset,homeBaseYOffset;
extern float xTarget,yTarget,zTarget;
extern boolean enterState;

#endif
