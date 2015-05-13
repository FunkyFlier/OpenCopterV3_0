#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>

void HandShake();
void Radio();
void HandleGSRCData();
void TuningTransmitter();
void TryHandShake();
void CalibrateSensors();

extern boolean USBFlag,handShake,calibrationMode,newGSRC,gsCTRL;
extern uint16_t GSRCValue[8];

#endif
