#ifndef RADIO_H
#define RADIO_H
#include "Types.h"
#include <Arduino.h>

void HandShake();
void Radio();
void TuningTransmitter();
void TryHandShake();
void CalibrateSensors();

void SendPage(uint8_t*);
void SendEraseComplete();
void SendDumpComplete();

extern boolean USBFlag,handShake,calibrationMode,newGSRC,gsCTRL;
extern int16_t GSRCValue[8];
extern uint16_u flashOutputPacketNumber;

#endif

