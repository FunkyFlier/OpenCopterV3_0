#ifndef RADIO_H
#define RADIO_H

#include <Arduino.h>

void HandShake();

extern boolean USBFlag,handShake,calibrationMode;
extern Print* radioPrint;
extern Stream* radioStream;

#endif
