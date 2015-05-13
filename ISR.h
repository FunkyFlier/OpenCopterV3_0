#ifndef ISR_H
#define ISR_H

#include <Arduino.h>

extern volatile boolean watchDogStartCount;

void _200HzISRConfig();

#endif
