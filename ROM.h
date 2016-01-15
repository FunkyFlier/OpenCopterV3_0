#ifndef ROM_H
#define ROM_H

#include <Arduino.h>

void AssignPointerArray();
void ROMFlagsCheck();
void LoadROM();

extern float* floatPointerArray[204];
extern int16_t* int16PointerArray[22];
extern uint8_t* bytePointerArray[19];

#endif

