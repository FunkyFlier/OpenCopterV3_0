#ifndef ROM_H
#define ROM_H

#include <Arduino.h>

void AssignPointerArray();
void ROMFlagsCheck();
void LoadROM();

extern float* floatPointerArray[186];
extern int16_t* int16PointerArray[14];
extern uint8_t* bytePointerArray[14];

#endif

