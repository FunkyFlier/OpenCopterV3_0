#ifndef ROM_H
#define ROM_H

#include <Arduino.h>

void AssignPointerArray();
void ROMFlagsCheck();
void LoadROM();

extern float* floatPointerArray[197];
extern int16_t* int16PointerArray[15];
extern uint8_t* bytePointerArray[17];

#endif

