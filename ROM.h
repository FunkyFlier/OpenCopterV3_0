#ifndef ROM_H
#define ROM_H

#include "Enums.h"
#include <Arduino.h>

void AssignPointerArray();
void ROMFlagsCheck();
void LoadROM();
void LoadRC();
void VerifyRcType();

extern float* floatPointerArray[END_FLOATS];
extern int16_t* int16PointerArray[END_INT_16S];
extern uint8_t* bytePointerArray[END_BYTES];

#endif

