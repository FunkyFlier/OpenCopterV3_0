#ifndef RCSIGNALS_H
#define RCSIGNALS_H


#include <Arduino.h>


typedef struct {
  int16_t max;
  int16_t min;
  int16_t mid;
  volatile int16_t rcvd;
  uint8_t chan;
  float scale;
  uint8_t reverse;
}
RC_t;

void FeedLine();
void DetectRC();
void LoadRCValuesFromRom();
void CheckTXPositions();
void GetSwitchPositions();

extern RC_t rcData[8];
extern int16_t RCValue[8];
volatile extern boolean rcDetected;
extern volatile uint8_t rcType;
extern uint8_t ISRState;
extern volatile boolean RCFailSafe;
extern volatile boolean newRC;
extern int16_t cmdElev,cmdAile,cmdRudd,throCommand;
extern uint8_t switchPositions;
extern float pitchSetPointTX,rollSetPointTX;


#endif
