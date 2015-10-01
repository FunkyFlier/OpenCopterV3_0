#include "LED.h"
#include "Motors.h"
#include "Enums.h"
#include "Definitions.h"
//Green Yellow Blue Red

void LEDPatternSet(uint8_t, uint8_t, uint8_t, uint8_t);
void LEDPatternHandler(uint32_t);

uint8_t LEDPatternMatrix[4];
uint8_t LEDPattern[4];
uint8_t OverRideMatrix[4];
boolean displayFSData = false;
uint8_t ctrlByte;

void LEDInit(){
  GreenLEDOutput();
  YellowLEDOutput();
  BlueLEDOutput();
  RedLEDOutput();  
  ControlLED(0x0F);

}
void LEDPatternSet(uint8_t green,uint8_t yellow, uint8_t blue, uint8_t red){
  LEDPattern[GREEN_] = green;
  LEDPattern[YELLOW_] = yellow;
  LEDPattern[BLUE_] = blue;
  LEDPattern[RED_] = red;

}
void LEDPatternHandler(uint32_t currentTime){
  static boolean onOff = true,fastIndicator = true,updatePattern;
  static uint8_t patternCount;
  static uint32_t patternTime;
  static uint32_t fastBlinkTime;

  if (displayFSData == true && motorState == HOLD){
    LEDPatternMatrix[GREEN_] = OverRideMatrix[GREEN_];
    LEDPatternMatrix[YELLOW_] = OverRideMatrix[YELLOW_];
    LEDPatternMatrix[BLUE_] = OverRideMatrix[BLUE_];
    LEDPatternMatrix[RED_] = OverRideMatrix[RED_]; 
  }
  else{
    LEDPatternMatrix[GREEN_] = LEDPattern[GREEN_];
    LEDPatternMatrix[YELLOW_] = LEDPattern[YELLOW_];
    LEDPatternMatrix[BLUE_] = LEDPattern[BLUE_];
    LEDPatternMatrix[RED_] = LEDPattern[RED_];
  }

  if (currentTime - patternTime >= PATTERN_TIME){
    patternTime = currentTime;
    if (onOff == true){
      patternCount++;
      ctrlByte = 0xFF;
      for (uint8_t i = 0; i <= 3; i++){
        if (LEDPatternMatrix[i] == 0){
          ctrlByte &= ~(1<<i);
        }
        if (LEDPatternMatrix[i] >= 2 && LEDPatternMatrix[i] < 6){
          if (patternCount % LEDPatternMatrix[i] == 0){
            ctrlByte &= ~(1<<i);
          }
        }
      }
      onOff = false;
    }
    else{
      ctrlByte = 0x00;
      for (uint8_t i = 0; i <= 3; i++){
        if (LEDPatternMatrix[i] == 1){
          ctrlByte |= 1<<i;
        }
      }
      onOff = true;
    }
    updatePattern = true;
  }
  if (currentTime - fastBlinkTime >= PATTERN_TIME_FAST){
    fastBlinkTime = currentTime;
    fastIndicator ^= 1;
    for (uint8_t i = 0; i <= 3; i++){
      if (LEDPatternMatrix[i] == 7){
        if (fastIndicator == true){
          ctrlByte |= 1<<i;
        }
        else{
          ctrlByte &= ~(1<<i);
        }

      }
    }
    updatePattern = true;
  }
  if (updatePattern == true){
    ControlLED(ctrlByte);
    updatePattern = false;
  }

}


void ControlLED(uint8_t controlByte){
  controlByte = controlByte & 0x0F;
  switch(controlByte){
  case 0:
    GreenLEDLow();
    YellowLEDLow();
    BlueLEDLow();
    RedLEDLow();
    break;
  case 1:

    RedLEDHigh();
    GreenLEDLow();
    YellowLEDLow();
    BlueLEDLow();
    break;
  case 2:
    BlueLEDHigh();
    GreenLEDLow();
    YellowLEDLow();
    RedLEDLow();
    break;
  case 3:

    BlueLEDHigh();
    RedLEDHigh();
    GreenLEDLow();
    YellowLEDLow();

    break;
  case 4:
    YellowLEDHigh();
    GreenLEDLow();
    BlueLEDLow();
    RedLEDLow();
    break;
  case 5:
    YellowLEDHigh();
    RedLEDHigh();
    GreenLEDLow();
    BlueLEDLow();
    break;
  case 6:
    YellowLEDHigh();
    BlueLEDHigh();
    GreenLEDLow();
    RedLEDLow();
    break;
  case 7:
    YellowLEDHigh();
    BlueLEDHigh();
    RedLEDHigh();
    GreenLEDLow();

    break;
  case 8:
    GreenLEDHigh();
    YellowLEDLow();
    BlueLEDLow();
    RedLEDLow();

    break;
  case 9:
    GreenLEDHigh();
    RedLEDHigh();
    YellowLEDLow();
    BlueLEDLow();
    break;
  case 10:
    GreenLEDHigh();
    BlueLEDHigh();
    YellowLEDLow();
    RedLEDLow();
    break;
  case 11:
    GreenLEDHigh();
    BlueLEDHigh();
    RedLEDHigh();
    YellowLEDLow();
    break;
  case 12:
    GreenLEDHigh();
    YellowLEDHigh();
    BlueLEDLow();
    RedLEDLow();
    break;
  case 13:
    GreenLEDHigh();
    YellowLEDHigh();
    RedLEDHigh();
    BlueLEDLow();
    break;
  case 14:
    GreenLEDHigh();
    YellowLEDHigh();
    BlueLEDHigh();
    RedLEDLow();
    break;
  case 15:
    GreenLEDHigh();
    YellowLEDHigh();
    BlueLEDHigh();
    RedLEDHigh();
    break;
  }
}










