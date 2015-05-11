#include "LED.h"
//Green Yellow Blue Red
void LEDInit(){
  GreenLEDOutput();
  YellowLEDOutput();
  BlueLEDOutput();
  RedLEDOutput();  

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


