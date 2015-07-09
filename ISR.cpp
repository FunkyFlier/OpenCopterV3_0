#include "ISR.h"
#include "Definitions.h"
#include "RCSignals.h"
#include "LED.h"
#include "FlightControl.h"
#include "GPS.h"
#include "Enums.h"
#include "Radio.h"

volatile boolean watchDogStartCount = false;

void _200HzISRConfig(){
  TCCR5A = (1<<COM5A1);
  TCCR5B = (1<<CS51)|(1<<WGM52);
  TIMSK5 = (1<<OCIE5A);
  OCR5A = 10000;
}

ISR(TIMER5_COMPA_vect, ISR_NOBLOCK){
  if (watchDogStartCount == true){
    watchDogFailSafeCounter++;
    if (rcDetected == true){
      RCFailSafeCounter++;
    }
    groundFSCount++;
    GPSFailSafeCounter++;
  }
  else{
    if (handShake == true){
      Radio();
      TuningTransmitter(); 
    }
  }
  if (rcType != RC){
    FeedLine();
  }
  if (watchDogFailSafeCounter >=200){
    TIMSK5 = (0<<OCIE5A);
    Motor1WriteMicros(0);//set the output compare value
    Motor2WriteMicros(0);
    Motor3WriteMicros(0);
    Motor4WriteMicros(0);
    Motor5WriteMicros(0);
    Motor6WriteMicros(0);
    Motor7WriteMicros(0);
    Motor8WriteMicros(0);
    while(1){
      ControlLED(0x09);
      delay(500);
      ControlLED(0x0D);
      delay(500);
      ControlLED(0x0F);
      delay(500);
      ControlLED(0x0B);
      delay(500);
    }
  }
}




