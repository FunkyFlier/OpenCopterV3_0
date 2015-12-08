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
    baroFSCount++;
  }
  else{
    if (handShake == true && calibrationMode == false && getFlashMode == false){
      Radio();
      TuningTransmitter(); 
    }
    LEDPatternHandler(micros());
  }
  if (rcType != RC){
    FeedLine();
  }
  if (watchDogFailSafeCounter >=200){
    TIMSK5 = (0<<OCIE5A);
    Motor1WriteMicros(0);
    Motor2WriteMicros(0);
    Motor3WriteMicros(0);
    Motor4WriteMicros(0);
    Motor5WriteMicros(0);
    Motor6WriteMicros(0);
    Motor7WriteMicros(0);
    Motor8WriteMicros(0);
    LEDPatternSet(0,1,0,1);
    while(1){
      LEDPatternHandler(millis());
    }
  }
}






