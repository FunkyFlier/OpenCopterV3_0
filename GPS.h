#ifndef GPS_h
#define GPS_h
#include <Arduino.h>

#include "Math.h"
#include "Definitions.h"
#include "Types.h"

#define RADIUS_EARTH 6372795
#define SACC_MAX 10000//0.1
#define HACC_MAX 10000//1

void GPSInit();
void DistBearing(int32_t*, int32_t*, int32_t*, int32_t*, float*, float*, float*, float*);
void GPSMonitor();
void GPSStart();

typedef union{
  struct{
    uint32_t iTWO;
    int32_t lon;
    int32_t lat;
    int32_t height;
    int32_t hMSL;
    uint32_t hAcc;
    uint32_t vAcc;
    int32_t velN;
    int32_t velE;
    int32_t velD;
    uint32_t speed3D;
    uint32_t speed2D;
    int32_t heading;
    uint32_t sAcc;
    uint32_t cAcc;
    uint8_t gpsFix;
  }
  vars;
  byte buffer[61];
}
GPS_Union_t;

extern boolean newGPSData,GPSDetected;
extern boolean gpsFailSafe;
extern GPS_Union_t GPSData;
extern float gpsAlt,floatLat, floatLon,velN, velE, velD;
extern int32_t homeLat,homeLon;
extern float homeLatFloat,homeLonFloat;
extern float hAcc,sAcc;
extern volatile uint32_t GPSFailSafeCounter;
extern uint8_t gpsStartState;


#endif

