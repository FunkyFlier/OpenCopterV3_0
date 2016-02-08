#ifndef COMM_H
#define COMM_H
#include "SerialPort_.h"

extern SerialPort<0, 255, 255>  Port0;
extern SerialPort<1, 255, 255>  RC_SERIAL_PORT;
extern SerialPort<2, 255, 255>  Port2;
extern SerialPort<3, 255, 255>  gpsPort;

void AssignRadioUSB();
void AssignRadioUART();
uint8_t RadioAvailableForWrite();
uint8_t RadioAvailable();
uint8_t RadioRead();
void RadioWrite(uint8_t);

void GPSSerialBegin(uint32_t);
void  GPSSerialWrte(uint8_t);
uint8_t  GPSSerialRead();
uint8_t  GPSSerialAvailable();

void RCSerialWrite(uint8_t);
uint8_t RCSerialRead();
uint8_t RCSerialAvailable(); 
void RCSerialBegin(uint32_t);
void RCSerialBegin(uint32_t,uint8_t);

void RadioSerialBegin();

uint8_t SPITransfer(uint8_t);
void    SPIInit(uint8_t, uint8_t, uint8_t);
void    SPISetMode(uint8_t);

void    I2CInit();
uint8_t I2CRead(uint8_t, uint8_t, uint8_t);
uint8_t I2CReadStopStart(uint8_t, uint8_t, uint8_t);
uint8_t I2CWrite(uint8_t, uint8_t, uint8_t);
uint8_t I2CReceive();

void    EEPROMWrite(uint16_t,uint16_t);
uint8_t EEPROMRead(uint16_t);

#endif//#ifndef COMM_H

