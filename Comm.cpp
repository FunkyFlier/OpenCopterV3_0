#include <SPI.h>
#include "I2C_.h"
#include <EEPROM.h>
#include <Arduino.h>
#include "Definitions.h"

Print* radioPrint;
Stream* radioStream;

void AssignRadioUSB(){
  radioStream = &Port0;
  radioPrint = &Port0;
}

void AssignRadioUART(){
  radioStream = &Port2;
  radioPrint = &Port2;

}


uint8_t RadioAvailable(){
  return radioStream->available();
}

uint8_t RadioRead(){
  return radioStream->read();
}

void RadioWrite(uint8_t outByte){
  radioPrint->write(outByte);
}


void RCSerialBegin(uint32_t baudRate, uint8_t config){
  RC_SERIAL_PORT.begin(baudRate,config);
}

void RCSerialBegin(uint32_t baudRate){
  RC_SERIAL_PORT.begin(baudRate);
}

void RCSerialWrte(uint8_t outByte){
  RC_SERIAL_PORT.write(outByte);
}

uint8_t RCSerialRead(){
  return RC_SERIAL_PORT.read();
}

uint8_t RCSerialAvailable(){
  return RC_SERIAL_PORT.available();
}

void RadioSerialBegin(){
  Port0.begin(115200);
  Port2.begin(115200);
  Port2.write(0x0D);//in case a programmable XBEE is attached
  delay(500);
  Port2.print("B");
}

void SPIInit(uint8_t endian,uint8_t clockDiv,uint8_t mode){
  SPI.begin();
  SPI.setBitOrder(endian);
  SPI.setClockDivider(clockDiv);
  SPI.setDataMode(mode);
}

uint8_t SPITransfer(uint8_t writeByte){
  return SPI.transfer(writeByte);
}

void SPISetMode(uint8_t mode){
  SPI.setDataMode(mode);
}


void I2CInit(){
  I2c.begin();
  I2c.setSpeed(1);
  I2c.timeOut(2);
}

uint8_t I2CReadStopStart(uint8_t deviceAddress, uint8_t registerAddress, uint8_t numBytes){
  return I2c.readStopStart(deviceAddress, registerAddress, numBytes);
}

uint8_t I2CRead(uint8_t deviceAddress, uint8_t registerAddress, uint8_t numBytes){
  return I2c.read(deviceAddress, registerAddress, numBytes);
}

uint8_t I2CWrite(uint8_t deviceAddress, uint8_t registerAddress, uint8_t value){
  return I2c.write(deviceAddress, registerAddress, value);
}

uint8_t I2CReceive(){
  return I2c.receive();
}


void EEPROMWrite(uint16_t address,uint16_t data){
  EEPROM.write(address,data);
}

uint8_t EEPROMRead(uint16_t address){
  return EEPROM.read(address);
}



