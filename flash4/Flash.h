#ifndef FLASH_H
#define FLASH_H

#include <Arduino.h>

void LoggingStateMachine();
void LogHandler();

void WriteBufferRemainder();
boolean WriteBufferHandler(uint8_t, uint8_t*);
void CheckEraseToPageBounds(uint16_t );
void VerifyPageWriteReady();
void LogBuilder();  
void SearchForLastRecord();
boolean GetRecordNumber(uint16_t, uint16_t*, uint16_t*, uint8_t*);
void CompleteRecord(uint16_t,uint16_t*,uint16_t*);

void WriteBufferRemainder();
boolean WriteBufferHandler(uint8_t , uint8_t*);
  
void FlashInit();
void LoggingInit();

uint8_t FlashGetByte(uint16_t,uint8_t);
boolean FlashGetArray(uint16_t,uint8_t,uint8_t, uint8_t*);
boolean FlashGetPage(uint16_t,uint16_t,uint8_t*);

boolean FlashWriteByte(uint16_t,uint8_t, uint8_t);
boolean FlashWriteByteBlocking(uint16_t,uint8_t, uint8_t);
boolean FlashWritePartialPage(uint16_t,uint8_t, uint8_t, uint8_t*);
boolean FlashWritePage(uint16_t, uint16_t, uint8_t*);

boolean ClearPage(uint16_t);
boolean FlashEraseBlock4k(uint16_t);
boolean FlashEraseBlock32k(uint16_t);
boolean FlashEraseBlock64k(uint16_t);
boolean FlashEraseChip();

boolean VerifyWriteReady();
boolean CheckForSuccessfulWrite();
uint8_t GetStatusReg();
void DispStatRegs();
//to do move after debugging
extern uint8_t loggingState;
extern boolean startNewLog,endCurrentLog,writePageStarted,loggingReady;

#endif
//
