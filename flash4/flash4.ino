#include <SPI.h>
#include <Streaming.h>
#include "Flash.h"
#include "Defines.h"

uint8_t byteBuffer[256];
uint8_t logTestBuffer[100];
void setup(){
  GyroSSOutput();
  AccSSOutput();
  BaroSSOutput();
  MagSSOutput();
  FlashSSOutput();
  GyroSSHigh();
  AccSSHigh();
  BaroSSHigh();
  MagSSHigh();
  FlashSSHigh();

  SPI.begin();
  SPI.setBitOrder(MSBFIRST);
  SPI.setClockDivider(SPI_CLOCK_DIV2);   
  SPI.setDataMode(SPI_MODE0);

  Serial.begin(115200);

  FlashInit();

  /*Serial<<"erase chip\r\n";
  FlashEraseChip();
  Serial<<"last record search\r\n";*/
  //FlashEraseBlock4k(0);
  //FlashEraseBlock64k(0);
  //FlashEraseChip();
  //MakeRecord();
  SearchForLastRecord();
  VerifyPageWriteReady();
  Serial<<"after verify ready----------------------------------\r\n";
  FlashDump(0x00,0x100);
  FlashDump(0x3FE0,0x3FFF);
  //FlashDump(0x00,0x04);
  //Serial<<"verify erase\r\n";
  //VerifyErase();
  /*Serial<<"Setup record\r\n";
  MakeRecord();
  Serial<<"last record search\r\n";
  SearchForLastRecord();
  Serial<<"last record search complete\r\n";
  VerifyPageWriteReady();
  Serial<<"---\r\n";
  Serial<<"byte dump0\r\n";
  FlashDumpBytes(0x00,0x00,7);
  FlashDumpBytes(0x0E,0x0E,7);
  FlashDumpBytes(0x0F,0x0F,7);
  FlashDumpBytes(0x10,0x10,7);
  FlashDumpBytes(0x11,0x11,7);
  FlashDumpBytes(0x1F,0x1F,7);
  FlashDumpBytes(0x3FFD,0x3FFD,7);
  FlashDumpBytes(0x3FFE,0x3FFE,7);
  FlashDumpBytes(0x3FFF,0x3FFF,7);*/
  LoggingTest();
  
  /*Serial<<"fill flash\r\n";
   FillFlash();
   Serial<<"verify fill\r\n";
   VerifyFill();
   Serial<<"4k\r\n";
   Test4KErase();
   Serial<<"32k\r\n";
   Test32KErase();
   Serial<<"64k\r\n";
   Test64KErase();*/
  Serial<<"tests complete\r\n";

}

void loop(){

}

void LoggingTest(){
  uint16_t logCount = 0;
  for(uint8_t i = 0; i < 100; i++){
    logTestBuffer[i] = i;
  }
  logTestBuffer[0] = 0xAA;
  logTestBuffer[99] = 0x55;
  startNewLog = true;
  while(logCount < 60){
    LoggingStateMachine();
    if (loggingReady == true){
      //Serial<<"log count: "<<logCount<<"\r\n";
      //Serial<<WriteBufferHandler(sizeof(logTestBuffer),logTestBuffer)<<"\r\n";
      //writePageStarted = WriteBufferHandler(sizeof(logTestBuffer),logTestBuffer);
      WriteBufferHandler(sizeof(logTestBuffer),logTestBuffer);
      logCount++;
    }
  }
  endCurrentLog = true;
  LoggingStateMachine();
  while(loggingState != END_CURRENT_LOG){
    LoggingStateMachine();
  }
  while(loggingState != WRITE_READY){
    LoggingStateMachine();
  }
  FlashDump(0x00,0x100);
  FlashDump(0x3FE0,0x3FFF);
}

void MakeRecord(){
  for(uint16_t i = 0; i < 256; i++){
    byteBuffer[i] = 0xAA;
  }
  byteBuffer[0] = 0x1F;
  byteBuffer[1] = 0x00;
  byteBuffer[2] = 0x00;
  byteBuffer[3] = 0xFF;
  byteBuffer[4] = 0xFF;
  byteBuffer[5] = 0xFF;
  while(VerifyWriteReady() == false){
    //Serial<<"1\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x3FE0,256,byteBuffer);

  byteBuffer[0] = 0x3F;
  byteBuffer[1] = 0x00;
  byteBuffer[2] = 0x00;
  for (uint16_t i = 0x3FE1; i < 0x3FF1; i++){
    while(VerifyWriteReady() == false){
      //Serial<<"2\r\n";
      //DispStatRegs();
    }
    FlashWritePage(i,256,byteBuffer);
  }

  byteBuffer[0] = 0x2F;
  byteBuffer[1] = 0x00;
  byteBuffer[2] = 0x00;
  while(VerifyWriteReady() == false){
    //Serial<<"3\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x3FF1,256,byteBuffer);
  
 /* byteBuffer[0] = 0x7F;
  byteBuffer[1] = 0x01;
  byteBuffer[2] = 0x00;
  byteBuffer[3] = 0xFF;
  byteBuffer[4] = 0xFF;
  byteBuffer[5] = 0xFF;
  while(VerifyWriteReady() == false){
    //Serial<<"1\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x12,256,byteBuffer);*/
  Serial<<"initial falsh dump----------------------------------\r\n";
  FlashDump(0x00,0x100);
  FlashDump(0x3FE0,0x3FFF);
/*  byteBuffer[0] = 0x3F;
  byteBuffer[1] = 0x09;
  byteBuffer[2] = 0x00;
  while(VerifyWriteReady() == false){
    //Serial<<"3\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x0E,256,byteBuffer);


  //Serial<<"0*\r\n";*/

  //-------------------------------------------------
 /* byteBuffer[0] = 0x7F;
  byteBuffer[1] = 0x01;
  byteBuffer[2] = 0x00;
  byteBuffer[3] = 0xFF;
  byteBuffer[4] = 0xFF;
  byteBuffer[5] = 0xFF;
  while(VerifyWriteReady() == false){
    //Serial<<"1\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x10,256,byteBuffer);*/
/*
  byteBuffer[0] = 0x3F;
  byteBuffer[1] = 0x01;
  byteBuffer[2] = 0x00;
  for (uint16_t i = 0x10; i < 0x3FFD; i++){
    //for (uint16_t i = 0x102; i < 0x200; i++){
    while(VerifyWriteReady() == false){
      //Serial<<"2\r\n";
      //DispStatRegs();
    }
    FlashWritePage(i,256,byteBuffer);
  }

  byteBuffer[0] = 0x3F;
  byteBuffer[1] = 0x01;
  byteBuffer[2] = 0x00;
  while(VerifyWriteReady() == false){
    //Serial<<"3\r\n";
    //DispStatRegs();
  }
  FlashWritePage(0x3FFD,256,byteBuffer);
  //Serial<<"0x100*\r\n";
  FlashDumpBytes(0x00,0x00,7);
  FlashDumpBytes(0x0E,0x0E,7);
  FlashDumpBytes(0x0F,0x0F,7);
  FlashDumpBytes(0x10,0x10,7);
  FlashDumpBytes(0x11,0x11,7);
  FlashDumpBytes(0x1F,0x1F,7);
  FlashDumpBytes(0x3FFD,0x3FFD,7);
  FlashDumpBytes(0x3FFE,0x3FFE,7);
  FlashDumpBytes(0x3FFF,0x3FFF,7);
  //-------------------------------------------------

*/
}

/*void FillCompleteRecords(){
 for(uint16_t i = 0; i <= 0x3F; i++){
 
 }
 }*/
void VerifyFillPartial(uint16_t startPage, uint16_t endPage){
  uint16_u inAddress;
  uint16_t pageNumber;
  uint32_t fullAddr;
  uint8_t inByte;
  if (endPage > 0x3FFF){
    endPage = 0x3FFF;
  }
  if (startPage >= endPage){
    startPage = 0;
  }
  for(uint16_t i = startPage; i <= endPage; i++){
    while(VerifyWriteReady() == false){
      Serial<<"* ";
      DispStatRegs();
    }
    inAddress.buffer[0] = FlashGetByte(i,0);
    inAddress.buffer[1] = FlashGetByte(i,1);
    if (inAddress.val != i){
      Serial<<"address set wrong\r\n"<<i<<"\r\n";
      Serial<<_HEX(inAddress.buffer[0])<<","<<_HEX(inAddress.buffer[1])<<","<<inAddress.val<<"\r\n";
    }
    for(uint16_t j = 2; j < 256; j++){
      inByte = FlashGetByte(i,j);
      if (inByte != (uint8_t)j){
        fullAddr = ((uint32_t)i << 8) + (uint32_t)j;
        Serial<<"byte set wrong\r\n"<<fullAddr<<","<<i<<","<<j<<","<<inByte<<"\r\n";
      }
    }
  }
}

void VerifyErasePartial(uint16_t startPage, uint16_t endPage){
  uint16_t pageNumber;
  uint8_t inByte;
  if (endPage > 0x3FFF){
    endPage = 0x3FFF;
  }
  if (startPage >= endPage){
    startPage = 0;
  }
  for(uint16_t i = startPage; i <= endPage; i++){
    while(VerifyWriteReady() == false){
      Serial<<"* ";
      DispStatRegs();
    }
    for(uint16_t j = 0; j < 256; j++){
      inByte = FlashGetByte(i,(uint8_t)j);
      if (inByte != 0xFF){
        pageNumber = (i << 8) + j;
        Serial<<"page not erased\r\n";
        Serial<<pageNumber<<","<<i<<","<<j<<"\r\n";
      }
    }
  }
}
void VerifyFill(){
  uint16_u inAddress;
  uint16_t pageNumber;
  uint32_t fullAddr;
  uint8_t inByte;
  //DumpTheTwoBytes();
  for(uint16_t i = 0; i <= 0x3FFF; i++){
    //for(uint16_t i = 0; i < 1024; i++){  
    //Serial<<"1 - "<<i<<"\r\n";
    while(VerifyWriteReady() == false){
      Serial<<"* ";
      DispStatRegs();
    }
    inAddress.buffer[0] = FlashGetByte(i,0);
    inAddress.buffer[1] = FlashGetByte(i,1);
    //Serial<<"2\r\n";
    if (inAddress.val != i){
      Serial<<"address set wrong\r\n"<<i<<"\r\n";
      Serial<<_HEX(inAddress.buffer[0])<<","<<_HEX(inAddress.buffer[1])<<","<<inAddress.val<<"\r\n";
    }
    //Serial<<"3\r\n";
    for(uint16_t j = 2; j < 256; j++){
      //Serial<<"*";
      inByte = FlashGetByte(i,j);
      if (inByte != (uint8_t)j){
        fullAddr = ((uint32_t)i << 8) + (uint32_t)j;
        Serial<<"byte set wrong\r\n"<<fullAddr<<","<<i<<","<<j<<","<<inByte<<"\r\n";
        //DumpTheTwoBytes();
      }
    }
    //Serial<<"\r\n4\r\n";
  }

}
void VerifyErase(){
  uint16_t pageNumber;
  uint8_t inByte;
  for(uint16_t i = 0; i <= 0x3FFF; i++){
    for(uint16_t j = 0; j < 256; j++){
      while(VerifyWriteReady() == false){
        Serial<<"* ";
        DispStatRegs();
      }
      inByte = FlashGetByte(i,(uint8_t)j);
      if (inByte != 0xFF){
        pageNumber = (i << 8) + j;
        Serial<<"full not erased\r\n";
        Serial<<pageNumber<<","<<i<<","<<j<<"\r\n";
      }
    }
  }
}

void Test4KErase(){
  uint8_t eraseByte;
  Serial<<"-----\r\n4kdump before erase\r\n";
  VerifyFillPartial(0,15);
  Serial<<"a\r\n";
  while(VerifyWriteReady() == false){
    DispStatRegs();
  }
  Serial<<"b\r\n";
  eraseByte = FlashEraseBlock4k(0); 
  if (eraseByte == true){
    Serial<<"erase succeded\r\n";
    Serial<<"----------\r\n4kdump after erase\r\n"; 
    VerifyErasePartial(0,15);

  }
  else{
    Serial<<"erase failed\r\n";
  }

}

void Test32KErase(){

  uint8_t eraseByte;
  Serial<<"-----\r\n32kdump before erase\r\n";
  VerifyFillPartial(128,255);

  Serial<<"a\r\n";
  while(VerifyWriteReady() == false){
    DispStatRegs();
  }
  Serial<<"b\r\n";
  eraseByte = FlashEraseBlock32k(128); 
  if (eraseByte == true){
    Serial<<"erase succeded\r\n";
    Serial<<"----------\r\n32kdump after erase\r\n"; 
    VerifyErasePartial(128,255);

  }
  else{
    Serial<<"erase 32 1failed\r\n";
  }

  Serial<<"-----\r\n32k 2 dump before erase\r\n";
  VerifyFillPartial(256,383);

  Serial<<"c\r\n";
  while(VerifyWriteReady() == false){
    DispStatRegs();
  }
  Serial<<"d\r\n";
  eraseByte = FlashEraseBlock32k(256); 
  if (eraseByte == true){
    Serial<<"erase succeded\r\n";
    Serial<<"----------\r\n32kdump after erase\r\n"; 
    VerifyErasePartial(256,383);

  }
  else{
    Serial<<"erase 32 2 1failed\r\n";
  }

}

void Test64KErase(){
  uint8_t eraseByte;
  Serial<<"-----\r\n64kdump before erase\r\n";
  VerifyFillPartial(512,767);
  Serial<<"e\r\n";
  while(VerifyWriteReady() == false){
    DispStatRegs();
  }
  Serial<<"f\r\n";
  eraseByte = FlashEraseBlock64k(512); 
  if (eraseByte == true){
    Serial<<"erase succeded\r\n";
    Serial<<"----------\r\n64kdump after erase\r\n"; 
    VerifyErasePartial(512,767);

  }
  else{
    Serial<<"erase 64 failed\r\n";
  }
}
void FillFlash(){
  uint16_u pageNumber;
  uint8_t outputArray[256];
  for(uint16_t j = 0; j <= 256; j++){
    outputArray[j] = (uint8_t)j;
  }


  for(uint16_t i = 0; i <= 0x3FFF; i++){
    pageNumber.val = i;
    outputArray[0] = pageNumber.buffer[0];
    outputArray[1] = pageNumber.buffer[1];
    //Serial<<i<<","<<pageNumber.val<<","<<_HEX(outputArray[0])<<","<<_HEX(outputArray[1])<<"\r\n";
    while(VerifyWriteReady() == false){
    }
    FlashWritePage(i,sizeof(outputArray),outputArray);
  }
}
void FlashDump(uint16_t lowerBound, uint16_t upperBound){
  uint8_t outputArray[256];
  uint8_t inByte;
  if (upperBound > 0x3FFF){
    upperBound = 0x3FFF;
  }
  if (lowerBound > upperBound){
    lowerBound = 0;
  }
  for(uint16_t i = lowerBound; i <= upperBound; i++){
    Serial<<"page: "<<i<<"\r\n";
    for(uint16_t j = 0; j < 256; j++){
      outputArray[j] = 0;
    }
    while(VerifyWriteReady() == false){
      Serial<<"* ";
      DispStatRegs();
    }
    if (FlashGetPage(i,sizeof(outputArray),outputArray) == false){
      Serial<<"failed to get page\r\n";
      while(1){
        Serial<<"fail\r\n";
        delay(2000);
      }
    }
    else{
      for(uint16_t j = 0; j < 256; j++){
        Serial<<_HEX(outputArray[j])<<",";
      }
      Serial<<"\r\n";
    }
  }
}


void FlashDumpBytes(uint16_t lowerBound, uint16_t upperBound,uint8_t numBytes){
  uint8_t outputArray[256];
  uint8_t inByte;
  if (upperBound > 0x3FFF){
    upperBound = 0x3FFF;
  }
  if (lowerBound > upperBound){
    lowerBound = 0;
  }
  for(uint16_t i = lowerBound; i <= upperBound; i++){
    Serial<<"page: "<<i<<"\r\n";
    for(uint16_t j = 0; j < 256; j++){
      outputArray[j] = 0;
    }
    while(VerifyWriteReady() == false){
      //Serial<<"* ";
      //DispStatRegs();
    }
    if (FlashGetPage(i,sizeof(outputArray),outputArray) == false){
      Serial<<"failed to get page\r\n";
      while(1){
        Serial<<"fail\r\n";
        delay(2000);
      }
    }
    else{
      for(uint16_t j = 0; j < numBytes; j++){
        Serial<<_HEX(outputArray[j])<<"\r\n";
      }
    }
  }
}



























