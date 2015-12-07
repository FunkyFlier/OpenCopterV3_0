#include "Flash.h"
#include "Defines.h"
#include "SPI.h"
#include <Streaming.h>
#include <Arduino.h> 

uint16_t currentRecordNumber, currentRecordAddress, currentPageAddress, lowestRecordNumber, lowestRecordAddress;
uint32_u currentTime;

uint8_t writeBuffer[256];

uint8_t writeBufferIndex = 0;

boolean startNewLog = false,endCurrentLog = false,writePageStarted = false,loggingReady=false;

uint8_t loggingState = WRITE_READY;

void LoggingStateMachine(){

  static uint16_t nextBlockAddress = 0;
  static uint8_t pageCount = 0;
  uint16_u outInt16;

  switch(loggingState){
  case CHECK_4K:
    if(VerifyWriteReady() == false){
      break;
    }
    if (FlashGetByte((nextBlockAddress + pageCount),0) != 0xFF){
      loggingState = ERASE;
      break;
    }
    pageCount++;
    if(pageCount == 16){
      pageCount = 0;
      loggingState = WRITE_READY;
    }
    break;
  case ERASE:
    if(VerifyWriteReady() == false){
      break;
    }
    FlashEraseBlock4k(nextBlockAddress);
    loggingState = WRITE_READY;
    break;
  case WRITE_READY:
    if(VerifyWriteReady() == false){
      break;
    }
    if (startNewLog == true){
      endCurrentLog = false;
      startNewLog = false;
      loggingState = START_NEW_LOG;
      break;
    }
    if (endCurrentLog == true){
      endCurrentLog = false;
      startNewLog = false;
      loggingState = END_CURRENT_LOG;
      break;
    }
    if (startNewLog == false && endCurrentLog == false){
      loggingReady = true;
      if (writePageStarted == true){
        loggingState = COMPLETE_PAGE;
        loggingReady = false;
        writePageStarted = false;
      }
    }
    break;
  case COMPLETE_PAGE:
    if(VerifyWriteReady() == false){
      break;
    }
    FlashWriteByte(currentPageAddress,0,WRITE_COMPLETE);
    currentPageAddress += 1;
    if (currentPageAddress > 0x3FFF){
      currentPageAddress = 0;
    }
    if ((currentPageAddress & 0x000F) == 0x000F){
      loggingState = CHECK_4K;
      pageCount = 0;
      nextBlockAddress = (currentPageAddress & 0xFFF0) + 0x0010;
      if (nextBlockAddress > 0x3FF0){
        nextBlockAddress = 0;
      }
      break;
    }
    loggingState = WRITE_READY;
    break;
  case START_NEW_LOG:
    if(VerifyWriteReady() == false){
      break;
    }
    outInt16.val = currentRecordNumber;
    writeBuffer[0] = WRITE_STARTED_REC_START;
    writeBuffer[1] = outInt16.buffer[0];
    writeBuffer[2] = outInt16.buffer[1];
    writeBuffer[3] = 0xFF;
    writeBuffer[4] = 0xFF;
    writeBuffer[5] = 0xFF;
    FlashWritePartialPage(currentPageAddress,0,6,writeBuffer);
    writeBufferIndex = 6;
    loggingState = WRITE_READY;
    break;
  case END_CURRENT_LOG:
    if(VerifyWriteReady() == false){
      break;
    }
    WriteBufferRemainder();
    break;
  case COMPLETE_LAST_PAGE:  
    if(VerifyWriteReady() == false){
      break;
    }
    FlashWriteByte(currentPageAddress,0,WRITE_COMPLETE);
    loggingState = UPDATE_FIRST_PAGE;
    break;
  case UPDATE_FIRST_PAGE:
    if(VerifyWriteReady() == false){
      break;
    }
    outInt16.val = currentPageAddress;
    writeBuffer[0] = 0x00;
    writeBuffer[1] = outInt16.buffer[0];
    writeBuffer[2] = outInt16.buffer[1];
    FlashWritePartialPage(currentRecordAddress,3,3,writeBuffer);
    loggingState = BOUND_CHECK;
    break;
  case BOUND_CHECK:
    if(VerifyWriteReady() == false){
      break;
    }
    startNewLog = false;
    endCurrentLog = false;
    writePageStarted = false;
    loggingReady = false;
    currentPageAddress += 1;
    currentRecordAddress = currentPageAddress;
    if (currentPageAddress > 0x3FFF){
      currentPageAddress = 0;
    }
    if ((currentPageAddress & 0x000F) == 0x000F){
      loggingState = CHECK_4K;
      pageCount = 0;
      nextBlockAddress = (currentPageAddress & 0xFFF0) + 0x0010;
      if (nextBlockAddress > 0x3FF0){
        nextBlockAddress = 0;
      }
      break;
    }
    loggingState = WRITE_READY;
    break;

  }
}





void LogHandler(){

}

void WriteBufferRemainder(){
  //any data remaining in the output buffer is written 
  uint16_u outInt16;
  outInt16.val = currentRecordNumber;
  if (writeBufferIndex == 0){
    currentPageAddress -= 1;
    FlashWriteByte(currentPageAddress,0,WRITE_COMPLETE_REC_END);
    loggingState = UPDATE_FIRST_PAGE;
    return;
  }
  writeBuffer[0] = WRITE_STARTED_REC_END;
  writeBuffer[1] = outInt16.buffer[0];
  writeBuffer[2] = outInt16.buffer[1];
  for(uint16_t i = writeBufferIndex; i < 256; i++){
    writeBuffer[i] = 0xFF;
  }
  writeBufferIndex = 0;
  FlashWritePage(currentPageAddress,writeBuffer);
  loggingState = COMPLETE_LAST_PAGE;

}


boolean WriteBufferHandler(uint8_t numBytes, uint8_t inputBuffer[]){//make void?
  //takes data from the loghandler and writes it to the flash memory
  uint16_u outInt16;
  boolean bufferToFlash = false;

  for(uint16_t i = 0; i < numBytes; i++){
    writeBuffer[writeBufferIndex] = inputBuffer[i];
    writeBufferIndex++;
    if (writeBufferIndex == 0){
      outInt16.val = currentRecordNumber;
      FlashWritePage(currentPageAddress,writeBuffer);
      loggingState = COMPLETE_PAGE;
      writeBuffer[0] = WRITE_STARTED;
      writeBuffer[1] = outInt16.buffer[0];
      writeBuffer[2] = outInt16.buffer[1];
      writeBufferIndex = 4;
      loggingReady = false;
      bufferToFlash = true;

    }
  }

  return bufferToFlash;
}

void LoggingInit(){
  SearchForLastRecord();
  VerifyPageWriteReady(); 
}

void VerifyPageWriteReady(){
  //called during init 
  //checks to see if the next 4k block is erased 
  uint16_t next4KBoundary; 
  next4KBoundary = (currentPageAddress & 0xFFF0) + 0x0010;
  if (next4KBoundary > 0x3FF0){
    next4KBoundary = 0;
  }
  if ((currentPageAddress & BLOCK_MASK_4K) == 0x00){
    while(VerifyWriteReady() == false){
    }
    FlashEraseBlock4k(currentPageAddress);
    currentRecordAddress = currentPageAddress;
  }
  else{
    while(VerifyWriteReady() == false){
    }
    FlashEraseBlock4k(next4KBoundary);
    CheckEraseToPageBounds(currentPageAddress);
  }
  currentPageAddress = currentRecordAddress;
}

void CheckEraseToPageBounds(uint16_t currentAddress){
  //checks if the remaining pages in the block are erased
  //if the remaineder of the 4k block is not erased it is set to zero
  uint8_t numPagesToCheck;
  uint16_t next4KBoundary;
  numPagesToCheck = 0x10 - (currentAddress & 0x00F);
  next4KBoundary = (currentAddress & 0xFFF0) + 0x0010;
  if (next4KBoundary > 0x3FF0){
    next4KBoundary = 0;
  }
  for(uint8_t i = 0; i < numPagesToCheck; i++){
    while(VerifyWriteReady() == false){
    }
    if(FlashGetByte((currentAddress + i),0) != 0xFF){
      for(uint8_t i = 0; i < numPagesToCheck; i++){
        while(VerifyWriteReady() == false){
        }
        ClearPage(currentAddress + i);
      }
      currentRecordAddress = next4KBoundary;
      return;
    }
  }
  currentRecordAddress = currentAddress;

}
void SearchForLastRecord(){
  //called during init
  //set the record number and address to start logging
  //if start of log data is partial the log is completed
  boolean firstRecord = true;
  uint8_t  firstByte;
  uint16_t recordNumber,lastPageAddress;
  boolean validRecord,recordComplete;
  uint32_u fullAddress;
  for(uint16_t i = 0; i <= 0x3FFF; i++){
    //for(uint16_t i = 0; i <= 0x00FF; i++){
    fullAddress.val = (uint32_t)i << 8;
    FlashSSLow();
    SPI.transfer(READ_ARRAY);
    SPI.transfer(fullAddress.buffer[2]);
    SPI.transfer(fullAddress.buffer[1]);
    SPI.transfer(fullAddress.buffer[0]);
    firstByte = SPI.transfer(0);
    FlashSSHigh();

    if (firstByte == WRITE_COMPLETE_REC_START || firstByte == WRITE_COMPLETE_REC_START_END){
      GetRecordNumber(i,&recordNumber,&lastPageAddress,&recordComplete);
      if (recordComplete == false){
        CompleteRecord(i,&recordNumber,&lastPageAddress);
      }
      if (recordNumber >= currentRecordNumber){ 
        currentRecordNumber = recordNumber + 1;
        currentPageAddress = lastPageAddress + 1;
        if (currentPageAddress > 0x3FFF){
          currentPageAddress = 0;
        }
      }
      if (recordNumber == 0x3FFF){
        currentRecordNumber = 0;
        currentPageAddress = 0;
      }
      if (firstRecord == true){
        firstRecord = false;
        lowestRecordNumber = recordNumber;
      }
      if (recordNumber <= lowestRecordNumber){
        lowestRecordNumber = recordNumber;
        lowestRecordAddress = i;
      }
    }

  }

}

void CompleteRecord(uint16_t index, uint16_t* startingRecordNumber, uint16_t* finalAddress){
  //completes the start of log data
  boolean endOfRecordFound = false;
  uint8_t startByte;
  uint16_t searchCount = 0;
  uint16_u recordNumber,endAddress;
  uint16_t searchAddress,startingAddress;

  while(endOfRecordFound == false){
    searchAddress = index + searchCount;
    if (searchAddress > 0x3FFF){
      searchAddress -= 0x4000;
    }
    startByte = FlashGetByte(searchAddress,0);
    switch(startByte){
    case WRITE_COMPLETE://verify record number
      while(VerifyWriteReady() == false){
      }
      FlashGetArray(searchAddress,1,sizeof(recordNumber.buffer),recordNumber.buffer);
      if (recordNumber.val != *startingRecordNumber){
        endOfRecordFound = true;
        searchAddress -= 1;
        endAddress.val =  searchAddress;
        startingAddress = index;
        FlashWriteByteBlocking(searchAddress,   0,WRITE_COMPLETE_REC_END);
        FlashWriteByteBlocking(startingAddress, 3,0x00);
        FlashWriteByteBlocking(startingAddress, 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress, 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }
      break;
    case WRITE_COMPLETE_REC_START:
      if (searchAddress != index){
        endOfRecordFound = true;
        if (searchAddress == 0){
          searchAddress = 0x3FFF;
        }
        else{
          searchAddress -= 1;
        }
        endAddress.val =  searchAddress;
        startingAddress = index;

        FlashWriteByteBlocking(searchAddress,    0,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress , 3,0x00);
        FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
        break;
      }
      searchAddress += 1;
      if (searchAddress > 0x3FFF){
        searchAddress = 0;
      }
      while(VerifyWriteReady() == false){
      }
      FlashGetArray(searchAddress, 1,2,recordNumber.buffer);
      if (recordNumber.val != *startingRecordNumber){
        endOfRecordFound = true;
        startingAddress = index ;
        endAddress.val =  index;
        FlashWriteByteBlocking(startingAddress,  0,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress , 3,0x00);
        FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }
      break;
    case WRITE_COMPLETE_REC_END://
      while(VerifyWriteReady() == false){
      }
      FlashGetArray((searchAddress + 1) , 2,sizeof(recordNumber.buffer),recordNumber.buffer);
      startingAddress = index;
      if (recordNumber.val != *startingRecordNumber){
        endAddress.val =  searchAddress;
        FlashWriteByteBlocking(endAddress.val, 0 ,WRITE_COMPLETE_REC_END);
      }
      FlashWriteByteBlocking(startingAddress , 3,0x00);
      FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
      FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
      *finalAddress = endAddress.val;
      endOfRecordFound = true;
      break;
    default:
      endOfRecordFound = true;
      if (searchAddress == 0){
        searchAddress = 0x3FFF;
      }
      else{
        searchAddress -= 1;
      }
      if (searchAddress == index){
        while(VerifyWriteReady() == false){
        }
        startingAddress = index;
        endAddress.val =  index;
        FlashWriteByteBlocking(startingAddress, 0,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress , 3,0x00);
        FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }
      else{
        while(VerifyWriteReady() == false){
        }
        startingAddress = index;
        endAddress.val =  searchAddress;
        FlashWriteByteBlocking(endAddress.val, 0 ,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress , 3,0x00);
        FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }

      break;
    }

    searchCount++;
    if (searchCount == 0x3FFF){
      endOfRecordFound = true;
      startingAddress = index;

      if (index == 0){
        endAddress.val =  0x3FFF;
      }
      else{
        endAddress.val =  index - 1;
      }
      FlashWriteByteBlocking(startingAddress,  0,WRITE_COMPLETE_REC_END);

      FlashWriteByteBlocking(startingAddress , 3,0x00);
      FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
      FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
      *finalAddress = endAddress.val;
    }
  }

}
boolean GetRecordNumber(uint16_t index, uint16_t *recordNumber, uint16_t *endAddress, uint8_t *recordComplete){
  //if a start of record is found this sets the bounds for the record
  //todo return void
  uint16_u inInt16;
  uint32_u fullAddress;
  uint8_t inByte;


  fullAddress.val = ((uint32_t)index << 8) + 1;
  FlashSSLow();
  SPI.transfer(READ_ARRAY);
  SPI.transfer(fullAddress.buffer[2]);
  SPI.transfer(fullAddress.buffer[1]);
  SPI.transfer(fullAddress.buffer[0]);
  for(uint8_t i = 0; i < START_OF_REC_LEN; i++){
    inByte = SPI.transfer(0);
    switch(i){
    case 0:
      inInt16.buffer[0] = inByte;
      break;
    case 1:
      inInt16.buffer[1] = inByte;
      *recordNumber = inInt16.val;
      break;
    case 2://record complete
      if (inByte == 0x00){
        *recordComplete = true;
      }
      else{
        *recordComplete = false;
      }
      break;
    case 3://last page LSB
      if (*recordComplete == true){
        inInt16.buffer[0] = inByte;
      }
      break;
    case 4://last page LSB
      if (*recordComplete == true){
        inInt16.buffer[1] = inByte;
        *endAddress = inInt16.val;
      }
      else{
        *endAddress = 0;
      }
      break;
    }
  }
  FlashSSHigh();
  return false;



}


//low level functions read / write / erase /init
//init
void FlashInit(){
  //waits for the chip to be ready then unprotects all registers
  while(VerifyWriteReady() == false){
  } 

  FlashSSLow();
  SPI.transfer(WRITE_ENABLE);
  FlashSSHigh();
  FlashSSLow();
  SPI.transfer(STATUS_WRITE);
  SPI.transfer(0x00);
  FlashSSHigh();
  while(VerifyWriteReady() == false){
  } 
}
//read
uint8_t FlashGetByte(uint16_t pageAddress, uint8_t byteAddress){
  //returns a single byte
  uint32_u addressOutput;
  uint8_t inByte;
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(READ_ARRAY);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  inByte = SPI.transfer(0);
  FlashSSHigh();
  return inByte;
}

void FlashGetArray(uint16_t pageAddress, uint8_t byteAddress,uint8_t numBytes, uint8_t readBuffer[]){
  //returns an array of arbitrary length
  //to do make non blocking
  uint32_u addressOutput;
  
  if (numBytes > (256 - byteAddress) ){
    numBytes = 256 - byteAddress;
  }
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(READ_ARRAY);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < numBytes; i++){
    readBuffer[i] = SPI.transfer(0x00);
  }
  FlashSSHigh();
}
void FlashGetPage(uint16_t pageAddress,uint8_t readBuffer[]){
  //returns an entire page of data
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8);

  FlashSSLow();
  SPI.transfer(READ_ARRAY);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    readBuffer[i] = SPI.transfer(0x00);
  }
  FlashSSHigh();

}
//write
void FlashWriteByte(uint16_t pageAddress, uint8_t byteAddress, uint8_t writeByte){
  //writes a single byte
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  SPI.transfer(writeByte);
  FlashSSHigh();
}
//todo return for write functions? 
void FlashWriteByteBlocking(uint16_t pageAddress, uint8_t byteAddress, uint8_t writeByte){
  //waits for device ready then writes a single byte
  uint32_u addressOutput;
  while(VerifyWriteReady() == false){
  }
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  SPI.transfer(writeByte);
  FlashSSHigh();
}
void FlashWritePartialPage(uint16_t pageAddress, uint8_t byteAddress, uint8_t numBytes, uint8_t writeBuffer[]){
  //writes
  uint32_u addressOutput;
  if (numBytes > (256 - byteAddress) ){
    numBytes = 256 - byteAddress;//avoids wrap around to first byte
  }
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < numBytes; i++){
    SPI.transfer(writeBuffer[i]);
  }
  FlashSSHigh();
}

void FlashWritePage(uint16_t pageAddress, uint8_t writeBuffer[]){
  //writes 256 bytes to flash chip
  uint32_u addressOutput;


  addressOutput.val = ((uint32_t)pageAddress << 8);
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    SPI.transfer(writeBuffer[i]);
  }
  FlashSSHigh();
}
//erase
void ClearPage(uint16_t pageAddress){
  //sets all data on a page to zero
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8);
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    SPI.transfer(0x00);
  }
  FlashSSHigh();
}

boolean FlashEraseBlock4k(uint16_t blockAddress){
  //sets a 4k block to 0xFF
  uint32_u addressOutput;
  if (blockAddress > 0x3FF0){
    return false;
  }

  if ((blockAddress & BLOCK_MASK_4K) == 0x0000){
    addressOutput.val = ((uint32_t)blockAddress << 8);
    FlashSSLow();
    SPI.transfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPI.transfer(ERASE_4K);
    SPI.transfer(addressOutput.buffer[2]);
    SPI.transfer(addressOutput.buffer[1]);
    SPI.transfer(addressOutput.buffer[0]);
    FlashSSHigh();
    return true;
  }
  else{

    return false;
  } 
}
boolean FlashEraseBlock32k(uint16_t blockAddress){
  //sets a 32k block to 0xFF
  uint32_u addressOutput;
  if (blockAddress > 0x3F80){
    return false;
  }

  if ((blockAddress & BLOCK_MASK_32K) == 0x0000 ){
    addressOutput.val = ((uint32_t)blockAddress << 8);
    FlashSSLow();
    SPI.transfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPI.transfer(ERASE_32K);
    SPI.transfer(addressOutput.buffer[2]);
    SPI.transfer(addressOutput.buffer[1]);
    SPI.transfer(addressOutput.buffer[0]);
    FlashSSHigh();
    return true;
  }
  else{
    return false;
  } 
}
boolean FlashEraseBlock64k(uint16_t blockAddress){
  //sets a 64k block to 0xFF
  uint32_u addressOutput;
  if (blockAddress > 0x3F00){
    return false;
  }

  if ((blockAddress & BLOCK_MASK_64K) == 0x0000){
    addressOutput.val = ((uint32_t)blockAddress << 8);
    FlashSSLow();
    SPI.transfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPI.transfer(ERASE_64K);
    SPI.transfer(addressOutput.buffer[2]);
    SPI.transfer(addressOutput.buffer[1]);
    SPI.transfer(addressOutput.buffer[0]);
    FlashSSHigh();
    return true;
  }
  else{
    return false;
  } 
}
boolean FlashEraseChip(){
  //sets entire chip to 0xFF
  while(VerifyWriteReady() == false){
  }
  
  FlashSSLow();
  SPI.transfer(WRITE_ENABLE);
  FlashSSHigh();

  FlashSSLow();
  SPI.transfer(ERASE_CHIP);
  FlashSSHigh();
  
  while(VerifyWriteReady() == false){
  }
  
  if (CheckForSuccessfulWrite() == true){
    return true;
  }
  return false;

}
//status
boolean CheckForSuccessfulWrite(){
  //todo remove
  uint8_t statusReg;
  statusReg = GetStatusReg();
  if ( (statusReg & WRITE_ERROR_MASK) == 0x00){
    return true;
  }
  else{
    return false;
  }
}

uint8_t GetStatusReg(){
  //returns the first byte of the status registers
  uint8_t inByte1;//,inByte2;
  FlashSSLow();
  SPI.transfer(READ_STATUS_REG);
  inByte1 = SPI.transfer(0);
  //inByte2 = SPI.transfer(0);
  FlashSSHigh(); 
  return inByte1;
  //return 0 device ready write not enabled
  //return 1 device busy wrtie not enabled
  //return 2 device ready write enabled 
  //return 3 device busy write enabled 
}


boolean VerifyWriteReady(){
  uint8_t statusReg;
  statusReg = GetStatusReg() & 0x03;
  switch(statusReg){
  case 0://device ready write not enabled
    FlashSSLow();
    SPI.transfer(WRITE_ENABLE);
    FlashSSHigh();
    return true;
    break;
  case 1://device busy wrtie not enabled
    return false;
    break;
  case 2://device ready write enabled
    return true;
    break;
  case 3://device busy write enabled 
    return false;
    break;
  default:
    return false;
    break;
  }
}



void DispStatRegs(){
  //todo remove after debugging is complete
  uint8_t inByte1,inByte2;
  FlashSSLow();
  SPI.transfer(READ_STATUS_REG);
  inByte1 = SPI.transfer(0);
  inByte2 = SPI.transfer(0);
  FlashSSHigh(); 
  Serial<<"SB1: "<<_HEX(inByte1)<<"\r\nSB2: "<<_HEX(inByte2)<<"\r\n";
}














