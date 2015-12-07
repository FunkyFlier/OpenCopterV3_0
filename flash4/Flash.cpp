#include "Flash.h"
#include "Defines.h"
#include "SPI.h"
#include <Streaming.h>
#include <Arduino.h> 

#define BLOCK_MASK_4K 0x000F
#define BLOCK_MASK_32K 0x007F
#define BLOCK_MASK_64K 0x00FF
#define LOG_RATE 100



uint16_t currentRecordNumber, currentRecordAddress, currentPageAddress, lowestRecordNumber, lowestRecordAddress;
uint32_u currentTime;

uint8_t writeBuffer[256];//,loggingBuffer[256];

uint8_t writeBufferIndex = 0;

boolean startNewLog = false,endCurrentLog = false,writePageStarted = false,loggingReady=false;

uint8_t loggingState = WRITE_READY;

void LoggingStateMachine(){
  //handles chip erasure and ready status 
  //set the ready to log boolean
  static uint16_t nextBlockAddress = 0;
  static uint8_t pageCount = 0;
  //static uint8_t loggingState = WRITE_READY;
  uint16_u outInt16;

  switch(loggingState){
  case CHECK_4K:
    Serial<<"CHECK_4K\r\n";
    //loggingReady = false;
    if(VerifyWriteReady() == false){
      break;
    }
    Serial<<_HEX(FlashGetByte((nextBlockAddress + pageCount),0))<<"\r\n";
    if (FlashGetByte((nextBlockAddress + pageCount),0) != 0xFF){
      Serial<<"found to be erased\r\n";
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
    Serial<<"ERASE\r\n";
    if(VerifyWriteReady() == false){
      break;
    }
    FlashEraseBlock4k(nextBlockAddress);
    loggingState = WRITE_READY;
    break;
  case WRITE_READY:
    Serial<<"WRITE_READY\r\n";
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
        Serial<<"writePageStarted\r\n";        
        loggingState = COMPLETE_PAGE;
        loggingReady = false;
        writePageStarted = false;
      }
    }
    break;
  case COMPLETE_PAGE:
    Serial<<"COMPLETE_PAGE\r\n";
    if(VerifyWriteReady() == false){
      Serial<<"not write ready\r\n";
      break;
    }
    FlashWriteByte(currentPageAddress,0,WRITE_COMPLETE);
    currentPageAddress += 1;
    if (currentPageAddress > 0x3FFF){
      currentPageAddress = 0;
    }
    //Serial<<"test &: "<<","<<_HEX(currentPageAddress)<<","<<_HEX(currentPageAddress & 0x000F)<<"\r\n";
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
    Serial<<"START_NEW_LOG\r\n";
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
    Serial<<"END_CURRENT_LOG\r\n";
    if(VerifyWriteReady() == false){
      break;
    }

    WriteBufferRemainder();
    /*currentRecordAddress = currentPageAddress + 1;
     if (currentRecordAddress > 0x3FFF){
     currentRecordAddress = 0;
     }*/
    //loggingState = COMPLETE_LAST_PAGE;
    break;
  case COMPLETE_LAST_PAGE:  
    Serial<<"COMPLETE_LAST_PAGE\r\n";
    if(VerifyWriteReady() == false){
      Serial<<"not write ready\r\n";
      break;
    }
    FlashWriteByte(currentPageAddress,0,WRITE_COMPLETE);
    loggingState = UPDATE_FIRST_PAGE;
    break;
  case UPDATE_FIRST_PAGE:
    Serial<<"UPDATE_FIRST_PAGE\r\n";
    if(VerifyWriteReady() == false){
      Serial<<"not write ready\r\n";
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
    Serial<<"BOUND_CHECK\r\n";
    if(VerifyWriteReady() == false){
      Serial<<"not write ready\r\n";
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
  //checks loggingReady and time interval to call the function to write the buffer
  //do logging
  //call WriteBufferHandler
}
void FlashDump2(uint16_t lowerBound, uint16_t upperBound){
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
        Serial<<_HEX(outputArray[j])<<"\r\n";
      }
    }
  }
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
  FlashWritePage(currentPageAddress,256,writeBuffer);
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
      FlashWritePage(currentPageAddress,256,writeBuffer);
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
  Serial<<_HEX(next4KBoundary)<<","<<_HEX(currentPageAddress)<<"\r\n";
  if (next4KBoundary > 0x3FF0){
    next4KBoundary = 0;
  }
  if ((currentPageAddress & BLOCK_MASK_4K) == 0x00){
    Serial<<"at 4k bound\r\n";
    while(VerifyWriteReady() == false){
    }
    FlashEraseBlock4k(currentPageAddress);
    currentRecordAddress = currentPageAddress;
  }
  else{
    Serial<<"not at 4k bound\r\n";
    while(VerifyWriteReady() == false){
    }
    FlashEraseBlock4k(next4KBoundary);
    CheckEraseToPageBounds(currentPageAddress);
  }
  currentPageAddress = currentRecordAddress;
  Serial<<"current addr: "<<currentRecordAddress<<","<<currentPageAddress<<"\r\n";
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
  Serial<<"cetpb"<<numPagesToCheck<<","<<next4KBoundary<<"\r\n";
  for(uint8_t i = 0; i < numPagesToCheck; i++){
    while(VerifyWriteReady() == false){
    }
    if(FlashGetByte((currentAddress + i),0) != 0xFF){
      Serial<<"first byte failed\r\n";
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
      Serial<<"start byte found\r\n";
      Serial<<_HEX(firstByte)<<"\r\n";
      //validRecord = GetRecordNumber(i,&recordNumber,&lastPageAddress,&recordComplete);
      GetRecordNumber(i,&recordNumber,&lastPageAddress,&recordComplete);
      Serial<<recordNumber<<","<<lastPageAddress<<","<<recordComplete<<"\r\n";
      //Serial<<validRecord<<"\r\n";
      //handle incomplete record for WRITE_COMPLETE_REC_START_END
      if (recordComplete == false){
        Serial<<"incomplete record found\r\n";
        CompleteRecord(i,&recordNumber,&lastPageAddress);
        Serial<<"record completed\r\n";
      }
      if (recordNumber >= currentRecordNumber){ 
        currentRecordNumber = recordNumber + 1;
        currentPageAddress = lastPageAddress + 1;
        if (currentPageAddress > 0x3FFF){
          currentPageAddress = 0;
        }
      }
      if (recordNumber == 0x3FFF){//or 3FFF?
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

      Serial<<recordNumber<<","<<currentRecordNumber<<","<<_HEX(currentPageAddress)<<"\r\n";
      /*if(validRecord == true){
       if (recordComplete == false){
       CompleteRecord(i,&recordNumber);
       }
       if (recordNumber >= currentRecordNumber){ 
       currentRecordNumber = recordNumber + 1;
       currentPageAddress = lastPageAddress + 1;
       }
       if (recordNumber == 0x3FFF){//or 3FFF?
       currentRecordNumber = 0;
       currentPageAddress = 0;
       }
       
       if (recordNumber <= lowestRecordNumber){
       lowestRecordNumber = recordNumber;
       lowestRecordAddress = i;
       }
       //Serial<<recordNumber<<","<<currentRecordNumber<<","<<currentPageAddress<<"\r\n";
       }*/
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

    //searchAddress = ((uint32_t)index + searchCount) << 8;
    searchAddress = index + searchCount;
    //Serial<<"search addr1: "<<searchAddress<<","<<searchCount<<"\r\n";
    if (searchAddress > 0x3FFF){
      //searchAddress -= 0x3FFF;
      searchAddress -= 0x4000;
    }
    //Serial<<"search addr2: "<<searchAddress<<","<<searchCount<<"\r\n";
    startByte = FlashGetByte(searchAddress,0);
    switch(startByte){
    case WRITE_COMPLETE://verify record number
      //Serial<<"a\r\n";

      FlashGetArray(searchAddress,1,sizeof(recordNumber.buffer),recordNumber.buffer);
      if (recordNumber.val != *startingRecordNumber){
        Serial<<"a**\r\n";

        endOfRecordFound = true;
        searchAddress -= 1;
        endAddress.val =  searchAddress;

        startingAddress = index;
        Serial<<_HEX(startByte)<<","<<searchAddress<<","<<recordNumber.val<<","<<*startingRecordNumber<<"\r\n";
        FlashWriteByteBlocking(searchAddress,   0,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress, 3,0x00);
        FlashWriteByteBlocking(startingAddress, 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress, 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }
      break;
    case WRITE_COMPLETE_REC_START:
      Serial<<"b\r\n";
      if (searchAddress != index){
        Serial<<"1b\r\n";
        endOfRecordFound = true;
        if (searchAddress == 0){
          Serial<<"2b\r\n";
          searchAddress = 0x3FFF;
        }
        else{
          Serial<<"3b\r\n";
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
      Serial<<"4b\r\n";
      if (searchAddress > 0x3FFF){
        Serial<<"5b\r\n";
        searchAddress = 0;
      }

      FlashGetArray(searchAddress, 1,2,recordNumber.buffer);
      Serial<<"6b\r\n";
      Serial<<recordNumber.val<<","<<searchAddress<<"\r\n";
      //Serial<<_HEX(recordNumber.buffer[0])<<","<<_HEX(recordNumber.buffer[1])<<"\r\n";
      //Serial<<
      if (recordNumber.val != *startingRecordNumber){
        Serial<<"7b\r\n";
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
      Serial<<"c\r\n";
      FlashGetArray((searchAddress + 1) , 2,sizeof(recordNumber.buffer),recordNumber.buffer);
      startingAddress = index;
      if (recordNumber.val != *startingRecordNumber){
        //startingAddress = index;
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
      Serial<<"d\r\n";
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
        Serial<<"d1\r\n";
        startingAddress = index;
        endAddress.val =  index;
        FlashWriteByteBlocking(startingAddress, 0,WRITE_COMPLETE_REC_END);

        FlashWriteByteBlocking(startingAddress , 3,0x00);
        FlashWriteByteBlocking(startingAddress , 4,endAddress.buffer[0]);
        FlashWriteByteBlocking(startingAddress , 5,endAddress.buffer[1]);
        *finalAddress = endAddress.val;
      }
      else{
        //endOfRecordFound = true;
        while(VerifyWriteReady() == false){
        }
        Serial<<"d2\r\n";
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
    //Serial<<_HEX(searchCount)<<"\r\n";
    if (searchCount == 0x3FFF){
      Serial<<"search count limit\r\n";
      endOfRecordFound = true;
      startingAddress = index;

      if (index == 0){
        endAddress.val =  0x3FFF;
      }
      else{
        endAddress.val =  index - 1;
      }
      Serial<<_HEX(endAddress.val)<<"\r\n";
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
      /*case 0:
       if (inByte != 0xAA){
       FlashSSHigh();
       return false;
       }
       break;*/
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
      /*case 6:
       if (inByte!= 0x55){
       FlashSSHigh();
       return true;
       }
       else{
       FlashSSHigh();
       return false;
       }
       break;*/
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
    Serial<<"init waiting for status reg to clear 1\r\n";
    Serial<<GetStatusReg()<<"\r\n";
    delay(1000);
  } 

  FlashSSLow();
  SPI.transfer(WRITE_ENABLE);
  FlashSSHigh();
  FlashSSLow();
  SPI.transfer(STATUS_WRITE);
  SPI.transfer(0x00);
  FlashSSHigh();
  while(VerifyWriteReady() == false){
    Serial<<"init waiting for status reg to clear 2\r\n";
    Serial<<GetStatusReg()<<"\r\n";
    delay(1000);
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

boolean FlashGetArray(uint16_t pageAddress, uint8_t byteAddress,uint8_t numBytes, uint8_t readBuffer[]){
  //returns an array of arbitrary length
  //to do make non blocking
  uint32_u addressOutput;
  /*if (sizeof(readBuffer) != numBytes){
   return false;
   }*/
  while(VerifyWriteReady() == false){
  } 
  /*if (numBytes > 256){
   Serial<<"--==\r\n";
   return false;
   }*/
  if (numBytes > (256 - byteAddress) ){
    Serial<<"++==\r\n";
    return false;
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
  return true;
}
boolean FlashGetPage(uint16_t pageAddress,uint16_t numBytes,uint8_t readBuffer[]){
  //returns an entire page of data
  uint32_u addressOutput;
  if (numBytes != 256){
    return false;
  }
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
  return true;

}
//write
boolean FlashWriteByte(uint16_t pageAddress, uint8_t byteAddress, uint8_t writeByte){
  //writes a single byte
  uint32_u addressOutput;

  while(VerifyWriteReady() == false){
  } //todo why is this blocking?
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  SPI.transfer(writeByte);
  FlashSSHigh();
  return true;
}
//todo return for write functions? 
boolean FlashWriteByteBlocking(uint16_t pageAddress, uint8_t byteAddress, uint8_t writeByte){
  //waits for device ready then writes a single byte
  uint32_u addressOutput;
  while(VerifyWriteReady() == false){
  }
  /*if (VerifyWriteReady() == false){
   return false;
   }*/
  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPI.transfer(PROGRAM_PAGE);
  SPI.transfer(addressOutput.buffer[2]);
  SPI.transfer(addressOutput.buffer[1]);
  SPI.transfer(addressOutput.buffer[0]);
  SPI.transfer(writeByte);
  FlashSSHigh();
  return true;
}
boolean FlashWritePartialPage(uint16_t pageAddress, uint8_t byteAddress, uint8_t numBytes, uint8_t writeBuffer[]){
  //writes
  uint32_u addressOutput;

  if (VerifyWriteReady() == false){
    return false;
  }//todo remove if write functions return void
  /*if (sizeof(writeBuffer) != numBytes){
   return false;
   }*/
  if (numBytes > (256 - byteAddress) ){
    return false;
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
  return true;
}

boolean FlashWritePage(uint16_t pageAddress, uint16_t numBytes, uint8_t writeBuffer[]){
  //writes 256 bytes to flash chip
  uint32_u addressOutput;

  if (VerifyWriteReady() == false){
    return false;
  }
  if (numBytes != 256){
    return false;
  }
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
  /*for(uint16_t i = 0; i < 256; i++){
   Serial<<writeBuffer[i]<<"=";
   }
   Serial<<"\r\n";*/
  return true;  
}
//erase
boolean ClearPage(uint16_t pageAddress){
  //sets all data on a page to zero
  uint32_u addressOutput;
  if (VerifyWriteReady() == false){
    return false;
  }
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
  return true; 
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

  if ((blockAddress & BLOCK_MASK_32K) == 0x0000 ){//|| (blockAddress & BLOCK_MASK_32K) == 0x0080){
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
    Serial<<"Erase wait 1\r\n";
    delay(3000);
  }
  FlashSSLow();
  SPI.transfer(WRITE_ENABLE);
  FlashSSHigh();

  FlashSSLow();
  SPI.transfer(ERASE_CHIP);
  FlashSSHigh();
  while(VerifyWriteReady() == false){
    Serial<<"Erase wait 2\r\n";
    delay(1000);
  }
  Serial<<_HEX(GetStatusReg())<<"\r\n";
  if (CheckForSuccessfulWrite() == true){
    Serial<<"erase successful\r\n";
  }
  else{
    Serial<<"erase failed\r\n";
    Serial<<_HEX(GetStatusReg())<<"\r\n";
    while(1){
      Serial<<"erased failed holding\r\n";
      delay(3000);
    }
  }

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

















