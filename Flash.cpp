#include "Flash.h"
#include "Definitions.h"
#include "Enums.h"
#include "Comm.h"
#include "Types.h"
#include "Inertial.h"
#include "GPS.h"
#include "Calibration.h"
#include "Attitude.h"
#include "Sensors.h"
#include "RCSignals.h"
#include "FlightControl.h"
#include "Radio.h"
#include "Motors.h"
#include <Arduino.h> 


uint16_t currentRecordNumber, currentRecordAddress, currentPageAddress, lowestRecordNumber, lowestRecordAddress;

uint8_t writeBuffer[256];

uint8_t writeBufferIndex = 0;

boolean startNewLog = false,endCurrentLog = false,writePageStarted = false,loggingReady=false,logEnabled = false;
boolean startOfRecordDataToFlash = false;
boolean eraseLogs = false, dumpLogs = false;
uint8_t loggingState = WRITE_READY;

void HighRateLog(uint32_t);
void MedRateLog(uint32_t);
void LowRateLog(uint32_t);
void LogDump();
void OutputRecord(uint16_t ,uint16_t);
void GainsToFlash();
void MotorMixToFlash();

void LogOutput(){
  while(1){
    Radio();
    if (eraseLogs == true){
      if(VerifyWriteReady() == true){
        eraseLogs = false;
        while(VerifyWriteReady() == false){
          Radio();
        }
        FlashEraseChip();
        while(VerifyWriteReady() == false){
          Radio();
        }
        SendEraseComplete();
      }
    }
    if (dumpLogs == true){
      flashOutputPacketNumber.val = 0;
      LogDump();
      SendDumpComplete();
      dumpLogs = false;
    }
  }
}
void LogDump(){
  uint32_u fullAddress;
  uint8_t  firstByte;
  uint16_t recordNumber,lastPageAddress;
  uint8_t pageBuffer[256];
  boolean validRecord,recordComplete;

  for(uint16_t i = 0; i <= 0x3FFF; i++){
    Radio();
    fullAddress.val = (uint32_t)i << 8;
    FlashSSLow();
    SPI.transfer(READ_ARRAY);
    SPI.transfer(fullAddress.buffer[2]);
    SPI.transfer(fullAddress.buffer[1]);
    SPI.transfer(fullAddress.buffer[0]);
    firstByte = SPI.transfer(0);
    FlashSSHigh();

    if (firstByte == WRITE_COMPLETE_REC_START || firstByte == WRITE_COMPLETE_REC_START_END || firstByte == WRITE_COMPLETE_REC_END || firstByte == WRITE_COMPLETE){
      FlashGetPage(i,pageBuffer);
      SendPage(pageBuffer);
      /*GetRecordNumber(i,&recordNumber,&lastPageAddress,&recordComplete);
       if (recordComplete == false){
       CompleteRecord(i,&recordNumber,&lastPageAddress);
       }
       OutputRecord(i,lastPageAddress);*/

    }

  }

}

/*void OutputRecord(uint16_t startAddress,uint16_t endAddress){
 uint8_t pageBuffer[256];
 uint16_t numPagesToOutput,currentOutputAddress;
 currentOutputAddress = startAddress;
 if (endAddress < startAddress){
 numPagesToOutput = (endAddress + 0x4001) - startAddress;
 }
 else{
 numPagesToOutput = endAddress - startAddress + 1;
 }
 
 for(uint16_t i = 0; i < numPagesToOutput; i++){
 while(VerifyWriteReady() == false){
 }
 FlashGetPage(currentOutputAddress,pageBuffer);
 SendPage(pageBuffer);
 currentOutputAddress++;
 if (currentOutputAddress > 0x3FFF){
 currentOutputAddress = 0;
 }
 }
 
 }*/

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
    loggingReady = true;
    if (startNewLog == true){
      endCurrentLog = false;
      startNewLog = false;
      startOfRecordDataToFlash = true;
      loggingState = START_NEW_LOG;
      break;
    }
    if (endCurrentLog == true){
      logEnabled = false;
      endCurrentLog = false;
      startNewLog = false;
      loggingState = END_CURRENT_LOG;
      loggingReady = false;
      break;
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
    logEnabled = true;
    loggingReady = false;
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
  uint32_t logTime;
  static uint8_t startOfRecordOutputState = 0;
  static uint32_t previousHighRate,previousMedRate,previousLowRate;
  if (logEnabled == true){
    if (loggingReady == true){
      logTime = millis();
      if (startOfRecordDataToFlash == true){
        GainsToFlash();
        MotorMixToFlash();
        startOfRecordDataToFlash = false;
      }
      else{
        if (logTime - previousHighRate > HIGH_RATE_INTERVAL){
          previousHighRate = logTime;
          HighRateLog(logTime);
        }
        if (logTime - previousMedRate > MED_RATE_INTERVAL){
          previousMedRate = logTime;
          MedRateLog(logTime);
        }
        if (logTime - previousLowRate > LOW_RATE_INTERVAL){
          previousLowRate = logTime;
          LowRateLog(logTime);
        }
      }
    }
  }

}

void GainsToFlash(){
  float_u outFloat;
  //uint8_t 
  uint8_t outByte = 0;
  //WriteBufferHandler(6,writeBuffer);
  WriteBufferHandler(1,&outByte);

  outFloat.val = kp_pitch_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_pitch_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_pitch_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_pitch_rate;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_roll_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_roll_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_roll_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_roll_rate;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_yaw_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_yaw_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_yaw_rate;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_yaw_rate;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_pitch_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_pitch_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_pitch_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_pitch_attitude;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_roll_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_roll_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_roll_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_roll_attitude;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_yaw_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_yaw_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_yaw_attitude;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_yaw_attitude;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_altitude_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_altitude_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_altitude_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_altitude_position;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_altitude_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_altitude_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_altitude_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_altitude_velocity;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_loiter_pos_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_loiter_pos_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_loiter_pos_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_loiter_pos_x;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_loiter_velocity_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_loiter_velocity_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_loiter_velocity_x;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_loiter_velocity_x;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_loiter_pos_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_loiter_pos_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_loiter_pos_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_loiter_pos_y;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_loiter_velocity_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_loiter_velocity_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_loiter_velocity_y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_loiter_velocity_y;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_waypoint_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_waypoint_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_waypoint_position;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_waypoint_position;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_waypoint_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_waypoint_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_waypoint_velocity;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_waypoint_velocity;
  WriteBufferHandler(4,outFloat.buffer);

  outFloat.val = kp_cross_track;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ki_cross_track;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = kd_cross_track;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = fc_cross_track;
  WriteBufferHandler(4,outFloat.buffer);
}

void MotorMixToFlash(){
  float_u outFloat;

  outFloat.val = m1X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m1Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m1Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m2X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m2Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m2Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m3X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m3Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m3Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m4X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m4Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m4Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m5X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m5Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m5Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m6X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m6Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m6Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m7X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m7Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m7Z;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m8X;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m8Y;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = m8Z;
  WriteBufferHandler(4,outFloat.buffer);
  WriteBufferHandler(1,&txLossRTB);
  WriteBufferHandler(1,&magDetected);
  WriteBufferHandler(1,&GPSDetected);
}



void HighRateLog(uint32_t time){
  uint32_u currentTime;
  float_u outFloat;
  uint8_t outByte;

  currentTime.val = time;

  outByte = 1;
  WriteBufferHandler(1,&outByte);
  WriteBufferHandler(4,currentTime.buffer);
  //gyro------------------------------------------
  outFloat.val = degreeGyroX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = degreeGyroY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = degreeGyroZ;
  WriteBufferHandler(4,outFloat.buffer);
  //acc-------------------------------------------
  outFloat.val = filtAccX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = filtAccY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = filtAccZ;
  WriteBufferHandler(4,outFloat.buffer);
  //mag--------------------------------------------
  outFloat.val = scaledMagX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = scaledMagY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = scaledMagZ;
  WriteBufferHandler(4,outFloat.buffer);
  //att--------------------------------------------
  outFloat.val = yawInDegrees;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = pitchInDegrees;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = rollInDegrees;
  WriteBufferHandler(4,outFloat.buffer);
  //pos--------------------------------------------
  outFloat.val = XEst;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = YEst;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = ZEst;
  WriteBufferHandler(4,outFloat.buffer);
  //vel--------------------------------------------
  outFloat.val = velX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = velY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = velZ;
  WriteBufferHandler(4,outFloat.buffer);
  //inertial--------------------------------------------
  outFloat.val = inertialX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = inertialY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = inertialZ;
  WriteBufferHandler(4,outFloat.buffer);
  //inertial--------------------------------------------
  outFloat.val = accelBiasX;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = accelBiasY;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = accelBiasZ;
  WriteBufferHandler(4,outFloat.buffer);
}
void MedRateLog(uint32_t time){
  uint32_u currentTime;
  float_u outFloat;
  int16_u outInt16;
  uint8_t outByte;
  currentTime.val = time;

  outByte = 2;
  WriteBufferHandler(1,&outByte);
  WriteBufferHandler(4,currentTime.buffer);
  outFloat.val = pressure;
  WriteBufferHandler(4,outFloat.buffer);
  WriteBufferHandler(1,&baroFS);
  outInt16.val = RCValue[0];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[1];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[2];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[3];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[4];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[5];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[6];
  WriteBufferHandler(2,outInt16.buffer);
  outInt16.val = RCValue[7];
  WriteBufferHandler(2,outInt16.buffer);

}



void LowRateLog(uint32_t time){
  uint32_u currentTime;
  float_u outFloat;
  uint8_t outByte;

  currentTime.val = time;
  outByte = 3;
  WriteBufferHandler(1,&outByte);
  WriteBufferHandler(4,currentTime.buffer);
  outFloat.val = floatLat;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = floatLon;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = homeLatFloat;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = homeLonFloat;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = velN;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = velE;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = hAcc;
  WriteBufferHandler(4,outFloat.buffer);
  outFloat.val = sAcc;
  WriteBufferHandler(4,outFloat.buffer);
  WriteBufferHandler(1,&gpsFailSafe);
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


void WriteBufferHandler(uint8_t numBytes, uint8_t inputBuffer[]){
  //takes data from the loghandler and writes it to the flash memory
  uint16_u outInt16;

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
      writeBufferIndex = 3;
      loggingReady = false;
    }
  }

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
    SPITransfer(READ_ARRAY);
    SPITransfer(fullAddress.buffer[2]);
    SPITransfer(fullAddress.buffer[1]);
    SPITransfer(fullAddress.buffer[0]);
    firstByte = SPITransfer(0);
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
  SPITransfer(READ_ARRAY);
  SPITransfer(fullAddress.buffer[2]);
  SPITransfer(fullAddress.buffer[1]);
  SPITransfer(fullAddress.buffer[0]);
  for(uint8_t i = 0; i < START_OF_REC_LEN; i++){
    inByte = SPITransfer(0);
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
  SPITransfer(WRITE_ENABLE);
  FlashSSHigh();
  FlashSSLow();
  SPITransfer(STATUS_WRITE);
  SPITransfer(0x00);
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
  SPITransfer(READ_ARRAY);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  inByte = SPITransfer(0);
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
  SPITransfer(READ_ARRAY);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < numBytes; i++){
    readBuffer[i] = SPITransfer(0x00);
  }
  FlashSSHigh();
}
void FlashGetPage(uint16_t pageAddress,uint8_t readBuffer[]){
  //returns an entire page of data
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8);

  FlashSSLow();
  SPITransfer(READ_ARRAY);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    readBuffer[i] = SPITransfer(0x00);
  }
  FlashSSHigh();

}
//write
void FlashWriteByte(uint16_t pageAddress, uint8_t byteAddress, uint8_t writeByte){
  //writes a single byte
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8) + (uint32_t)byteAddress;
  FlashSSLow();
  SPITransfer(PROGRAM_PAGE);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  SPITransfer(writeByte);
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
  SPITransfer(PROGRAM_PAGE);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  SPITransfer(writeByte);
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
  SPITransfer(PROGRAM_PAGE);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < numBytes; i++){
    SPITransfer(writeBuffer[i]);
  }
  FlashSSHigh();
}

void FlashWritePage(uint16_t pageAddress, uint8_t writeBuffer[]){
  //writes 256 bytes to flash chip
  uint32_u addressOutput;


  addressOutput.val = ((uint32_t)pageAddress << 8);
  FlashSSLow();
  SPITransfer(PROGRAM_PAGE);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    SPITransfer(writeBuffer[i]);
  }
  FlashSSHigh();
}
//erase
void ClearPage(uint16_t pageAddress){
  //sets all data on a page to zero
  uint32_u addressOutput;

  addressOutput.val = ((uint32_t)pageAddress << 8);
  FlashSSLow();
  SPITransfer(PROGRAM_PAGE);
  SPITransfer(addressOutput.buffer[2]);
  SPITransfer(addressOutput.buffer[1]);
  SPITransfer(addressOutput.buffer[0]);
  for(uint16_t i = 0; i < 256; i++){
    SPITransfer(0x00);
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
    SPITransfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPITransfer(ERASE_4K);
    SPITransfer(addressOutput.buffer[2]);
    SPITransfer(addressOutput.buffer[1]);
    SPITransfer(addressOutput.buffer[0]);
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
    SPITransfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPITransfer(ERASE_32K);
    SPITransfer(addressOutput.buffer[2]);
    SPITransfer(addressOutput.buffer[1]);
    SPITransfer(addressOutput.buffer[0]);
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
    SPITransfer(WRITE_ENABLE);
    FlashSSHigh();
    FlashSSLow();
    SPITransfer(ERASE_64K);
    SPITransfer(addressOutput.buffer[2]);
    SPITransfer(addressOutput.buffer[1]);
    SPITransfer(addressOutput.buffer[0]);
    FlashSSHigh();
    return true;
  }
  else{
    return false;
  } 
}
void FlashEraseChip(){
  //sets entire chip to 0xFF
  /*while(VerifyWriteReady() == false){
   }*/

  FlashSSLow();
  SPITransfer(WRITE_ENABLE);
  FlashSSHigh();

  FlashSSLow();
  SPITransfer(ERASE_CHIP);
  FlashSSHigh();

  /*while(VerifyWriteReady() == false){
   }
   
   if (CheckForSuccessfulWrite() == true){
   //return true;
   }*/
  //return false;

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
  SPITransfer(READ_STATUS_REG);
  inByte1 = SPITransfer(0);
  //inByte2 = SPITransfer(0);
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
    SPITransfer(WRITE_ENABLE);
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






























