#include "Radio.h"
#include "Comm.h"
#include "Sensors.h"
#include "Types.h"
#include "Attitude.h"
#include "RCSignals.h"
#include "FlightControl.h"
#include "Rom.h"
#include "Definitions.h"
#include "Motors.h"
#include "Enums.h"

boolean USBFlag=false,handShake=false,calibrationMode=false,gsCTRL=false;

int16_t GSRCValue[8];
boolean newGSRC;

void SetTransmissionRate();
void WriteCalibrationDataToRom();
void OrderedSet();
void SendOrdAck();
void SendOrdMis();
void OrderedQuery();
void SendUnAck();
void SendUnMis();
void UnReliableTransmit();
void SendHandShakeResponse();
void SendCalData();
void HandleGSRCData();


uint8_t typeNum,cmdNum,itemBuffer[255],calibrationNumber,hsRequestNumber,lsRequestNumber,hsNumItems,lsNumItems, hsList[40], lsList[40];
uint16_t localPacketNumberOrdered, remotePacketNumberOrdered, remotePacketNumberUn, packetTemp[2];
boolean hsTX,lsTX,sendCalibrationData;
uint32_t hsMillis,lsMillis;
uint8_t groundStationID;

void TryHandShake(){
  AssignRadioUART();
  HandShake();

  if (handShake == false) {
    USBFlag = true;
    AssignRadioUSB();
    HandShake();
  }
}

void Radio() {
  static uint8_t rxSum=0,rxDoubleSum=0,packetLength=0,numRXbytes=0,radioState = 0,itemIndex=0;
  uint8_t radioByte;
  float_u outFloat;
  uint8_t j;
  while (RadioAvailable() > 0) { //---

    radioByte = RadioRead();
    switch (radioState) { //+++
    case SB_CHECK://check for start byte
      rxSum = 0;
      rxDoubleSum = 0;
      if (radioByte == 0xAA) {
        radioState = CHECK_GS_ID;
      }
      break;
    case CHECK_GS_ID:
      if (radioByte == groundStationID){
        radioState = PKT_LEN;
      }
      else{
        radioState = SB_CHECK;
      }
      break;
    case PKT_LEN:
      packetLength = radioByte;
      numRXbytes = 0;
      radioState = ITEM_TYPE;
      break;

    case ITEM_TYPE:
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (radioByte == 0xFA) { //reliable querie
        radioState = REL_QRY_PKT_LSB;
        break;
      }
      if (radioByte == 0xFD) { //reliable set
        radioState = REL_SET_PKT_LSB;
        break;
      }
      typeNum = radioByte;
      if (typeNum == GS_CONTROL && packetLength == 18) {
        radioState = GS_RC_CMD_NUM;
        break;
      }


      if (packetLength == 2) { //length for unrelaible will always be 2
        radioState = UNREL_START;//unrelaible data
      }
      else {
        radioState = SB_CHECK;
      }
      break;

    case UNREL_START://unrelaible
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      radioState = UNREL_CK_SM1;
      break;
    case UNREL_CK_SM1://unreliable checksum 1
      if (rxSum == radioByte) {
        radioState = UNREL_CK_SM2;
        break;
      }
      radioState = SB_CHECK;
      break;
    case UNREL_CK_SM2://unreliable check sum 2
      if (rxDoubleSum == radioByte) {
        //UnReliableTransmit();
      }
      radioState = SB_CHECK;
      break;
    case REL_QRY_PKT_LSB://reliable queries - get packet num LSB
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = REL_QRY_PKT_MSB;
      break;
    case REL_QRY_PKT_MSB://packet num MSB and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberUn = (packetTemp[1] << 8 ) | packetTemp[0];
      radioState = REL_QRY_TYP_NUM;
      break;
    case REL_QRY_TYP_NUM://get typeNum
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      radioState = REL_QRY_CMD_NUM;
      break;
    case REL_QRY_CMD_NUM://get cmdNum
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = REL_QRY_SUM1;
      break;

    case REL_QRY_SUM1://check the first sum
      if (rxSum == radioByte) {
        radioState = REL_QRY_SUM2;
        break;
      }
      radioState = SB_CHECK;
      break;

    case REL_QRY_SUM2://check the second sum
      if (rxDoubleSum == radioByte) {
        SendUnAck();
      }
      radioState = SB_CHECK;

      break;

    case REL_SET_PKT_LSB://reliable set get packet num lsb
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = REL_SET_PKT_MSB;
      break;

    case REL_SET_PKT_MSB://get packet num msb and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberOrdered = (packetTemp[1] << 8 ) | packetTemp[0];
      radioState = REL_SET_TYPE;
      break;

    case REL_SET_TYPE:
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (typeNum == GS_PING || typeNum == RESET_PR_OFFSET){
        radioState = REL_SET_SUM1;
        break;
      }
      radioState = REL_SET_CMD;
      break;

    case REL_SET_CMD:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = REL_SET_BUFFER;
      if (typeNum == START_CAL_DATA || typeNum == SET_PR_OFFSETS || typeNum == ESC_CAL) {
        radioState = REL_SET_SUM1;
      }
      if (typeNum == END_CAL_DATA && (cmdNum == 3 || cmdNum == 7)) {
        radioState = REL_SET_SUM1;
      }
      break;

    case REL_SET_BUFFER://buffer in data
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (packetLength > 250) {
        radioState = SB_CHECK;
      }
      if (numRXbytes == packetLength) {
        radioState = REL_SET_SUM1;
      }
      break;
    case REL_SET_SUM1://check first sum
      if (rxSum != radioByte) {
        radioState = SB_CHECK;
        break;
      }
      radioState = REL_SET_SUM2;
      break;
    case REL_SET_SUM2:
      if (rxDoubleSum == radioByte) {
        if (remotePacketNumberOrdered != localPacketNumberOrdered) {
          SendOrdMis();
          radioState = SB_CHECK;
          break;
        }
        if (typeNum == RESET_PR_OFFSET){
          pitchOffset = 0;
          rollOffset = 0;
          j = 0;
          outFloat.val = pitchOffset;
          for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
            EEPROMWrite(i, outFloat.buffer[j++]);
          }
          j = 0;
          outFloat.val = rawRoll;
          for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
            EEPROMWrite(i, outFloat.buffer[j++]);
          }
          EEPROMWrite(PR_FLAG, 0xAA);
        }
        if (calibrationMode == true) {
          if (typeNum == START_CAL_DATA) {
            sendCalibrationData = true;
            calibrationNumber = cmdNum;
          }
          if (typeNum == END_CAL_DATA) {
            WriteCalibrationDataToRom();
            sendCalibrationData = false;
          }
        }
        else {
          if (typeNum < UINT8) {
            OrderedSet();
          }
          if (typeNum == HS_DATA || typeNum == LS_DATA) {
            SetTransmissionRate();
          }
          if (typeNum == SET_PR_OFFSETS) {

            pitchOffset = rawPitch;
            rollOffset = rawRoll;
            j = 0;
            outFloat.val = pitchOffset;
            for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
              EEPROMWrite(i, outFloat.buffer[j++]);
            }
            j = 0;
            outFloat.val = rawRoll;
            for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
              EEPROMWrite(i, outFloat.buffer[j++]);
            }
            EEPROMWrite(PR_FLAG, 0xAA);
          }
          if (typeNum == ESC_CAL && calibrationModeESCs == true){
            switch(cmdNum){
            case 0:
              calibrateESCs = true;
              break;
            case 1:
              SendOrdAck();
              delay(500);
              asm volatile ("  jmp 0");
              break;

            }
          }

        }
        SendOrdAck();
      }
      radioState = SB_CHECK;
      break;
    case GS_RC_CMD_NUM:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      itemIndex = 0;
      if (cmdNum == 8) {
        radioState = GS_RC_BUFFER;
      }
      else {
        radioState = SB_CHECK;
      }
      break;
    case GS_RC_BUFFER:
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      if (itemIndex == (cmdNum * 2)) {
        radioState = GS_RC_SUM1;
      }
      break;
    case GS_RC_SUM1:
      if (rxSum == radioByte) {
        radioState = GS_RC_SUM2;
      }
      else {
        radioState = SB_CHECK;
      }
      break;
    case GS_RC_SUM2:
      if (rxDoubleSum == radioByte) {
        HandleGSRCData();

      }
      radioState = SB_CHECK;
      break;

    }//+++
  }//---

}

void CalibrateSensors() {
  uint32_t generalPurposeTimer = millis();
  while (1) {
    if ( millis() - generalPurposeTimer >= 10) {
      generalPurposeTimer = millis();
      GetAcc();
      GetMag();
      if (sendCalibrationData == true) {
        SendCalData();
      }
    }
    Radio();
  }

}

void HandleGSRCData() {
  int16_u inShort;
  uint8_t itemIndex = 0;
  for (uint8_t i = 0; i < cmdNum; i++) {
    switch (i) {
    case THRO:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[THRO] = inShort.val;
      break;
    case AILE:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AILE] = inShort.val;
      break;
    case ELEV:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[ELEV] = inShort.val;
      break;
    case RUDD:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[RUDD] = inShort.val;
      break;
    case GEAR:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[GEAR] = inShort.val;
      break;
    case AUX1:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX1] = inShort.val;
      break;
    case AUX2:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX2] = inShort.val;
      break;
    case AUX3:
      inShort.buffer[0] = itemBuffer[itemIndex++];
      inShort.buffer[1] = itemBuffer[itemIndex++];
      GSRCValue[AUX3] = inShort.val;
      break;
    }
  }
  newGSRC = true;
  groundFSCount = 0;
}

void TuningTransmitter() { //
  uint32_u now;
  float_u outFloat;
  int16_u outInt16;
  uint8_t packetLength,txSum,txDoubleSum,hsListIndex,lsListIndex,tuningItemIndex,liveDataBuffer[200];
  static uint32_t hsTXTimer=0,lsTXTimer=0;


  now.val = millis();


  if (hsTX == true) { //---

    if (now.val - hsTXTimer >= hsMillis) { //+++
      hsTXTimer = now.val;
      txSum = 0;
      txDoubleSum = 0;
      hsListIndex = 0;
      tuningItemIndex = 0;
      packetLength = 0;
      //assemble the transmit buffer
      liveDataBuffer[tuningItemIndex++] = 0xAA;
      liveDataBuffer[tuningItemIndex++] = groundStationID;
      tuningItemIndex++;//skip packet length for now
      liveDataBuffer[tuningItemIndex++] = 4;
      txSum += 4;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = hsRequestNumber;
      txSum += hsRequestNumber;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = hsNumItems;
      txSum += hsNumItems;
      txDoubleSum += txSum;
      packetLength++;

      for (uint8_t i = 0; i < 4; i++) { //always include millis
        liveDataBuffer[tuningItemIndex++] = now.buffer[i];
        txSum += now.buffer[i];
        txDoubleSum += txSum;
        packetLength++;
      }


      for (uint8_t i = 0; i < hsNumItems; i++) { //***
        switch (hsList[hsListIndex++]) {
        case 0://floats
          outFloat.val = *floatPointerArray[hsList[hsListIndex]];
          for (uint8_t j = 0; j < 4; j++) {
            liveDataBuffer[tuningItemIndex++] = outFloat.buffer[j];
            txSum += outFloat.buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          hsListIndex++;
          break;
        case 1://int16
          outInt16.val = *int16PointerArray[hsList[hsListIndex]];
          for (uint8_t j = 0; j < 2; j++) {
            liveDataBuffer[tuningItemIndex++] = outInt16.buffer[j];
            txSum += outInt16.buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          hsListIndex++;
          break;
        case 2://int32
          /*for(uint8_t j = 0; j < 4; j++){
           liveDataBuffer[tuningItemIndex++] = (*int32PointerArray[hsList[hsListIndex]]).buffer[j];
           txSum += (*int32PointerArray[hsList[hsListIndex]]).buffer[j];
           txDoubleSum += txSum;
           packetLength++;
           }
           hsListIndex++;*/
          break;
        case 3:
          liveDataBuffer[tuningItemIndex++] = *bytePointerArray[hsList[hsListIndex]];
          txSum += *bytePointerArray[hsList[hsListIndex]];
          txDoubleSum += txSum;
          packetLength++;

          hsListIndex++;
          break;
        }
      }//***

      liveDataBuffer[2] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 3); i++) {
        RadioWrite(liveDataBuffer[i]);
      }
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);


    }//+++

  }//---

  if (lsTX == true) { //---

    if (now.val - lsTXTimer >= lsMillis) { //+++
      lsTXTimer = now.val;
      txSum = 0;
      txDoubleSum = 0;
      lsListIndex = 0;
      tuningItemIndex = 0;
      packetLength = 0;
      //assemble the transmit buffer
      liveDataBuffer[tuningItemIndex++] = 0xAA;
      liveDataBuffer[tuningItemIndex++] = groundStationID;
      tuningItemIndex++;//skip packet length for now
      liveDataBuffer[tuningItemIndex++] = 5;
      txSum += 5;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = lsRequestNumber;
      txSum += lsRequestNumber;
      txDoubleSum += txSum;
      packetLength++;

      liveDataBuffer[tuningItemIndex++] = lsNumItems;
      txSum += lsNumItems;
      txDoubleSum += txSum;
      packetLength++;

      for (uint8_t i = 0; i < 4; i++) { //always include millis
        liveDataBuffer[tuningItemIndex++] = now.buffer[i];
        txSum += now.buffer[i];
        txDoubleSum += txSum;
        packetLength++;
      }


      for (uint8_t i = 0; i < lsNumItems; i++) { //***
        switch (lsList[lsListIndex++]) {
        case 0://floats
          for (uint8_t j = 0; j < 4; j++) {
            outFloat.val = *floatPointerArray[lsList[lsListIndex]];
            liveDataBuffer[tuningItemIndex++] = outFloat.buffer[j];
            txSum += outFloat.buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          lsListIndex++;
          break;
        case 1://int16
          outInt16.val = *int16PointerArray[lsList[lsListIndex]];
          for (uint8_t j = 0; j < 2; j++) {
            liveDataBuffer[tuningItemIndex++] = outInt16.buffer[j];
            txSum += outInt16.buffer[j];
            txDoubleSum += txSum;
            packetLength++;
          }
          lsListIndex++;
          break;
        case 2://int32
          /*for(uint8_t j = 0; j < 4; j++){
           liveDataBuffer[tuningItemIndex++] = (*int32PointerArray[lsList[lsListIndex]]).buffer[j];
           txSum += (*int32PointerArray[lsList[lsListIndex]]).buffer[j];
           txDoubleSum += txSum;
           packetLength++;
           }
           lsListIndex++;*/
          break;
        case 3:
          liveDataBuffer[tuningItemIndex++] = *bytePointerArray[lsList[lsListIndex]];
          txSum += *bytePointerArray[lsList[lsListIndex]];
          txDoubleSum += txSum;
          packetLength++;

          lsListIndex++;
          break;
        default:
          break;
        }
      }//***

      liveDataBuffer[2] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 3); i++) {
        RadioWrite(liveDataBuffer[i]);
      }
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);

    }//+++

  }//---

}//

void SetTransmissionRate() {

  if (typeNum == HS_DATA) {
    if (cmdNum == 0) {
      hsTX = false;
    }
    else {
      hsTX = true;
      hsMillis = uint32_t((1.0 / cmdNum) * 1000);
      hsRequestNumber = itemBuffer[0];
      hsNumItems = itemBuffer[1];
      if (hsNumItems > 20) {
        hsTX = false;
      }
      else {
        memcpy( &hsList[0], &itemBuffer[2], (hsNumItems * 2) );
      }
    }



  }
  else {

    if (cmdNum == 0) {
      lsTX = false;
    }
    else {
      lsTX = true;
      lsMillis = uint32_t((1.0 / cmdNum) * 1000);
      lsRequestNumber = itemBuffer[0];
      lsNumItems = itemBuffer[1];
      if (lsNumItems > 20) {
        lsTX = false;
      }
      else {
        memcpy( &lsList[0], &itemBuffer[2], (lsNumItems * 2) );
      }
    }


  }
}

void WriteCalibrationDataToRom() {
  uint8_t temp,calibrationFlags;
  uint8_t itemIndex = 0;
  switch (cmdNum) {
  case 0://mag calibration data
    if (magDetected == true) {
      for (uint16_t i = MAG_CALIB_START; i <= MAG_CALIB_END; i++) {
        EEPROMWrite(i, itemBuffer[itemIndex++]);
      }

      calibrationFlags = EEPROMRead(CAL_FLAGS);
      calibrationFlags &= ~(1 << MAG_FLAG);
      EEPROMWrite(CAL_FLAGS, calibrationFlags);
    }
    break;//--------------------------------------------
  case 1://acc calibration data
    for (uint16_t i = ACC_CALIB_START; i <= ACC_CALIB_END; i++) {
      EEPROMWrite(i, itemBuffer[itemIndex++]);
    }

    calibrationFlags = EEPROMRead(CAL_FLAGS);
    calibrationFlags &= ~(1 << ACC_FLAG);
    EEPROMWrite(CAL_FLAGS, calibrationFlags);


    break;//--------------------------------------------


  case 2://RC calibration data
    for (uint16_t i = RC_DATA_START; i <= RC_DATA_END; i++) {
      EEPROMWrite(i, itemBuffer[itemIndex++]);
    }
    calibrationFlags = EEPROMRead(CAL_FLAGS);
    calibrationFlags &= ~(1 << RC_FLAG);
    EEPROMWrite(CAL_FLAGS, calibrationFlags);
    break;//--------------------------------------------
  case 3://command to end calibration and reset controller
    //save the packet numbers
    if (USBFlag == true) {
      EEPROMWrite(HS_FLAG, 0xBB); //set handshake compelte flag in EEPROM
    }
    else {
      EEPROMWrite(HS_FLAG, 0xAA); //set handshake compelte flag in EEPROM
    }
    TIMSK5 = (0<<OCIE5A);
    SendOrdAck();
    //save the packet numbers
    temp = localPacketNumberOrdered & 0x00FF;
    EEPROMWrite(PKT_LOCAL_ORD_L, temp);
    temp = localPacketNumberOrdered >> 8;
    EEPROMWrite(PKT_LOCAL_ORD_M, temp);
    Motor1WriteMicros(0);//set the output compare value
    Motor2WriteMicros(0);
    Motor3WriteMicros(0);
    Motor4WriteMicros(0);
    Motor5WriteMicros(0);
    Motor6WriteMicros(0);
    Motor7WriteMicros(0);
    Motor8WriteMicros(0);
    delay(500);
    asm volatile ("  jmp 0");

    break;//--------------------------------------
  case 4://tx failsafe
    txLossRTB = itemBuffer[0];
    if (txLossRTB < 0 ||txLossRTB > 1){
      txLossRTB = 0;
    }

    EEPROMWrite(TX_FS_FLAG, 0xAA);
    EEPROMWrite(TX_FS, txLossRTB);
    txLossRTB = EEPROMRead(TX_FS);
    break;//--------------------------------------------
  case 5://pwms
    EEPROMWrite(HOVER_THRO, itemBuffer[itemIndex++]);
    EEPROMWrite(HOVER_THRO_FLAG, 0xAA);
    EEPROMWrite(PROP_IDLE, itemBuffer[itemIndex++]);
    EEPROMWrite(PROP_IDLE_FLAG, 0xAA);
    EEPROMWrite(PWM_LIM_HIGH_START, itemBuffer[itemIndex++]);
    EEPROMWrite(PWM_LIM_HIGH_END, itemBuffer[itemIndex++]);
    EEPROMWrite(PWM_LIM_LOW_START, itemBuffer[itemIndex++]);
    EEPROMWrite(PWM_LIM_LOW_END, itemBuffer[itemIndex++]);
    EEPROMWrite(PWM_FLAG, 0xAA);

    break;//--------------------------------------------
  case 6://MODES
    EEPROMWrite(MODE_FLAG,0xAA);
    for(uint16_t i = MODE_START; i <= MODE_END; i++){
      EEPROMWrite(i,itemBuffer[itemIndex++]);
    }

    break;//--------------------------------------------
  case 7:
    EEPROMWrite(ESC_CAL_FLAG,0xBB);
    break;
  }




}


void OrderedSet() {
  float_u outFloat;
  switch (typeNum) {
  case FLOAT:
    if (cmdNum >= KP_PITCH_RATE_ && cmdNum <= MAG_DEC_) {
      for (uint8_t i = 0; i < 4; i++) {
        outFloat.buffer[i] =  itemBuffer[i];
      }
      *floatPointerArray[cmdNum] = outFloat.val;
      if (cmdNum == MAG_DEC_){
        cosDec = cos(declination);
        sinDec = sin(declination);
      }
      saveGainsFlag = true;
      romWriteDelayTimer = millis();

    }
    break;
  case INT16:
    /*
      for (uint8_t i = 0; i < 2; i++){
     (*int16PointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
     }*/
    break;
  case INT32:
    /*
      for (uint8_t i = 0; i < 4; i++){
     (*int32PointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
     }
     */
    break;
  }

}

void SendOrdAck() {

  uint8_t txSum = 0,txDoubleSum = 0,temp;

  RadioWrite(0xAA);
  RadioWrite(groundStationID);
  RadioWrite(3);
  RadioWrite(0xFC);
  txSum = 0xFC;
  txDoubleSum += txSum;
  temp = localPacketNumberOrdered & 0x00FF;
  RadioWrite(temp);
  txSum += temp;
  txDoubleSum += txSum;
  temp = (localPacketNumberOrdered >> 8) & 0x00FF;
  RadioWrite(temp);
  txSum += temp;
  txDoubleSum += txSum;
  RadioWrite(txSum);
  RadioWrite(txDoubleSum);
  localPacketNumberOrdered++;

}

void SendOrdMis() {
  uint8_t txSum = 0,txDoubleSum = 0,temp;

  RadioWrite(0xAA);
  RadioWrite(groundStationID);
  RadioWrite(3);
  RadioWrite(0xFB);
  txSum = 0xFB;
  txDoubleSum += txSum;
  temp = localPacketNumberOrdered & 0x00FF;
  RadioWrite(temp);
  txSum += temp;
  txDoubleSum += txSum;
  temp = (localPacketNumberOrdered >> 8) & 0x00FF;
  RadioWrite(temp);
  txSum += temp;
  txDoubleSum += txSum;
  RadioWrite(txSum);
  RadioWrite(txDoubleSum);
}

void OrderedQuery() {
  float_u outFloat;
  int16_u outInt16;
  switch (typeNum) {
  case FLOAT:
    outFloat.val = *floatPointerArray[cmdNum];
    for (uint8_t i = 0; i < 4; i++) {
      itemBuffer[i] = outFloat.buffer[i];
    }
    break;
  case INT16:
    outInt16.val = *int16PointerArray[cmdNum];
    for (uint8_t i = 0; i < 2; i++) {
      itemBuffer[i] = outInt16.buffer[i];
    }
    break;
  case INT32:
    /*for (uint8_t i = 0; i < 4; i++){
     itemBuffer[i] = (*int32PointerArray[cmdNum]).buffer[i];
     }*/
    break;
  case UINT8:

    break;
  }

}

void SendUnAck() {
  uint8_t txSum = 0,txDoubleSum = 0,temp;
  float_u outFloat;
  float_u outInt16;
  RadioWrite(0XAA);
  RadioWrite(groundStationID);
  switch (typeNum) {
  case FLOAT:
    RadioWrite(9);
    RadioWrite(0xF9);
    txSum = 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;

    outFloat.val = *floatPointerArray[cmdNum];
    RadioWrite(outFloat.buffer[0]);
    txSum += outFloat.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(outFloat.buffer[1]);
    txSum += outFloat.buffer[1];
    txDoubleSum += txSum;
    RadioWrite(outFloat.buffer[2]);
    txSum += outFloat.buffer[2];
    txDoubleSum += txSum;
    RadioWrite(outFloat.buffer[3]);
    txSum += outFloat.buffer[3];

    txDoubleSum += txSum;
    RadioWrite(txSum);
    RadioWrite(txDoubleSum);

    break;
  case INT16:
    RadioWrite(7);
    RadioWrite(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    outInt16.val = *int16PointerArray[cmdNum];
    RadioWrite(outInt16.buffer[0]);
    txSum += outInt16.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(outInt16.buffer[1]);
    txSum += outInt16.buffer[1];
    txDoubleSum += txSum;
    RadioWrite(txSum);
    RadioWrite(txDoubleSum);

    break;
  case INT32:
    /*RadioWrite(9);
     RadioWrite(0xF9);
     txSum = 0xF9;
     txDoubleSum += txSum;
     temp = remotePacketNumberUn & 0x00FF;
     RadioWrite(temp);
     txSum += temp;
     txDoubleSum += txSum;
     temp = (remotePacketNumberUn >> 8) & 0x00FF;
     RadioWrite(temp);
     txSum += temp;
     txDoubleSum += txSum;
     RadioWrite(typeNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     RadioWrite(cmdNum);
     txSum += cmdNum;
     txDoubleSum += txSum;
     RadioWrite((*int32PointerArray[cmdNum]).buffer[0]);
     txSum += (*int32PointerArray[cmdNum]).buffer[0];
     txDoubleSum += txSum;
     RadioWrite((*int32PointerArray[cmdNum]).buffer[1]);
     txSum += (*int32PointerArray[cmdNum]).buffer[1];
     txDoubleSum += txSum;
     RadioWrite((*int32PointerArray[cmdNum]).buffer[2]);
     txSum += (*int32PointerArray[cmdNum]).buffer[2];
     txDoubleSum += txSum;
     RadioWrite((*int32PointerArray[cmdNum]).buffer[3]);
     txSum += (*int32PointerArray[cmdNum]).buffer[3];
     txDoubleSum += txSum;
     RadioWrite(txSum);
     RadioWrite(txDoubleSum);*/

    break;
  case UINT8:
    RadioWrite(6);
    RadioWrite(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    RadioWrite(*bytePointerArray[cmdNum]);
    txSum += *bytePointerArray[cmdNum];
    txDoubleSum += txSum;

    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
    break;
  }



  if (typeNum == INT16) {
    RadioWrite(7);
    RadioWrite(0xF9);
    txSum += 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[0]);
    txSum += itemBuffer[0];
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[1]);
    txSum += itemBuffer[1];
    txDoubleSum += txSum;
    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
  }
  else {
    RadioWrite(9);
    RadioWrite(0xF9);
    txSum = 0xF9;
    txDoubleSum += txSum;
    temp = remotePacketNumberUn & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    temp = (remotePacketNumberUn >> 8) & 0x00FF;
    RadioWrite(temp);
    txSum += temp;
    txDoubleSum += txSum;
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += cmdNum;
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[0]);
    txSum += itemBuffer[0];
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[1]);
    txSum += itemBuffer[1];
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[2]);
    txSum += itemBuffer[2];
    txDoubleSum += txSum;
    RadioWrite(itemBuffer[3]);
    txSum += itemBuffer[3];
    txDoubleSum += txSum;
    RadioWrite(txSum);
    RadioWrite(txDoubleSum);

  }
}

/*void SendUnMis() {
 
 uint8_t txSum = 0,txDoubleSum = 0,temp;
 
 RadioWrite(0xAA);
 RadioWrite(groundStationID);
 RadioWrite(3);
 RadioWrite(0xF8);
 txSum += 0xF8;
 txDoubleSum += txSum;
 temp = localPacketNumberUn & 0x00FF;
 RadioWrite(temp);
 txSum += temp;
 txDoubleSum += txSum;
 temp = (localPacketNumberUn >> 8) & 0x00FF;
 RadioWrite(temp);
 txSum += temp;
 txDoubleSum += txSum;
 RadioWrite(txSum);
 RadioWrite(txDoubleSum);
 
 }*/

void UnReliableTransmit() {
  uint8_t txSum = 0, txDoubleSum = 0;
  float_u outFloat;
  int16_u outInt16;
  RadioWrite(0xAA);
  RadioWrite(groundStationID);
  switch (typeNum) {
  case FLOAT://float
    RadioWrite(4);
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    outFloat.val = *floatPointerArray[cmdNum];
    for (uint8_t i = 0; i < 4; i++) {
      RadioWrite(outFloat.buffer[i]);
      txSum += outFloat.buffer[i];
      txDoubleSum += txSum;
    }
    break;
  case INT16://int16
    RadioWrite(2);
    RadioWrite(typeNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    RadioWrite(cmdNum);
    txSum += typeNum;
    txDoubleSum += txSum;
    outInt16.val = *int16PointerArray[cmdNum];
    for (uint8_t i = 0; i < 2; i++) {
      RadioWrite(outInt16.buffer[i]);
      txSum += outInt16.buffer[i];
      txDoubleSum += txSum;
    }
    break;
  case INT32://int32
    /*RadioWrite(4);
     RadioWrite(typeNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     RadioWrite(cmdNum);
     txSum += typeNum;
     txDoubleSum += txSum;
     for(uint8_t i = 0; i < 4; i++){
     RadioWrite((*int32PointerArray[cmdNum]).buffer[i]);
     txSum += (*int32PointerArray[cmdNum]).buffer[i];
     txDoubleSum += txSum;
     }*/
    break;
  }
  RadioWrite(txSum);
  RadioWrite(txDoubleSum);
}



void HandShake() {
  uint8_t handShakeState = 0,radioByte,rxSum=0,rxDoubleSum=0;
  uint32_t radioTimer = millis();

  if (EEPROMRead(HS_FLAG) == 0xAA || EEPROMRead(HS_FLAG) == 0xBB) { //Check for handshake from calibration
    if (EEPROMRead(HS_FLAG) == 0xBB) {
      AssignRadioUSB();
    }
    else{
      AssignRadioUART();
    }
    EEPROMWrite(HS_FLAG, 0xFF);
    packetTemp[0] = EEPROMRead(PKT_LOCAL_ORD_L);//lsb for packetNumberLocalOrdered
    packetTemp[1] = EEPROMRead(PKT_LOCAL_ORD_M);//msb for packetNumberLocalOrdered
    localPacketNumberOrdered = (packetTemp[1] << 8) | packetTemp[0];
    groundStationID = EEPROMRead(GS_ID_INDEX);
    /*packetTemp[0] = EEPROMRead(PKT_LOCAL_UN_L);//lsb for packetNumberLocalUnOrdered
     packetTemp[1] = EEPROMRead(PKT_LOCAL_UN_M);//msb for packetNumberLocalUnOrdered
     localPacketNumberUn = (packetTemp[1] << 8) | packetTemp[0];*/
    handShake = true;
    return;
  }
  localPacketNumberOrdered = 0;
  //localPacketNumberUn = 0;
  while (RadioAvailable() > 0) {
    RadioRead();//clear any data in the buffer
  }
  radioTimer = millis();
  while (millis() - radioTimer < 1000 && handShake == false) {
    //look for data on the radio port
    if (RadioAvailable() > 0) {
      handShake = true;
    }
  }
  if (handShake == false) {
    return;
  }
  handShake = false;
  calibrationModeESCs = false;
  gsCTRL = false;
  calibrationMode = false;
  while (millis() - radioTimer < 1000 && handShake == false) { //***

    if (RadioAvailable() > 0) { //---

      while (RadioAvailable() > 0) { //+++

        radioByte = RadioRead();

        switch (handShakeState) { //^^^

        case START_BYTE://check for 0xAA
          rxSum = 0;
          rxDoubleSum = 0;
          calibrationMode = false;
          if (radioByte == 0xAA) {
            handShakeState = GET_GS_ID;
          }
          break;
        case GET_GS_ID:
          groundStationID = radioByte;
          handShakeState = LENGTH;
          break;

        case LENGTH://get and verify the length
          if (radioByte == 0x02) { //len will always be 2 for the HS
            handShakeState = CMD_BYTE;
          }
          else {
            handShakeState = START_BYTE;
          }
          break;

        case CMD_BYTE://check for correct command byte
          if (radioByte == 0xFF) {
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = HS_TYPE;
          }
          else {
            handShakeState = START_BYTE;
          }
          break;

        case HS_TYPE://check handshake type
          if (radioByte == 0x04){
            calibrationModeESCs = true;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = HS_SUM_1;
            break;
          }
          if (radioByte == 0x03) {//gs ctrl calibration 
            gsCTRL = true;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = HS_SUM_1;
            calibrationMode = true;
            break;
          }
          if (radioByte == 0x02) {//gs ctrl
            gsCTRL = true;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = HS_SUM_1;
            break;
          }
          if (radioByte == 0x01) {//calibration 
            gsCTRL = false;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = HS_SUM_1;
            calibrationMode = true;
            break;
          }
          if (radioByte == 0x00) {//normal
            gsCTRL = false;
            rxDoubleSum += rxSum;
            handShakeState = HS_SUM_1;
            break;
          }
          handShakeState = START_BYTE;

          break;

        case HS_SUM_1://verify sum
          if (radioByte == rxSum) {
            handShakeState = HS_SUM_2;
            break;
          }
          handShakeState = START_BYTE;
          break;

        case HS_SUM_2://verify double sum
          if (radioByte == rxDoubleSum) {
            SendHandShakeResponse();
            handShake = true;
            handShakeState = START_BYTE;
            break;
          }

          handShakeState = START_BYTE;
          break;

        }//^^^

      }//+++

    }//---

  }//***

  if (handShake == false) {
    calibrationModeESCs = false;
    gsCTRL = false;
    calibrationMode = false;
  }
  else{
    EEPROMWrite(GS_ID_INDEX,groundStationID);
  }

}

void SendHandShakeResponse() {
  uint8_t txSum = 0, txDoubleSum = 0;
  RadioWrite(0xAA);
  RadioWrite(groundStationID);
  RadioWrite(0x04);//packet length
  if (calibrationMode == true) {

    RadioWrite(0xF7);//cmd byte
    txSum += 0xF7;
    txDoubleSum += txSum;
    RadioWrite(PROTOCOL_VER_NUM);//version number
    txSum += 1;
    txDoubleSum += txSum;
    RadioWrite(PROTOCOL_VER_SUB_NUM);//sub version number
    txSum += 1;
    txDoubleSum += txSum;
    RadioWrite(NUM_WAY_POINTS);
    txSum += NUM_WAY_POINTS;
    txDoubleSum += txSum;
    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
    for (uint8_t i = 0; i < 15; i++) {
      RadioWrite(0xAA);
      RadioWrite(groundStationID);
      RadioWrite(0x04);//packet length
      RadioWrite(0xF7);//cmd byte
      RadioWrite(1);//version number
      RadioWrite(1);//sub version number
      RadioWrite(NUM_WAY_POINTS);
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);
    }

  }
  else {
    if(calibrationModeESCs == true){
      RadioWrite(0xF6);//cmd byte
      txSum += 0xF6;
      txDoubleSum += txSum;
      RadioWrite(1);//version number
      txSum += 1;
      txDoubleSum += txSum;
      RadioWrite(1);//sub version number
      txSum += 1;
      txDoubleSum += txSum;
      RadioWrite(NUM_WAY_POINTS);
      txSum += NUM_WAY_POINTS;
      txDoubleSum += txSum;
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);
      for (uint8_t i = 0; i < 15; i++) {
        RadioWrite(0xAA);
        RadioWrite(groundStationID);
        RadioWrite(0x04);//packet length
        RadioWrite(0xFE);//cmd byte
        RadioWrite(1);//version number
        RadioWrite(1);//sub version number
        RadioWrite(NUM_WAY_POINTS);
        RadioWrite(txSum);
        RadioWrite(txDoubleSum);
      }
    }
    else{
      RadioWrite(0xFE);//cmd byte
      txSum += 0xFE;
      txDoubleSum += txSum;
      RadioWrite(1);//version number
      txSum += 1;
      txDoubleSum += txSum;
      RadioWrite(1);//sub version number
      txSum += 1;
      txDoubleSum += txSum;
      RadioWrite(NUM_WAY_POINTS);
      txSum += NUM_WAY_POINTS;
      txDoubleSum += txSum;
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);
      for (uint8_t i = 0; i < 15; i++) {
        RadioWrite(0xAA);
        RadioWrite(groundStationID);
        RadioWrite(0x04);//packet length
        RadioWrite(0xFE);//cmd byte
        RadioWrite(1);//version number
        RadioWrite(1);//sub version number
        RadioWrite(NUM_WAY_POINTS);
        RadioWrite(txSum);
        RadioWrite(txDoubleSum);
      }
    }

  }
}

void SendCalData() {
  int16_u temp;

  uint8_t txSum = 0;
  uint8_t txDoubleSum = 0;
  RadioWrite(0xAA);
  RadioWrite(groundStationID);
  switch (calibrationNumber) {
  case 0:
    RadioWrite(7);


    RadioWrite((uint8_t)0x00);
    txSum += 0;
    txDoubleSum += txSum;

    RadioWrite(magX.buffer[0]);
    txSum += magX.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(magX.buffer[1]);
    txSum += magX.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(magY.buffer[0]);
    txSum += magY.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(magY.buffer[1]);
    txSum += magY.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(magZ.buffer[0]);
    txSum += magZ.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(magZ.buffer[1]);
    txSum += magZ.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
    break;
  case 1:
    RadioWrite(7);


    RadioWrite(1);
    txSum += 1;
    txDoubleSum += txSum;

    RadioWrite(accX.buffer[0]);
    txSum += accX.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(accX.buffer[1]);
    txSum += accX.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(accY.buffer[0]);
    txSum += accY.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(accY.buffer[1]);
    txSum += accY.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(accZ.buffer[0]);
    txSum += accZ.buffer[0];
    txDoubleSum += txSum;

    RadioWrite(accZ.buffer[1]);
    txSum += accZ.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
    break;
  case 2:
    RadioWrite(17);

    RadioWrite(2);
    txSum += 2;
    txDoubleSum += txSum;

    temp.val = rcData[0].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[1].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[2].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[3].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[4].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[5].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[6].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    temp.val = rcData[7].rcvd;
    RadioWrite(temp.buffer[0]);
    txSum += temp.buffer[0];
    txDoubleSum += txSum;
    RadioWrite(temp.buffer[1]);
    txSum += temp.buffer[1];
    txDoubleSum += txSum;

    RadioWrite(txSum);
    RadioWrite(txDoubleSum);
    break;
  }
}


















