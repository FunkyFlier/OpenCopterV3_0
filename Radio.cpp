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

uint16_t GSRCValue[8];
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



uint8_t typeNum,cmdNum,itemBuffer[255],calibrationNumber,hsRequestNumber,lsRequestNumber,hsNumItems,lsNumItems, hsList[40], lsList[40];
uint16_t localPacketNumberOrdered, localPacketNumberUn, remotePacketNumberOrdered, remotePacketNumberUn, packetTemp[2];
boolean hsTX,lsTX,sendCalibrationData;
uint32_t hsMillis,lsMillis;
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
    case 0://check for start byte
      rxSum = 0;
      rxDoubleSum = 0;
      if (radioByte == 0xAA) {
        radioState = 1;
      }
      break;
    case 1:
      packetLength = radioByte;
      numRXbytes = 0;
      radioState = 2;
      break;

    case 2:
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (radioByte == 0xFA) { //reliable querie
        radioState = 6;
        break;
      }
      if (radioByte == 0xFD) { //reliable set
        radioState = 12;
        break;
      }
      typeNum = radioByte;
      if (typeNum == 11 && packetLength == 18) {
        radioState = 19;
        break;
      }
      if (packetLength == 2) { //length for unrelaible will always be 2
        radioState = 3;//unrelaible data
      }
      else {
        radioState = 0;
      }
      break;

    case 3://unrelaible
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      radioState = 4;
      break;
    case 4://unreliable checksum 1
      if (rxSum == radioByte) {
        radioState = 5;
        break;
      }
      radioState = 0;
      break;
    case 5://unreliable check sum 2
      if (rxDoubleSum == radioByte) {
        //UnReliableTransmit();
      }
      radioState = 0;
      break;
    case 6://reliable queries - get packet num LSB
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = 7;
      break;
    case 7://packet num MSB and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberUn = (packetTemp[1] << 8 ) | packetTemp[0];
      if (remotePacketNumberUn > localPacketNumberUn) {
        if ( (remotePacketNumberUn - localPacketNumberUn) > 1000) {
          radioState = 8;
          break;
        }
        SendUnMis();
        radioState = 0;
        break;
      }
      if (remotePacketNumberUn == localPacketNumberUn) {
        radioState = 8;
        break;
      }
      if (remotePacketNumberUn < localPacketNumberUn) {
        if ((localPacketNumberUn - remotePacketNumberUn) > 1000) {
          SendUnMis();
          radioState = 0;
        }
        radioState = 8;
      }
      break;
    case 8://get typeNum
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      radioState = 9;
      break;
    case 9://get cmdNum
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = 10;
      break;

    case 10://check the first sum
      if (rxSum == radioByte) {
        radioState = 11;
        break;
      }
      radioState = 0;
      break;

    case 11://check the second sum
      if (rxDoubleSum == radioByte) {
        SendUnAck();
        if (remotePacketNumberUn == localPacketNumberUn) {
          localPacketNumberUn++;
        }
      }
      radioState = 0;

      break;

    case 12://reliable set get packet num lsb
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[0] = radioByte;
      radioState = 13;
      break;

    case 13://get packet num msb and verify
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      packetTemp[1] = radioByte;
      remotePacketNumberOrdered = (packetTemp[1] << 8 ) | packetTemp[0];
      radioState = 14;
      break;

    case 14:
      typeNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      radioState = 15;
      break;

    case 15:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      itemIndex = 0;
      radioState = 16;
      if (typeNum == 6 || typeNum == 8 ) {
        radioState = 17;
      }
      if (typeNum == 7 && cmdNum == 3) {
        radioState = 17;
      }
      break;

    case 16://buffer in data
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      numRXbytes++;
      if (packetLength > 250) {
        radioState = 0;
      }
      if (numRXbytes == packetLength) {
        radioState = 17;
      }
      break;
    case 17://check first sum
      if (rxSum != radioByte) {
        radioState = 0;
        break;
      }
      radioState = 18;
      break;
    case 18:
      if (rxDoubleSum == radioByte) {
        if (remotePacketNumberOrdered != localPacketNumberOrdered) {
          SendOrdMis();
          radioState = 0;
          break;
        }
        if (calibrationMode == true) {
          if (typeNum == 6) {
            sendCalibrationData = true;
            calibrationNumber = cmdNum;
          }
          if (typeNum == 7) {
            WriteCalibrationDataToRom();
            sendCalibrationData = false;
          }
        }
        else {
          if (typeNum < 3) {
            OrderedSet();
          }
          if (typeNum == 4 || typeNum == 5) {
            SetTransmissionRate();
          }
          if (typeNum == 8) {

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
        }
        SendOrdAck();
      }
      radioState = 0;
      break;
    case 19:
      cmdNum = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      itemIndex = 0;
      if (cmdNum == 8) {
        radioState = 20;
      }
      else {
        radioState = 0;
      }
      break;
    case 20:
      itemBuffer[itemIndex++] = radioByte;
      rxSum += radioByte;
      rxDoubleSum += rxSum;
      if (itemIndex == (cmdNum * 2)) {
        radioState = 21;
      }
      break;
    case 21:
      if (rxSum == radioByte) {
        radioState = 22;
      }
      else {
        radioState = 0;
      }
      break;
    case 22:
      if (rxDoubleSum == radioByte) {
        HandleGSRCData();

      }
      radioState = 0;
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

      liveDataBuffer[1] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 2); i++) {
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

      liveDataBuffer[1] = packetLength;
      for (uint8_t i = 0; i < (packetLength + 2); i++) {
        RadioWrite(liveDataBuffer[i]);
      }
      RadioWrite(txSum);
      RadioWrite(txDoubleSum);

    }//+++

  }//---

}//

void SetTransmissionRate() {

  if (typeNum == 4) {
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
  //int16_u temp16;
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
    SendOrdAck();
    //save the packet numbers
    temp = localPacketNumberOrdered & 0x00FF;
    EEPROMWrite(PKT_LOCAL_ORD_L, temp);
    temp = localPacketNumberOrdered >> 8;
    EEPROMWrite(PKT_LOCAL_ORD_M, temp);

    temp = localPacketNumberUn & 0x00FF;
    EEPROMWrite(PKT_LOCAL_UN_L, temp);
    temp = localPacketNumberUn >> 8;
    EEPROMWrite(PKT_LOCAL_UN_M, temp);
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
  }



}


void OrderedSet() {
  float_u outFloat;
  switch (typeNum) {
  case 0:
    if (cmdNum >= KP_PITCH_RATE_ && cmdNum <= MAG_DEC_) {
      for (uint8_t i = 0; i < 4; i++) {
        outFloat.buffer[i] =  itemBuffer[i];
      }
      *floatPointerArray[cmdNum] = outFloat.val;
      saveGainsFlag = true;
      romWriteDelayTimer = millis();

    }
    break;
  case 1:
    /*
      for (uint8_t i = 0; i < 2; i++){
     (*int16PointerArray[cmdNum]).buffer[i] =  itemBuffer[i];
     }*/
    break;
  case 2:
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
  case 0:
    outFloat.val = *floatPointerArray[cmdNum];
    for (uint8_t i = 0; i < 4; i++) {
      itemBuffer[i] = outFloat.buffer[i];
    }
    break;
  case 1:
    outInt16.val = *int16PointerArray[cmdNum];
    for (uint8_t i = 0; i < 2; i++) {
      itemBuffer[i] = outInt16.buffer[i];
    }
    break;
  case 2:
    /*for (uint8_t i = 0; i < 4; i++){
     itemBuffer[i] = (*int32PointerArray[cmdNum]).buffer[i];
     }*/
    break;
  case 3:

    break;
  }

}

void SendUnAck() {
  uint8_t txSum = 0,txDoubleSum = 0,temp;
  float_u outFloat;
  float_u outInt16;
  RadioWrite(0XAA);
  switch (typeNum) {
  case 0:
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
  case 1:
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
  case 2:
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
  case 3:
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



  if (typeNum == 1) {
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

void SendUnMis() {

  uint8_t txSum = 0,txDoubleSum = 0,temp;

  RadioWrite(0xAA);
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

}

void UnReliableTransmit() {
  uint8_t txSum = 0, txDoubleSum = 0;
  float_u outFloat;
  int16_u outInt16;
  RadioWrite(0xAA);
  switch (typeNum) {
  case 0://float
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
  case 1://int16
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
  case 2://int32
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
    packetTemp[0] = EEPROMRead(PKT_LOCAL_UN_L);//lsb for packetNumberLocalUnOrdered
    packetTemp[1] = EEPROMRead(PKT_LOCAL_UN_M);//msb for packetNumberLocalUnOrdered
    localPacketNumberUn = (packetTemp[1] << 8) | packetTemp[0];
    handShake = true;
    return;
  }
  localPacketNumberOrdered = 0;
  localPacketNumberUn = 0;
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

  while (millis() - radioTimer < 1000 && handShake == false) { //***

    if (RadioAvailable() > 0) { //---

      while (RadioAvailable() > 0) { //+++

        radioByte = RadioRead();

        switch (handShakeState) { //^^^

        case 0://check for 0xAA
          rxSum = 0;
          rxDoubleSum = 0;
          calibrationMode = false;
          if (radioByte == 0xAA) {
            handShakeState = 1;
          }
          break;

        case 1://get and verify the length
          if (radioByte == 0x02) { //len will always be 2 for the HS
            handShakeState = 2;
          }
          else {
            handShakeState = 0;
          }
          break;

        case 2://check for correct command byte
          if (radioByte == 0xFF) {
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 3;
          }
          else {
            handShakeState = 0;
          }
          break;

        case 3://check handshake type
          if (radioByte == 0x03) {//gs ctrl calibration 
            gsCTRL = true;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 4;
            calibrationMode = true;
            break;
          }
          if (radioByte == 0x02) {//gs ctrl
            gsCTRL = true;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 4;
            break;
          }
          if (radioByte == 0x01) {//calibration 
            gsCTRL = false;
            rxSum += radioByte;
            rxDoubleSum += rxSum;
            handShakeState = 4;
            calibrationMode = true;
            break;
          }
          if (radioByte == 0x00) {//normal
            gsCTRL = false;
            rxDoubleSum += rxSum;
            handShakeState = 4;
            break;
          }
          handShakeState = 0;

          break;

        case 4://verify sum
          if (radioByte == rxSum) {
            handShakeState = 5;
            break;
          }
          handShakeState = 0;
          break;

        case 5://verify double sum
          if (radioByte == rxDoubleSum) {
            SendHandShakeResponse();
            handShake = true;
          }
          handShakeState = 0;
          break;

        }//^^^

      }//+++

    }//---

  }//***

  if (handShake == false) {
    calibrationMode = false;
    gsCTRL = false;
  }

}

void SendHandShakeResponse() {
  uint8_t txSum = 0, txDoubleSum = 0;
  RadioWrite(0xAA);
  RadioWrite(0x04);//packet length
  if (calibrationMode == true) {

    RadioWrite(0xF7);//cmd byte
    txSum += 0xF7;
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

void SendCalData() {
  int16_u temp;

  uint8_t txSum = 0;
  uint8_t txDoubleSum = 0;
  RadioWrite(0xAA);
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








