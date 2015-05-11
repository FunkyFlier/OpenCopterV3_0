#include "Sensors.h"





#ifdef ROT_45
int16_t tempX, tempY;
#endif
//gyro-----------------------------------
int16_u gyroX, gyroY, gyroZ;
void GyroInit() {

  GyroSSLow();
  SPITransfer(L3G_WHO_AM_I  | READ | SINGLE);
  Serial << _HEX(SPITransfer(0x00)) << "\r\n";
  GyroSSHigh();

  GyroSSLow();
  SPITransfer(L3G_CTRL_REG2 | WRITE | SINGLE);
  SPITransfer(0x00); //high pass filter disabled
  GyroSSHigh();

  GyroSSLow();
  SPITransfer(L3G_CTRL_REG3 | WRITE | SINGLE);
  SPITransfer(0x00); //not using interrupts
  GyroSSHigh();

  GyroSSLow();
  SPITransfer(L3G_CTRL_REG4 | WRITE | SINGLE);
  SPITransfer(0x20); //2000dps scale
  GyroSSHigh();

  GyroSSLow();
  SPITransfer(L3G_CTRL_REG5 | WRITE | SINGLE);
  SPITransfer(0x02); //out select to use the second LPF
  //not using HPF or interrupts
  GyroSSHigh();

  GyroSSLow();
  SPITransfer(L3G_CTRL_REG1 | WRITE | SINGLE);
  SPITransfer(0x8F);
  GyroSSHigh();
}

void GetGro() {

  GyroSSLow();
  SPITransfer(L3G_OUT_X_L  | READ | MULTI);
  gyroX.buffer[0] = SPITransfer(0x00);
  gyroX.buffer[1] = SPITransfer(0x00);
  gyroY.buffer[0] = SPITransfer(0x00);
  gyroY.buffer[1] = SPITransfer(0x00);
  gyroZ.buffer[0] = SPITransfer(0x00);
  gyroZ.buffer[1] = SPITransfer(0x00);

  GyroSSHigh();
#ifdef V1
  gyroY.val *= -1;
  gyroZ.val *= -1;
#endif
#ifdef ROT_45
  tempX = gyroX.val *  0.7071067 + gyroY.val * 0.7071067;
  tempY = gyroX.val * -0.7071067 + gyroY.val * 0.7071067;
  gyroX.val = tempX;
  gyroY.val = tempY;
#endif

}
//end gyro------------------------------

//baro---------------------------------

float initialPressure, pressure, alti;
boolean newBaro = false;

#ifdef V1

long Pressure(unsigned long );
short Temperature(unsigned int );

void StartUT(void);
unsigned int ReadUT(void);
void StartUP(void);
unsigned long ReadUP(void) ;
//v1 vars
//barometer variables
//int32_t pres;
short temperature;
uint32_t baroTimer;
int pressureState;
int ac1;
int ac2;
int ac3;
unsigned int ac4;
unsigned int ac5;
unsigned int ac6;
int b1;
int b2;
int mb;
int mc;
int md;
unsigned char msb;
unsigned char lsb;
unsigned char xlsb;
long x1;
long x2;
long x3;
long b3;
long b5;
long b6;
long p;
unsigned long b4;
unsigned long b7;
unsigned int ut;
unsigned long up;
uint32_t baroPollTimer;
int baroCount;
float baroSum;
long pressureInitial;
#endif//#ifdef V1
//end v1 vars


//v2 vars
#ifdef V2
void GetBaro();
void CheckCRC();

//barometer
uint16_u C1, C2, C3, C4, C5, C6, promSetup, promCRC;
uint32_u D_rcvd;
float D1, D2;
float pres, temperature, dT, TEMP, OFF, SENS, P;
uint8_t baroState;
uint32_t baroRateTimer, baroDelayTimer;


#endif//#ifdef V2
//end barometer

//end v2 vars

#ifdef V2
void PollPressure() {
  if (millis() - baroRateTimer >= BARO_CONV_TIME) {
    switch (baroState) {
    case 0://start temp conv
      BaroSSLow();
      SPITransfer(CONVERT_D2_OSR4096);
      BaroSSHigh();
      baroState = 1;
      baroDelayTimer = millis();
      break;
    case 1:
      if (millis() - baroDelayTimer >= 10) {
        BaroSSLow();
        SPITransfer(ADC_READ);
        D_rcvd.buffer[2] = SPITransfer(0x00);
        D_rcvd.buffer[1] = SPITransfer(0x00);
        D_rcvd.buffer[0] = SPITransfer(0x00);
        D2 = (float)D_rcvd.val;
        BaroSSHigh();
        baroState = 2;
      }
      break;
    case 2:
      BaroSSLow();
      SPITransfer(CONVERT_D1_OSR4096);
      BaroSSHigh();
      baroState = 3;
      baroDelayTimer = millis();
      break;
    case 3:
      if (millis() - baroDelayTimer >= 10) {
        BaroSSLow();
        SPITransfer(ADC_READ);
        D_rcvd.buffer[2] = SPITransfer(0x00);
        D_rcvd.buffer[1] = SPITransfer(0x00);
        D_rcvd.buffer[0] = SPITransfer(0x00);
        D1 = (float)D_rcvd.val;
        BaroSSHigh();
        baroState = 0;
        baroRateTimer = millis();
        GetBaro();
        newBaro = true;
      }
      break;
    }
  }
}



void GetBaro() {


  dT = D2 - (((uint32_t)C5.val) << 8);
  TEMP = (dT * C6.val) / 8388608;
  OFF = C2.val * 65536.0 + (C4.val * dT) / 128;
  SENS = C1.val * 32768.0 + (C3.val * dT) / 256;

  if (TEMP < 0) {
    // second order temperature compensation when under 20 degrees C
    float T2 = (dT * dT) / 0x80000000;
    float Aux = TEMP * TEMP;
    float OFF2 = 2.5 * Aux;
    float SENS2 = 1.25 * Aux;
    TEMP = TEMP - T2;
    OFF = OFF - OFF2;
    SENS = SENS - SENS2;
  }

  P = (D1 * SENS / 2097152 - OFF) / 32768;
  temperature = TEMP + 2000;
  pressure = P;


}

void BaroInit() {
  BaroSSLow();
  SPITransfer(MS5611_RESET);
  BaroSSHigh();
  delay(5);

  BaroSSLow();
  SPITransfer(MS5611_PROM_Setup);
  promSetup.buffer[1] = SPITransfer(0x00);
  promSetup.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C1);
  C1.buffer[1] = SPITransfer(0x00);
  C1.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C2);
  C2.buffer[1] = SPITransfer(0x00);
  C2.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C3);
  C3.buffer[1] = SPITransfer(0x00);
  C3.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C4);
  C4.buffer[1] = SPITransfer(0x00);
  C4.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C5);
  C5.buffer[1] = SPITransfer(0x00);
  C5.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_C6);
  C6.buffer[1] = SPITransfer(0x00);
  C6.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();
  delay(1);
  BaroSSLow();
  SPITransfer(MS5611_PROM_CRC);
  promCRC.buffer[1] = SPITransfer(0x00);
  promCRC.buffer[0] = SPITransfer(0x00);
  BaroSSHigh();

  Serial << C1.val << "," << C2.val << "," << C3.val << "," << C4.val << "," << C5.val << "," << C6.val << "\r\n";

  CheckCRC();


  baroRateTimer = millis();
  while (newBaro == false) {
    PollPressure();
  }
  if (newBaro == true) {
    newBaro = false;
    initialPressure = pressure;
  }

}

void CheckCRC() {
  int16_t cnt;
  uint16_t n_rem;
  uint16_t crc_read;
  uint8_t n_bit;
  uint16_t n_prom[8] = {
    promSetup.val, C1.val, C2.val, C3.val, C4.val, C5.val, C6.val, promCRC.val
  };
  //uint16_t n_prom[8] = {0,0,0,0,0,0,0,0};
  n_rem = 0x00;

  crc_read = n_prom[7];

  n_prom[7] = (0xFF00 & (n_prom[7]));

  for (cnt = 0; cnt < 16; cnt++) {
    if (cnt & 1) {
      n_rem ^= (uint8_t)((n_prom[cnt >> 1]) & 0x00FF);

    }
    else {
      n_rem ^= (uint8_t)(n_prom[cnt >> 1] >> 8);
    }

    for (n_bit = 8; n_bit > 0; n_bit--) {
      if (n_rem & 0x8000) {
        n_rem = (n_rem << 1) ^ 0x3000;

      }
      else {
        n_rem = (n_rem << 1);
      }
    }
  }

  n_rem = (0x000F & (n_rem >> 12));
  n_prom[7] = crc_read;


  if ((0x000F & crc_read) == (n_rem ^ 0x00)) {
    Serial << "CRC passed\r\n";
  }
  else {
    Serial << "CRC failed\r\n";
  }
}

#endif//#ifdef V2



#ifdef V1
void PollPressure(void) {
  if (millis() - baroPollTimer > POLL_RATE) {
    switch (pressureState) {
    case 0://read ut
      StartUT();
      pressureState = 1;
      baroTimer = millis();
      break;
    case 1://wait for ready signal
      if (millis() - baroTimer > 5) {
        pressureState = 2;
        ut = ReadUT();
        StartUP();
        baroTimer = millis();
      }

      break;
    case 2://read up
      if (millis() - baroTimer > CONV_TIME) {
        up = ReadUP();
        temperature = Temperature(ut);
        pressure = (float)Pressure(up);
        pressureState = 0;
        newBaro = true;
        baroPollTimer = millis();
      }
      break;

    }
  }
}

long Pressure(unsigned long up) {


  b6 = b5 - 4000;
  // Calculate B3
  x1 = (b2 * (b6 * b6) >> 12) >> 11;
  x2 = (ac2 * b6) >> 11;
  x3 = x1 + x2;
  b3 = (((((long)ac1) * 4 + x3) << OSS) + 2) >> 2;

  // Calculate B4
  x1 = (ac3 * b6) >> 13;
  x2 = (b1 * ((b6 * b6) >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (unsigned long)(x3 + 32768)) >> 15;

  b7 = ((unsigned long)(up - b3) * (50000 >> OSS));
  if (b7 < 0x80000000)
    p = (b7 << 1) / b4;
  else
    p = (b7 / b4) << 1;

  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  p += (x1 + x2 + 3791) >> 4;

  return p;
}

short Temperature(unsigned int ut) {

  x1 = (((long)ut - (long)ac6) * (long)ac5) >> 15;
  x2 = ((long)mc << 11) / (x1 + md);
  b5 = x1 + x2;

  return ((b5 + 8) >> 4);
}

void StartUT(void) {
  I2CWrite(BMP085_ADDRESS, 0xF4, 0x2E);
}

unsigned int ReadUT(void) {



  I2CRead(BMP085_ADDRESS, 0xF6, 2);
  msb = I2CReceive();
  lsb = I2CReceive();

  return ((msb << 8) | lsb);
}

void StartUP(void) {
  I2CWrite(BMP085_ADDRESS, 0xF4, (0x34 + (OSS << 6)));
}

unsigned long ReadUP(void) {

  I2CRead(BMP085_ADDRESS, 0xF6, 3);
  msb = I2CReceive();
  lsb = I2CReceive();
  xlsb = I2CReceive();
  return ((((unsigned long) msb << 16) | ((unsigned long) lsb << 8) | (unsigned long) xlsb) >> (8 - OSS));
}

void BaroInit(void) {
  pressureState = 0;
  newBaro = false;
  I2CRead(BMP085_ADDRESS, 0xAA, 22);
  msb = I2CReceive();
  lsb = I2CReceive();
  ac1 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  ac2 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  ac3 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  ac4 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  ac5 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  ac6 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  b1 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  b2 = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  mb = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  mc = (msb << 8) | lsb;

  msb = I2CReceive();
  lsb = I2CReceive();
  md = (msb << 8) | lsb;

  I2CRead(BMP085_ADDRESS,0xD0,1);
  Serial<<"bmp 085 "<<_HEX(I2CReceive())<<"\r\n";
  //this is to get the ground pressure for relative altitude
  //lower pressure than this means positive altitude
  //higher pressure than this means negative altitude
  baroCount = 0;
  baroSum = 0;
  while (baroCount < 10) { //use a while instead of a for loop because the for loop runs too fast
    PollPressure();
    if (newBaro == true) {
      newBaro = false;
      baroCount++;
      baroSum += pressure;
    }
  }
  initialPressure = baroSum / 10;



}

#endif//#ifdef V1


boolean GetAltitude(float *press, float *pressInit, float *alti) {
  if (*pressInit <= 0 || *press <= 0){
    *alti = 0;
    return false;
  }
  float pressureRatio =  *press /  *pressInit;
  *alti = (1.0f - pow(pressureRatio, 0.190295f)) * 44330.0f;
  return true;
}

//end baro-----------------------------

//acc----------------------------------
int16_u  accX, accY, accZ;

#ifdef V1



void AccInit() {

  SPISetMode(SPI_MODE3);

  AccSSLow();
  SPITransfer(WRITE | SINGLE | BW_RATE);
  SPITransfer(0x0C);
  AccSSHigh();

  AccSSLow();
  SPITransfer(WRITE | SINGLE | POWER_CTL);
  SPITransfer(0x08);//start measurment
  AccSSHigh();

  AccSSLow();
  SPITransfer(WRITE | SINGLE | DATA_FORMAT);
  SPITransfer(0x08);//full resolution + / - 16g
  AccSSHigh();
  SPISetMode(SPI_MODE0);

  GetAcc();

}

void GetAcc() {
  SPISetMode(SPI_MODE3);
  AccSSLow();
  SPITransfer(DATAX0 | READ | MULTI);
  accX.buffer[0] = SPITransfer(0x00);
  accX.buffer[1] = SPITransfer(0x00);
  accY.buffer[0] = SPITransfer(0x00);
  accY.buffer[1] = SPITransfer(0x00);
  accZ.buffer[0] = SPITransfer(0x00);
  accZ.buffer[1] = SPITransfer(0x00);
  AccSSHigh();
  SPISetMode(SPI_MODE0);

  accY.val *= -1;
  accZ.val *= -1;
#ifdef ROT_45
  tempX = accX.val *  0.7071067 + accY.val * 0.7071067;
  tempY = accX.val * -0.7071067 + accY.val * 0.7071067;
  accX.val = tempX;
  accY.val = tempY;
#endif
  GetAcc();


}
#endif//#ifdef V1

#ifdef V2

void GetAcc() {
  AccSSLow();
  SPITransfer(OUT_X_L_A | READ | MULTI);
  accX.buffer[0] = SPITransfer(0x00);
  accX.buffer[1] = SPITransfer(0x00);
  accY.buffer[0] = SPITransfer(0x00);
  accY.buffer[1] = SPITransfer(0x00);
  accZ.buffer[0] = SPITransfer(0x00);
  accZ.buffer[1] = SPITransfer(0x00);
  accX.val = accX.val >> 4;
  accY.val = accY.val >> 4;
  accZ.val = accZ.val >> 4;

  AccSSHigh();
#ifdef ROT_45
  tempX = accX.val *  0.7071067 + accY.val * 0.7071067;
  tempY = accX.val * -0.7071067 + accY.val * 0.7071067;
  accX.val = tempX;
  accY.val = tempY;
#endif
  //Serial<<"* "<<accX.val<<","<<accY.val<<","<<accZ.val<<"\r\n";
}
void AccInit() {
  AccSSLow();
  SPITransfer(CTRL_REG1_A | WRITE | SINGLE);
  SPITransfer(0x77);//400Hz all axes enabled
  AccSSHigh();

  AccSSLow();
  SPITransfer(CTRL_REG2_A | WRITE | SINGLE);
  SPITransfer(0x00);//high pass filter not used
  AccSSHigh();

  AccSSLow();
  SPITransfer(CTRL_REG3_A | WRITE | SINGLE);
  SPITransfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPITransfer(CTRL_REG4_A | WRITE | SINGLE);
  SPITransfer(0x18);//little endian
  AccSSHigh();

  AccSSLow();
  SPITransfer(CTRL_REG5_A | WRITE | SINGLE);
  SPITransfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  AccSSLow();
  SPITransfer(CTRL_REG6_A | WRITE | SINGLE);
  SPITransfer(0x00);//not using interrupts for polling sensor
  AccSSHigh();

  GetAcc();
}
#endif//#ifdef V2

//end acc------------------------------

//mag----------------------------------
int16_u magX,magY,magZ;
uint8_t i2cTimeOutStatus,i2cTimeOutCount;
boolean magDetected = true;
void VerifyMag();

void VerifyMag() {
  I2CRead((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_ID_A, (uint8_t)3);
  
  if (I2CReceive() != 0x48) {
    Serial<<"id1\r\n";
    magDetected = false;
    return;
  }

  if (I2CReceive() != 0x34) {
    Serial<<"id2\r\n";
    magDetected = false;
    return;
  }

  if (I2CReceive() != 0x33) {
    Serial<<"id3\r\n";
    magDetected = false;
    return;
  }
}

void MagInit() {
  //continous conversion 220Hz
  I2CWrite((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_CRA_REG, (uint8_t)0x9C);
  I2CWrite((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_CRB_REG, (uint8_t)0x60);
  I2CWrite((uint8_t)MAG_ADDRESS, (uint8_t)HMC5983_MR_REG, (uint8_t)0x80);


  VerifyMag();



  I2CRead(MAG_ADDRESS, HMC5983_OUT_X_H, 6);
  magX.buffer[1] = I2CReceive();//X
  magX.buffer[0] = I2CReceive();
  magZ.buffer[1] = I2CReceive();//Z
  magZ.buffer[0] = I2CReceive();
  magY.buffer[1] = I2CReceive();//Y
  magY.buffer[0] = I2CReceive();
#ifdef V1
#ifndef EXT_MAG
      magY.val *= -1;
      magZ.val *= -1;
#endif
#endif
  GetMag();

}
void GetMag() {
  i2cTimeOutStatus = I2CRead(MAG_ADDRESS, HMC5983_OUT_X_H, 6);
  if (i2cTimeOutCount == 10){
    magDetected = false;
    //imu.magDetected = false;
    Serial<<"mag lost\r\n";
    return;
  }
  if (i2cTimeOutStatus != 0){
    i2cTimeOutCount++;
    return;
  }
  i2cTimeOutCount = 0;
  magX.buffer[1] = I2CReceive();//X
  magX.buffer[0] = I2CReceive();
  magZ.buffer[1] = I2CReceive();//Z
  magZ.buffer[0] = I2CReceive();
  magY.buffer[1] = I2CReceive();//Y
  magY.buffer[0] = I2CReceive();
#ifdef V1
#ifndef EXT_MAG
      magY.val *= -1;
      magZ.val *= -1;
#endif
#endif



}

//end mag------------------------------

