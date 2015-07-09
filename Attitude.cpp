#include "Attitude.h"

#include <EEPROM.h>
#include "Streaming_.h"

void SetVariables();
void GetPitch();
void GetRoll();
void GetYaw();
void SetInitialAccelerometerMagnitude();

float yawInDegrees,pitchInDegrees,rollInDegrees;
float yawInRadians,pitchInRadians,rollInRadians;
float R11,R12,R13,R21,R22,R23,R31,R32,R33;
float R11_,R12_,R13_,R21_,R22_,R23_,R31_,R32_,R33_;
float declination;
float cosDec,sinDec;

float q0=1,q1=0,q2=0,q3=0;
float rawPitch,rawRoll,pitchOffset,rollOffset;

float q0q0,q1q1,q2q2,q3q3,q0q1,q0q2,q0q3,q1q2,q1q3,q2q3;
float acc_x,acc_y,acc_z,mag_x,mag_y,mag_z,gro_x,gro_y,gro_z;
float initialAccMagnitude;


void SetInitialAccelerometerMagnitude(){
  float accSumX = 0,accSumY = 0,accSumZ = 0;
  float avgX,avgY,avgZ;
  for(uint16_t i = 0; i < SAMPLES_FOR_ACC_MAGNITUDE; i++){
    GetAcc();
    ACCScale();
    accSumX += scaledAccX;
    accSumY += scaledAccY;
    accSumZ += scaledAccZ;

    if (StationaryGyro() == false){
      accSumX = scaledAccX;
      accSumY = scaledAccY;
      accSumZ = scaledAccZ;
      i = 1;
    }
  }


  avgX = accSumX / SAMPLES_FOR_ACC_MAGNITUDE;
  avgY = accSumY / SAMPLES_FOR_ACC_MAGNITUDE;
  avgZ = accSumZ / SAMPLES_FOR_ACC_MAGNITUDE;  
  initialAccMagnitude = sqrt(avgX * avgX + avgY * avgY + avgZ * avgZ);
}

void AHRSupdate(float dt) {

  static float integralFBX = 0,integralFBY = 0,integralFBZ = 0;
  float magnitude,recipNorm;
  float qa,qb,qc;

  float kiDTAcc,kiDTMag,dtby2;
  float bx,bz,wx,wy,wz,vx,vy,vz;

  float hx,hy,hz,exm,eym,ezm,exa,eya,eza;

  float magnitudeDifference;

  SetVariables();

  magnitude =  sqrt(acc_x * acc_x + acc_y * acc_y + acc_z * acc_z);
  magnitudeDifference = fabs(initialAccMagnitude -  magnitude);
  if (magnitudeDifference < FEEDBACK_LIMIT ){

    recipNorm = 1/magnitude;
    acc_x *= recipNorm;
    acc_y *= recipNorm;
    acc_z *= recipNorm;

    if (magDetected == true){
      recipNorm = 1.0/sqrt(mag_x * mag_x + mag_y * mag_y + mag_z * mag_z);
      mag_x *= recipNorm;
      mag_y *= recipNorm;
      mag_z *= recipNorm;

      hx = R11 * mag_x + R21 * mag_y + R31 * mag_z;
      hy = R12 * mag_x + R22 * mag_y + R32 * mag_z;
      hz = R13 * mag_x + R23 * mag_y + R33 * mag_z;


      bx = sqrt(hx * hx + hy * hy);
      bz = hz;


      wx = R11*bx + R13*bz;
      wy = R21*bx + R23*bz;
      wz = R31*bx + R33*bz;


      exm = (mag_y * wz - mag_z * wy);
      eym = (mag_z * wx - mag_x * wz);
      ezm = (mag_x * wy - mag_y * wx);
    }
    else{
      exm = 0;
      eym = 0;
      ezm = 0;
    }


    vx = R13;
    vy = R23;
    vz = R33;


    exa = (acc_y * vz - acc_z * vy);
    eya = (acc_z * vx - acc_x * vz);
    eza = (acc_x * vy - acc_y * vx);

    kiDTAcc = KI_ACC * dt;
    kiDTMag = KI_MAG * dt;
    if (KI_ACC > 0){
      integralFBX += exa * kiDTAcc+ exm * kiDTMag;
      integralFBY += eya * kiDTAcc+ eym * kiDTMag;
      integralFBZ += eza * kiDTAcc+ ezm * kiDTMag;
      gro_x = gro_x + integralFBX;
      gro_y = gro_y + integralFBY;
      gro_z = gro_z + integralFBZ;  
    }
    else{
      integralFBX = 0;
      integralFBY = 0;
      integralFBZ = 0;  
    }
    gro_x += exa * KP_ACC + exm * KP_MAG;
    gro_y += eya * KP_ACC + eym * KP_MAG;
    gro_z += eza * KP_ACC + ezm * KP_MAG;
  }


  dtby2 = dt * 0.5;
  qa = q0;
  qb = q1;
  qc = q2;
  q0 += -1.0 * dtby2*(gro_x * qb + gro_y * qc + gro_z * q3);
  q1 +=      dtby2*(gro_x * qa - gro_y * q3 + gro_z * qc);
  q2 +=      dtby2*(gro_x * q3 + gro_y * qa - gro_z * qb);
  q3 +=      dtby2*(gro_y * qb - gro_x * qc + gro_z * qa);



  //normalize the quaternion
  recipNorm = 1/sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
  q0 *= recipNorm;
  q1 *= recipNorm;
  q2 *= recipNorm;
  q3 *= recipNorm;

}

void SetInitialQuaternion(){
  float magnitude;
  float bx,by;

  for(uint8_t i = 0; i < 100; i++){
    while(StationaryGyro() == false){
    }
    PollAcc();
    PollMag();
    delay(3);
  }
  SetVariables();
  //calculate the ypr from sensors convert to quaternion and rotation matrix
  pitchInRadians = atan2(-acc_x,sqrt(acc_y * acc_y + acc_z * acc_z));
  rollInRadians = atan2(acc_y,acc_z);
  
  yawInRadians = 0;
  q0 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q1 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0) - sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0); 
  q2 = cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q3 = sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) - cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0);
  magnitude = sqrt(q0 *  q0 + q1 *  q1 + q2 *  q2 + q3 *  q3); 
  q0 = q0 / magnitude;
  q1 = q1 / magnitude;
  q2 = q2 / magnitude;
  q3 = q3 / magnitude;
  cosDec = 1;
  sinDec = 0;
  GenerateRotationMatrix();
  GetEuler();

  if (magDetected == true){
    bx = mag_x * cos(pitchInRadians) + mag_y * sin(pitchInRadians) * sin(rollInRadians) + mag_z * sin(pitchInRadians) * cos(rollInRadians);
    by = mag_z * sin(rollInRadians) - mag_y * cos(rollInRadians);
    yawInRadians = atan2(by, bx);
  }
  else{
    yawInRadians = 0;
  }

  q0 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q1 = cos(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0) - sin(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0); 
  q2 = cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*cos(rollInRadians/2.0) + sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*sin(rollInRadians/2.0); 
  q3 = sin(yawInRadians/2.0)*cos(pitchInRadians/2.0)*cos(rollInRadians/2.0) - cos(yawInRadians/2.0)*sin(pitchInRadians/2.0)*sin(rollInRadians/2.0);
  magnitude = sqrt(q0 *  q0 + q1 *  q1 + q2 *  q2 + q3 *  q3); 
  q0 = q0 / magnitude;
  q1 = q1 / magnitude;
  q2 = q2 / magnitude;
  q3 = q3 / magnitude;

  cosDec = cos(declination);
  sinDec = sin(declination);
  GenerateRotationMatrix();
  GetEuler();

  SetGyroOffsets();
 
  SetInitialAccelerometerMagnitude();
}

void GenerateRotationMatrix(){
  q0q0 = q0*q0;
  q1q1 = q1*q1;
  q2q2 = q2*q2;
  q3q3 = q3*q3;

  q0q1 = q0*q1;
  q0q2 = q0*q2;
  q0q3 = q0*q3;

  q1q2 = q1*q2;
  q1q3 = q1*q3;

  q2q3 = q2*q3;
  //generate rotation matrix
  R11 = 2.0*(q0q0-0.5+q1q1);
  R12 = 2.0*(q1q2+q0q3);
  R13 = 2.0*(q1q3-q0q2);
  R21 = 2.0*(q1q2-q0q3);
  R22 = 2.0*(q0q0-0.5+q2q2);
  R23 = 2.0*(q2q3+q0q1);
  R31 = 2.0*(q1q3+q0q2);
  R32 = 2.0*(q2q3-q0q1);
  R33 = 2.0*(q0q0-0.5+q3q3);  
  //rotate by declination so the inertial navigation can be done in the ture north frame instead of mag north frame
  R11_ = R11*cosDec - R12*sinDec;
  R12_ = R12*cosDec + R11*sinDec;
  R13_ = R13;

  R21_ = R21*cosDec - R22*sinDec;
  R22_ = R22*cosDec + R21*sinDec;
  R23_ = R23;

  R31_ = R31*cosDec - R32*sinDec;
  R32_ = R32*cosDec + R31*sinDec;
  R33_ = R33;



}

void SetVariables(){
  acc_x = -filtAccX;
  acc_y = -filtAccY;
  acc_z = -filtAccZ;
  mag_x =  scaledMagX;
  mag_y =  scaledMagY;
  mag_z =  scaledMagZ;
  gro_x =  radianGyroX;
  gro_y =  radianGyroY;
  gro_z =  radianGyroZ;
}

void GetEuler(){
  GetPitch();
  GetRoll();
  GetYaw();
}
void GetPitch(){
  pitchInRadians = asin(2.0 * (q0 * q2 - q3 * q1));
  rawPitch =  ToDeg(pitchInRadians);
  pitchInDegrees = rawPitch - pitchOffset;
}

void GetRoll(){
  rollInRadians = atan2(2 * (q0 * q1 + q2 * q3),1 - 2.0 * (q1 * q1 + q2 * q2));
  rawRoll = ToDeg(rollInRadians);
  rollInDegrees = rawRoll - rollOffset;
}

void GetYaw(){
  yawInRadians = atan2(2.0 * (q0 * q3 + q1 * q2) , 1 - 2.0* (q2 * q2 + q3 * q3)) - declination;
  yawInDegrees = ToDeg(yawInRadians);
  
  if (yawInDegrees < 0){
    yawInDegrees +=360;
  }
}

















