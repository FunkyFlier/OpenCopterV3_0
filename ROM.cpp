#include "ROM.h"
#include "Attitude.h"
#include "Calibration.h"
#include "Comm.h"
#include "Definitions.h"
#include "Enums.h"
#include "FlightControl.h"
#include "GPS.h"
#include "Inertial.h"
#include "LED.h"
#include "Motors.h"
#include "Sensors.h"
#include "Radio.h"
#include "RCSignals.h"
#include "Types.h"
#include "Flash.h"

#include <EEPROM.h>

void LoadModes();
void LoadDEC();
void LoadPROff();
void LoadGains();
void LoadRC();
void LoadPWMLimits();
void LoadCeilingFloor();
void SetDefaultGains();
void LoadMotorMix();
void LoadEstimatorGains();

float* floatPointerArray[197];

int16_t* int16PointerArray[15];

uint8_t* bytePointerArray[17];

uint8_t propIdlePercent,hoverPercent;

void AssignPointerArray() {
  floatPointerArray[GYRO_X_DEG] = &degreeGyroX;//calibartion 
  floatPointerArray[GYRO_Y_DEG] = &degreeGyroY;
  floatPointerArray[GYRO_Z_DEG] = &degreeGyroZ;

  floatPointerArray[ACC_X_FILT] = &filtAccX;//calibartion
  floatPointerArray[ACC_Y_FILT] = &filtAccY;
  floatPointerArray[ACC_Z_FILT] = &filtAccZ;

  floatPointerArray[ACC_X_SC] = &scaledAccX;//calibration
  floatPointerArray[ACC_Y_SC] = &scaledAccY;
  floatPointerArray[ACC_Z_SC] = &scaledAccZ;

  floatPointerArray[MAG_X_SC] = &scaledMagX;//cailbration
  floatPointerArray[MAG_Y_SC] = &scaledMagY;
  floatPointerArray[MAG_Z_SC] = &scaledMagZ;

  floatPointerArray[DIST_TO_CRAFT] = &distToCraft;//inertial
  floatPointerArray[HEAD_TO_CRAFT] = &headingToCraft;

  floatPointerArray[RAW_X] = &gpsX;//inertial
  floatPointerArray[RAW_Y] = &gpsY;
  floatPointerArray[RAW_Z] = &baroZ;

  floatPointerArray[VEL_N] = &velN;//gps
  floatPointerArray[VEL_E] = &velE;
  floatPointerArray[VEL_D] = &velD;

  floatPointerArray[VEL_BARO] = &baroVel;

  floatPointerArray[PITCH_] = &pitchInDegrees;//attitude
  floatPointerArray[ROLL_] = &rollInDegrees;
  floatPointerArray[YAW_] = &yawInDegrees;

  floatPointerArray[QUAT_0] = &q0;//attitude
  floatPointerArray[QUAT_1] = &q1;
  floatPointerArray[QUAT_2] = &q2;
  floatPointerArray[QUAT_3] = &q3;

  floatPointerArray[X_EST] = &XEst;//inertial
  floatPointerArray[Y_EST] = &YEst;
  floatPointerArray[Z_EST] = &ZEstUp;

  floatPointerArray[VEL_X] = &velX;//inertial
  floatPointerArray[VEL_Y] = &velY;
  floatPointerArray[VEL_Z] = &velZUp;

  floatPointerArray[ACC_BIAS_X] = &accelBiasX;//inerital
  floatPointerArray[ACC_BIAS_Y] = &accelBiasY;
  floatPointerArray[ACC_BIAS_Z] = &accelBiasZ;

  floatPointerArray[INERTIAL_X] = &inertialX;//inerital
  floatPointerArray[INERTIAL_Y] = &inertialY;
  floatPointerArray[INERTIAL_Z] = &inertialZ;

  floatPointerArray[INERTIAL_X_BIASED] = &inertialXBiased;//inerital
  floatPointerArray[INERTIAL_Y_BIASED] = &inertialYBiased;
  floatPointerArray[INERTIAL_Z_BIASED] = &inertialZBiased;

  floatPointerArray[RAW_PITCH] = &rawPitch;//attitude
  floatPointerArray[RAW_ROLL] = &rawRoll;
  floatPointerArray[PITCH_OFFSET] = &pitchOffset;
  floatPointerArray[ROLL_OFFSET] = &rollOffset;


  floatPointerArray[KP_PITCH_RATE_] = &kp_pitch_rate;//flight control
  floatPointerArray[KI_PITCH_RATE_] = &ki_pitch_rate;
  floatPointerArray[KD_PITCH_RATE_] = &kd_pitch_rate;
  floatPointerArray[FC_PITCH_RATE_] = &fc_pitch_rate;

  floatPointerArray[KP_ROLL_RATE_] = &kp_roll_rate;
  floatPointerArray[KI_ROLL_RATE_] = &ki_roll_rate;
  floatPointerArray[KD_ROLL_RATE_] = &kd_roll_rate;
  floatPointerArray[FC_ROLL_RATE_] = &fc_roll_rate;

  floatPointerArray[KP_YAW_RATE_] = &kp_yaw_rate;
  floatPointerArray[KI_YAW_RATE_] = &ki_yaw_rate;
  floatPointerArray[KD_YAW_RATE_] = &kd_yaw_rate;
  floatPointerArray[FC_YAW_RATE_] = &fc_yaw_rate;

  floatPointerArray[KP_PITCH_ATT_] = &kp_pitch_attitude;
  floatPointerArray[KI_PITCH_ATT_] = &ki_pitch_attitude;
  floatPointerArray[KD_PITCH_ATT_] = &kd_pitch_attitude;
  floatPointerArray[FC_PITCH_ATT_] = &fc_pitch_attitude;

  floatPointerArray[KP_ROLL_ATT_] = &kp_roll_attitude;
  floatPointerArray[KI_ROLL_ATT_] = &ki_roll_attitude;
  floatPointerArray[KD_ROLL_ATT_] = &kd_roll_attitude;
  floatPointerArray[FC_ROLL_ATT_] = &fc_roll_attitude;

  floatPointerArray[KP_YAW_ATT_] = &kp_yaw_attitude;
  floatPointerArray[KI_YAW_ATT_] = &ki_yaw_attitude;
  floatPointerArray[KD_YAW_ATT_] = &kd_yaw_attitude;
  floatPointerArray[FC_YAW_ATT_] = &fc_yaw_attitude;


  floatPointerArray[KP_ALT_POS_] = &kp_altitude_position;
  floatPointerArray[KI_ALT_POS_] = &ki_altitude_position;
  floatPointerArray[KD_ATL_POS_] = &kd_altitude_position;
  floatPointerArray[FC_ALT_POS_] = &fc_altitude_position;

  floatPointerArray[KP_ALT_VEL_] = &kp_altitude_velocity;
  floatPointerArray[KI_ALT_VEL_] = &ki_altitude_velocity;
  floatPointerArray[KD_ALT_VEL_] = &kd_altitude_velocity;
  floatPointerArray[FC_ALT_VEL_] = &fc_altitude_velocity;

  floatPointerArray[KP_LOIT_X_POS_] = &kp_loiter_pos_x;
  floatPointerArray[KI_LOIT_X_POS_] = &ki_loiter_pos_x;
  floatPointerArray[KD_LOIT_X_POS_] = &kd_loiter_pos_x;
  floatPointerArray[FC_LOIT_X_POS_] = &fc_loiter_pos_x;

  floatPointerArray[KP_LOIT_X_VEL_] = &kp_loiter_velocity_x;
  floatPointerArray[KI_LOIT_X_VEL_] = &ki_loiter_velocity_x;
  floatPointerArray[KD_LOIT_X_VEL_] = &kd_loiter_velocity_x;
  floatPointerArray[FC_LOIT_X_VEL_] = &fc_loiter_velocity_x;

  floatPointerArray[KP_LOIT_Y_POS_] = &kp_loiter_pos_y;
  floatPointerArray[KI_LOIT_Y_POS_] = &ki_loiter_pos_y;
  floatPointerArray[KD_LOIT_Y_POS_] = &kd_loiter_pos_y;
  floatPointerArray[FC_LOIT_Y_POS_] = &fc_loiter_pos_y;

  floatPointerArray[KP_LOIT_Y_VEL_] = &kp_loiter_velocity_y;
  floatPointerArray[KI_LOIT_Y_VEL_] = &ki_loiter_velocity_y;
  floatPointerArray[KD_LOIT_Y_VEL_] = &kd_loiter_velocity_y;
  floatPointerArray[FC_LOIT_Y_VEL_] = &fc_loiter_velocity_y;

  floatPointerArray[KP_WP_POS_] = &kp_waypoint_position;
  floatPointerArray[KI_WP_POS_] = &ki_waypoint_position;
  floatPointerArray[KD_WP_POS_] = &kd_waypoint_position;
  floatPointerArray[FC_WP_POS_] = &fc_waypoint_position;

  floatPointerArray[KP_WP_VEL_] = &kp_waypoint_velocity;
  floatPointerArray[KI_WP_VEL_] = &ki_waypoint_velocity;
  floatPointerArray[KD_WP_VEL_] = &kd_waypoint_velocity;
  floatPointerArray[FC_WP_VEL_] = &fc_waypoint_velocity;

  floatPointerArray[KP_CT_] = &kp_cross_track;
  floatPointerArray[KI_CT_] = &ki_cross_track;
  floatPointerArray[KD_CT_] = &kd_cross_track;
  floatPointerArray[FC_CT_] = &fc_cross_track;

  floatPointerArray[MAG_DEC_] = &declination;//attitude

  floatPointerArray[RATE_SP_X] = &rateSetPointX;//flight control
  floatPointerArray[RATE_SP_Y] = &rateSetPointY;
  floatPointerArray[RATE_SP_Z] = &rateSetPointZ;

  floatPointerArray[ADJ_X] = &adjustmentX;//flight control
  floatPointerArray[ADJ_Y] = &adjustmentY;
  floatPointerArray[ADJ_Z] = &adjustmentZ;

  floatPointerArray[PITCH_SP] = &pitchSetPoint;//flight control
  floatPointerArray[ROLL_SP] = &rollSetPoint;
  floatPointerArray[YAW_SP] = &yawSetPoint;

  floatPointerArray[X_TARG] = &xTarget;//flight control
  floatPointerArray[Y_TARG] = &yTarget;
  floatPointerArray[Z_TARG] = &zTarget;

  floatPointerArray[VEL_SP_X] = &velSetPointX;//flight control
  floatPointerArray[VEL_SP_Y] = &velSetPointY;
  floatPointerArray[VEL_SP_Z] = &velSetPointZ;

  floatPointerArray[TILT_X] = &tiltAngleX;//flight control
  floatPointerArray[TILT_Y] = &tiltAngleY;
  floatPointerArray[THRO_ADJ] = &throttleAdjustment;

  floatPointerArray[PITCH_SP_TX] = &pitchSetPointTX;
  floatPointerArray[ROLL_SP_TX] = &rollSetPointTX;

  floatPointerArray[DIST_TO_WP] = &distToWayPoint;//flight control
  floatPointerArray[TARGET_VEL_WP] = &landingThroAdjustment;
  floatPointerArray[MOTOR_CMD_1] = &motorCommand1;//motors
  floatPointerArray[MOTOR_CMD_2] = &motorCommand2;
  floatPointerArray[MOTOR_CMD_3] = &motorCommand3;
  floatPointerArray[MOTOR_CMD_4] = &motorCommand4;
  floatPointerArray[MOTOR_CMD_5] = &motorCommand5;
  floatPointerArray[MOTOR_CMD_6] = &motorCommand6;
  floatPointerArray[MOTOR_CMD_7] = &motorCommand7;
  floatPointerArray[MOTOR_CMD_8] = &motorCommand8;

  floatPointerArray[PRESSURE_] = &pressure;//sensors
  floatPointerArray[CTRL_BEARING] = &controlBearing;//flight control
  floatPointerArray[YAW_INITIAL] = &initialYaw;//flight control
  floatPointerArray[GPS_ALT] = &gpsAlt;//gps


  floatPointerArray[LAT_] = &floatLat;//gps
  floatPointerArray[LON_] = &floatLon;
  floatPointerArray[HB_LAT] = &homeLatFloat;
  floatPointerArray[HB_LON] = &homeLonFloat;
  floatPointerArray[H_ACC] = &hAcc;
  floatPointerArray[S_ACC] = &sAcc;


  floatPointerArray[M1_X] = &m1X;
  floatPointerArray[M1_Y] = &m1Y;
  floatPointerArray[M1_Z] = &m1Z;

  floatPointerArray[M2_X] = &m2X;
  floatPointerArray[M2_Y] = &m2Y;
  floatPointerArray[M2_Z] = &m2Z;

  floatPointerArray[M3_X] = &m3X;
  floatPointerArray[M3_Y] = &m3Y;
  floatPointerArray[M3_Z] = &m3Z;

  floatPointerArray[M4_X] = &m4X;
  floatPointerArray[M4_Y] = &m4Y;
  floatPointerArray[M4_Z] = &m4Z;

  floatPointerArray[M5_X] = &m5X;
  floatPointerArray[M5_Y] = &m5Y;
  floatPointerArray[M5_Z] = &m5Z;

  floatPointerArray[M6_X] = &m6X;
  floatPointerArray[M6_Y] = &m6Y;
  floatPointerArray[M6_Z] = &m6Z;

  floatPointerArray[M7_X] = &m7X;
  floatPointerArray[M7_Y] = &m7Y;
  floatPointerArray[M7_Z] = &m7Z;

  floatPointerArray[M8_X] = &m8X;
  floatPointerArray[M8_Y] = &m8Y;
  floatPointerArray[M8_Z] = &m8Z;

  floatPointerArray[KP_ACC] = &kpAcc;
  floatPointerArray[KI_ACC] = &kiAcc;
  floatPointerArray[KP_MAG] = &kpMag;
  floatPointerArray[KI_MAG] = &kiMag;
  floatPointerArray[FEEDBACK_LIMIT] = &feedbackLimit;
  floatPointerArray[K_P_GPS] = &kPosGPS;
  floatPointerArray[K_V_GPS] = &kVelGPS;
  floatPointerArray[K_B_GPS] = &kBiasGPS;
  floatPointerArray[K_P_BARO] = &kPosBaro;
  floatPointerArray[K_V_BARO] = &kVelBaro;
  floatPointerArray[K_B_BARO] = &kBiasBaro;
  floatPointerArray[K_P_BARO] = &kPosBaro;
  floatPointerArray[K_P_V_BARO] = &kPosVelBaro;
  floatPointerArray[K_P_B_BARO] = &kPosBiasBaro;


  floatPointerArray[M_AH] = &mAh;
  floatPointerArray[BATT_PERCENT] = &batteryPercent;
  floatPointerArray[CELL_VOLT] = &cellVoltage;

  floatPointerArray[X_ERROR_POS] = &xPosOutput;
  floatPointerArray[X_ERROR_VEL] = &xVelOutput;
  floatPointerArray[Y_ERROR_POS] = &yPosOutput;
  floatPointerArray[Y_ERROR_VEL] = &yVelOutput;
  floatPointerArray[Z_ERROR_POS] = &zPosError;
  floatPointerArray[Z_ERROR_VEL] = &zVelError;
  floatPointerArray[ACC_ERROR_X] = &exa;
  floatPointerArray[ACC_ERROR_Y] = &eya;
  floatPointerArray[ACC_ERROR_Z] = &eza;


  int16PointerArray[GYRO_X] = &gyroX.val;//sensors
  int16PointerArray[GYRO_Y] = &gyroY.val;
  int16PointerArray[GYRO_Z] = &gyroZ.val;
  int16PointerArray[ACC_X] = &accX.val;
  int16PointerArray[ACC_Y] = &accY.val;
  int16PointerArray[ACC_Z] = &accZ.val;
  int16PointerArray[MAG_X] = &magX.val;
  int16PointerArray[MAG_Y] = &magY.val;
  int16PointerArray[MAG_Z] = &magZ.val;

  int16PointerArray[THRO_CMD] = &throttleCommand;//motors

  int16PointerArray[PWM_HIGH] = &pwmHigh;//mtors
  int16PointerArray[PWM_LOW] = &pwmLow;

  int16PointerArray[CEILING_LIMIT] = &ceilingLimit;//mtors
  int16PointerArray[FLOOR_LIMIT] = &floorLimit;


  bytePointerArray[F_MODE_] = &flightMode;//flight control
  bytePointerArray[GPS_FIX] = &GPSData.vars.gpsFix;//GPS
  bytePointerArray[XY_LOIT_STATE] = &XYLoiterState;//flight control
  bytePointerArray[Z_LOIT_STATE] = &flightModeControl;//flight control

    bytePointerArray[RTB_STATE] = &RTBState;//flight control
  bytePointerArray[MOTOR_STATE] = &motorState;//motors
  bytePointerArray[TELEM_FS] = &GSCTRLFailSafe;//flight control
  bytePointerArray[GPS_FS] = &gpsFailSafe;//gps

  bytePointerArray[SWITCH_POS] = &wayPointState;//&switchPositions;//&switchPositions;//&wayPointState;//RC


  bytePointerArray[IDLE_PERCENT] = &GSCTRLFailSafe ;//rom
  bytePointerArray[HOVER_PERCENT] = &hoverPercent;//rom
  bytePointerArray[TX_LOSS_RTB] = &baroFS;//flight control
  bytePointerArray[MAG_DET] = &magDetected;//sensors
  bytePointerArray[TX_FS_STATUS] = &txFailSafe;

  bytePointerArray[GPS_START_STATE] = &gpsStartState;//sensors
  bytePointerArray[INIT_PROG] = &initProgress;


} 

void ROMFlagsCheck() {
  uint16_t j;
  float_u outFloat;
  int16_u outInt16;
  uint8_t LEDControlByte = 0,calibrationFlags;
  initProgress = 2;
  if (EEPROMRead(VER_FLAG_1) != VER_NUM_1 || EEPROMRead(VER_FLAG_2) != VER_NUM_2) {
    for (uint16_t i = 0; i < 700; i++) {
      EEPROMWrite(i, 0xFF);
    }
    EEPROMWrite(VER_FLAG_1, VER_NUM_1);
    EEPROMWrite(VER_FLAG_2, VER_NUM_2);
  }
  if (EEPROMRead(EST_FLAG) != 0xAA){
    EEPROMWrite(EST_FLAG,0xAA);
    kpAcc = 1.5;
    kiAcc = 0;
    kpMag = 1.5;
    kiMag = 0;
    feedbackLimit = 0.25;

    kPosGPS = 0.1;
    kVelGPS = 0.35;
    kBiasGPS = 0.001;
    kPosBaro = 0.07;
    kVelBaro = 0.08;
    kBiasBaro = 0.001;
    kPosVelBaro = 0;
    kPosBiasBaro = 0;    
    j = EST_GAIN_START;
    for(uint16_t i = KP_ACC; i <= K_P_B_BARO; i++){
      outFloat.val = *floatPointerArray[i];
      EEPROMWrite(j++, outFloat.buffer[0]);
      EEPROMWrite(j++, outFloat.buffer[1]);
      EEPROMWrite(j++, outFloat.buffer[2]);
      EEPROMWrite(j++, outFloat.buffer[3]);
    }
  }
  if (EEPROMRead(CEILING_FLOOR_FLAG) != 0xAA){
    EEPROMWrite(CEILING_FLOOR_FLAG,0xAA);
    outInt16.val = (int16_t)CEILING;
    EEPROMWrite(CEILING_START, outInt16.buffer[0]);
    EEPROMWrite(CEILING_END, outInt16.buffer[1]);
    outInt16.val = (int16_t)FLOOR;
    EEPROMWrite(FLOOR_START, outInt16.buffer[0]);
    EEPROMWrite(FLOOR_END, outInt16.buffer[1]);
  }
  if(EEPROMRead(SWIFT_X_FLAG) !=0xAA){
    EEPROMWrite(SWIFT_X_FLAG,0xAA);
    EEPROMWrite(ROT_45,0x00);
  }
  if (EEPROMRead(ROT_45) == 0x01){
    rotateSensor45Deg = true;
  }
  else{
    rotateSensor45Deg = false;
  }
  if (EEPROMRead(MIX_FLAG) != 0xAA){
    EEPROMWrite(MIX_FLAG,0xAA);
    m1X = m1Y = m2Y = m2Z = m4X = m4Z = m5X = m5Y = m6Y = m6Z = m8X = m8Z =  1.0;
    m1Z = m2X = m3X = m3Y = m3Z = m4Y = m5Z = m6X = m7X = m7Y = m7Z = m8Y = -1.0;
    j = MIX_START;
    for(uint16_t i = M1_X; i <= M8_Z; i++){
      outFloat.val = *floatPointerArray[i];
      EEPROMWrite(j++, outFloat.buffer[0]);
      EEPROMWrite(j++, outFloat.buffer[1]);
      EEPROMWrite(j++, outFloat.buffer[2]);
      EEPROMWrite(j++, outFloat.buffer[3]);
    }
  }
  if (EEPROMRead(TX_FS_FLAG) != 0xAA) {
    EEPROMWrite(TX_FS, 0);
    EEPROMWrite(TX_FS_FLAG, 0xAA);
  }
  if (EEPROMRead(PR_FLAG) != 0xAA) {
    pitchOffset = 0;
    rollOffset = 0;
    j = 0;
    outFloat.val = 0;
    for (uint16_t i = PITCH_OFFSET_START; i <= PITCH_OFFSET_END; i++) {
      EEPROMWrite(i, outFloat.buffer[j++]);
    }
    j = 0;
    for (uint16_t i = ROLL_OFFSET_START; i <= ROLL_OFFSET_END; i++) {
      EEPROMWrite(i, outFloat.buffer[j++]);
    }
    EEPROMWrite(PR_FLAG, 0xAA);
  }
  if (EEPROMRead(PWM_FLAG) != 0xAA) {
    pwmHigh = 2000;
    pwmLow = 1000;
    outInt16.val = pwmHigh;
    EEPROMWrite(PWM_LIM_HIGH_START, outInt16.buffer[0]);
    EEPROMWrite(PWM_LIM_HIGH_END, outInt16.buffer[1]);
    outInt16.val = pwmLow;
    EEPROMWrite(PWM_LIM_LOW_START, outInt16.buffer[0]);
    EEPROMWrite(PWM_LIM_LOW_END, outInt16.buffer[1]);
    EEPROMWrite(PWM_FLAG, 0xAA);
  }
  if (EEPROMRead(PROP_IDLE_FLAG) != 0xAA) {
    EEPROMWrite(PROP_IDLE_FLAG, 0xAA);
    EEPROMWrite(PROP_IDLE, 12);
  }
  if (EEPROMRead(HOVER_THRO_FLAG) != 0xAA) {
    EEPROMWrite(HOVER_THRO_FLAG, 0xAA);
    EEPROMWrite(HOVER_THRO, 55);

  }

  if (EEPROMRead(MODE_FLAG) != 0xAA){
    EEPROMWrite(MODE_FLAG,0xAA);
    j = MODE_START;
    EEPROMWrite(j++,L0);
    EEPROMWrite(j++,L1);
    EEPROMWrite(j++,L2);
    EEPROMWrite(j++,ATT);
    EEPROMWrite(j++,ATT);
    EEPROMWrite(j++,ATT_TRIM);
    EEPROMWrite(j++,RATE);
    EEPROMWrite(j++,RATE);
    EEPROMWrite(j++,RATE_TRIM);
  }
  calibrationFlags = EEPROMRead(CAL_FLAGS);
  VerifyMag();
  if ( (((calibrationFlags & (1 << RC_FLAG)) >> RC_FLAG) == 0x01 && rcDetected == true && RCFailSafe == false)|| ((calibrationFlags & (1 << ACC_FLAG)) >> ACC_FLAG) == 0x01 || ( ((calibrationFlags & (1 << MAG_FLAG)) >> MAG_FLAG) == 0x01 && magDetected ) ) {
    TryHandShake();
    if (calibrationMode == true) {
      return;
    }
    if ( ((calibrationFlags & (1 << RC_FLAG)) >> RC_FLAG) == 0x01 ) {
      LEDControlByte |= 1<<0;
    }
    if ( ((calibrationFlags & (1 << ACC_FLAG)) >> ACC_FLAG) == 0x01 ) {
      LEDControlByte |= 1<<1;
    }
    if ( ((calibrationFlags & (1 << MAG_FLAG)) >> MAG_FLAG) == 0x01 ) {
      LEDControlByte |= 1<<2;
    }
    LEDControlByte |= 1<<3;
    ControlLED(LEDControlByte);

    TIMSK5 = (0<<OCIE5A);
    while (1) {
      ControlLED(0x08);
      delay(500);
      ControlLED(LEDControlByte);
      delay(500);
    }
  }

  if ( ((calibrationFlags & (1 << GAINS_FLAG)) >> GAINS_FLAG) == 0x01 ) {
    SetDefaultGains();
  }
  LoadROM();


}
void SetDefaultGains() {

  uint16_t j;
  float_u outFloat;

  kp_pitch_rate = 0.7;
  ki_pitch_rate = 6.0;
  kd_pitch_rate = 0.055;
  fc_pitch_rate = 30.0;

  kp_roll_rate = 0.7;
  ki_roll_rate = 6.0;
  kd_roll_rate = 0.05;
  fc_roll_rate = 30.0;

  kp_yaw_rate = 5.0;
  ki_yaw_rate = 0.85;
  kd_yaw_rate = 0.03;
  fc_yaw_rate = 30.0;

  kp_pitch_attitude = 3.0;
  ki_pitch_attitude = 0;
  kd_pitch_attitude = 0.05;
  fc_pitch_attitude = 30.0;

  kp_roll_attitude = 3.0;
  ki_roll_attitude = 0;
  kd_roll_attitude = 0.05;
  fc_roll_attitude = 30;

  kp_yaw_attitude = 5;
  ki_yaw_attitude = 0;
  kd_yaw_attitude = 0.01;
  fc_yaw_attitude = 30.0;

  kp_altitude_position = 0.75;
  ki_altitude_position = 0;
  kd_altitude_position = -0.002;
  fc_altitude_position = 30;

  kp_altitude_velocity = 140;
  ki_altitude_velocity = 210;
  kd_altitude_velocity = 25;
  fc_altitude_velocity = 30;

  kp_loiter_pos_x = 0.75;
  ki_loiter_pos_x = 0;
  kd_loiter_pos_x = -0.001;
  fc_loiter_pos_x = 30;

  kp_loiter_velocity_x = 7.5;
  ki_loiter_velocity_x = 0.5;
  kd_loiter_velocity_x = 0.1;
  fc_loiter_velocity_x = 30;

  kp_loiter_pos_y = 0.75;
  ki_loiter_pos_y = 0.0;
  kd_loiter_pos_y = -0.001;
  fc_loiter_pos_y = 30;

  kp_loiter_velocity_y = 7.5;
  ki_loiter_velocity_y = 0.5;
  kd_loiter_velocity_y = 0.1;
  fc_loiter_velocity_y = 30;

  kp_waypoint_position = 0.75;
  ki_waypoint_position = 0;
  kd_waypoint_position = -0.001;
  fc_waypoint_position = 0;

  kp_waypoint_velocity = 7.5;
  ki_waypoint_velocity = 0.5;
  kd_waypoint_velocity = 0.1;
  fc_waypoint_velocity = 30;

  kp_cross_track = 7.5;
  ki_cross_track = 0.5;
  kd_cross_track = 0.1;
  fc_cross_track = 30;

  declination = ToRad(3.3);
  j = GAINS_START;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++) {
    outFloat.val = *floatPointerArray[i];
    EEPROMWrite(j++, outFloat.buffer[0]);
    EEPROMWrite(j++, outFloat.buffer[1]);
    EEPROMWrite(j++, outFloat.buffer[2]);
    EEPROMWrite(j++, outFloat.buffer[3]);
  }
  j = DEC_START;
  outFloat.val = *floatPointerArray[MAG_DEC_];
  EEPROMWrite(j++, outFloat.buffer[0]);
  EEPROMWrite(j++, outFloat.buffer[1]);
  EEPROMWrite(j++, outFloat.buffer[2]);
  EEPROMWrite(j++, outFloat.buffer[3]);


}
void LoadPWMLimits() {
  int16_u outInt16;
  outInt16.buffer[0] = EEPROMRead(PWM_LIM_HIGH_START);
  outInt16.buffer[1] = EEPROMRead(PWM_LIM_HIGH_END);
  pwmHigh = outInt16.val;
  if (pwmHigh > 2000) {
    pwmHigh = 2000;
  }
  if (pwmHigh < 1800) {
    pwmHigh = 1800;
  }
  outInt16.buffer[0] = EEPROMRead(PWM_LIM_LOW_START);
  outInt16.buffer[1] = EEPROMRead(PWM_LIM_LOW_END);
  pwmLow = outInt16.val;
  if (pwmLow < 1000) {
    pwmLow = 1000;
  }
  if (pwmLow > 1200) {
    pwmLow = 1200;
  }
  propIdlePercent = EEPROMRead(PROP_IDLE);
  if (propIdlePercent > 20) {
    propIdleCommand = pwmLow * (1 + (20.0 / 100.0));
  }
  else {
    propIdleCommand = pwmLow * (1 + ((float)propIdlePercent / 100.0));
  }

}
void LoadRC() {
  uint16_t j = 0; //index for input buffers
  uint16_t k = 0; //index for start of each channel's data in rom
  uint16_t l = 0; //index for each channel
  uint16_t switchControl;
  int16_u outInt16;
  float_u outFloat;

  for (uint16_t i = RC_DATA_START; i <= RC_DATA_END; i++) { //index for each rom location
    switchControl = i - k;
    if (switchControl < CHAN_INDEX) { //first 16 bit ints
      outInt16.buffer[j++] = EEPROMRead(i);
    }
    if (switchControl > CHAN_INDEX && i - k < REV_INDEX) { //scale factor
      outFloat.buffer[j++] = EEPROMRead(i);
    }

    switch (switchControl) {
    case MAX_INDEX://max
      rcData[l].max = outInt16.val;
      j = 0;
      break;
    case MIN_INDEX://min
      rcData[l].min = outInt16.val;
      j = 0;
      break;
    case MID_INDEX://mid
      rcData[l].mid = outInt16.val;
      j = 0;
      break;
    case CHAN_INDEX://chan
      rcData[l].chan = EEPROMRead(i);
      break;
    case SCALE_INDEX://scale
      rcData[l].scale = outFloat.val;
      j = 0;
      break;
    case REV_INDEX://reverse
      rcData[l].reverse = EEPROMRead(i);
      k += 12;
      l += 1;
      break;
    }
  }
  txLossRTB = EEPROMRead(TX_FS);
  if (txLossRTB > 1) {
    txLossRTB = 0;
  }

}
void LoadACC() {
  uint8_t outFloatIndex = 0;
  float_u outFloat;
  for (uint16_t i = ACC_CALIB_START; i <= ACC_CALIB_END; i++) { //load acc values
    outFloat.buffer[outFloatIndex] = EEPROMRead(i);
    outFloatIndex++;
    switch (i) {
    case ACC_S_X_INDEX:
      accXScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_S_Y_INDEX:
      accYScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_S_Z_INDEX:
      accZScale = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_X_INDEX:
      accXOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_Y_INDEX:
      accYOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    case ACC_O_Z_INDEX:
      accZOffset = outFloat.val;
      outFloatIndex = 0;
      break;
    default:
      break;
    }
  }
}

void LoadMAG() {

  float_u outFloat;
  uint8_t outFloatIndex = 0;
  for (uint16_t i = MAG_CALIB_START; i <= MAG_CALIB_END; i++) { //load the compass values

    outFloat.buffer[outFloatIndex] = EEPROMRead(i);
    outFloatIndex++;
    switch (i) {
    case MAG_OFF_X_INDEX:
      magOffSetX = outFloat.val;
      outFloatIndex = 0;
      break;
    case MAG_OFF_Y_INDEX:
      magOffSetY = outFloat.val;
      outFloatIndex = 0;
      break;
    case MAG_OFF_Z_INDEX:
      magOffSetZ = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_00_INDEX:
      magWInv00 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_01_INDEX:
      magWInv01 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_02_INDEX:
      magWInv02 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_10_INDEX:
      magWInv10 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_11_INDEX:
      magWInv11 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_12_INDEX:
      magWInv12 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_20_INDEX:
      magWInv20 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_21_INDEX:
      magWInv21 = outFloat.val;
      outFloatIndex = 0;
      break;
    case W_22_INDEX:
      magWInv22 = outFloat.val;
      outFloatIndex = 0;
      break;
    default:
      break;
    }
  }
}

void LoadGains() {
  float_u outFloat;
  uint16_t j = GAINS_START;
  for (uint16_t i = KP_PITCH_RATE_; i <= FC_CT_; i++) { //gains
    outFloat.buffer[0] = EEPROMRead(j++);
    outFloat.buffer[1] = EEPROMRead(j++);
    outFloat.buffer[2] = EEPROMRead(j++);
    outFloat.buffer[3] = EEPROMRead(j++);
    *floatPointerArray[i] = outFloat.val;
  }
}

void LoadPROff() {
  float_u outFloat;
  uint16_t j = PITCH_OFFSET_START;
  for (uint16_t i = PITCH_OFFSET; i <= ROLL_OFFSET; i++) { //pitch and roll offsets
    outFloat.buffer[0] = EEPROMRead(j++);
    outFloat.buffer[1] = EEPROMRead(j++);
    outFloat.buffer[2] = EEPROMRead(j++);
    outFloat.buffer[3] = EEPROMRead(j++);
    *floatPointerArray[i] = outFloat.val;
  }
}

void LoadDEC() {
  uint16_t j = DEC_START;
  float_u outFloat;
  outFloat.buffer[0] = EEPROMRead(j++);
  outFloat.buffer[1] = EEPROMRead(j++);
  outFloat.buffer[2] = EEPROMRead(j++);
  outFloat.buffer[3] = EEPROMRead(j++);
  *floatPointerArray[MAG_DEC_] = outFloat.val;
  cosDec = cos(declination);
  sinDec = sin(declination);
}

void LoadModes(){
  uint8_t j = 0;
  for(uint16_t i = MODE_START; i <= MODE_END; i++){
    modeArray[j++] = EEPROMRead(i);
 }
}
void LoadCeilingFloor(){
  int16_u outInt16;
  outInt16.buffer[0] = EEPROMRead(CEILING_START);
  outInt16.buffer[1] = EEPROMRead(CEILING_END);
  ceilingLimit = outInt16.val;

  outInt16.buffer[0] = EEPROMRead(FLOOR_START);
  outInt16.buffer[1] = EEPROMRead(FLOOR_END);
  floorLimit = outInt16.val;

}
void LoadMotorMix(){
  float_u outFloat;
  uint16_t j = MIX_START;
  for (uint16_t i = M1_X; i <= M8_Z; i++) { //pitch and roll offsets
    outFloat.buffer[0] = EEPROMRead(j++);
    outFloat.buffer[1] = EEPROMRead(j++);
    outFloat.buffer[2] = EEPROMRead(j++);
    outFloat.buffer[3] = EEPROMRead(j++);
    *floatPointerArray[i] = outFloat.val;
  }

}
void LoadEstimatorGains(){
  float_u outFloat;
  uint16_t j = EST_GAIN_START;
  for (uint16_t i = KP_ACC; i <= K_P_B_BARO; i++) { //pitch and roll offsets
    outFloat.buffer[0] = EEPROMRead(j++);
    outFloat.buffer[1] = EEPROMRead(j++);
    outFloat.buffer[2] = EEPROMRead(j++);
    outFloat.buffer[3] = EEPROMRead(j++);
    *floatPointerArray[i] = outFloat.val;
  }
}
void LoadROM() {
  LoadRC();
  LoadACC();
  LoadMAG();
  LoadGains();
  LoadPROff();
  LoadPWMLimits();
  LoadModes();
  LoadDEC();
  LoadCeilingFloor();
  LoadMotorMix();
  LoadEstimatorGains();
}

















