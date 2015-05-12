#ifndef ENUMS_H
#define ENUMS_H
enum _100HzStates{
  GET_GYRO,GET_MAG,ATT_UPDATE,ROT_MATRIX,GET_EULER,GET_INERTIAL,POS_VEL_PREDICTION,UPDATE_LAG_INDEX,POLL_GPS,POLL_BARO,PROCESS_CONTROL_SIGNALS,LAST_100HZ_TASK
};

enum Floats {
  GYRO_X_DEG,
  GYRO_Y_DEG,
  GYRO_Z_DEG,
  ACC_X_FILT,
  ACC_Y_FILT,
  ACC_Z_FILT,
  ACC_X_SC,
  ACC_Y_SC,
  ACC_Z_SC,
  MAG_X_SC,
  MAG_Y_SC,
  MAG_Z_SC,
  DIST_TO_CRAFT,
  HEAD_TO_CRAFT,
  RAW_X,
  RAW_Y,
  RAW_Z,
  VEL_N,
  VEL_E,
  VEL_D,
  VEL_BARO,
  PITCH_,
  ROLL_,
  YAW_,
  QUAT_0,
  QUAT_1,
  QUAT_2,
  QUAT_3,
  X_EST,
  Y_EST,
  Z_EST,
  VEL_X,
  VEL_Y,
  VEL_Z,
  ACC_BIAS_X,
  ACC_BIAS_Y,
  ACC_BIAS_Z,
  INERTIAL_X,
  INERTIAL_Y,
  INERTIAL_Z,
  INERTIAL_X_BIASED,
  INERTIAL_Y_BIASED,
  INERTIAL_Z_BIASED,
  RAW_PITCH,
  RAW_ROLL,
  PITCH_OFFSET,
  ROLL_OFFSET,
  KP_PITCH_RATE_,
  KI_PITCH_RATE_,
  KD_PITCH_RATE_,
  FC_PITCH_RATE_,
  KP_ROLL_RATE_,
  KI_ROLL_RATE_,
  KD_ROLL_RATE_,
  FC_ROLL_RATE_,
  KP_YAW_RATE_,
  KI_YAW_RATE_,
  KD_YAW_RATE_,
  FC_YAW_RATE_,
  KP_PITCH_ATT_,
  KI_PITCH_ATT_,
  KD_PITCH_ATT_,
  FC_PITCH_ATT_,
  KP_ROLL_ATT_,
  KI_ROLL_ATT_,
  KD_ROLL_ATT_,
  FC_ROLL_ATT_,
  KP_YAW_ATT_,
  KI_YAW_ATT_,
  KD_YAW_ATT_,
  FC_YAW_ATT_,
  KP_ALT_POS_,
  KI_ALT_POS_,
  KD_ATL_POS_,
  FC_ALT_POS_,
  KP_ALT_VEL_,
  KI_ALT_VEL_,
  KD_ALT_VEL_,
  FC_ALT_VEL_,
  KP_LOIT_X_POS_,
  KI_LOIT_X_POS_,
  KD_LOIT_X_POS_,
  FC_LOIT_X_POS_,
  KP_LOIT_X_VEL_,
  KI_LOIT_X_VEL_,
  KD_LOIT_X_VEL_,
  FC_LOIT_X_VEL_,
  KP_LOIT_Y_POS_,
  KI_LOIT_Y_POS_,
  KD_LOIT_Y_POS_,
  FC_LOIT_Y_POS_,
  KP_LOIT_Y_VEL_,
  KI_LOIT_Y_VEL_,
  KD_LOIT_Y_VEL_,
  FC_LOIT_Y_VEL_,
  KP_WP_POS_,
  KI_WP_POS_,
  KD_WP_POS_,
  FC_WP_POS_,
  KP_WP_VEL_,
  KI_WP_VEL_,
  KD_WP_VEL_,
  FC_WP_VEL_,
  KP_CT_,
  KI_CT_,
  KD_CT_,
  FC_CT_,
  MAG_DEC_,
  RATE_SP_X,
  RATE_SP_Y,
  RATE_SP_Z,
  ADJ_X,
  ADJ_Y,
  ADJ_Z,
  PITCH_SP,
  ROLL_SP,
  YAW_SP,
  X_TARG,
  Y_TARG,
  Z_TARG,
  VEL_SP_X,
  VEL_SP_Y,
  VEL_SP_Z,
  TILT_X,
  TILT_Y,
  THRO_ADJ,
  PITCH_SP_TX,
  ROLL_SP_TX,
  DIST_TO_WP,
  TARGET_VEL_WP,
  MOTOR_CMD_1,
  MOTOR_CMD_2,
  MOTOR_CMD_3,
  MOTOR_CMD_4,
  MOTOR_CMD_5,
  MOTOR_CMD_6,
  MOTOR_CMD_7,
  MOTOR_CMD_8,
  PRESSURE_,
  CTRL_BEARING,
  YAW_INITIAL,
  GPS_ALT,
  LAT_,
  LON_,
  HB_LAT,
  HB_LON,
  H_ACC,
  S_ACC

};
enum motorControlStates {
  HOLD,
  TO,
  FLIGHT,
  LANDING
};


enum loiterStates {
  LOITERING,
  RCINPUT,
  LAND,
  WAIT
};

enum FlightStates {
  RATE,
  RATE_TRIM,
  ATT,
  ATT_TRIM,
  L0,
  L1,
  L2,
  FOLLOW,
  WP,
  RTB
};



enum Int16s {
  GYRO_X,
  GYRO_Y,
  GYRO_Z,
  ACC_X,
  ACC_Y,
  ACC_Z,
  MAG_X,
  MAG_Y,
  MAG_Z,
  THRO_CMD,
  PWM_HIGH,
  PWM_LOW

};

enum BYTES {
  F_MODE_,
  GPS_FIX,
  XY_LOIT_STATE,
  Z_LOIT_STATE,
  RTB_STATE,
  MOTOR_STATE,
  TELEM_FS,
  GPS_FS,
  SWITCH_POS,
  IDLE_PERCENT,
  HOVER_PERCENT,
  TX_LOSS_RTB,
  MAG_DET,
  TX_FS_STATUS

};

enum CalibrationFlags {
  RC_FLAG,
  ACC_FLAG,
  MAG_FLAG,
  GAINS_FLAG
};

enum ISR_States {
  STAND,PPM};

enum RC_Types {
  DSM10, DSM11, SBUS, RC};
enum RC_Chan {
  THRO, AILE, ELEV, RUDD, GEAR, AUX1, AUX2, AUX3};
#endif
