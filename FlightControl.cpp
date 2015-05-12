#include "FlightControl.h"


void PollAcc();
void PollMag();
void PollGro();

uint32_t _100HzTimer,_400HzTimer;
volatile uint32_t RCFailSafeCounter,watchDogFailSafeCounter;
float initialYaw;
boolean integrate;
uint8_t HHState;
float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
uint8_t XYLoiterState, ZLoiterState,flightMode,RTBState,txLossRTB;
boolean telemFailSafe,txFailSafe;

float homeBaseXOffset,homeBaseYOffset;
float xTarget,yTarget,zTarget;
float velSetPointX,velSetPointY,velSetPointZ;
float tiltAngleX,tiltAngleY;
float distToWayPoint;
float controlBearing;
boolean enterState;

uint8_t modeArray[9] = {
  RATE,L1,ATT,ATT,ATT,ATT_TRIM,RATE,RATE,L0};

float kp_pitch_rate;//52
float ki_pitch_rate;//56
float kd_pitch_rate;//60
float fc_pitch_rate;//64

float kp_roll_rate;//68
float ki_roll_rate;//72
float kd_roll_rate;//76
float fc_roll_rate;//80

float kp_yaw_rate;//84
float ki_yaw_rate;//88
float kd_yaw_rate;//92
float fc_yaw_rate;//96

float kp_pitch_attitude;//100
float ki_pitch_attitude;//104
float kd_pitch_attitude;//108
float fc_pitch_attitude;//112

float kp_roll_attitude;//116
float ki_roll_attitude;//120
float kd_roll_attitude;//124
float fc_roll_attitude;//128

float kp_yaw_attitude;//132
float ki_yaw_attitude;//136
float kd_yaw_attitude;//140
float fc_yaw_attitude;//144

float kp_altitude_position;//148
float ki_altitude_position;//152
float kd_altitude_position;//156
float fc_altitude_position;//160

float kp_altitude_velocity;//164
float ki_altitude_velocity;//168
float kd_altitude_velocity;//172
float fc_altitude_velocity;///176
//float mul_altitude_velocity;

float kp_loiter_pos_x;//180
float ki_loiter_pos_x;//184
float kd_loiter_pos_x;//188
float fc_loiter_pos_x;//192

float kp_loiter_velocity_x;//196
float ki_loiter_velocity_x;//200
float kd_loiter_velocity_x;//204
float fc_loiter_velocity_x;//208

float kp_loiter_pos_y;//212
float ki_loiter_pos_y;//216
float kd_loiter_pos_y;//220
float fc_loiter_pos_y;//224

float kp_loiter_velocity_y;//228
float ki_loiter_velocity_y;//232
float kd_loiter_velocity_y;//236
float fc_loiter_velocity_y;//240

float kp_waypoint_position;//244
float ki_waypoint_position;//248
float kd_waypoint_position;//252
float fc_waypoint_position;//256

float kp_waypoint_velocity;//260
float ki_waypoint_velocity;//264
float kd_waypoint_velocity;//268
float fc_waypoint_velocity;//272

float kp_cross_track;//276
float ki_cross_track;//280
float kd_cross_track;//284
float fc_cross_track;//288

float pitchSetPoint;
float rollSetPoint;
float yawSetPoint;
float rateSetPointX;
float rateSetPointY;
float rateSetPointZ;



void _400HzTask() {
  uint32_t _400HzTime;

  _400HzTime = micros();
  if ( _400HzTime - _400HzTimer  >= 2500) {
    //what to do with DT calculation
    D22High();
    _400HzTimer = _400HzTime;
    PollAcc();
    D22Low();
  }
}

void _100HzTask(uint32_t loopTime){
  static uint8_t _100HzState = 0;

  float _100HzDt;

  if (loopTime - _100HzTimer >= 10000){
    _100HzDt = (loopTime - _100HzTimer) * 0.000001;
    _100HzTimer = loopTime;
    D23High();
    while(_100HzState < LAST_100HZ_TASK){
      switch (_100HzState){
      case GET_GYRO:
        PollGro();
        if(magDetected == true){
          _100HzState = GET_MAG;
        }
        else{
          _100HzState = ATT_UPDATE;
        }
        break;
      case GET_MAG:
        PollMag();  
        _100HzState = ATT_UPDATE;
        break;
      case ATT_UPDATE:
        AHRSupdate(_100HzDt);
        _100HzState = ROT_MATRIX;
        break;
      case ROT_MATRIX:
        GenerateRotationMatrix();
        _100HzState = GET_EULER;
        break;
      case GET_EULER:
        GetEuler();
        _100HzState = GET_INERTIAL;
        break;
      case GET_INERTIAL:
        GetInertial();
        _100HzState = POS_VEL_PREDICTION;
        break;
      case POS_VEL_PREDICTION:
        Predict(_100HzDt);
        _100HzState = POLL_GPS;
        break;
      case UPDATE_LAG_INDEX:
        UpdateLagIndex();
        _100HzState = POLL_GPS;
        break;
      case POLL_GPS:
        GPSMonitor();
        if (newGPSData == true) {
          newGPSData = false;
          CorrectXY();
        }
        _100HzState = POLL_BARO;
        break;
      case POLL_BARO:
        PollPressure();
        if (newBaro == true) {
          newBaro = false;
          CorrectZ();
        }
        _100HzState = PROCESS_CONTROL_SIGNALS;
        break;
      case PROCESS_CONTROL_SIGNALS:
        if (newRC == true) {//9
          newRC = false;
          ProcessChannels();
          RCFailSafeCounter = 0;
        }
        if (RCFailSafeCounter > 200){
          RCFailSafe = true;
        }
        _100HzState = LAST_100HZ_TASK;
        break;
      default:
        _100HzState = GET_GYRO;
        break;
      }
      _400HzTask();

    }
    D23Low();
    _100HzState = GET_GYRO;
  }




}
void PollAcc(){
  GetAcc();
  ACCScale();
}
void PollMag(){
  GetMag();
  MAGScale();
}
void PollGro(){
  GetGro();
  GROScale();
}

