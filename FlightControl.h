#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include "PID.h"
#include <Arduino.h>

void _400HzTask();
void _100HzTask(uint32_t);
void ProcessChannels();
void Arm();

extern uint8_t flightMode;
extern uint32_t _100HzTimer,_400HzTimer;
extern volatile uint32_t RCFailSafeCounter,watchDogFailSafeCounter,groundFSCount;
extern float initialYaw;
extern boolean integrate;
extern uint8_t HHState;
extern float landingThroAdjustment,throttleAdjustment,adjustmentX,adjustmentY,adjustmentZ;
extern uint8_t XYLoiterState,ZLoiterState,flightMode,RTBState,txLossRTB;
extern float homeBaseXOffset,homeBaseYOffset;
extern float xTarget,yTarget,zTarget;
extern float velSetPointX,velSetPointY,velSetPointZ;
extern float tiltAngleX,tiltAngleY;
extern float distToWayPoint;
extern float controlBearing;
extern boolean telemFailSafe,txFailSafe,tuningTrasnmitOK;

extern boolean enterState;
extern float xFromTO,yFromTO;
extern int16_t floorLimit,ceilingLimit;

extern uint8_t modeArray[9];


extern float kp_pitch_rate;//52
extern float ki_pitch_rate;//56
extern float kd_pitch_rate;//60
extern float fc_pitch_rate;//64

extern float kp_roll_rate;//68
extern float ki_roll_rate;//72
extern float kd_roll_rate;//76
extern float fc_roll_rate;//80

extern float kp_yaw_rate;//84
extern float ki_yaw_rate;//88
extern float kd_yaw_rate;//92
extern float fc_yaw_rate;//96

extern float kp_pitch_attitude;//100
extern float ki_pitch_attitude;//104
extern float kd_pitch_attitude;//108
extern float fc_pitch_attitude;//112

extern float kp_roll_attitude;//116
extern float ki_roll_attitude;//120
extern float kd_roll_attitude;//124
extern float fc_roll_attitude;//128

extern float kp_yaw_attitude;//132
extern float ki_yaw_attitude;//136
extern float kd_yaw_attitude;//140
extern float fc_yaw_attitude;//144

extern float kp_altitude_position;//148
extern float ki_altitude_position;//152
extern float kd_altitude_position;//156
extern float fc_altitude_position;//160

extern float kp_altitude_velocity;//164
extern float ki_altitude_velocity;//168
extern float kd_altitude_velocity;//172
extern float fc_altitude_velocity;///176
//extern float mul_altitude_velocity;

extern float kp_loiter_pos_x;//180
extern float ki_loiter_pos_x;//184
extern float kd_loiter_pos_x;//188
extern float fc_loiter_pos_x;//192

extern float kp_loiter_velocity_x;//196
extern float ki_loiter_velocity_x;//200
extern float kd_loiter_velocity_x;//204
extern float fc_loiter_velocity_x;//208

extern float kp_loiter_pos_y;//212
extern float ki_loiter_pos_y;//216
extern float kd_loiter_pos_y;//220
extern float fc_loiter_pos_y;//224

extern float kp_loiter_velocity_y;//228
extern float ki_loiter_velocity_y;//232
extern float kd_loiter_velocity_y;//236
extern float fc_loiter_velocity_y;//240

extern float kp_waypoint_position;//244
extern float ki_waypoint_position;//248
extern float kd_waypoint_position;//252
extern float fc_waypoint_position;//256

extern float kp_waypoint_velocity;//260
extern float ki_waypoint_velocity;//264
extern float kd_waypoint_velocity;//268
extern float fc_waypoint_velocity;//272

extern float kp_cross_track;//276
extern float ki_cross_track;//280
extern float kd_cross_track;//284
extern float fc_cross_track;//288

extern float pitchSetPoint;
extern float rollSetPoint;
extern float yawSetPoint;
extern float rateSetPointX;
extern float rateSetPointY;
extern float rateSetPointZ;
extern float adjustmentX;
extern float adjustmentY;
extern float adjustmentZ;

extern float wpVelSetPoint,wpPathVelocity,wpCrossTrackVelocity,wpTilX,wpTiltY,headingToWayPoint;
extern float angleDiffOutput;

extern PID PitchRate;
extern PID RollRate;
extern PID YawRate;

extern PID PitchAngle;
extern PID RollAngle;
extern YAW YawAngle;

extern PID LoiterXPosition;
extern PID LoiterXVelocity;

extern PID LoiterYPosition;
extern PID LoiterYVelocity;

extern PID AltHoldPosition;
extern PID AltHoldVelocity;

extern PID WPPosition;
extern PID WPVelocity;

extern PID WPCrossTrack;


#endif
