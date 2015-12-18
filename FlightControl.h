#ifndef FLIGHTCONTROL_H
#define FLIGHTCONTROL_H

#include "PID.h"
#include <Arduino.h>

void highRateTasks();
void _100HzTask(uint32_t);
void ProcessChannels();
void StartCalibration();
void Rotate2dVector(float*, float*, float*, float*, float*, float*);

void WayPointUpdate(float lat, float lon, float alt, float yaw);
void WayPointLookAt(float lat, float lon, boolean lookAt );
void WayPointStateMachine();

void WayPointStop();
void WayPointLandGS();
void WayPointReturnToBase();

void WayPointTakeOff();
void WayPointLandTX();

extern uint8_t flightMode;
extern uint32_t _100HzTimer,_400HzTimer;
extern volatile uint32_t RCFailSafeCounter,watchDogFailSafeCounter,groundFSCount,baroFSCount;
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
extern boolean telemFailSafe,txFailSafe,tuningTrasnmitOK,baroFS;

extern boolean enterState;
extern float xFromTO,yFromTO;
extern int16_t floorLimit,ceilingLimit;
extern boolean batteryFSOverride;

extern uint8_t modeArray[9];
extern float highRateDT,_100HzDt;

extern float bodyVelX,bodyVelY;

extern uint8_t initProgress;

extern float kp_pitch_rate;
extern float ki_pitch_rate;
extern float kd_pitch_rate;
extern float fc_pitch_rate;

extern float kp_roll_rate;
extern float ki_roll_rate;
extern float kd_roll_rate;
extern float fc_roll_rate;

extern float kp_yaw_rate;
extern float ki_yaw_rate;
extern float kd_yaw_rate;
extern float fc_yaw_rate;

extern float kp_pitch_attitude;
extern float ki_pitch_attitude;
extern float kd_pitch_attitude;
extern float fc_pitch_attitude;

extern float kp_roll_attitude;
extern float ki_roll_attitude;
extern float kd_roll_attitude;
extern float fc_roll_attitude;

extern float kp_yaw_attitude;
extern float ki_yaw_attitude;
extern float kd_yaw_attitude;
extern float fc_yaw_attitude;

extern float kp_altitude_position;
extern float ki_altitude_position;
extern float kd_altitude_position;
extern float fc_altitude_position;

extern float kp_altitude_velocity;
extern float ki_altitude_velocity;
extern float kd_altitude_velocity;
extern float fc_altitude_velocity;

extern float kp_loiter_pos_x;
extern float ki_loiter_pos_x;
extern float kd_loiter_pos_x;
extern float fc_loiter_pos_x;

extern float kp_loiter_velocity_x;
extern float ki_loiter_velocity_x;
extern float kd_loiter_velocity_x;
extern float fc_loiter_velocity_x;

extern float kp_loiter_pos_y;
extern float ki_loiter_pos_y;
extern float kd_loiter_pos_y;
extern float fc_loiter_pos_y;

extern float kp_loiter_velocity_y;
extern float ki_loiter_velocity_y;
extern float kd_loiter_velocity_y;
extern float fc_loiter_velocity_y;

extern float kp_waypoint_position;
extern float ki_waypoint_position;
extern float kd_waypoint_position;
extern float fc_waypoint_position;

extern float kp_waypoint_velocity;
extern float ki_waypoint_velocity;
extern float kd_waypoint_velocity;
extern float fc_waypoint_velocity;

extern float kp_cross_track;
extern float ki_cross_track;
extern float kd_cross_track;
extern float fc_cross_track;

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
extern float alhpaForPressure;
extern float debugVariable;

extern float baroErrorLim,countsOff,countsOn,landErrorLim;

extern PID PitchRate;
extern PID RollRate;
extern PID_2 YawRate;

extern PID_2 PitchAngle;
extern PID_2 RollAngle;
extern YAW_2 YawAngle;

extern PID_2 LoiterXPosition;
extern PID_2 LoiterXVelocity;

extern PID_2 LoiterYPosition;
extern PID_2 LoiterYVelocity;

extern PID_2 AltHoldPosition;
extern PID_2 AltHoldVelocity;

extern PID_2 WPPosition;
extern PID_2 WPVelocity;

extern PID_2 WPCrossTrack;


#endif

