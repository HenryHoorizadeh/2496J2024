 #include "api.h"
#include "main.h"
#include "okapi/api.hpp"
#include "pros/api_legacy.h"

//header guards?
#ifndef PIDH
#define PIDH

// extern void setConstants(float kp, float ki, float kd);
extern void resetEncoders();
// extern void chasMove(int voltageLF, int voltageLB, int voltageRF, int voltageRB);
// extern float calcPID(int target, float input, int integralKi, int maxI);

extern void driveStraight(int target);
extern void driveTurn(int target);
extern void driveAim(int target);
extern void driveSlow(int target);
extern void driveSmall(int target);
extern void driveShoot(int target);
extern void chasMove2(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRB, int voltageRM);
extern float error;
extern float viewvol;
extern double heading_error;

//tune straight constants here: setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
#define STRAIGHT_KP 1.09 // 
#define STRAIGHT_KI 0.01 // 
#define STRAIGHT_KD 8.15  // 

//tune straight integral-specific here: voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL);
#define STRAIGHT_INTEGRAL_KI 40
#define STRAIGHT_MAX_INTEGRAL 14.5


//tune turn constants here: setConstants(TURN_KP, TURN_KI, TURN_KD);
#define TURN_KP 7.325 //5.25//8.75
#define TURN_KI 0.025 //0.125//0.115
#define TURN_KD 70 //38 //105

//tune turn integral-specific here: voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL);
#define TURN_INTEGRAL_KI 30
#define TURN_MAX_INTEGRAL 25

#endif