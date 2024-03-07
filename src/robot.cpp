//constructors for everything electrical

#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"

//2496j
// #define OPTICAL_PORT 4
// #define IMU_PORT 18
// #define LF_PORT 6
// #define LM_PORT 4
// #define LB_PORT 8
// #define RF_PORT 5
// #define RM_PORT 16
// #define RB_PORT 10
// #define GPS_PORT 1
// #define CATA_PORT 20
// #define INTAKE_PORT 11



// 2496j
#define OPTICAL_PORT 4
#define IMU_PORT 13
#define LF_PORT 11
#define LM_PORT 17
#define LB_PORT 4
#define RF_PORT 15
#define RM_PORT 14
#define RB_PORT 20
#define GPS_PORT 21
#define CATA_PORT 7
#define LIFT_PORT 19
#define INTAKE_PORT 9
#define LIFTROTO_PORT 8
#define CATAROTO_PORT 6

pros::Rotation liftroto(LIFTROTO_PORT);
pros::Rotation cataroto(CATAROTO_PORT);

pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, true);

//intake
pros::Motor LIFT (LIFT_PORT, pros::E_MOTOR_GEARSET_18, false);

//cata
pros::Motor CATA (CATA_PORT, pros::E_MOTOR_GEARSET_36, false);

pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_06, false);

//angler for intake
pros::ADIDigitalOut wing1 ('E', false);
pros::ADIDigitalOut wing2 ('H', false);
pros::ADIDigitalOut awp ('C', false);
pros::ADIDigitalOut liftp ('A', true);
pros::ADIDigitalOut intakep ('D', false);
pros::ADIDigitalOut rachet ('B', false);

//cata limit switch
pros::ADIDigitalIn catalim ('I');

//extender
pros::ADIDigitalOut extender ('J');

//auton selector
pros::ADIDigitalIn selec ('G');

//sensors
pros::ADIEncoder encLeft ({{1, 1, 2}, false});
pros::ADIEncoder encRight ({{1, 3, 4}, false});
pros::ADIEncoder encMid ({{1, 5, 6}, false});

pros::Imu imu (IMU_PORT);

//controller
pros::Controller con (pros::E_CONTROLLER_MASTER);

//gps
#define X_OFFSET .225
#define Y_OFFSET .223
#define X_INITIAL 1.54
#define Y_INITIAL 1.14
#define HEADING_INITIAL 90
pros::Gps gps1(GPS_PORT, X_INITIAL, Y_INITIAL, HEADING_INITIAL, X_OFFSET, Y_OFFSET);
pros::c::gps_status_s_t gpsData;

//optical
pros::Optical optical(OPTICAL_PORT);


    pros::Vision vision (18);