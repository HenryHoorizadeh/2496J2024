//constructors for everything electrical

#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pros/motors.h"


#define OPTICAL_PORT 4
#define IMU_PORT 18
#define LF_PORT 6
#define LM_PORT 4
#define LB_PORT 8
#define RF_PORT 5
#define RM_PORT 16
#define RB_PORT 10
#define GPS_PORT 1
#define CATA_PORT 20
#define INTAKE_PORT 11


pros::Motor LF (LF_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LM (LM_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor LB (LB_PORT, pros::E_MOTOR_GEARSET_06, true);
pros::Motor RF (RF_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RM (RM_PORT, pros::E_MOTOR_GEARSET_06, false);
pros::Motor RB (RB_PORT, pros::E_MOTOR_GEARSET_06, false);

//intake
pros::Motor INTAKE (INTAKE_PORT, pros::E_MOTOR_GEARSET_18, false);

//cata
pros::Motor CATA (CATA_PORT, pros::E_MOTOR_GEARSET_06, false);

//angler for intake
pros::ADIDigitalOut wing1 ('B', false);
pros::ADIDigitalOut wing2 ('C', false);
pros::ADIDigitalOut awp ('A', false);
pros::ADIDigitalOut blocker ('D', false);

//cata limit switch
pros::ADIDigitalIn catalim ('H');

//extender
pros::ADIDigitalOut extender ('C');

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