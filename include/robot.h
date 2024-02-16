//externing to make sure that these electrical component declarations are defined (the names can't be used more than once) and will be used multiple times in multiple files
//extern: external linkage

#include "main.h"
#include "pros/adi.hpp"
#include "pros/motors.h"

#ifndef ROBOTH
#define ROBOTH

//controller
extern pros::Controller con;


extern pros::Rotation liftroto;
extern pros::Rotation cataroto;

//chassis motors
extern pros::Motor LF;
extern pros::Motor LB;
extern pros::Motor RF;
extern pros::Motor RB;
extern pros::Motor RM;
extern pros::Motor LM;

//intake
extern pros::Motor LIFT;
extern pros::Motor INTAKE;

//cata
extern pros::Motor CATA;
extern pros::Motor CATA2;

//sensors
extern pros::ADIEncoder encLeft;
extern pros::ADIEncoder encRight;
extern pros::ADIEncoder encMid;

//imu
extern pros::Imu imu;

//indexer
extern pros::Motor INDEXER;

//angler
extern pros::ADIDigitalOut wing1;
extern pros::ADIDigitalOut wing2;
//extern pros::ADIDigitalOut blocker;
extern pros::ADIDigitalOut awp;

extern pros::ADIDigitalOut liftp;
extern pros::ADIDigitalOut intakep;
extern pros::ADIDigitalOut rachet;




//gps
extern pros::Gps gps1;
extern pros::c::gps_status_s_t gpsData;

//expansion
extern pros::ADIDigitalOut expand;

//auton selector
extern pros::ADIDigitalIn selec;

//cata limit switch
extern pros::ADIDigitalIn catalim;

//extender
extern pros::ADIDigitalOut extender;

//optical
extern pros::Optical optical;
extern pros::Vision vision;


#endif