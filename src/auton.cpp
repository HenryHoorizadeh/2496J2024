#include "api.h"
#include "auton.h"
#include "main.h"
#include "robot.h"
#include "pid.h"

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {
  //INDEX
    //AUTON 0: SKILLS
    //AUTON 1:
    //AUTON 2: 
    //AUTON 3:
    //AUTON 4: 
    //AUTON 5: 

//111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111111


  if (atn == 0) {
    INTAKE.move(-110);
    delay(350);
    int count = 0;
    CATA.move(127);
    while (count < 47){
      if (catalim.get_value() == true){
        count ++;
        delay(300);
      }
    }
    CATA.move(0);

    driveStraight(-290);
    driveTurn(-155);
    driveStraight(3200);
    driveTurn(-122);
    //break
    driveStraight(1000);
    driveTurn(-60);
    driveStraight(600);
    driveTurn(90);
    driveStraight(1600);
    driveTurn(90);
    wing1.set_value(true);
    driveStraight(1400);
    wing1.set_value(false);
    driveStraight(-1300);
    driveTurn(20);
    wing1.set_value(true);
    driveStraight(1400);
    wing1.set_value(false);
    driveStraight(-1000);
    driveTurn(57);
    driveStraight(2200);
    driveTurn(71);//78
    driveStraight(-1200);
    driveStraight(800);
    driveTurn(-10);
    driveStraight(-1200);
    driveStraight(800);
    //driveTurn(-122);
    // driveStraight(2200);
    // driveTurn(25);
    // driveStraight(700);
    // driveTurn(90);
    // wing1.set_value(true);
    // driveStraight(1400);
    // wing1.set_value(false);
    // driveStraight(-1300);
    // driveTurn(20);
    // wing1.set_value(true);
    // driveStraight(1400);
    // wing1.set_value(false);
    // driveStraight(00);
    
  } else if (atn == 1) {
   INTAKE.move(-127);
  }


//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222

  else if (atn == 2) {
   INTAKE.move(-127);
   driveStraight(-1400);
   driveTurn(-15);
   driveStraight(900);
   driveTurn(100);

  }

//3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

  
  else if (atn == 3) {
  
  INTAKE.move(-127);
  driveStraight(-1200);
  driveStraight(1150);
  awp.set_value(true);
  delay(350);
  driveTurn(-55);
  awp.set_value(false);
  driveStraight(1500);

  }

//44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444


  else if (atn == 4) {
    
  }

//55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555

  else if(atn == 6) {
   
  }

//////////////////////////////ARCHIVE/////////////////////////////////////////////////////////////////////////////////////////////////////////

}