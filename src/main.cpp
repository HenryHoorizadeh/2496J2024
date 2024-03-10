// basically run all of the code here (PID, auton, etc.)

#include "main.h"
#include "api.h"
#include "auton.h"
#include "pid.h"
#include "robot.h"

using namespace pros;
using namespace std;


// void resetEncoders() { //we can't add this to main.h because main.h doesn't
// refer to robot.h (where LF, LB, etc. are located) 	LF.tare_position(); //or
// set_zero_position(0) or set_zero_position(LF.get_position()); (sets current
// encoder position to 0) 	LB.tare_position(); 	RF.tare_position();
// 	RB.tare_position();
// }

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
  static bool pressed = false;
  pressed = !pressed;
  if (pressed) {
    pros::lcd::set_text(2, "I was pressed!");
  } else {
    pros::lcd::clear_line(2);
  }
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");

  // pros::lcd::register_btn1_cb(on_center_button);
  // optical.set_led_pwm(100);


}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */

int atn = 1;
string autstr;
 
void competition_initialize() {
    while(true) {
      if(selec.get_value() == true) {
        atn ++;  
        delay(350);
      }
      //resetEncoders();
      
      if (atn == 0) {
        autstr = "Skills";
        con.print(0, 0, "Aut 0: %s", autstr);
      }
      else if (atn == 1) {
        autstr = "NONE";
        con.print(0, 0, "Aut 1: %s", autstr);
      }
      else if (atn == 2) {
        autstr = "AWPON";
        con.print(0, 0, "Aut 2: %s", autstr);
      }
      else if (atn == 3) {
       autstr = "AWPOFF";
        con.print(0, 0, "Aut 3: %s", autstr);
      }
      else if (atn == 4) {
       autstr = "ML1";
        con.print(0, 0, "Aut 4: %s", autstr);
      }
      else if (atn == 5) {
       autstr = "DEFN1";
        con.print(0, 0, "Aut 5: %s", autstr);
      }else if (atn == 6) {
       autstr = "DISRUPT";
        con.print(0, 0, "Aut 6: %s", autstr);
      } else if (atn == 7) {
       atn = 0;
      }
  
      con.clear();
    }
}


/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

int cycle = 0;
int wait = 900;
int wait2 = 250;
int lift_count = 0;

int angle = 0;

bool rachetToggle = false;

bool wingsToggle = false;
bool wingsToggle2 = false;
bool awpToggle = false;
bool liftToggle = true;
bool liftToggle90 = true;
bool deployExpansion = false;
bool intakeToggle = true;
bool cataToggle = false;
bool cataToggle2 = false;
bool NEWL1 = false;
bool NEWL2 = false;
bool NEWR2 = false;
bool NEWR1 = false;


bool extenderToggle = false;
bool ccon = false;
bool slow = true;
bool printtog = true;
int speed = 0;
bool up;
bool down;
bool L1 = false;
bool R1 = false;

bool rollerOn = false;

void opcontrol() {
  int time = 0;
  bool arcToggle = true;
  bool tankToggle = false;
  bool flyToggle = true;
  
  //ONLY for Pressed/Primed CataControl
  bool cataPressed;
  bool cataPrimed;
  CATA.tare_position();
  imu.tare_heading();
  double mrpm = 0;

  double prevrpm = 0;

	while (true) {
    //printing stuff

		double chasstempC = ((RF.get_temperature() + RB.get_temperature() + LF.get_temperature() + LB.get_temperature())/4);
    double catat = (CATA.get_temperature());
    double catatempC = CATA.get_temperature();
    prevrpm = mrpm;
    if (prevrpm < LM.get_actual_velocity()){
    mrpm = LM.get_actual_velocity();
    }
		
    //if (time % 100 == 0) con.clear();
		
    //if (time % 2 == 0) {

      //con.clear();
      if(printtog){

			//cycle++;
      //int encoderAvg = (LF.get_position() + RF.get_position()) / 2;
      // if (cycle % 3 == 0) con.print(0, 0, "Aut: %s", ); //autstr //%s
      if (time % 50 == 0 && time % 100 != 0 && time != 150){
        con.print(0, 0, "ERROR: %s           ", autstr);
      } 
      if (time % 50 == 0 && time % 100 != 0){
        con.print(1, 0, "CataTemp: %f           ", float(imu.get_heading()));
      } 
      if (time % 50 == 0){
        setConstants(0.075, 0, 0.1);
        con.print(2, 0, "Temp: %f        ", float(CATA.get_temperature())); // //imu.get_heading() //mrpm
      } 



      // if ((cycle+1) % 6 == 0){
      //   con.clear_line(0);
      // } 
      // if ((cycle+2) % 6 == 0){
      //   con.print(0, 0, "ERROR: %s", autstr);  //autstr //%s
      // } 
      // if ((cycle+3) % 6 == 0){
      //    con.clear_line(1);
      // }
      // if ((cycle+4) % 6 == 0){
      //    con.print(1, 0, "CataTemp: %f", float(error)); //autstr //%s float(imu.get_heading()) //catat
      // }
		  // if ((cycle+5) % 6 == 0){
      //    con.clear_line(2);
      // } 
      // if ((cycle+6) % 6 == 0){
      //    con.print(2, 0, "Temp: %f", float(time2));  //con.print(2, 0, "Temp: %f", chasstempC);
      // }
	  }
    //}
     

		//chassis arcade drive
		int power = con.get_analog(ANALOG_LEFT_Y); //power is defined as forward or backward
		int RX = con.get_analog(ANALOG_RIGHT_X); //turn is defined as left (positive) or right (negative)


		//int turn = int(abs(RX) * RX / 75);
    
		int turn = int(RX);
		int left = power + turn;
		int right = power - turn;

    // //switch between arcade and tank
    // if (con.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_Y)) {

    //   // arcToggle = !arcToggle;
    //   // tankToggle = !tankToggle;
    // }

   
    if (tankToggle) {
      LF.move(con.get_analog(ANALOG_LEFT_Y));
      LM.move(con.get_analog(ANALOG_LEFT_Y));
      LB.move(con.get_analog(ANALOG_LEFT_Y));
      RF.move(con.get_analog(ANALOG_RIGHT_Y));
      RM.move(con.get_analog(ANALOG_RIGHT_Y));
      RB.move(con.get_analog(ANALOG_RIGHT_Y));
    }
    if (arcToggle) {
      LF.move(left);
      LM.move(left);
      LB.move(left);
      RF.move(right);
      RM.move(right);
      RB.move(right);
    }

    //auton selector
    if (selec.get_value() == true) { // brain was here
      atn++;
     // delay(350);
    }
    
    if (atn == 0) {
      autstr = "Skills";
    }
    if (atn == 1) {
      autstr = "NONE";
    }
    else if (atn == 2) {
      autstr = "AWPON";
    }
    else if (atn == 3) {
      autstr = "AWPOFF";
    }
    else if (atn == 4) {
      autstr = "ML1";
    }
    else if (atn == 5) {
      autstr = "DEFN1";
    } else if (atn == 6) {
      autstr = "DISRUPT";
    }
    else if (atn == 7) {
      atn = 0;
    }

  if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R1)){
    NEWR1 = true;
  } else {
    NEWR1 = false;
  }

  if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_R2)){
    NEWR2 = true;
  } else {
    NEWR2 = false;
  }

  if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L2)){
    NEWL2 = true;
  } else {
    NEWL2 = false;
  }

  
  if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
    NEWL1 = true;
  } else {
    NEWL1 = false;
  }



    if (((con.get_digital(E_CONTROLLER_DIGITAL_R1) && NEWR2) || (NEWR1 && con.get_digital(E_CONTROLLER_DIGITAL_R2))) || ((NEWR1 && NEWR2) || (con.get_digital(E_CONTROLLER_DIGITAL_R1) && con.get_digital(E_CONTROLLER_DIGITAL_R2)))){
     LIFT.move(-127);
     liftToggle = true;
     liftToggle90 = true;
     INTAKE.move(0);
     down = true;
     R1 = false;
    } else if  (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
			INTAKE.move(-127);
      down = false;
      R1 = true;
		} else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
			INTAKE.move(127);
      down = false;
      R1 = false;
		} else {
			INTAKE.move(0);
      down = false;
      R1 = false;
		}

    // if  (con.get_digital(E_CONTROLLER_DIGITAL_R1)) {
		// 	INTAKE.move(-127);
		// }
		// else if (con.get_digital(E_CONTROLLER_DIGITAL_R2)) {
		// 	INTAKE.move(127);
		// }
		// else {
		// 	INTAKE.move(0);
		// }




//cata roto sensor 
if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_UP)){
    ccon = !ccon;
  }  
if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_B)){
    intakeToggle = !intakeToggle;
  } 
   
// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)){
//     slow = !slow;
//   }

// if (slow){
//   speed = 127;
// } else{
//   speed = 110;
// }
CATA.set_brake_mode(MOTOR_BRAKE_COAST);

  // if(ccon){

  // } else {
  //   if(con.get_digital(E_CONTROLLER_DIGITAL_L2) ||  catalim.get_value() == false){
  //    // CATA.move(127);
  //    CATA.move_velocity(600);
  //   } else {
  //     CATA.move(0);
  //   }
  // }

  

    //Simple single button 
  // if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)){
  //   slow = !slow;
  // }

  // if(CATA.get_position() > 360){
  //   CATA.tare_position();
  // }

///CONTROL1

  // if(slow){
  //   // if(((CATA.get_position() < 330) && (CATA.get_position() > 30)) || con.get_digital(E_CONTROLLER_DIGITAL_L2)){
  //   //   if ((CATA.get_position() < 200) && (CATA.get_position() > 30)){
  //   //     CATA.move(70);
  //   //   } else {
  //   //     CATA.move(127);
  //   //   }
  //   // } else {
  //   //   CATA.move(0);
  //   // }
  // } else {
  //   float pos = CATA.get_position();
  //   if(((CATA.get_position() < 320) && (CATA.get_position() > 30)) || con.get_digital(E_CONTROLLER_DIGITAL_L2)){
  //     if ((CATA.get_position() < 200) && (CATA.get_position() > 30)){
  //       CATA.move(127);
  //     } else {
  //       CATA.move(127);
  //     }
  //   } else {
  //     CATA.move(0);
  //   }
  // }
 
///CONTROL2
// if(slow){
//     if(CATA.get_position() < 330 || con.get_digital(E_CONTROLLER_DIGITAL_L2)){
//       CATA.move(127);
//     } else {
//       CATA.move(0);
//     }
//   } else {
//     if(CATA.get_position() < 330 || con.get_digital(E_CONTROLLER_DIGITAL_L2)){
//       CATA.move(60);
//     } else {
//       CATA.move(0);
//     }
//   }


//old lim control
// if(catalim.get_value() == false || con.get_digital(E_CONTROLLER_DIGITAL_L2)){
//       CATA.move(127);
//     } else {
//       CATA.move(0);
//     }
//   } else

//hi
  ////CATA Primed/Pressed Control
  //   cataPrimed = catalim.get_value();
  //   if(con.get_digital(E_CONTROLLER_DIGITAL_DOWN)){
  //     CATA.move(-127);
  //   } else {
  //     // printf("Catalim %i", cataPrimed);
  //     if (con.get_digital(E_CONTROLLER_DIGITAL_L1)) {
  //       cataPressed = true;
  //     }
  //     if (cataPrimed == false){
  //       CATA.move(-127);
  //       cataPressed = false;
  //     }
  //     else if (cataPressed == true && cataPrimed == true) {
  //       CATA.move(-127);
  //     }
  //     else {
  //       CATA.move(0);
  //     }
  // }

    
    //pid tester
		if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_X)) {
// driveTurn2(-10);

// driveArcR(90, 550, 2500);


      // driveTurn2(-55);
      temp_lift = true;
      driveStraight2(2000);
      // driveStraightC(1000);
      // driveArcLF(180, 500, 10000);
      // // driveArcL(150, 500, 10000);
      // //driveArcLF(150, 500, 10000);
      // //driveStraightC(500);
      //  driveArcRF(90, 400, 10000);
      //  driveStraightC(1000);
     //driveTurn2(90);
      // driveTurn2(170);
      // driveTurn2(-170);

     //driveTurn2(113);
     //driveTurn2(25);
    // driveTurn2(-153);

// driveStraight(150);
 // driveTurn(-90); //70
  //driveTurn2(54);
  //driveTurn2(180);
  // driveTurn2(-40);
  // driveTurn2(180);
  //driveTurn(30);

    }

    //Wings
    if (((con.get_digital(E_CONTROLLER_DIGITAL_L1) && NEWL2) || (NEWL1 && con.get_digital(E_CONTROLLER_DIGITAL_L2))) || ((NEWL1 && NEWL2) || (con.get_digital(E_CONTROLLER_DIGITAL_L1) && con.get_digital(E_CONTROLLER_DIGITAL_L2)))){
     LIFT.move(127);
     liftToggle = true;
     liftToggle90 = true;
     up = true;
    } else if(NEWL1){
      L1 = true;
      //CATA.move(127);
      // if(wingsToggle == true){
      //   wingsToggle = false;
      //   speed = 70;
      // } else {
      //   wingsToggle = true;
      //   speed = 68;
      // }
      // } else {
      //   wingsToggle = true;
      // }
      /////////////////////////////////////////////////////
      wingsToggle2 = !wingsToggle2;
      wingsToggle = false;
      speed = 69;
      ///////////////////////////////////////////////////////////////////
      //CATA.move_velocity(480);
       up = false;
    } 
    //else if (NEWL2){
    // wingsToggle2 = !wingsToggle2;
    // wingsToggle = false;
    // } 
    else if (con.get_digital(E_CONTROLLER_DIGITAL_L2)) {
			//
      up = false;
      cataToggle = true;
      
		} else {
      cataToggle = false;
      //CATA.move(0);
       up = false;
    }

if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)){
  cataToggle2 = !cataToggle2;
}
 
 if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_Y)){
  wingsToggle = !wingsToggle;
 }
//speed = 0;


if(ccon){
  if((cataToggle2 || cataToggle) || ((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14500))){ //14000
    //CATA.move(127);
    CATA.move_velocity(100);//50 ///57.5
  } else {
    CATA.move(0);

  }
} else {
    if(cataToggle2 || cataToggle){
    //CATA.move(127);
    CATA.move_velocity(85); //67
  } else {
     CATA.move(0);
    //CATA.move_velocity(0);

  }
}

    //wingsToggle = !wingsToggle;
    // if(con.get_digital_new_press(E_CONTROLLER_DIGITAL_L1)){
    //   wingsToggle = !wingsToggle;
    // }

    
  if (wingsToggle2 == true || wingsToggle == true){
      wing1.set_value(true); 
  } else {
    wing1.set_value(false);
  }

  if (wingsToggle2 == true && wingsToggle == false){
    wing2.set_value(true); 
  } else {
    wing2.set_value(false);
  }
  

    // if (wingsToggle2 == false) {
		// 		wing1.set_value(false);
    //     wing2.set_value(false);
		// 	} else {
    //     wing1.set_value(true);
    //     wing2.set_value(true);
		// 	}

    //   if (wingsToggle == false) {
		// 		wing1.set_value(false);
    //     // wing2.set_value(false);
		// 	} else {
    //     wing1.set_value(true);
    //     // wing2.set_value(true);
		// 	}




    //   if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
		// 	rachetToggle = !rachetToggle;
		// }
    LIFT.set_brake_mode(pros::E_MOTOR_BRAKE_COAST);
    // if (rachetToggle == false) {
		// 		rachet.set_value(false); 
		// 	} else {
    //     rachet.set_value(true);
		// 	}
if (intakeToggle == false) {
				intakep.set_value(false);
			} else {
        intakep.set_value(true);
			}

      if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_RIGHT)) {
			rachetToggle = !rachetToggle;
		}
    if (rachetToggle == false) {
				rachet.set_value(false);
			} else {
        rachet.set_value(true);
    LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    RF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    RM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    RB.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LIFT.set_brake_mode(E_MOTOR_BRAKE_COAST);
    liftToggle = true;
     liftToggle90 = true;
			}
   
    // if (flyToggle == false) {
		// 		CATA.move(0);
		// 	} else {
    //     CATA.move(127);
		// 	}
   
  //  if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
	// 		awpToggle = !awpToggle;
  //    wingsToggle = !wingsToggle;
	// 	}
    if (awpToggle == false) {
				awp.set_value(false);
			} else {
        awp.set_value(true);
			}
   
    //   if (con.get_digital(E_CONTROLLER_DIGITAL_UP)) {
    //     LIFT.move(-127);
		// } else if (con.get_digital(E_CONTROLLER_DIGITAL_DOWN)){
    //     LIFT.move(127);
    // } else {
    //     LIFT.move(0);
    // }
    
    if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_LEFT)) {
			liftToggle = !liftToggle;
      liftToggle90 = true;
		 } else if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_A)) {
			liftToggle90 = !liftToggle90;
      liftToggle = true;
		}

    angle = liftroto.get_angle();

    if (angle > 30000){
      angle = angle-36000;
    }


    if (liftToggle == false) {
      //liftToggle90 == true;
      setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID(15400, angle, 40, 140, false)); //15000
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //liftToggle = true;
        lift_count = 0;
      }

      

      // if((angle < 150 || angle > 300)){
      //   LIFT.move(127);
      // } else if(angle > 152){
      //   LIFT.move(-127);
      // } else {
      //   LIFT.set_brake_mode(E_MOTOR_BRAKE_BRAKE);
      //   liftToggle = true;
      // }

				//liftp.set_value(false);
			} else if (liftToggle90 == false) {
      setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID(4500, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //liftToggle = true;
        lift_count = 0;
      }} else {

      if (up == false && down == false){
      LIFT.move(0);
    }

        //liftp.set_value(true);
			}

    //reset all motor encoders
		// if (con.get_digital_new_press(E_CONTROLLER_DIGITAL_DOWN)) {
		// 	//resetEncoders();
    //   printtog = !printtog;
		// }
	  	time += 1;
		  pros::delay(1);
	  }
  }