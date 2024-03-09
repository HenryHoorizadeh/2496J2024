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


  void liftU_fn(void* param) {
    int lift_count = 0;
    bool angle = 0;
    //std::uint32_t now = pros::millis();
    while(true){
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    //setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID(4500, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        break;
        //liftToggle = true;
        lift_count = 0;
      }
      //delay(500);
      //pros::Task::delay_until(&now, 50);

    }
 }

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
tempre = false;
    //startnew
    // INTAKE.move(120);
    // RF.move(-10);
    // RM.move(-10);
    // RB.move(-10);
    LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LB.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    intakep.set_value(true);
    LIFT.move(-127);
    delay(550); //350

    int count = 0;
    int time = 0;


    int time3 = 0;

    LIFT.move(0);
    cataroto.reset_position();
    
    ///////////////////////////////////////////////////////////////////
    /*
    CATA.move_velocity(75); //67
     while ((time3 < 27000) && (-cataroto.get_position() < 828000)){
        con.print(1, 0, "Roto: %f           ", float(cataroto.get_position()));
        delay(10);
        time3 += 10;
    }
    */
    ///////////////////////////////////////////////////////////////////
    CATA.move(0);

time3 = 0;

while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time3 < 1000){
     CATA.move_velocity(45);
     delay(1);
     time3 += 1;
    }
    CATA.move(0);



tempre = false;
imu.tare();
intakep.set_value(true);
driveStraight2(100);





driveTurn2(28); //30
     
driveStraightC(3700);
temp_lift = true;
lift_target = 4500;
//pros::Task liftU(liftU_fn);
wing1.set_value(true);
wing2.set_value(true);


driveArcLF(30, 1200, 1700); //500

driveArcLF(55, 1250, 1200);
wing1.set_value(false);
wing2.set_value(false);


driveStraightC(600);//first push on goal right
driveStraight2(-600);
driveTurn2(-67); //-62
driveStraight2(1800);
tempre = false;
/*
time = 0;
int lift_count = 0;
double angle = 0;


while(true){
setConstants(0.09, 0, 0.2); //0.075
angle = liftroto.get_angle();


    if (angle > 30000){
      angle = angle-36000;
    }
      LIFT.move(calcPID(6000, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 900){ //1000
        lift_count ++;
      }
      delay(10);
      time += 10;
      if ((lift_count > 400) || time > 750){
        LIFT.move(0);
        time += 10;
        break;
        lift_count = 0;
      }
}*/


//intakep.set_value(false);
driveStraightC(-200);






driveTurn2(-150);
wing1.set_value(true);
driveStraightC(400); //700

// wing2.set_value(false);

/*split arc
driveArcRF(110, 200, 1700);//make sure corner ball are near goal
wing1.set_value(false);
driveStraightC(200);
driveArcRF(70, 200, 1200);
*/
temp_lift = true;
lift_target = 4500;

driveArcRF(190, 200, 1800);
driveTurn2(28); 
driveStraight(800);

wing1.set_value(false);
driveStraightC(-200);


//driveArcRF(-220, 200, 1000);
//driveStraightC(-1100);
driveArcR(-180, 200, 1800); //-110 //1300 //1100 
//intakep.set_value(true);


driveStraightC(425); //400
driveArcRF(160, 230, 2500); //180
driveStraight2(2000);
driveTurn2(0);
driveStraightC(-1000);//-2000
//driveTurn2(-60);
driveArcR(-75, 200, 750); //150//1000 //800
driveTurn2(-62);
wing2.set_value(true);
driveStraightC(350);//600




driveArcRF(90, 300, 1000);
driveStraightC(1500);
driveTurn2(0);
wing2.set_value(false);
driveStraightC(-1000);
driveArcR(-75, 200, 750); //1000 ///800
driveTurn2(-62);
//driveTurn2(-60);
wing2.set_value(true);
driveStraightC(550); //700




driveArcRF(100, 230, 1000);
driveStraightC(1500);
driveTurn2(28);
wing2.set_value(false);
driveStraightC(-1000);
driveArcR(-90, 200, 800); //1000 //800
driveTurn2(-62);
driveStraightC(900);
wing1.set_value(true);
wing2.set_value(true);
driveArcRF(165, 100, 2000);
// wing1.set_value(true);
// wing2.set_value(true);
driveStraightC(400);
driveArcLF(75, 350, 2000); //250
driveStraight2(300);
wing1.set_value(false);
// wing2.set_value(false);
driveStraight2(-400);
driveTurn2(28);
driveStraight2(700);
wing2.set_value(false);
driveTurn2(28);
driveStraightC(-700); ///-500


driveArcR(-68, 180, 700); //-80 //-60 //-65 //1000
// wing2.set_value(true);
// wing1.set_value(true);
driveTurn2(-40); //-37
driveStraightC(1300);
driveArcLF(22, 110, 800);
driveStraight2(1200);
driveTurn2(28);

//final side push
wing1.set_value(true);
wing2.set_value(true);
driveArcRF(30, 1200, 1700); //500
driveArcRF(55, 1250, 1200);
wing1.set_value(false);
wing2.set_value(false);
driveStraightC(600);//first push on goal right
driveStraight2(-600);
driveTurn2(118); //-62
driveStraight2(1800);







  
  } else if (atn == 1) {
/*
while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
     CATA.move_velocity(45);
     delay(1);
     time += 1;
    }
    CATA.move(0);
tempre = false;
*/
tempre = false;
    //startnew
    // INTAKE.move(120);
    // RF.move(-10);
    // RM.move(-10);
    // RB.move(-10);
    LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
    LB.set_brake_mode(E_MOTOR_BRAKE_HOLD);

    intakep.set_value(true);
    LIFT.move(-127);
    delay(550); //350

    int count = 0;
    int time = 0;


    int time3 = 0;

    LIFT.move(0);
    cataroto.reset_position();
    
    ///////////////////////////////////////////////////////////////////
    
    CATA.move_velocity(75); //67
     while ((time3 < 27000) && (-cataroto.get_position() < 828000)){
        con.print(1, 0, "Roto: %f           ", float(cataroto.get_position()));
        delay(10);
        time3 += 10;
    }
    
    ///////////////////////////////////////////////////////////////////
    CATA.move(0);

time3 = 0;


tempre = false;
imu.tare();
intakep.set_value(true);
driveStraight2(100);
imu.tare();




driveTurn2(28); //30
     
driveStraightC(3500);
temp_lift = true;
lift_target = 4500;
//pros::Task liftU(liftU_fn);
wing1.set_value(true);
wing2.set_value(true);


driveArcLF(30, 1200, 1700); //500

driveArcLF(55, 1250, 1200);
wing1.set_value(false);
wing2.set_value(false);


driveStraightC(600);//first push on goal right
driveStraight2(-500);//s2
driveTurn2(-56); //-62
driveStraight2(1000);
tempre = false;
/*
time = 0;
int lift_count = 0;
double angle = 0;


while(true){
setConstants(0.09, 0, 0.2); //0.075
angle = liftroto.get_angle();


    if (angle > 30000){
      angle = angle-36000;
    }
      LIFT.move(calcPID(6000, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 900){ //1000
        lift_count ++;
      }
      delay(10);
      time += 10;
      if ((lift_count > 400) || time > 750){
        LIFT.move(0);
        time += 10;
        break;
        lift_count = 0;
      }
}*/

temp_lift = true;
lift_target = 4500;

//intakep.set_value(false);
driveStraightC(-200);






driveTurn2(-150);
wing1.set_value(true);
driveStraightC(400); //700

// wing2.set_value(false);
driveArcRF(110, 200, 1700);//make sure corner ball are near goal
 wing1.set_value(false);
driveStraightC(200);
driveTurn2(-30);


//driveArcRF(-220, 200, 1000);
//driveStraightC(-1100);
driveArcR(-110, 200, 1000); //-110 //1300
//intakep.set_value(true);


driveStraightC(425); //400
driveArcRF(160, 230, 2500); //180
driveStraight2(2000);
driveTurn2(0);
driveStraightC(-1000);//-2000
//driveTurn2(-60);
driveArcR(-75, 200, 800); //150 //1000
driveTurn2(-62);
wing2.set_value(true);
driveStraightC(350);//600




driveArcRF(90, 300, 1000);
driveStraightC(1500);
driveTurn2(0);
wing2.set_value(false);
driveStraightC(-1000);
driveArcR(-75, 200, 700); //1000
driveTurn2(-62);
//driveTurn2(-60);
wing2.set_value(true);
driveStraightC(550); //700




driveArcRF(100, 230, 1000);
driveStraightC(1500);
driveTurn2(28);
wing2.set_value(false);
driveStraightC(-1000);
driveArcR(-90, 200, 900); //1000
driveTurn2(-62);
driveStraightC(900);
wing1.set_value(true);
wing2.set_value(true);
driveArcRF(165, 100, 2000);
// wing1.set_value(true);
// wing2.set_value(true);
driveStraightC(400);
driveArcLF(75, 350, 2000); //250
driveStraight2(300);
wing1.set_value(false);
// wing2.set_value(false);
driveStraight2(-400);
driveTurn2(28);
driveStraight2(700);
wing2.set_value(false);




driveTurn2(28);
driveStraightC(-500);
driveArcR(-68, 180, 800); //-80 //-60 //-65 //1000
wing2.set_value(true);
driveTurn2(-40); //-37
driveStraightC(1300);
//wing1.set_value(true);
//wing2.set_value(true);

driveArcRF(160, 525, 1700);  //280 //480
wing1.set_value(false);
wing1.set_value(false);
driveTurn2(110); //118
driveStraight2(600); //700
driveStraight2(-400); //-500
driveStraight2(900); 
driveStraight2(-500);



  

  
  }


//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222

  else if (atn == 2) { //3ball
    intakep.set_value(true);
  INTAKE.move(127);
  wing2.set_value(true);
  delay(150);
  driveStraightC(500);
  wing2.set_value(false);
  driveStraight2(2100);
  driveStraightC(-2200); //-2400
  INTAKE.move(-127);
  driveTurn2(90);
  driveTurn2(-75);
  INTAKE.move(127);
  LIFT.move(-127);
  driveStraight2(1700);
  LIFT.move(0);
  driveStraight2(-1500); //-1700
  driveTurn2(97);

  wing1.set_value(true);
  wing2.set_value(true);
  driveArcLF(30, 1200, 1700); //500
  INTAKE.move(-127);
  driveArcLF(45, 1250, 1000);
  wing1.set_value(false);
  wing2.set_value(false);
  driveStraightC(1500);
  driveStraightC(-800);
  driveTurn2(15);
  driveStraightC(1500);
  driveStraight2(-800);
  driveTurn2(-45);
  INTAKE.move(127);
  // driveStraight2(1800);
  // driveStraightC(-700);
  // //Cut
  // driveArcLF(90, 500, 1500);
  // driveStraight2(-1000);
  


  }

//3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

  
  else if (atn == 3) { //rush
  imu.tare();
  intakep.set_value(true);
  INTAKE.move(127);
  driveStraight2(2300);
  driveStraight2(-250);
  driveTurn(75);
  wing1.set_value(true);
  driveStraight2(1200);
  wing1.set_value(false);
  driveStraight2(-400);
  INTAKE.move(-127);
  driveStraight2(1000);
  driveStraight2(-800);
    
  //   tempre = false;
  //  awp.set_value(true);
  // delay(500);
//   driveStraight2(-2500);
//   driveStraight2(1000);
//  // driveTurn2(170);
//   //awp.set_value(false);
//   // driveTurn2(45);
//   // //intakep.set_value(true);
//   // driveStraight2(-1500);
//   // driveStraight2(1400);
//   // driveTurn2(0);
//   // driveStraight2(800);
//   // driveTurn2(-45);
//   // driveStraight2(1550);
//   // intakep.set_value(true);
    
  
  }

//44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444


  else if (atn == 4) { //out of corner
     tempre = false;
     driveStraight2(100);
   LIFT.move(127);
   delay(1500);
  // LIFT.move(-127);
  //  delay(150);
   LIFT.move(0);
   INTAKE.move(-127);
   driveTurn2(-100);
   LIFT.move(-127);
   driveTurn2(60);
   LIFT.move(0);
   driveStraight2(1000);
   driveTurn2(30);
   intakep.set_value(true);
   driveStraight2(1000);

  }

//55555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555555

  else if(atn == 5) {
    tempre = false;
imu.tare_heading();
  intakep.set_value(true);
  awp.set_value(true);
  driveStraight2(600);

  driveTurn2(-120);
  awp.set_value(false);
  driveTurn2(-30); //-40
  intakep.set_value(false);
  driveStraight2(2000);
  driveStraight2(-700);
  intakep.set_value(true);
  driveTurn2(-120); //-145
  driveStraight2(2350); //1200
  // driveTurn2(-85);
  // driveStraight2(900);
  driveTurn2(25);
  intakep.set_value(false);
  driveStraight2(500);
  intakep.set_value(true);
  driveStraight2(-1000);
  driveTurn2(-50);
  driveStraight2(1100); //900

  driveTurn2(50);
  wing1.set_value(true);
  driveStraight2(2100);
  wing1.set_value(false);
  driveStraight2(-1000);
  tempre = true;



  

  } else if (atn == 6){
    LF.move(10);
    LM.move(10);
    LB.move(10);

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
    imu.tare();
    while(catalim.get_value() == false){
     CATA.move(127); 
    }
    CATA.move(0);
    
  }
  
  else if (atn == 6) {
    imu.tare_heading();
   driveStraight2(3500);
   driveTurn(361);

  }


//////////////////////////////ARCHIVE/////////////////////////////////////////////////////////////////////////////////////////////////////////
// tempre = false;
//     //startnew
//     // INTAKE.move(120);
//     // RF.move(-10);
//     // RM.move(-10);
//     // RB.move(-10);
//     LF.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//     LM.set_brake_mode(E_MOTOR_BRAKE_HOLD);
//     LB.set_brake_mode(E_MOTOR_BRAKE_HOLD);

//     intakep.set_value(true);
//     LIFT.move(-127);
//     delay(350);

//     int count = 0;
//     int time = 0;

//     // while (count < 30000){
//     //   delay(1);
//     //   CATA.move(127);
//     //   count ++;
//     // }
    
//     // while ((count < 47) && (time < 30000)){
//     //   if ((catalim.get_value() == true)){
//     //     count ++;
//     //   }
//     //     delay(300);
//     //     time += 300;
//     // }
//     LIFT.move(0);
//     cataroto.reset_position();
//     ///////////////////////////////////////////////////////////////////
//     CATA.move_velocity(67); //
//      while ((time < 27000) && (cataroto.get_position() < 828000)){
//       // if (!((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000))){
//       //   count ++;
//       // }
//         delay(10);
//         time += 10;
//     }
//     ///////////////////////////////////////////////////////////////////
//     CATA.move(0);
//     //imu.tare();
//     // while(catalim.get_value() == false){
//     //  CATA.move(127); 
//     // }
//     CATA.move(0);
//     LF.set_brake_mode(E_MOTOR_BRAKE_COAST);
//     LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
//     LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
//     // CATA.move(127);
//    // delay(1000);
//  driveStraight2(100);
// while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
//      CATA.move_velocity(45);
//      delay(1);
//      time += 1;
//     }
//     CATA.move(0);
// tempre = false;
// // driveTurn(90); 
// // driveTurn(90);  //-100
// //     resetEncoders();
//      imu.tare();
// driveTurn2(30);//27 //30 //27 //30 /30
// //imu.tare();
// driveStraight2(4100); //3850

// while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
//      CATA.move_velocity(40);
//      delay(1);
//      time += 1;
//     }
//     CATA.move(0);
// tempre = false;




// time = 0;
// int lift_count = 0;
// double angle = 0;





// while(true){
// setConstants(0.09, 0, 0.2); //0.075
// angle = liftroto.get_angle();

//     if (angle > 30000){
//       angle = angle-36000;
//     }
//       LIFT.move(calcPID(6000, angle, 40, 140, false));
//       if (abs(liftroto.get_angle() - 15000) < 900){ //1000
//         lift_count ++;
//       }
//       delay(10);
//       time += 10;
//       if ((lift_count > 400) || time > 750){
//         LIFT.move(0);
//         time += 10;
//         break;
//         lift_count = 0;
//       }
// }
// intakep.set_value(false);
// // driveTurn2(-20);
// // driveStraight2(-1200); //800
// // driveTurn2(-50);
// // driveStraight2(-1601);
// // driveStraight2(800);
// // driveTurn2(-45);
// // driveStraight2(-1601);
// // //xtra
// // driveStraight2(800);
// // driveTurn2(-50);
// // driveStraight2(-1601);
// // //xtra
// // driveStraight2(200);
// // driveTurn2(25);
// // driveStraight2(2000);
// // driveTurn2(110);
// // driveStraight2(800); //1300 //2000 //1300
// // driveTurn2(-170);//-153
// // wing1.set_value(true);
// // delay(500);
// // driveStraight2(2101);
// // wing1.set_value(false);
// // driveStraight2(-1500);
// // driveTurn2(110); //-65 //115
// // driveStraight2(1200);//1000
// // driveTurn2(-145); //-142 //-155 //-167
// // wing1.set_value(true);
// // delay(500);
// // driveStraight2(2101);
// // wing1.set_value(false);
// // driveStraight2(-1500);
// // //xtra
// // driveTurn2(110); //-65 //115
// // driveStraight2(450);//1000
// // driveTurn2(-120);
// // driveStraight2(2101);
// // driveStraight2(-1500);
// // //xtra
// // driveTurn2(-20);
// // driveStraight2(-1500);
// // driveTurn2(-63);

// //endnew

// ///old v2
// driveTurn2(-15);
// wing1.set_value(true);//Gerald was here
// driveStraight2(1200); //1400
// wing1.set_value(false); //800
// driveTurn2(-60);
// driveStraight2(1601);
// ////xtrra2start
// // driveStraight2(800);
// // driveTurn2(-30); //-45
// // driveStraight2(-1601);
// // driveTurn2(-50);
// ////xxtra2sart
// //xtra
// driveStraight2(-700);//-800
// driveTurn2(-55);
// driveStraight2(1601);
// //xtra
// driveStraight2(-700); //-800
// driveTurn2(-140); //-150
// driveStraight2(2000);//2000 //1650 //1850
// //old side///////////////////////
// // driveTurn2(-60);
// // driveStraight2(300); //1300 //2000 //2500
// // driveTurn2(-35); //-153 //-125 //-35
// // wing1.set_value(true);
// // driveStraight2(700); //700
// // wing1.set_value(false);//robot.disconnect();
// // driveTurn2(30);
// // driveStraight2(1500);
// // old side////////////////////////////////////
// driveTurn2(-60);
// driveStraight2(500); //1300 //2000 //2500
// driveTurn2(20);
// driveStraight2(2101);
// driveStraight2(-1500);
// driveTurn2(-60); //-65

// driveStraight2(800);//1000 //1400 //700
// driveTurn2(30); //-142 //-155 //
// wing1.set_value(true);
// delay(350);
// driveStraight2(2101);
// wing1.set_value(false);

// driveStraight2(-1500);
// driveTurn2(-60); //-65
// //wing1.set_value(true); 
// driveStraight2(1600); //2000
// //wing1.set_value(false);//1000 //1400 //1100
// //driveTurn2(-60);
// //driveStraight2(800);
// driveTurn2(75);
// wing1.set_value(true);
// driveStraight2(1200);
// driveTurn2(30);
// driveStraight2(2101);
// wing1.set_value(false);
// driveStraight2(-1500);
// wing1.set_value(true);
// delay(550);
// driveTurn2(40);
// driveStraight2(2101);
// wing1.set_value(false);
// driveStraight2(-1500);
// // driveStraight2(2101);
// // driveStraight2(-1500);

// //old last push///////////////
// // driveTurn2(-60); //-65
// // driveStraight2(700);//1000 //1400 //1100
// // driveTurn2(60);
// // wing1.set_value(true);
// // driveStraight2(1200);
// // wing1.set_value(false);
// // driveTurn2(30);
// // driveStraight2(2101);
// // driveStraight2(-1500);
// //old last push///////////////////

// // driveTurn2(0);
// // wing1.set_value(true);
// // driveStraight2(2100);
// // wing1.set_value(false);
// // driveTurn2(75);
// // driveStraight2(800);
// // wing1.set_value(true);
// // driveTurn2(120);
// // driveStraight2(2100);



// ////old v2

//     // delay(350);
//     // int count = 0;
//     // CATA.move(127);
//     // while (count < 47){
//     //   if (catalim.get_value() == true){
//     //     count ++;
//     //     delay(300);
//     //   }
//     // }
//     // CATA.move(0);

//     // while(catalim.get_value() == false){
//     //  CATA.move(127); 
//     // }
//     // CATA.move(0);
//     // INTAKE.move(-110);

//     // driveStraight2(-190); //290
//     // driveTurn2(-158);
//     // driveStraight2(3300); //3250
//     // driveTurn2(83);
//     // //break
//     // driveStraight2(1000);
//     // driveTurn2(23);
//     // driveStraight2(600);
//     // driveTurn2(113);
//     // driveStraight2(1650); //1600
//     // driveTurn2(-153);
//     // wing1.set_value(true);
//     // driveStraight2(1401);
//     // wing1.set_value(false);
//     // driveStraight2(-1300);
//     // driveTurn2(-133);
//     // wing1.set_value(true);
//     // driveStraight2(1401);
//     // wing1.set_value(false);
//     // driveStraight2(-1000);
//     // driveTurn2(90); //57 //101 //97
//     // driveStraight2(-2400);//2200

//     // ////cut
//     // driveTurn2(11);//78 //71 //62
//     // driveStraight2(-601);
//     // driveTurn2(-38); //34
//     // driveStraight2(-1201);
//     // driveStraight2(800);
//     // driveStraight2(-2001);
//     // driveStraight2(800);
//     // driveTurn2(-40); //-10//15
//     // driveStraight2(-2001);
//     // driveStraight2(800);



// //FULL LOCAL START//
//     // driveStraight(-290);
//     // driveTurn(-155);
//     // driveStraight(3250);
//     // driveTurn(-122);
//     // //break
//     // driveStraight(1000);
//     // driveTurn(-60);
//     // driveStraight(600);
//     // driveTurn(90);
//     // driveStraight(1600);
//     // driveTurn(90);
//     // wing1.set_value(true);
//     // driveStraight(1400);
//     // wing1.set_value(false);
//     // driveStraight(-1300);
//     // driveTurn(20);
//     // wing1.set_value(true);
//     // driveStraight(1400);
//     // wing1.set_value(false);
//     // driveStraight(-1000);
//     // driveTurn(-126); //57
//     // driveStraight(-2400);//2200

//     // ////cut
//     // driveTurn(-90);//78 //71 //62
//     // driveStraight(-601);
//     // driveTurn(-45);
//     // driveStraight(-1201);
//     // driveStraight(800);
//     // driveStraight(-2001);
//     // driveStraight(800);
//     // driveTurn(15); //-10//15
//     // driveStraight(-2001);
//     // driveStraight(800);
// //FULL LOCAL END

//     // driveTurn(-118);//78 //71 //62
//     // driveStraight(-1200);
//     // driveStraight(800);
//     // driveTurn(-10);
//     // driveStraight(-1200);
//     // driveStraight(800);
//     // driveStraight(-2001);
//     // driveStraight(800);
//     // driveTurn(-10);
//     // driveStraight(-2001);
//     // driveStraight(800);



//     //driveTurn(-122);
//     // driveStraight(2200);
//     // driveTurn(25);
//     // driveStraight(700);
//     // driveTurn(90);
//     // wing1.set_value(true);
//     // driveStraight(1400);
//     // wing1.set_value(false);
//     // driveStraight(-1300);
//     // driveTurn(20);
//     // wing1.set_value(true);
//     // driveStraight(1400);
//     // wing1.set_value(false);
//     // driveStraight(00);
    
///////////////////////////////////////////////////////////old prog skills/////////////////////////////////////////////////////////////////////////////////////

//driveTurn2(30);//27 //30 //27 //30 /30
//imu.tare();
//driveStraight2(4100); //3850

//while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
     //CATA.move_velocity(40);
     //delay(1);
     //time += 1;
    }
    //CATA.move(0);
//tempre = false;




//time = 0;
//int lift_count = 0;
//double angle = 0;





//while(true){
//setConstants(0.09, 0, 0.2); //0.075
//angle = liftroto.get_angle();

    //if (angle > 30000){
      //angle = angle-36000;
    //}
      //LIFT.move(calcPID(6000, angle, 40, 140, false));
      //if (abs(liftroto.get_angle() - 15000) < 900){ //1000
        //lift_count ++;
      //}
      //delay(10);
      //time += 10;
      //if ((lift_count > 400) || time > 750){
        //LIFT.move(0);
        //time += 10;
        //break;
        //lift_count = 0;
      //}
//}
//intakep.set_value(false);

///old v2
//driveTurn2(-15);
//wing1.set_value(true);//Gerald was here
//driveStraight2(1200); //1400
//wing1.set_value(false); //800
//driveTurn2(-60);
//driveStraight2(1601);
////xtrra2start
// driveStraight2(800);
// driveTurn2(-30); //-45
// driveStraight2(-1601);
// driveTurn2(-50);
////xxtra2sart
//xtra
//driveStraight2(-700);//-800
//driveTurn2(-55);
//driveStraight2(1601);
//xtra
//driveStraight2(-700); //-800
//////////////////////////////Alex Start here
//driveTurn2(-140);
//driveArcR(60, 1900, 5000);
/*
driveArcR(-50, 1900, 5000);
driveTurn2(-140); //-150
driveStraight2(1700);//2000 //1650 //1850
driveArcR(90, 1000, 5000);
//old side///////////////////////
// driveTurn2(-60);
// driveStraight2(300); //1300 //2000 //2500
// driveTurn2(-35); //-153 //-125 //-35
// wing1.set_value(true);
// driveStraight2(700); //700
// wing1.set_value(false);//robot.disconnect();
// driveTurn2(30);
// driveStraight2(1500);
// old side////////////////////////////////////
driveTurn2(-60);
driveStraight2(500); //1300 //2000 //2500
driveTurn2(20);
driveStraight2(2101);
driveStraight2(-1500);
driveTurn2(-60); //-65

driveStraight2(800);//1000 //1400 //700
driveTurn2(30); //-142 //-155 //
wing1.set_value(true);
delay(350);
driveStraight2(2101);
wing1.set_value(false);

driveStraight2(-1500);
driveTurn2(-60); //-65
//wing1.set_value(true); 
driveStraight2(1600); //2000
//wing1.set_value(false);//1000 //1400 //1100
//driveTurn2(-60);
//driveStraight2(800);
driveTurn2(75);
wing1.set_value(true);
driveStraight2(1200);
driveTurn2(30);
driveStraight2(2101);
wing1.set_value(false);
driveStraight2(-1500);
wing1.set_value(true);
delay(550);
driveTurn2(40);
driveStraight2(2101);
wing1.set_value(false);
driveStraight2(-1500);
// driveStraight2(2101);
// driveStraight2(-1500);

//old last push///////////////
// driveTurn2(-60); //-65
// driveStraight2(700);//1000 //1400 //1100
// driveTurn2(60);
// wing1.set_value(true);
// driveStraight2(1200);
// wing1.set_value(false);
// driveTurn2(30);
// driveStraight2(2101);
// driveStraight2(-1500);
//old last push///////////////////

// driveTurn2(0);
// wing1.set_value(true);
// driveStraight2(2100);
// wing1.set_value(false);
// driveTurn2(75);
// driveStraight2(800);
// wing1.set_value(true);
// driveTurn2(120);
// driveStraight2(2100);



////old v2

    // delay(350);
    // int count = 0;
    // CATA.move(127);
    // while (count < 47){
    //   if (catalim.get_value() == true){
    //     count ++;
    //     delay(300);
    //   }
    // }
    // CATA.move(0);

    // while(catalim.get_value() == false){
    //  CATA.move(127); 
    // }
    // CATA.move(0);
    // INTAKE.move(-110);

    // driveStraight2(-190); //290
    // driveTurn2(-158);
    // driveStraight2(3300); //3250
    // driveTurn2(83);
    // //break
    // driveStraight2(1000);
    // driveTurn2(23);
    // driveStraight2(600);
    // driveTurn2(113);
    // driveStraight2(1650); //1600
    // driveTurn2(-153);
    // wing1.set_value(true);
    // driveStraight2(1401);
    // wing1.set_value(false);
    // driveStraight2(-1300);
    // driveTurn2(-133);
    // wing1.set_value(true);
    // driveStraight2(1401);
    // wing1.set_value(false);
    // driveStraight2(-1000);
    // driveTurn2(90); //57 //101 //97
    // driveStraight2(-2400);//2200

    // ////cut
    // driveTurn2(11);//78 //71 //62
    // driveStraight2(-601);
    // driveTurn2(-38); //34
    // driveStraight2(-1201);
    // driveStraight2(800);
    // driveStraight2(-2001);
    // driveStraight2(800);
    // driveTurn2(-40); //-10//15
    // driveStraight2(-2001);
    // driveStraight2(800);



//FULL LOCAL START//
    // driveStraight(-290);
    // driveTurn(-155);
    // driveStraight(3250);
    // driveTurn(-122);
    // //break
    // driveStraight(1000);
    // driveTurn(-60);
    // driveStraight(600);
    // driveTurn(90);
    // driveStraight(1600);
    // driveTurn(90);
    // wing1.set_value(true);
    // driveStraight(1400);
    // wing1.set_value(false);
    // driveStraight(-1300);
    // driveTurn(20);
    // wing1.set_value(true);
    // driveStraight(1400);
    // wing1.set_value(false);
    // driveStraight(-1000);
    // driveTurn(-126); //57
    // driveStraight(-2400);//2200

    // ////cut
    // driveTurn(-90);//78 //71 //62
    // driveStraight(-601);
    // driveTurn(-45);
    // driveStraight(-1201);
    // driveStraight(800);
    // driveStraight(-2001);
    // driveStraight(800);
    // driveTurn(15); //-10//15
    // driveStraight(-2001);
    // driveStraight(800);
//FULL LOCAL END

    // driveTurn(-118);//78 //71 //62
    // driveStraight(-1200);
    // driveStraight(800);
    // driveTurn(-10);
    // driveStraight(-1200);
    // driveStraight(800);
    // driveStraight(-2001);
    // driveStraight(800);
    // driveTurn(-10);
    // driveStraight(-2001);
    // driveStraight(800);



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
    */
//}
