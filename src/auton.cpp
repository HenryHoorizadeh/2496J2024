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


    LIFT.move(0);
    cataroto.reset_position();
    ///////////////////////////////////////////////////////////////////
    CATA.move_velocity(60); //
     while ((time < 27000) && (-cataroto.get_position() < 828000)){
        con.print(1, 0, "Roto: %f           ", float(cataroto.get_position()));
        delay(10);
        time += 10;
    }
    ///////////////////////////////////////////////////////////////////
    CATA.move(0);

    CATA.move(0);
    LF.set_brake_mode(E_MOTOR_BRAKE_COAST);
    LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
    LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
///
driveStraight2(100);

while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
     CATA.move_velocity(45);
     delay(1);
     time += 1;
    }
    CATA.move(0);
tempre = false;
imu.tare();

//driveArcL(60,400,900);
//driveStraight2(400);
//driveTurn(90);
driveTurn2(28); //30
driveStraight2(4100);

tempre = false;

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
}

driveTurn2(-15);
wing1.set_value(true);//Gerald was here
driveStraight2(1200); //1400
wing1.set_value(false); //800
driveTurn2(-60);
intakep.set_value(false);
driveStraight2(1601);
driveStraight2(-700);//-800
driveTurn2(-55);
driveStraight2(1601);
///

//tempre = false;//was not here, can delete later

driveStraight2(-700);//back from goal
driveTurn2(-140);
driveStraight2(1400); //2200
//driveTurn2(-20); //-10
driveArcR(75, 550, 1300);
driveStraight2(-500);
//driveTurn2(-20);

/*
wing1.set_value(1);
driveStraight2(500);
driveTurn2(70);//turn to goal to push 1
wing1.set_value(0);

driveStraight2(1500);// PUSH 1
driveStraight2(-500);
driveStraight2(750);//second push

driveStraight2(-1500);
driveTurn2(-17);

driveStraight2(1000);
driveStraight2(-270);//back a bit to avoid ball
driveTurn2(80);
*/
//wing1.set_value(1);
// driveArcR(45, 450, 1000);
// wing1.set_value(0);
// driveArcR(45, 450, 1000);
//driveArcR(40, 1550, 1000);
//driveArcR(90, 450, 1000);
driveTurn2(0);

// wing1.set_value(1);
// driveArcR(80, 350, 1800);//arc push    does not work :(  works by itself



driveStraight2(2100);//push 2 //1400 //2800
wing1.set_value(0);
// driveStraight2(-500);
// driveStraight2(750);//second push
wing1.set_value(0);
driveTurn2(5);
driveStraight2(-1500);

driveTurn2(-60);//-17 //-73
driveStraight2(900);//drive to side 750
driveTurn2(30);
wing1.set_value(1);
delay(250);
driveStraight2(2000);
wing1.set_value(0);
driveStraight2(-1700); //-1500

driveTurn2(-73);//-17
driveStraight2(1800); //1300


driveTurn2(70); //70
wing1.set_value(1);
delay(300);
driveStraight2(700);

// wing1.set_value(1);
// delay(300);
driveArcL(45, 2500, 1000);//arc push 1 //2300
driveTurn2(30);
wing1.set_value(0);
driveStraight2(1000);
driveTurn2(10);
driveStraight2(-1500);
wing1.set_value(1);
/*
driveTurn2(30);
driveStraight2(1800);
wing1.set_value(0);
driveStraight2(-1500);
*/

/*old last push
driveTurn2(-73);//-17
driveStraight2(1400); //1300


driveTurn2(70); //70

wing1.set_value(1);
delay(300);
//wing1.set_value(1);
driveArcL(45, 2500, 1500);//arc push 1 //2300
driveTurn2(30);
wing1.set_value(0);
driveStraight2(1000);
driveStraight2(-1500);
*/

/*arc end
driveTurn2(-60);
driveStraight2(1000);
wing1.set_value(1);
driveArcR(90, 600, 1500);
driveStraight2(1500);
wing1.set_value(1);
driveTurn2(-60);
driveStraight2(1000);
driveTurn2(30);
driveStraight2(500);
wing1.set_value(1);
driveTurn2(750);
driveStraight2(1200);
driveTurn2(120);
driveStraight2(1600);
driveStraight2(-700);
*/ 

// wing1.set_value(1);
// driveTurn2(30);
// driveStraight2(1500);

// ///driveArcL(45, 2300, 2500); //arc push 2
// // driveStraight2(1000);
// //driveStraight2(-200);
// wing1.set_value(0);
// driveStraight2(-1000);

/*
driveTurn2(80);
driveStraight2(1600);
driveArcR(90, 200, 1000);
driveStraight2(600);
driveStraight2(-600);
driveStraight2(900);
*/


  } else if (atn == 1) {

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


    LIFT.move(0);
    cataroto.reset_position();
    ///////////////////////////////////////////////////////////////////
    CATA.move_velocity(65); //
     while ((time < 27000) && (-cataroto.get_position() < 828000)){
        con.print(1, 0, "Roto: %f           ", float(cataroto.get_position()));
        delay(10);
        time += 10;
    }
    ///////////////////////////////////////////////////////////////////
    CATA.move(0);

    CATA.move(0);
    LF.set_brake_mode(E_MOTOR_BRAKE_COAST);
    LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
    LB.set_brake_mode(E_MOTOR_BRAKE_COAST);
///
driveStraight2(100);

while(((cataroto.get_angle() < 34000 && cataroto.get_angle() > 16000) || (cataroto.get_angle() > 0 && cataroto.get_angle() < 14000)) && time < 1000){
     CATA.move_velocity(45);
     delay(1);
     time += 1;
    }
    CATA.move(0);
tempre = false;
imu.tare();

//driveArcL(60,400,900);
//driveStraight2(400);
//driveTurn(90);
driveTurn2(28); //30
driveStraight2(4100);

tempre = false;

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
}

driveTurn2(-15);
wing1.set_value(true);//Gerald was here
driveStraight2(1200); //1400
wing1.set_value(false); //800
driveTurn2(-60);
intakep.set_value(false);
driveStraight2(1601);
driveStraight2(-700);//-800
driveTurn2(-55);
driveStraight2(1601);
///

//tempre = false;//was not here, can delete later

driveStraight2(-700);//back from goal
driveTurn2(-140);
driveStraight2(1400); //2200
//driveTurn2(-20); //-10
driveArcR(75, 550, 1300);
driveStraight2(-500);
//driveTurn2(-20);

/*
wing1.set_value(1);
driveStraight2(500);
driveTurn2(70);//turn to goal to push 1
wing1.set_value(0);

driveStraight2(1500);// PUSH 1
driveStraight2(-500);
driveStraight2(750);//second push

driveStraight2(-1500);
driveTurn2(-17);

driveStraight2(1000);
driveStraight2(-270);//back a bit to avoid ball
driveTurn2(80);
*/
//wing1.set_value(1);
// driveArcR(45, 450, 1000);
// wing1.set_value(0);
// driveArcR(45, 450, 1000);
//driveArcR(40, 1550, 1000);
//driveArcR(90, 450, 1000);
driveTurn2(0);

// wing1.set_value(1);
// driveArcR(80, 350, 1800);//arc push    does not work :(  works by itself



driveStraight2(2800);//push 2 //1400
wing1.set_value(0);
// driveStraight2(-500);
// driveStraight2(750);//second push
wing1.set_value(0);
driveTurn2(5);
driveStraight2(-1500);

driveTurn2(-60);//-17 //-73
driveStraight2(900);//drive to side 750
driveTurn2(30);
wing1.set_value(1);
delay(250);
driveStraight2(2000);
wing1.set_value(0);
driveStraight2(-1700); //-1500

driveTurn2(-73);//-17
driveStraight2(1800); //1300


driveTurn2(70); //70
wing1.set_value(1);
delay(300);
driveStraight2(700);

// wing1.set_value(1);
// delay(300);
driveArcL(45, 2500, 1000);//arc push 1 //2300
driveTurn2(30);
wing1.set_value(0);
driveStraight2(1000);

driveStraight2(-750);
//wing1.set_value(1);
driveTurn2(-60);
driveArcR(90, 1000, 2000);
driveArcR(90, 200, 2000);
driveStraight2(1000);
driveStraight2(-750);


/*old last push
driveTurn2(-73);//-17
driveStraight2(1400); //1300


driveTurn2(70); //70

wing1.set_value(1);
delay(300);
//wing1.set_value(1);
driveArcL(45, 2500, 1500);//arc push 1 //2300
driveTurn2(30);
wing1.set_value(0);
driveStraight2(1000);
driveStraight2(-1500);
*/

/*arc end
driveTurn2(-60);
driveStraight2(1000);
wing1.set_value(1);
driveArcR(90, 600, 1500);
driveStraight2(1500);
wing1.set_value(1);
driveTurn2(-60);
driveStraight2(1000);
driveTurn2(30);
driveStraight2(500);
wing1.set_value(1);
driveTurn2(750);
driveStraight2(1200);
driveTurn2(120);
driveStraight2(1600);
driveStraight2(-700);
*/ 

// wing1.set_value(1);
// driveTurn2(30);
// driveStraight2(1500);

// ///driveArcL(45, 2300, 2500); //arc push 2
// // driveStraight2(1000);
// //driveStraight2(-200);
// wing1.set_value(0);
// driveStraight2(-1000);

/*
driveTurn2(80);
driveStraight2(1600);
driveArcR(90, 200, 1000);
driveStraight2(600);
driveStraight2(-600);
driveStraight2(900);
*/





///////////////start old1

// tempre = false;

// intakep.set_value(true);
// INTAKE.move(127);
// driveStraight2(2300);
// driveStraight2(-200);
//   driveTurn2(80);
// INTAKE.move(-127);
//   //wing1.set_value(true);
//   //delay(350);
//   driveStraight2(1600);
//   driveStraight2(-1600);
//   //delay(200);
//   wing1.set_value(true);
//   delay(350);
//   driveStraight2(1400);
//   wing1.set_value(false);
//    driveStraight2(-1000);
//    driveTurn2(10);
//    driveStraight2(-2500);

//   //  wing1.set_value(true);
//   //  driveStraight2(500);
//   //  wing1.set_value(false);



// //old rush///////////////
// // intakep.set_value(true);
// // driveStraight2(-1300);
// // driveStraight2(200);
// //   driveTurn2(-65);
// //   driveStraight2(1200);
// //   driveTurn2(-145);
// //   INTAKE.move(127);
// //   driveStraight2(500);
// //   driveTurn2(-55);
// //   INTAKE.move(-127);
// //   wing1.set_value(true);
// //   driveStraight2(1000);
//   //old rush///////////////////////////////





//   // awp.set_value(true);
//   // delay(500);
//   // driveTurn2(-120);
//   // awp.set_value(false);
//   // driveTurn2(0);
//   // intakep.set_value(true);
//   // driveStraight2(-1000);
//   // driveTurn(45);
//   // driveStraight2(-1500);
//   // driveStraight2(1000);
//   // driveTurn2(100);

  
  }


//22222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222222

  else if (atn == 2) {
    tempre = false;
   awp.set_value(true);
  delay(500);
  driveStraight2(-800);
  driveTurn2(170);
  awp.set_value(false);
  // driveTurn2(45);
  // //intakep.set_value(true);
  // driveStraight2(-1500);
  // driveStraight2(1400);
  driveTurn2(0);
  driveStraight2(800);
  driveTurn2(-45);
  driveStraight2(1550);
  intakep.set_value(true);


  }

//3333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333333

  
  else if (atn == 3) {
  //   tempre = false;
  //  awp.set_value(true);
  // delay(500);
  driveStraight2(-2500);
  driveStraight2(1000);
 // driveTurn2(170);
  //awp.set_value(false);
  // driveTurn2(45);
  // //intakep.set_value(true);
  // driveStraight2(-1500);
  // driveStraight2(1400);
  // driveTurn2(0);
  // driveStraight2(800);
  // driveTurn2(-45);
  // driveStraight2(1550);
  // intakep.set_value(true);
    
  
  }

//44444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444444


  else if (atn == 4) {
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
