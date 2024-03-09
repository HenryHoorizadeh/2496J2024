#include "api.h"
// #include "auton.h"
#include "main.h"
#include "pid.h"
#include "robot.h"
#include "auton.h"
// #include "<valarray>"
// #include "<sstream>"
// #include "<string>"


using namespace pros;
using namespace c;
using namespace std;

//constants used for calculating power/voltage
double vKp;
double vKi;
double vKd;
float error; //amount from target
float viewvol;
double viewIntegral;
double prevPower;
double currentPower; // temp
double lift_target = 0;
bool tempre = true;
bool temp_lift = false;

double prevError; //how is this specified/calculated??
double h;
//hi
int integral;
int derivative;
int time2;
int viewL = 0;

double power; //voltage provided to motors at any given time to reach the target


//calc2


double vKp2;
double vKi2;
double vKd2;
float error2; //amount from target
float viewvol2;
double viewIntegral2;
double prevPower2;
double currentPower2; // temp
bool tempre2 = true;
bool temp_lift2 = false;

double prevError2; //how is this specified/calculated??
double h2;
//hi
int integral2;
int derivative2;
int time22;

double power2;


void setConstants(double kp, double ki, double kd) {
    vKp = kp;
    vKi = ki;
    vKd = kd;
} 

void resetEncoders() { //reset the chassis motors every time a target is reached
    LF.tare_position(); //or set_zero_position(0) or set_zero_position(LF.get_position()); (sets current encoder position to 0)
    LB.tare_position();
	RF.tare_position();
	RB.tare_position();
    RM.tare_position();
	LM.tare_position();
}

//setting method for driving straight or turning (pos neg voltages change directions)
void chasMove(int voltageLF, int voltageLB, int voltageLM, int voltageRF, int voltageRB, int voltageRM) { //voltage to each chassis motor
    LF.move(voltageLF);
    LM.move(voltageLB);
    LB.move(voltageLB);
    RF.move(voltageRF);
    RM.move(voltageRF);
    RB.move(voltageRB);
}

int slew = 3;
double calcPID(double target, double input, int integralKi, int maxIntegral, bool slewOn) { //basically tuning i here
    int integral;
    prevError = error;
    error = target - input;
    prevPower = power;
    
    if(std::abs(error) < integralKi) {
        integral += error;
    } else {
        integral = 0;
    }

    if(integral >= 0) {
        integral = std::min(integral, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral = std::max(integral, -maxIntegral); //same thing but negative max
    }
    integral;
    
    derivative = error - prevError;

    power = (vKp * error) + (vKi * integral) + (vKd * derivative);

    return power;
} 

double calcPID2(double target, double input, int integralKi, int maxIntegral, bool slewOn) { //basically tuning i here
    int integral2;
    prevError2 = error2;
    error2 = target - input;
    prevPower2 = power2;
    
    if(std::abs(error2) < integralKi) {
        integral2 += error2;
    } else {
        integral2 = 0;
    }

    if(integral2 >= 0) {
        integral2 = std::min(integral2, maxIntegral); //min means take whichever value is smaller btwn integral and maxI
        //integral = integral until integral is greater than maxI (to keep integral limited to maxI)
    } else {
        integral2 = std::max(integral2, -maxIntegral); //same thing but negative max
    }
    integral2;
    
    derivative2 = error2 - prevError2;

    

    power2 = (vKp * error2) + (vKi * integral2) + (vKd * derivative2);

    return power2;
}

//driving straight
void driveStraight(int target) {
    int timeout = 0;
    // if (abs(target) < 800) {
    //     timeout = 2700;
    // } else {
    //     timeout = 5000;
    // }

    
    double x = 0;
    x = double(abs(target));
    timeout = (0.0000000000000214 * pow(x,5)) + (-0.00000000020623 * pow(x, 4)) + (0.00000074005 * pow(x, 3)) + (-0.00121409 * pow(x, 2)) + (1.27769 * x) + 426;


    imu.tare();

    // if (target == 1400){
    //     timeout = 1200;
    // }
    // if (target == -1201){
    //     timeout = 1100;
    // }
    // if (target == 1501){
    //     timeout = 1200;
    // }
    

    // if (target == 1800){
    //     timeout = 1200;
    // }
    // if (target == -1201){
    //     timeout = 1000;
    // } 

    // if (target == -1100){
    //     timeout = 500;
    // } 
    
    // if (target == -1400){
    //     timeout = 1100;
    // }

    // if (target == -601){
    //     timeout = 900;
    // }

    // if (target == 521){
    //     timeout = 500;
    // }
    // if (target == 151){
    //     timeout = 300;
    // }

    
    // if (target == -2001){
    //     timeout = 1500;
    // }
    // if (target < 0){
    //      setConstants(53, 0.4, 878); //0.4
    // } else{
    //      setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    // }

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;
    int maxPower = 10;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   
    

    while(true) {

     
    
        // // temp cata reset
        if (tempre){
        if (catalim.get_value() == false) CATA.move(127);
        else CATA.move(0);
        }

        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

        
        //heading correction
        // if(init_heading > 180) {
        //     init_heading = (360 - init_heading);
        // }

        if(imu.get_heading() < 180) {
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error = ((360 - imu.get_heading()) - init_heading);
        }

        heading_error = heading_error * 7; //7

        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        chasMove( (voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            CATA.move(0);
            break;
        } 

        delay(10);
        if (time2 % 50 == 0 && time2 % 100 != 0 && time2 != 150){
        con.print(0, 0, "ERROR: %f           ", float(encoderAvg));
      } 
      if (time2 % 50 == 0 && time2 % 100 != 0){
        con.print(1, 0, "CataTemp: %f           ", float(error));
      } 
      if (time2 % 50 == 0){
        con.print(2, 0, "Temp: %f        ", float(voltage));
      } 
        
        // if (time2 % 100 == 0) con.clear(); else if (time2 % 50 == 0) {
		// 	cycle++;
        //     if ((cycle+1) % 3 == 0) con.print(0, 0, "ERROR: %2f", encoderAvg); 
        //     if ((cycle+2) % 3 == 0) con.print(1, 0, "Volatge: %2f", heading_error); //autstr //%s
        //     if ((cycle+3) % 3 == 0) con.print(2, 0, "Temp: %f", 0.001);
		// }
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}


//driving straight
void driveStraight2(int target) {
    int timeout = 0;
    // if (abs(target) < 1100) {
    //     timeout = 2700;
    // } else {
    //     timeout = 5000;
    // }

    
    double x = 0;
    x = double(abs(target));
    timeout = (0.0000000000000214 * pow(x,5)) + (-0.00000000020623 * pow(x, 4)) + (0.00000074005 * pow(x, 3)) + (-0.00121409 * pow(x, 2)) + (1.27769 * x) + 426;

    

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;
    int maxPower = 10;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   
        int lift_count = 0;
    double angle = 0;
    int viewL = 0;

    while(true) {
    if(temp_lift){
    //temp lift 
    
    int lift_count = 0;
    int viewL = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      //LIFT.move(100);
      viewL = calcPID2(lift_target, angle, 40, 140, false);
      viewL=100;
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }

        setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
        // temp cata reset
        if (tempre){
        if (catalim.get_value() == false) CATA.move(127);
        else CATA.move(0);
        }

        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

        
   
        if(init_heading > 180) {
            init_heading = (360 - init_heading);
        }

        if(imu.get_heading() < 180) {
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error = ((360 - imu.get_heading()) - init_heading);
        }

        heading_error = heading_error * 7;
         //heading_error = 0;
        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        chasMove( (voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 4) count++;
        if (count >= 20 || time2 > timeout){
            CATA.move(0);
          break;
        } 

        delay(10);
        
        if (time2 % 100 == 0) con.clear(); else if (time2 % 50 == 0) {
			cycle++;
            setConstants(0.075, 0, 0.1);
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Enc: %2f", encoderAvg); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Heading: %2f", heading_error); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "ERROR: %f", float(calcPID2(4500, 10, 40, 140, false)));
		}
        time2 += 10;
        //hi
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveStraightC(int target) {
    int timeout = 0;
    bool over = false;
    //int targetN = target + 500;
    // if (abs(target) < 1100) {
    //     timeout = 2700;
    // } else {
    //     timeout = 5000;
    // }

    
    double x = 0;
    x = double(abs(target));
    timeout = (0.0000000000000214 * pow(x,5)) + (-0.00000000020623 * pow(x, 4)) + (0.00000074005 * pow(x, 3)) + (-0.00121409 * pow(x, 2)) + (1.27769 * x) + 426;
    if(target > 0){
    target = target + 500;
    } else {
        target = target - 500;
    }

    double voltage;
    double encoderAvg;
    int count = 0;
    double init_heading = imu.get_heading();
    double heading_error = 0;
    int cycle = 0; // Controller Display Cycle
    time2 = 0;
    int maxPower = 10;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   
    

    while(true) {

    if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

     
    
        // temp cata reset
        if (tempre){
        if (catalim.get_value() == false) CATA.move(127);
        else CATA.move(0);
        }

        encoderAvg = (LF.get_position() + RF.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

        
   
        if(init_heading > 180) {
            init_heading = (360 - init_heading);
        }

        if(imu.get_heading() < 180) {
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error = ((360 - imu.get_heading()) - init_heading);
        }

        heading_error = heading_error * 7;
         //heading_error = 0;
        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        chasMove( (voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (target > 0){
            if ((encoderAvg - (target-500)) > 0){
                over = true;
            }
        } else {
             if (((target+500) - encoderAvg) > 0){
                over = true;
            }
        }

        if (over || time2 > timeout){
            CATA.move(0);
          break;
        } 

        delay(10);
        
        if (time2 % 100 == 0) con.clear(); else if (time2 % 50 == 0) {
			cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Enc: %2f", encoderAvg); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Heading: %2f", heading_error); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "ERROR: %f", float(error));
		}
        time2 += 10;
        //hi
    }
    // LF.brake();
    // LM.brake();
    // LB.brake();
    // RF.brake();
    // RM.brake();
    // RB.brake();
}


//Turning
void driveTurn(int target) { //target is inputted in autons
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;

    // if (abs(target) < 10){
    // setConstants(7.325, 0.025, 90);     
    // } else if (abs(target) < 25){
    // setConstants(8, 0, 70);
    // } else if (abs(target) <= 39){
    // setConstants(8, 0.001, 91); //6.75 //88
    // } else if (abs(target) < 45){
    // setConstants(8, 0.001, 89);
    // } else if (abs(target) < 50){
    // setConstants(7.15, 0.001, 65);
    // } else if (abs(target) < 60){
    // setConstants(8.75, 0.001, 110);
    // } else if (abs(target) < 70){
    // setConstants(8.75, 0.025, 110);
    // } else if (abs(target) < 95){
    // setConstants(TURN_KP, TURN_KI, TURN_KD);
    // } else if(abs(target) < 115) {
    // setConstants(7.325, 0.025, 90);
    // } else if(abs(target) < 128){
    // setConstants(7.65, 0.025, 115);
    // } else if (abs(target) < 150){
    // setConstants(7.75, 0.025, 130);
    // } else if (abs(target) < 160){
    // setConstants(8, 0, 90);
    // } else if(abs(target) < 180){
    // setConstants(8.1, 0.025, 160);
    // }
    
    int timeout = 0;
    setConstants(TURN_KP, TURN_KI, TURN_KD);
    // if (abs(target) < 30) {
    //     timeout = 1900;
    // } else {
    //     timeout = 2100;
    // }



    double variKP = 0;
    double x = 0;
    x = double(abs(target));
    variKP = ( 0.0000000087421 * pow(x, 4)) + (-0.00000244862 * pow(x, 3)) + (-0.0000282007 * pow(x, 2)) + (0.0420013 * x) + 6.65754;

    double variKD = 0;
    x = double(abs(target));
    variKD = ( 0.00000040026 * pow(x, 4)) + (-0.00017729 * pow(x, 3)) + (0.0254803 * pow(x, 2)) + (-1.19824 * x) + 108.796;
    setConstants(variKP, TURN_KI, variKD);

    
    x = double(abs(target));
    timeout = (0.000000084191 * pow(x,5)) + (-0.0000359624 * pow(x, 4)) + (0.00592914 * pow(x, 3)) + (-0.471886 * pow(x, 2)) + (19.9573 * x) + 474.454;



    //timeout = (-0.00000536976 * pow(x, 4)) + (0.00247647 * pow(x, 3)) + (-0.337691 * pow(x, 2)) + (21.0318 * x) + 900;


    
    imu.tare_heading();

    while(true) {
        // temp cata reset
        if (tempre){
        if (catalim.get_value() == false) CATA.move(127);
        else CATA.move(0);

        }
        
        position = imu.get_heading(); //this is where the units are set to be degrees

    
        if (position > 180) {
            position = ((360 - position) * -1 );
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, false);
        // con.print(1, 0, "%2f", voltage);
        
        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);
        
        if (abs(target - position) <= 0.5) count++; //0.35
        if (count >= 20 || time2 > timeout) {
         //  imu.tare_heading();
           break; 
           
        }

        
        if (time2 % 100 == 0) con.clear(); else if (time2 % 50 == 0) {
			cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", target-position); 
            //if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", integral); //autstr //%s
            //if ((cycle+3) % 3 == 0) con.print(2, 0, "Integral: %f", integral);
		}
        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}



//Turning
void driveTurn2(int target) { //target is inputted in autons
    double voltage;
    double position;
    int count = 0;
    time2 = 0;
    int cycle = 0;
    int turnv = 0;
    int sav = 0;



    
    position = imu.get_heading(); //this is where the units are set to be degrees
    if (position > 180){
        position = ((360 - position) * -1 );
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
           position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }
 //fortnite - derrick

 
    double variKP = 0;
    double x = 0;
    x = double(abs(turnv));
    variKP = ( 0.0000000087421 * pow(x, 4)) + (-0.00000244862 * pow(x, 3)) + (-0.0000282007 * pow(x, 2)) + (0.0420013 * x) + 6.65754;

    double variKD = 0;
    x = double(abs(turnv));
    variKD = ( 0.00000040026 * pow(x, 4)) + (-0.00017729 * pow(x, 3)) + (0.0254803 * pow(x, 2)) + (-1.19824 * x) + 108.796;
    setConstants(variKP, TURN_KI, variKD);
    


    // if (abs(turnv) < 10){
    // setConstants(7.325, 0.025, 90);     
    // } else if (abs(turnv) < 25){
    // setConstants(8, 0, 70);
    // } else if (abs(turnv) <= 40){
    // setConstants(8, 0.001, 91); //6.75 //88
    // } else if (abs(turnv) < 45){
    // setConstants(8, 0.001, 89);
    // } else if (abs(turnv) < 50){
    // setConstants(7.15, 0.001, 65);
    // } else if (abs(turnv) < 60){
    // setConstants(8.75, 0.001, 110);
    // } else if (abs(turnv) < 75){
    // setConstants(8.75, 0.025, 110);
    // } else if (abs(turnv) < 95){
    // setConstants(7.325, 0.025, 73);
    // } else if(abs(turnv) < 115) {
    // setConstants(7.325, 0.025, 90);
    // } else if(abs(turnv) < 130){
    // setConstants(7.85, 0.025, 95); //7.5
    // } else if(abs(turnv) < 150){
    // setConstants(8.25, 0.025, 130); //7.75
    // } else if(abs(turnv) < 160){
    // setConstants(8, 0, 90); //7.75 //7.75//0.025//100
    // } else if(abs(turnv) < 180){
    // setConstants(8.1, 0.025, 155);
    // } else {
    // setConstants(7.325, 0.025, 73);  
    // }
    
    int timeout = 2100;

    // if (abs(turnv) < 30) {
    //     timeout = 1900;
    // } else {
    //     timeout = 2100;
    // }

    
    x = double(abs(turnv));
    timeout = (0.000000084191 * pow(x,5)) + (-0.0000359624 * pow(x, 4)) + (0.00592914 * pow(x, 3)) + (-0.471886 * pow(x, 2)) + (19.9573 * x) + 374.454; //474.45


    
    // double x = 0;
    //x = double(abs(turnv));
    //timeout = (-0.00000536976 * pow(x, 4)) + (0.00247647 * pow(x, 3)) + (-0.337691 * pow(x, 2)) + (21.0318 * x) + 600;


    
    

    while(true) {
        if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    //setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }
        // temp cata reset
        if (tempre){
        if (catalim.get_value() == false) CATA.move(127);
        else CATA.move(0);
        }
 
    position = imu.get_heading(); //this is where the units are set to be degrees
    if (position > 180){
        position = ((360 - position) * -1 );
    }

    if((target < 0) && (position > 0)){
        if((position - target) >= 180){
            target = target + 360;
            position = imu.get_heading();
            turnv = (target - position); // target + position
        } else {
             turnv = (abs(position) + abs(target));
        }
    } else if ((target > 0) && (position < 0)){
        if((target - position) >= 180){
           position = imu.get_heading();
            turnv = abs(abs(position) - abs(target));
        } else {
            turnv = (abs(position) + target);
        }
    } else {
         turnv = abs(abs(position) - abs(target));
    }
        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, false);
        // con.print(1, 0, "%2f", voltage);
        
        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);
        
        if (abs(target - position) <= 0.5) count++; //0.35
        if (count >= 20 || time2 > timeout) {
           break; 
        }

        
        if (time2 % 100 == 0) con.clear(); else if (time2 % 50 == 0) {
			cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", float(turnv)); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", float(variKP)); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "Integral: %2f", float(variKD));
		}
        time2 += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

void driveArcLF(double theta, double radius, int timeout){
//setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);

setConstants(0.25, 0, 0.01);
double ltarget = 0;
double rtarget = 0;
double ltargetFinal = 0;
double rtargetFinal = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
bool over = false;
//imu.tare_heading();
int count = 0;
int time = 0;
resetEncoders();
con.clear();
//int timeout = 5000;
ltargetFinal = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
rtargetFinal = double((theta / 360) * 2 * pi * (radius + 550));
theta = theta + 45;
ltarget = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
rtarget = double((theta / 360) * 2 * pi * (radius + 550));
while (true){

if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }
    setConstants(0.25, 0, 0.01);


    
double heading = imu.get_heading() - init_heading;
if (theta > 0){
    if (heading > 30){
        heading = heading - 360;
    }
} else {
    if (heading > 300){
        heading = heading - 360;
    }    
}
double encoderAvgL = LF.get_position();
//encoderAvgL = 100;
double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageL > 70){
     voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageR > 100){
     voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}

//delay(50);
//con.print(1, 0, "Aut 0: %f        ", float(encoderAvgL));
double leftcorrect = (encoderAvgL * 360) / (2 * pi * radius);
int fix = int(heading + leftcorrect);
fix = fix * 10;
con.print(0, 0, "Aut 0: %f        ", float(timeout-time));
 

chasMove( (voltageL - fix), (voltageL - fix), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
        if (radius > 0){
            if ((encoderAvgL - ltargetFinal) > 0){
                over = true;
            }
        } else {
             if ((ltargetFinal - encoderAvgL) > 0){
                over = true;
            }
        }

if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
if (over || time > timeout){
   CATA.move(0);
     break;
} 
time += 10;
delay(10);

}
}

void driveArcL(double theta, double radius, int timeout){
//setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
setConstants(0.25, 0, 0.01);
double ltarget = 0;
double rtarget = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
//imu.tare_heading();
int count = 0;
int time = 0;
resetEncoders();
con.clear();
//int timeout = 5000;
ltarget = double((theta / 360) * 2 * pi * radius); // * double(2) * pi * double(radius));
rtarget = double((theta / 360) * 2 * pi * (radius + 550));
while (true){



if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }
setConstants(0.25, 0, 0.01);

double heading = imu.get_heading() - init_heading;
if (theta > 0){
    if (heading > 30){
        heading = heading - 360;
    }
} else {
    if (heading > 300){
        heading = heading - 360;
    }    
}
double encoderAvgL = LF.get_position();
//encoderAvgL = 100;
double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageL > 70){
     voltageL = 70;
} else if (voltageL < -70){
    voltageL = -70;
}
int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageR > 100){
     voltageR = 100;
} else if (voltageR < -100){
    voltageR = -100;
}

//delay(50);
//con.print(1, 0, "Aut 0: %f        ", float(encoderAvgL));
double leftcorrect = (encoderAvgL * 360) / (2 * pi * radius);
int fix = int(heading + leftcorrect);
fix = fix * 10;
con.print(0, 0, "Aut 0: %f        ", float(voltageR + fix));
 

chasMove( (voltageL - fix), (voltageL - fix), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
if (count >= 20 || time > timeout){
   CATA.move(0);
     break;
} 
time += 10;
delay(10);

}
}






void driveArcR(double theta, double radius, int timeout){
//setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
setConstants(0.25, 0, 0.01);
double ltarget = 0;
double rtarget = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
if (init_heading > 180){
    init_heading = init_heading - 360;
}
//imu.tare_heading();
int count = 0;
int time = 0;
resetEncoders();
con.clear();
//int timeout = 5000;
ltarget = double((theta / 360) * 2 * pi * (radius + 550)); // * double(2) * pi * double(radius));
rtarget = double((theta / 360) * 2 * pi * (radius));
while (true){

if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }

    setConstants(0.25, 0, 0.01);


double heading = imu.get_heading() - init_heading;
if (theta > 0){
    if (heading > 300){
        heading = heading - 360;
    }
} else {
    if (heading > 30){
        heading = heading - 360;
    }    
}
double encoderAvgL = LF.get_position();
//encoderAvgL = 100;
double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageL > 100){
     voltageL = 100;
} else if (voltageL < -100){
    voltageL = -100;
}
int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageR > 70){
     voltageR = 70;
} else if (voltageR < -70){
    voltageR = -70;
}

//delay(50);
//con.print(1, 0, "Aut 0: %f        ", float(encoderAvgL));
double rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);
int fix = int(heading - rightcorrect);
fix = fix * 10;
con.print(0, 0, "Aut 0: %f        ", float(heading));
 

chasMove( (voltageL - fix), (voltageL - fix), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
if (count >= 20 || time > timeout){
   CATA.move(0);
     break;
} 
time += 10;
delay(10);

}
}



void driveArcRF(double theta, double radius, int timeout){
//setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
setConstants(0.25, 0, 0.01);
bool over = false;
double ltarget = 0;
double rtarget = 0;
double ltargetFinal = 0;
double rtargetFinal = 0;
double pi =  3.14159265359;
double init_heading = imu.get_heading();
if (init_heading > 180){
    init_heading = init_heading - 360;
}
//imu.tare_heading();
int count = 0;
int time = 0;
resetEncoders();
con.clear();
//int timeout = 5000;
ltargetFinal = double((theta / 360) * 2 * pi * (radius+550)); // * double(2) * pi * double(radius));
rtargetFinal = double((theta / 360) * 2 * pi * (radius));
theta = theta + 45;
ltarget = double((theta / 360) * 2 * pi * (radius + 550)); // * double(2) * pi * double(radius));
rtarget = double((theta / 360) * 2 * pi * (radius));
while (true){


if(temp_lift){
    //temp lift 
    int lift_count = 0;
    double angle = 0;
    angle = liftroto.get_angle();
    if (angle > 30000){
      angle = angle-36000;
    }
    setConstants(0.075, 0, 0.1);
      LIFT.move(calcPID2(lift_target, angle, 40, 140, false));
      if (abs(liftroto.get_angle() - 15000) < 1000){
        lift_count ++;
      }

      if (lift_count > 400){
        LIFT.move(0);
        //break;
        //liftToggle = true;
        lift_count = 0;
      }
    }
    setConstants(0.25, 0, 0.01);



double heading = imu.get_heading() - init_heading;
if (theta > 0){
    if (heading > 300){
        heading = heading - 360;
    }
} else {
    if (heading > 30){
        heading = heading - 360;
    }    
}
double encoderAvgL = LF.get_position();
//encoderAvgL = 100;
double encoderAvgR = (RB.get_position() +  RM.get_position()) / 2;
int voltageL = calcPID(ltarget, encoderAvgL, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageL > 100){
     voltageL = 100;
} else if (voltageL < -100){
    voltageL = -100;
}
int voltageR = calcPID(rtarget, encoderAvgR, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

 if(voltageR > 70){
     voltageR = 70;
} else if (voltageR < -70){
    voltageR = -70;
}

//delay(50);
//con.print(1, 0, "Aut 0: %f        ", float(encoderAvgL));
double rightcorrect = (encoderAvgR * 360) / (2 * pi * radius);
int fix = int(heading - rightcorrect);
fix = fix * 10;
con.print(0, 0, "Aut 0: %f        ", float(timeout-time));
   

chasMove( (voltageL - fix), (voltageL - fix), (voltageL - fix), (voltageR + fix), (voltageR + fix), (voltageR + fix));
        if (radius > 0){
            if ((encoderAvgR - (rtargetFinal)) > 0){
                over = true;
            }
        } else {
             if (((rtargetFinal) - encoderAvgR) > 0){
                over = true;
            }
        }

if ((abs(ltarget - encoderAvgL) <= 4) && (abs(rtarget - encoderAvgR) <= 4)) count++;
if (over || time > timeout){
   CATA.move(0);
     break;
} 
time += 10;
delay(10);

}
}
