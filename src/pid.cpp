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

double prevError; //how is this specified/calculated??
double h;

int integral;
int derivative;

double power; //voltage provided to motors at any given time to reach the target


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

    power = (vKp * error) + (vKi * integral) - (vKd * derivative);

    return power;
}

//driving straight
void driveStraight(int target) {
    int timeout = 3500;
    if (abs(target) < 800) {
        timeout = 2700;
    } else {
        timeout = 3500;
    }

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
    int time = 0;
    int maxPower = 10;

    setConstants(STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD);
    resetEncoders();
   
    

    while(true) {

     
    
        // temp cata reset
        if (catalim.get_value() == false) CATA.move(-127);
        else CATA.move(0);

        encoderAvg = (LB.get_position() + RB.get_position()) / 2;
        voltage = calcPID(target, encoderAvg, STRAIGHT_INTEGRAL_KI, STRAIGHT_MAX_INTEGRAL, true);

        
        //heading correction
        if(init_heading > 180) {
            init_heading = (360 - init_heading);
        }

        if(imu.get_heading() < 180) {
            heading_error = init_heading - imu.get_heading();
        }
        else {
            heading_error = ((360 - imu.get_heading()) - init_heading);
        }

        heading_error = heading_error * 5;

        if(voltage > 127){
            voltage = 127;
        } else if (voltage < -127){
            voltage = -127;
        }

        chasMove( (voltage + heading_error ), (voltage + heading_error), (voltage + heading_error), (voltage - heading_error), (voltage - heading_error), (voltage - heading_error));
        if (abs(target - encoderAvg) <= 3) count++;
        if (count >= 20 || time > timeout){
            CATA.move(0);
            break;
        } 

        delay(10);
        
        if (time % 100 == 0) con.clear(); else if (time % 50 == 0) {
			cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "ERROR: %2f", encoderAvg); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Volatge: %2f", voltage); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "Temp: %f", 0.001);
		}
        time += 10;
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

//Turning
void driveTurn(int target) { //target is inputted in autons
    double voltage;
    double position;
    int count = 0;
    int time = 0;
    int cycle = 0;

    setConstants(TURN_KP, TURN_KI, TURN_KD);
    
    int timeout = 2100;

    if (abs(target) < 30) {
        timeout = 1900;
    } else {
        timeout = 2100;
    }
    
    imu.tare_heading();

    while(true) {
        // temp cata reset
        if (catalim.get_value() == false) CATA.move(-127);
        else CATA.move(0);

        position = imu.get_heading(); //this is where the units are set to be degrees
        if (position > 180) {
            position = ((360 - position) * -1);
        }

        voltage = calcPID(target, position, TURN_INTEGRAL_KI, TURN_MAX_INTEGRAL, false);
        // con.print(1, 0, "%2f", voltage);
        
        chasMove(voltage, voltage, voltage, -voltage, -voltage, -voltage);
        
        if (abs(target - position) <= 0.4) count++; //0.35
        if (count >= 20 || time > timeout) {
            imu.tare_heading();
            break;
        }

        
        if (time % 100 == 0) con.clear(); else if (time % 50 == 0) {
			cycle++;
            if ((cycle+1) % 3 == 0) con.print(0, 0, "Error | %2f", target-position); 
            if ((cycle+2) % 3 == 0) con.print(1, 0, "Integral | %2f", integral); //autstr //%s
            if ((cycle+3) % 3 == 0) con.print(2, 0, "Integral: %f", integral);
		}
        time += 10;
        delay(10);
    }
    LF.brake();
    LM.brake();
    LB.brake();
    RF.brake();
    RM.brake();
    RB.brake();
}

