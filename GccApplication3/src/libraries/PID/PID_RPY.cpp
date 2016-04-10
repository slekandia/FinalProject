#include "libraries/PID/PID_RPY.h"


//PID_RPY::PID_RPY( double dt, double max, double min, double Kp, double Ki, double Kd )
//{
//PID roll_object = PID(0.001,1900,1000,0.15,0.1,0.04);
//PID pitch_object = PID(0.001,1900,1000,0.15,0.1,0.04);
//PID yaw_object = PID(0.001,1900,1000,0.2,0.02,0);

//int motor1 = user_throttle + PID_pitch - PID_yaw;
//int motor3 = user_throttle - PID_pitch - PID_yaw;
//int motor2 = user_throttle + PID_roll + PID_yaw;
//int motor4 = user_throttle - PID_roll + PID_yaw;
//}

//double PID_RPY::Calculate(double setpoint, double input)


//void PID_RPY::Update(PID_RPY roll_object, PID_RPY pitch_object, PID_RPY yaw_object)

//double PID_RPY::PID_Calculate(double setpoint, double input)
//{
	//// Calculate error
	//double error = setpoint - input;
//
	//// Proportional term
	//double Pout = kp * error;
//
	//// Integral term
	//integral += error * dt;
	//double Iout = ki * integral;
//
	//// Derivative term
	//double derivative = (error - pre_error) / dt;
	//double Dout = kd * derivative;
//
	//// Calculate total output
	//double output = Pout + Iout + Dout;
//
	//// Restrict to max/min
	//if( output > max )
	//output = max;
	//else if( output < min )
	//output = min;
//
	//// Save error to previous error
	//pre_error = error;
//
	//return output;
//};
//
//void PID_RPY::PID_Update(PID_RPY roll_object, PID_RPY pitch_object, PID_RPY yaw_object, PID_RPY altitude_object)
//{
	//PID_roll = roll_object.PID_Calculate(setpoint_roll, roll_input);
	//PID_pitch = pitch_object.PID_Calculate(setpoint_pitch, pitch_input);
	//PID_yaw = yaw_object.PID_Calculate(setpoint_yaw, yaw_input);
	//PID_alt = altitude_object.PID_Calculate(setpoint_alt, altitude_input);
//};

