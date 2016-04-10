#ifndef __PID_RPY
#define __PID_RPY




class PID_RPY
{
	public:

	//Desired state
	double setpoint_roll, setpoint_pitch, setpoint_yaw, setpoint_alt;
	double dt;
	double max;
	double min;
	double kp;
	double kd;
	double ki;
	double pre_error;
	double integral;

	//Input from the IMU;
	double roll_input, pitch_input, yaw_input;
	
	//Input from Rangefinder + Barometer
	double altitude_input;

	//PID Outputs
	double PID_roll, PID_pitch, PID_yaw, PID_alt;

	double user_throttle;
	

	double PID_Calculate(double setpoint, double input)
	{
		// Calculate error
		double error = setpoint - input;

		// Proportional term
		double Pout = kp * error;

		// Integral term
		integral += error * dt;
		double Iout = ki * integral;

		// Derivative term
		double derivative = (error - pre_error) / dt;
		double Dout = kd * derivative;

		// Calculate total output
		double output = Pout + Iout + Dout;

		// Restrict to max/min
		if( output > max )
		output = max;
		else if( output < min )
		output = min;

		// Save error to previous error
		pre_error = error;

		return output;
	};
	

	
	void PID_reset()
	{
		kp = 1;
		ki = 0;
		kd = 0;
	};
	void PID_Setup(double dtt, double maxx, double minn, double KP, double KI, double KD) {
		dt = dtt;
		max = maxx;
		min = minn;
		kp = KP;
		ki = KI;
		kd = KD;
	};
};

	//void PID_Update(PID_RPY roll_object, PID_RPY pitch_object, PID_RPY yaw_object, PID_RPY altitude_object)
	//{
		//roll_object.PID_Calculate(setpoint_roll, roll_input);
		//pitch_object.PID_Calculate(setpoint_pitch, pitch_input);
		//yaw_object.PID_Calculate(setpoint_yaw, yaw_input);
		//altitude_object.PID_Calculate(setpoint_alt, altitude_input);
	//};
#endif



