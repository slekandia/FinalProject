//
//
// /*
// * Stable.h
// *
// * Created: 9.3.2016 14:45:25
// * Author: Home
// */
//
//
#ifndef __STABLE_H__
#define __STABLE_H__
//
//
class Stable
{
	public:
	Stable() {

	};
	// 	//variables
	
	float Iout_RATE_RLL;
	float Iout_RATE_PTCH;
	float Iout_RATE_YAW;
	float Iout_RATE_DIST;
	float Iout_ACCEL_DIST;

	float Dout_RATE_RLL;
	float Dout_RATE_PTCH;
	float Dout_RATE_YAW;
	float Dout_RATE_DIST;
	float Dout_ACCEL_DIST;

	float pre_inter1_RLL;
	float pre_inter1_PTCH;
	float pre_inter1_YAW;
	float pre_inter2_DIST;
	float pre_inter1_ACCEL;
	
	float North;
	float South;
	float East;
	float West;
	
	float pre_measuredrollangle;
	float pre_measuredpitchangle;
	float pre_measuredyawangle;
	float pre_measuredaltitude;
	float pre_altituderate;
	
	float measuredrollangle;
	float measuredpitchangle;
	float measuredyawangle;
	float measuredaltitude;
	
	float rollrate;
	float pitchrate;
	float yawrate;
	float altituderate;
	float altitudeaccel;
	
	int K;
	
	void set_K(int K);
	
	float RLL_STB(float desired, float RLL_STB_P, float STB_LIMIT);
	float PTCH_STB(float desired, float PTCH_STB_P, float STB_LIMIT);
	float YAW_STB(float desired, float YAW_STB_P, float STB_LIMIT);
	float ALT_STB(float desired, float THR_ALT_P, float DIST_LIMIT, float RATE_THR_IMAX, float g_LIMIT, float RATE_THR_P, float RATE_THR_I, float RATE_THR_D, float dtt);
	
	float earthToBodyX(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out);
	float earthToBodyY(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out);
	float earthToBodyZ(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out);
	
	float RLL_RATE(float bodyframeX, float dtt, float RLL_RATE_P, float RLL_RATE_I, float RLL_RATE_D, float I_LIMIT, float RATE_LIMIT,float I_RESET_LIMIT);
	float PTCH_RATE(float  bodyframeY, float dtt, float PTCH_RATE_P, float PTCH_RATE_I, float PTCH_RATE_D, float I_LIMIT, float RATE_LIMIT,float I_RESET_LIMIT);
	float YAW_RATE(float  bodyframeZ,float dtt, float YAW_RATE_P, float YAW_RATE_I, float YAW_RATE_D, float I_LIMIT, float RATE_LIMIT,float I_RESET_LIMIT);
	float ALT_RATE(float bodyframeZ,float dtt, float ACCEL_THR_P, float ACCEL_THR_I, float ACCEL_THR_D, float RATE_THR_IMAX, float THR_LIMIT);
	
	float Roll_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt,float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float RLL_RATE_P, float RLL_RATE_I, float RLL_RATE_D);
	float Pitch_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt, float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float PTCH_RATE_P, float PTCH_RATE_I, float PTCH_RATE_D);
	float Yaw_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt, float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float YAW_RATE_P, float YAW_RATE_I, float YAW_RATE_D);
	float Altitude_Controller(float desired, float dtt, float THR_ALT_P, float DIST_LIMIT, float RATE_THR_IMAX, float g_LIMIT, float RATE_THR_P, float RATE_THR_I, float RATE_THR_D, float ACCEL_THR_P, float ACCEL_THR_I, float ACCEL_THR_D, float THR_LIMIT);
	
	void CONTROL_ALLOCATION (float ALT_IN, float RLL_IN, float PTCH_IN, float YAW_IN, float b, float l, float d);
	
	void getData(float measured_rollangle,float measured_pitchangle,float measured_yawangle, float measured_altitude, float dtt);
	
	float returnNorth();
	float returnSouth();
	float returnEast();
	float returnWest();
	
	float Roll_Countroller(float measured_roll, float desired_roll, float Kp_roll_1,float Kp_roll_2, float Ki_roll, float Kd_roll,float dtt) {
		float roll_rate;
			
		
		float error = desired_roll-measured_roll;
		
		float block_1 = Kp_roll_1*error;
		
		 
		
		
		
		
	}
	
	
	
	
}; //Stable
//
#endif //__STABLE_H__