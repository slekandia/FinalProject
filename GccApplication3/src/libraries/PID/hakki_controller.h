float pre_pitch = 0;
float pre_error2 = 0;
float integral_pitch = 0;
float derivative_pitch = 0;

float Pitch_Controller(float measured_pitch, float desired_pitch, float Kp_pitch_1,float Kp_pitch_2, float Ki_pitch, float Kd_pitch,float dtt) {
	float pitch_rate;
	pitch_rate = (measured_pitch-pre_pitch)/dtt;	
		
	float error,error2;
	error = desired_pitch-measured_pitch;
		
	float block_1 = Kp_pitch_1*error;
	
	error2 = block_1-pitch_rate;
	
	derivative_pitch = Kd_pitch*(error2-pre_error2)/dtt;
	
	integral_pitch = error2*Ki_pitch*dtt+integral_pitch;
	
	float output = Kp_pitch_2*error2 + integral_pitch + derivative_pitch;
	
	pre_pitch = measured_pitch;
	pre_error2 = error2;

	return output;
	}