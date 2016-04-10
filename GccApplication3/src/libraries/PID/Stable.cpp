// /*
// * Stable.cpp
// *
// * Created: 16.03.2016 21:32:21
// * Author: Metin
// */
// /*
// * Stable.cpp
// *
// * Created: 9.3.2016 14:45:24
// * Author: Home
// */
//
//
#include "libraries/PID/Stable.h"
#include "libraries/AP_HAL/AP_HAL.h"
#include "libraries/AP_HAL_Empty/AP_HAL_Empty.h"
//
extern "C"{
	#include <math.h>
};

extern const AP_HAL::HAL& hal;

void Stable::getData(float measured_rollangle,float measured_pitchangle,float measured_yawangle, float measured_altitude, float dtt){
	
	measuredrollangle = measured_rollangle;
	measuredpitchangle = measured_pitchangle;
	measuredyawangle = measured_yawangle;
	measuredaltitude = measured_altitude;
	
	rollrate = (measuredrollangle - pre_measuredrollangle)/dtt;
	pitchrate = (measuredpitchangle - pre_measuredpitchangle)/dtt;
	yawrate = (measuredyawangle - pre_measuredyawangle)/dtt;
	altituderate = (measuredaltitude - pre_measuredaltitude)/dtt;
	altitudeaccel = (altituderate-pre_altituderate)/dtt;
	
	pre_measuredrollangle = measuredrollangle;
	pre_measuredpitchangle = measuredpitchangle;
	pre_measuredyawangle = measuredyawangle;
	pre_measuredaltitude = measuredaltitude;
	pre_altituderate = altituderate;
	
};

void Stable::set_K(int Kin){
	K = Kin;
};

float Stable::RLL_STB(float desired, float RLL_STB_P, float STB_LIMIT){
	
	float inter1;
	float output;
	float error;
	
	error = desired - measuredrollangle;
	inter1 = K*error;
	
	if(inter1 >= STB_LIMIT){
		inter1 = STB_LIMIT;
	}
	else if(inter1 <= -STB_LIMIT){
		inter1 = -STB_LIMIT;
	};
	
	output = RLL_STB_P*inter1;
	
	hal.uartB->print("RLL_STB_OUT: ");
	hal.uartB->print(output);
	hal.uartB->print("\t");
	
	return output;
};

float Stable::PTCH_STB(float desired, float PTCH_STB_P, float STB_LIMIT){
	
	float inter1;
	float output;
	float error;
	
	error = desired - measuredpitchangle;
	inter1 = K*error;
	
	if(inter1 >= STB_LIMIT){
		inter1 = STB_LIMIT;
	}
	else if(inter1 <= -STB_LIMIT){
		inter1 = -STB_LIMIT;
	};
	
	output = PTCH_STB_P*inter1;
	
	hal.uartB->print("PTCH_STB_OUT: ");
	hal.uartB->print(output);
	hal.uartB->print("\t");
	
	return output;
};

float Stable::YAW_STB(float desired, float YAW_STB_P, float STB_LIMIT){
	
	float inter1;
	float output;
	float error;
	
	error = desired - measuredyawangle;
	inter1 = K*error;
	
	if(inter1 >= STB_LIMIT){
		inter1 = STB_LIMIT;
	}
	else if(inter1 <= -STB_LIMIT){
		inter1 = -STB_LIMIT;
	};
	
	output = YAW_STB_P*inter1;
	hal.uartB->print("YAW_STB_OUT: ");
	hal.uartB->println(output);
	
	return output;
};

float Stable::ALT_STB(float desired, float THR_ALT_P, float DIST_LIMIT, float RATE_THR_IMAX, float g_LIMIT, float RATE_THR_P, float RATE_THR_I, float RATE_THR_D, float dtt){
	float inter1;
	float inter2;
	float inter3;
	float output;
	float error;
	
	error = desired - measuredaltitude;
	inter1 = error*THR_ALT_P;
	
	if(inter1 >= DIST_LIMIT){
		inter1 = DIST_LIMIT;
	}
	else if(inter1 <= -DIST_LIMIT){
		inter1 = -DIST_LIMIT;
	};
	
	inter2 = inter1 - altituderate;
	
	float Pout = RATE_THR_P*inter2;
	Iout_RATE_DIST += RATE_THR_I*inter2*dtt;
	Dout_RATE_DIST = 0.05*(inter2 - pre_inter2_DIST)*dtt + 0.95*RATE_THR_D;
	
	inter3 = Pout + Iout_RATE_DIST + Dout_RATE_DIST;
	
	if(inter3 >= RATE_THR_IMAX){
		inter3 = RATE_THR_IMAX;
	}
	
	else if(inter3 <= -RATE_THR_IMAX){
		inter3 = -RATE_THR_IMAX;
	};
	
	if(inter3 >= g_LIMIT){
		inter3 = g_LIMIT;
	}
	
	else if(inter3 <= -g_LIMIT){
		inter3 = -g_LIMIT;
	};

	
	output = inter3;
	
	pre_inter2_DIST = inter2;
	return output;
	
};

float Stable::earthToBodyX(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out){
	
	float rad_measuredrollangle = measuredrollangle*3.14159265358979323846/180;
	float rad_measuredpitchangle = measuredpitchangle*3.14159265358979323846/180;
	float rad_measuredyawangle = measuredyawangle*3.14159265358979323846/180;
	
	float bodyframeX = RLL_STB_Out*cos(rad_measuredpitchangle)*cos(rad_measuredyawangle) + PTCH_STB_Out*cos(rad_measuredpitchangle)*sin(rad_measuredyawangle) - YAW_STB_Out*sin(rad_measuredpitchangle);
	
	return bodyframeX;
};

float Stable::earthToBodyY(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out){
	
	float rad_measuredrollangle = measuredrollangle*3.14159265358979323846/180;
	float rad_measuredpitchangle = measuredpitchangle*3.14159265358979323846/180;
	float rad_measuredyawangle = measuredyawangle*3.14159265358979323846/180;
	
	float constant1 = (sin(rad_measuredpitchangle)*sin(rad_measuredrollangle)*cos(rad_measuredyawangle)) - (cos(rad_measuredrollangle)*sin(rad_measuredyawangle));
	float constant2 = (sin(rad_measuredpitchangle)*sin(rad_measuredrollangle)*sin(rad_measuredyawangle)) + (cos(rad_measuredrollangle)*cos(rad_measuredyawangle));
	float constant3 = sin(rad_measuredrollangle)*cos(rad_measuredpitchangle);

	float bodyframeY = RLL_STB_Out*constant1 + PTCH_STB_Out*constant2 + YAW_STB_Out*constant3;
	
	return bodyframeY;
};

float Stable::earthToBodyZ(float RLL_STB_Out, float PTCH_STB_Out, float YAW_STB_Out){
	
	float rad_measuredrollangle = measuredrollangle*3.14159265358979323846/180;
	float rad_measuredpitchangle = measuredpitchangle*3.14159265358979323846/180;
	float rad_measuredyawangle = measuredyawangle*3.14159265358979323846/180;
	
	float constant1 = (sin(rad_measuredpitchangle)*cos(rad_measuredrollangle)*cos(rad_measuredyawangle)) + (sin(rad_measuredrollangle)*sin(rad_measuredyawangle));
	float constant2 = (sin(rad_measuredpitchangle)*cos(rad_measuredrollangle)*sin(rad_measuredyawangle)) - (sin(rad_measuredrollangle)*cos(rad_measuredyawangle));
	float constant3 = cos(rad_measuredrollangle)*cos(rad_measuredpitchangle);
	
	float bodyframeZ = RLL_STB_Out*constant1 + PTCH_STB_Out*constant2 + YAW_STB_Out*constant3;
	
	return bodyframeZ;
};



float Stable::RLL_RATE(float bodyframeX, float dtt, float RLL_RATE_P, float RLL_RATE_I, float RLL_RATE_D, float I_LIMIT, float RATE_LIMIT, float I_RESET_LIMIT){
	
	float inter1;
	float inter2;
	float output;
	float error;
	float difference;
	
	float Pout;
	
	inter1 =  bodyframeX - K*rollrate;

	Pout = inter1*RLL_RATE_P;
	
	Iout_RATE_RLL += RLL_RATE_I*inter1*dtt;
	
	if(Iout_RATE_RLL >= I_LIMIT){
		Iout_RATE_RLL = I_LIMIT;
	}
	else if(Iout_RATE_RLL <= -I_LIMIT){
		Iout_RATE_RLL = -I_LIMIT;
	};
	
	if(measuredrollangle < I_RESET_LIMIT && measuredrollangle > -1*I_RESET_LIMIT){
		Iout_RATE_RLL = 0;
	}
	
	
	Dout_RATE_RLL = 0.05*(inter1 - pre_inter1_RLL)*dtt + 0.95*Dout_RATE_RLL;
	
	inter2 = Pout + Iout_RATE_RLL + Dout_RATE_RLL;
	
	if(inter2 >= RATE_LIMIT){
		inter2 = RATE_LIMIT;
	}
	
	else if(inter2 <= -RATE_LIMIT){
		inter2 = -RATE_LIMIT;
	};
	
	output = inter2;
	pre_inter1_RLL = inter1;
	
	hal.uartB->print("RLL_RATE_OUT: ");
	hal.uartB->print(output);
	hal.uartB->print("\t");

	return output;
	

	
};


float Stable::PTCH_RATE(float bodyframeY,float dtt, float PTCH_RATE_P, float PTCH_RATE_I, float PTCH_RATE_D, float I_LIMIT, float RATE_LIMIT, float I_RESET_LIMIT){
	
	float inter1;
	float inter2;
	float output;
	float error;
	
	float Pout;
	
	inter1 = bodyframeY - K*pitchrate;
	
	
	Pout = inter1*PTCH_RATE_P;
	Iout_RATE_PTCH = inter1*PTCH_RATE_I*dtt;
	
	if(Iout_RATE_PTCH >= I_LIMIT){
		Iout_RATE_PTCH = I_LIMIT;
	}
	else if(Iout_RATE_PTCH <= -I_LIMIT){
		Iout_RATE_PTCH = -I_LIMIT;
	};
	
	if(measuredpitchangle < I_RESET_LIMIT && measuredpitchangle > -1*I_RESET_LIMIT){
		Iout_RATE_PTCH = 0;
	}
	
	Dout_RATE_PTCH = 0.05*(inter1 - pre_inter1_PTCH)*dtt + 0.95*Dout_RATE_PTCH;
	
	inter2 = Pout + Iout_RATE_PTCH + Dout_RATE_PTCH;
	
	if(inter2 >= RATE_LIMIT){
		inter2 = RATE_LIMIT;
	}
	
	else if(inter2 <= -RATE_LIMIT){
		inter2 = -RATE_LIMIT;
	};
	
	output = inter2;
	pre_inter1_PTCH = inter1;
	
	hal.uartB->print("PTCH_RATE_OUT: ");
	hal.uartB->print(output);
	hal.uartB->print("\t");

	return output;
	
};


float Stable::YAW_RATE(float bodyframeZ,float dtt, float YAW_RATE_P, float YAW_RATE_I, float YAW_RATE_D, float I_LIMIT, float RATE_LIMIT,float I_RESET_LIMIT){
	
	float inter1;
	float inter2;
	float output;
	float error;
	
	float Pout;
	
	inter1 = bodyframeZ - K*yawrate;
	
	
	Pout = inter1*YAW_RATE_P;
	Iout_RATE_YAW = inter1*YAW_RATE_I*dtt;
	
	if(Iout_RATE_YAW>= I_LIMIT){
		Iout_RATE_YAW = I_LIMIT;
	}
	else if(Iout_RATE_YAW <= -I_LIMIT){
		Iout_RATE_YAW = -I_LIMIT;
	};
	
	if(measuredyawangle < I_RESET_LIMIT && measuredyawangle > -1*I_RESET_LIMIT){
		Iout_RATE_YAW = 0;
	}
	
	Dout_RATE_YAW = 0.05*(inter1 - pre_inter1_YAW)*dtt + 0.95*Dout_RATE_YAW;
	
	inter2 = Pout + Iout_RATE_YAW + Dout_RATE_YAW;
	
	if(inter2 >= RATE_LIMIT){
		inter2 = RATE_LIMIT;
	}
	
	else if(inter2 <= -RATE_LIMIT){
		inter2 = -RATE_LIMIT;
	};
	
	output = inter2;
	pre_inter1_YAW = inter1;
	
	hal.uartB->print("YAW_RATE_OUT: ");
	hal.uartB->println(output);
	
	return output;
	
};

float Stable::ALT_RATE(float bodyframeZ, float dtt, float ACCEL_THR_P, float ACCEL_THR_I, float ACCEL_THR_D, float RATE_THR_IMAX, float THR_LIMIT){
	
	float inter1;
	float inter2;
	float output;
	float error;
	
	float Pout;
	
	inter1 = bodyframeZ - K*altitudeaccel;
	
	
	Pout = inter1*ACCEL_THR_P;
	Iout_ACCEL_DIST = inter1*ACCEL_THR_I*dtt;
	
	if(Iout_ACCEL_DIST>= RATE_THR_IMAX){
		Iout_ACCEL_DIST = RATE_THR_IMAX;
	}
	else if(Iout_ACCEL_DIST <= -RATE_THR_IMAX){
		Iout_ACCEL_DIST = -RATE_THR_IMAX;
	};
	
	Dout_ACCEL_DIST = 0.05*(inter1 - pre_inter1_ACCEL)*dtt + 0.95*Dout_ACCEL_DIST;
	
	inter2 = Pout + Iout_ACCEL_DIST + Dout_ACCEL_DIST;
	
	if(inter2 >= THR_LIMIT){
		inter2 = THR_LIMIT;
	}
	
	else if(inter2 <= -THR_LIMIT){
		inter2 = -THR_LIMIT;
	};
	
	output = inter2;
	pre_inter1_ACCEL = inter1;
	
	return output;
	
};

float Stable::Roll_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt,float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float RLL_RATE_P, float RLL_RATE_I, float RLL_RATE_D){
	
	float RLL_STB_Out =  RLL_STB(rolldesired, RLL_STB_P, STB_LIMIT);
	float PTCH_STB_Out = PTCH_STB(pitchdesired, PTCH_STB_P, STB_LIMIT);
	float YAW_STB_Out = YAW_STB(yawdesired, YAW_STB_P, STB_LIMIT);
	
	//float bodyframeX = earthToBodyX(RLL_STB_Out, PTCH_STB_Out, YAW_STB_Out);
	float bodyframeX = RLL_STB_Out;
	
	float roll_to_matrix = RLL_RATE(bodyframeX, dtt, RLL_RATE_P, RLL_RATE_I, RLL_RATE_D, I_LIMIT, RATE_LIMIT, I_RESET_LIMIT);
	
	return roll_to_matrix;
};

float Stable::Pitch_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt, float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float PTCH_RATE_P, float PTCH_RATE_I, float PTCH_RATE_D){
	
	float RLL_STB_Out =  RLL_STB(rolldesired, RLL_STB_P, STB_LIMIT);
	float PTCH_STB_Out = PTCH_STB(pitchdesired, PTCH_STB_P, STB_LIMIT);
	float YAW_STB_Out = YAW_STB(yawdesired, YAW_STB_P, STB_LIMIT);
	
	//float bodyframeY = earthToBodyY(RLL_STB_Out, PTCH_STB_Out, YAW_STB_Out);
	float bodyframeY = PTCH_STB_Out;
	
	float pitch_to_matrix = PTCH_RATE(bodyframeY, dtt, PTCH_RATE_P, PTCH_RATE_I, PTCH_RATE_D, I_LIMIT, RATE_LIMIT, I_RESET_LIMIT);

	return pitch_to_matrix;
};

float Stable::Yaw_Controller(float rolldesired,float pitchdesired, float yawdesired, float dtt, float I_LIMIT, float I_RESET_LIMIT, float STB_LIMIT, float RATE_LIMIT, float RLL_STB_P, float PTCH_STB_P, float YAW_STB_P, float YAW_RATE_P, float YAW_RATE_I, float YAW_RATE_D){
	
	float RLL_STB_Out =  RLL_STB(rolldesired, RLL_STB_P, STB_LIMIT);
	float PTCH_STB_Out = PTCH_STB(pitchdesired,PTCH_STB_P, STB_LIMIT);
	float YAW_STB_Out = YAW_STB(yawdesired, YAW_STB_P, STB_LIMIT);
	
	//float bodyframeZ = earthToBodyZ(RLL_STB_Out, PTCH_STB_Out, YAW_STB_Out);
	float bodyframeZ = YAW_STB_Out;
	
	float yaw_to_matrix = YAW_RATE(bodyframeZ, dtt, YAW_RATE_P, YAW_RATE_I, YAW_RATE_D, I_LIMIT, RATE_LIMIT, I_RESET_LIMIT);
	
	return yaw_to_matrix;
};


float Stable::Altitude_Controller(float desired, float dtt, float THR_ALT_P, float DIST_LIMIT, float RATE_THR_IMAX, float g_LIMIT, float RATE_THR_P, float RATE_THR_I, float RATE_THR_D, float ACCEL_THR_P, float ACCEL_THR_I, float ACCEL_THR_D, float THR_LIMIT){
	
	float ALT_STB_OUT = ALT_STB(desired, THR_ALT_P, DIST_LIMIT, RATE_THR_IMAX, g_LIMIT, RATE_THR_P, RATE_THR_I, RATE_THR_D, dtt);
	
	float bodyframeZ = earthToBodyZ(0, 0, ALT_STB_OUT);
	
	float alt_to_matrix = ALT_RATE(bodyframeZ, dtt, ACCEL_THR_P, ACCEL_THR_I, ACCEL_THR_D, RATE_THR_IMAX, THR_LIMIT);
	
	return alt_to_matrix;
};


void Stable::CONTROL_ALLOCATION (float ALT_IN, float RLL_IN, float PTCH_IN, float YAW_IN, float b, float l, float d){
	
	float w12 = -ALT_IN*l + sqrt(2) * (RLL_IN + PTCH_IN) + YAW_IN*b*l/d;
	float w22 = -ALT_IN*l - sqrt(2) * (PTCH_IN - RLL_IN) - YAW_IN*b*l/d;
	float w32 = -ALT_IN*l - sqrt(2) * (RLL_IN + PTCH_IN) + YAW_IN*b*l/d;
	float w42 = -ALT_IN*l - sqrt(2) * (RLL_IN - PTCH_IN) - YAW_IN*b*l/d;
	
	North = sqrt(w12);
	South = sqrt(w32);
	East = sqrt(w22);
	West = sqrt(w42);
	
// 	North = w12;
// 	South = w32;
// 	East = w22;
// 	West = w42;
	
};

float Stable::returnNorth(){
	return North;
};

float Stable::returnSouth(){
	return South;
};

float Stable::returnEast(){
	return East;
};

float Stable::returnWest(){
	return West;
};