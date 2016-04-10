extern "C" {
	#include <asf.h>
};

#include "libraries/PID/PID_RPY.h"


class Motors {
	public:
	Motors() {

	};
	unsigned short percent;
	
	void motors_init();
	void motor1_update(int percent, float pid_pitch, float pid_yaw, float pid_alt);
	void motor2_update(int percent, float pid_pitch, float pid_yaw, float pid_alt);
	void motor3_update(int percent, float pid_pitch, float pid_yaw, float pid_alt);
	void motor4_update(int percent, float pid_pitch, float pid_yaw, float pid_alt);
	
	void set_throttle(int percent_manual, float RPSin_N, float RPSin_E, float RPSin_S, float RPSin_W);
	void kill_motors();
};
