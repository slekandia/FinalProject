extern "C"{
	#include <avr32/io.h>
	#include <asf.h>
	#include "board.h"
	#include <math.h>
	#if UC3C // UC3C uses the TWIM module
	#include "twim.h"
	#else // default use the TWI module
	#include "eic.h"
	#include "twi.h"
	#endif
	#include "libraries/DataFlash/Dataflash.h"
	#include "gpio.h"
	#include "cycle_counter.h"
	#include "flashc.h"
	#include "MPU9150.h"
	#include "at42qt1060.h"
	#include "conf_at42qt1060.h"
	#include "libraries/AHRS/DCM.h"
	#include <stdio.h>
};

#include "libraries/PID/Stable.h"
#include "libraries/PID/hakki_controller.h"
#include "libraries/AP_HAL/AP_HAL.h"
#include "libraries/AP_HAL_Empty/AP_HAL_Empty.h"
#include "libraries/PID/PID_RPY.h"
#include "libraries/GPS/GPS.h"
#include "libraries/Motors/Motors.h"

#if BOARD == UC3C_EK
#define FCPU_HZ		16000000
#define FPBA_HZ		16000000
#else
#define FCPU_HZ		16000000
#define FPBA_HZ		16000000
#endif

const AP_HAL::HAL& hal = AP_HAL_Empty;

static pcl_freq_param_t pcl_freq_param;
static void init_sys_clocks(void)
{
	pcl_freq_param.cpu_f        = FCPU_HZ;
	pcl_freq_param.pba_f        = FPBA_HZ;
	pcl_freq_param.osc0_f       = FOSC0;
	pcl_freq_param.osc0_startup = OSC0_STARTUP;
	// Configure system clocks.
	if (pcl_configure_clocks(&pcl_freq_param) != PASS)
	while(1);
}


Motors motor;
Stable stable;
GPS gps;
MPU9150 mpu9150;
	float PID_Pitch;

// main function
int main(void) {
	t_cpu_time timeout;
	t_cpu_time timeout_mpu;
	float deltat_2;
	float roll_first, pitch_first;
	
	board_init();
	hal.init(0,NULL);
	hal.analogin->init();
	
	sysclk_init();
	init_sys_clocks();
	float initials[3];
	int16_t  magnetic[3];
	float yaw_first;
	float soft[3],hard[3];
	float desti[2];
	float kp = 4.8;
	float kpp = 0.7;
	float kd = 0;
  	mpu9150.initMPU9150(FCPU_HZ);
	mpu9150.initHMC58();
	mpu9150.calibrate_mpu9150(initials);

	int16_t myMagData[3];
	hal.uartB->println("NEE ZAFERINDEN BAHSEDIYORSUUN");
	
	
	float percent = 40;
	uint8_t c, last_c;
	uint16_t count = 0;
	cpu_set_timeout(cpu_ms_2_cy(10, FOSC0), &timeout_mpu);
	cpu_set_timeout(cpu_ms_2_cy(1000, FOSC0), &timeout);

	hal.uartB->println("VER COSKUYU");
	c = hal.uartB->read();
	motor.motors_init();

while(1){	 
	  
	if (usart_test_hit(&AVR32_USART4)) {
		hal.uartB->println("I am hit");
		last_c = c;

		c = hal.uartB->read();

		if(c == '9') {
			motor.kill_motors();
			hal.uartB->println("KIRDIN BENI GOD DAMNIT");
			while(1);
		}
		if (c == '8') {
			percent += 1;
			hal.uartB->print("Percent Increased to: ");
			hal.uartB->println(percent);
		}
		if (c == '2') {
			percent -= 1;
			hal.uartB->print("Percent Decreased to: ");
			hal.uartB->println(percent);
		}
		if (c == 'u') {
			kpp = kpp + 0.1;
			hal.uartB->print("Kpp is: ");
			hal.uartB->println(kpp);
		}
		if (c == 'j') {
			kpp = kpp - 0.1;
			hal.uartB->print("Kpp is: ");
			hal.uartB->println(kpp);
		}
		if (c == 't') {
			kd = kd + 0.001;
			hal.uartB->print("Kd is: ");
			hal.uartB->println(kd);
		}
		if (c == 'g') {
			kd = kd - 0.001;
			hal.uartB->print("Kd is: ");
			hal.uartB->println(kd);
		}		
		
		if (c == 'y') {
			kp = kp + 0.1;
			hal.uartB->print("Kp is: ");
			hal.uartB->println(kp);
		}
		if (c == 'h') {
			kp = kp - 0.1;
			hal.uartB->print("Kp is: ");
			hal.uartB->println(kp);
		}
		c = last_c;
	}

	if (c == '5') {		
// 		
		if(cpu_is_timeout(&timeout_mpu)) {
			
			cpu_stop_timeout(&timeout_mpu);
			mpu9150.update();
			PID_Pitch = Pitch_Controller(pitch,initials[1],kp,kpp,0,kd,0.01f);
			motor.motor1_update(percent,PID_Pitch,0,0);
			motor.motor3_update(percent,PID_Pitch,0,0);
			cpu_set_timeout(cpu_ms_2_cy(10, FOSC0), &timeout_mpu);
		}

// 		if(cpu_is_timeout(&timeout)) {
// 			cpu_stop_timeout(&timeout);
// 			hal.uartB->println(PID_Pitch);
// 			
// 			cpu_set_timeout(cpu_ms_2_cy(1000, FOSC0), &timeout);
// 		}

// 		float PID_Pitch = Pitch_Controller(pitch,initials[1],kp,kd,0,0,deltat);
// 		motor.motor1_update(percent,PID_Pitch,0,0);
		
// 		deltat =(float) cpu_cy_2_ms((Get_sys_count()),FOSC0)/1000;
//  		cpu_set_timeout( cpu_ms_2_cy(10000, FOSC0), &timeout);
//  		Set_sys_count(0);
		}
	}
}


