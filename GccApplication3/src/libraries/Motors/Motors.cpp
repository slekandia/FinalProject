/*
 * Motors.cpp
 *
 * Created: 12/12/2015 9:01:52 PM
 *  Author: Mert
 */ 


#include "Motors.h"
#include "libraries/PID/PID_RPY.h"
#include "libraries/AP_HAL/AP_HAL.h"
#include "libraries/AP_HAL_Empty/AP_HAL_Empty.h"

extern const AP_HAL::HAL& hal;


void Motors::motors_init() {
	static tc_waveform_opt_t WAVEFORM_OPT0;
	static tc_waveform_opt_t WAVEFORM_OPT1;
	static tc_waveform_opt_t WAVEFORM_OPT2;	
	static tc_waveform_opt_t WAVEFORM_OPT3;

	static tc_interrupt_t TC_INTERRUPT;
	gpio_enable_module_pin(57,0); //TC0_A2
	gpio_enable_module_pin(AVR32_TC0_B0_2_PIN,AVR32_TC0_B0_2_FUNCTION);
	
	gpio_enable_module_pin(AVR32_TC0_A1_0_2_PIN,AVR32_TC0_A1_2_FUNCTION);
	gpio_enable_module_pin(AVR32_TC0_B1_0_0_PIN,AVR32_TC0_B1_1_FUNCTION);
	
		
	WAVEFORM_OPT0.channel  = 0;									// Channel selection.


	WAVEFORM_OPT0.bswtrg   = TC_EVT_EFFECT_NOOP;                // Software trigger effect on TIOB.
	WAVEFORM_OPT0.beevt    = TC_EVT_EFFECT_NOOP;                // External event effect on TIOB.
	WAVEFORM_OPT0.bcpc     = TC_EVT_EFFECT_CLEAR;               // RC compare effect on TIOB.
	WAVEFORM_OPT0.bcpb     = TC_EVT_EFFECT_SET;                 // RB compare effect on TIOB.

	WAVEFORM_OPT0.aswtrg   = TC_EVT_EFFECT_NOOP;               // Software trigger effect on TIOA.
	WAVEFORM_OPT0.aeevt    = TC_EVT_EFFECT_NOOP;               // External event effect on TIOA.
	WAVEFORM_OPT0.acpc     = TC_EVT_EFFECT_NOOP;              // RC compare effect on TIOA: toggle.
	WAVEFORM_OPT0.acpa     = TC_EVT_EFFECT_NOOP;                // RA compare effect on TIOA: toggle (other possibilities are none; set and clear).

	WAVEFORM_OPT0.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER;// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	WAVEFORM_OPT0.enetrg   = false;                             // External event trigger enable.
	WAVEFORM_OPT0.eevt     = 1;                                 // External event selection.
	WAVEFORM_OPT0.eevtedg  = TC_SEL_NO_EDGE;                    // External event edge selection.
	WAVEFORM_OPT0.cpcdis   = false;                             // Counter disable when RC compare.
	WAVEFORM_OPT0.cpcstop  = false;                             // Counter clock stopped with RC compare.

	WAVEFORM_OPT0.burst    = false;                             // Burst signal selection.
	WAVEFORM_OPT0.clki     = false;                             // Clock inversion.
	WAVEFORM_OPT0.tcclks   = TC_CLOCK_SOURCE_TC3;               // Internal source clock 3, connected to fPBA / 8.
	
	
	WAVEFORM_OPT1.channel  = 1;									// Channel selection.


	WAVEFORM_OPT1.bswtrg   = TC_EVT_EFFECT_NOOP;                // Software trigger effect on TIOB.
	WAVEFORM_OPT1.beevt    = TC_EVT_EFFECT_NOOP;                // External event effect on TIOB.
	WAVEFORM_OPT1.bcpc     = TC_EVT_EFFECT_CLEAR;               // RC compare effect on TIOB.
	WAVEFORM_OPT1.bcpb     = TC_EVT_EFFECT_SET;                 // RB compare effect on TIOB.

	WAVEFORM_OPT1.aswtrg   = TC_EVT_EFFECT_NOOP;               // Software trigger effect on TIOA.
	WAVEFORM_OPT1.aeevt    = TC_EVT_EFFECT_NOOP;               // External event effect on TIOA.
	WAVEFORM_OPT1.acpc     = TC_EVT_EFFECT_CLEAR;              // RC compare effect on TIOA: toggle.
	WAVEFORM_OPT1.acpa     = TC_EVT_EFFECT_SET;                // RA compare effect on TIOA: toggle (other possibilities are none; set and clear).

	WAVEFORM_OPT1.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER;// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	WAVEFORM_OPT1.enetrg   = false;                             // External event trigger enable.
	WAVEFORM_OPT1.eevt     = 1;                                 // External event selection.
	WAVEFORM_OPT1.eevtedg  = TC_SEL_NO_EDGE;                    // External event edge selection.
	WAVEFORM_OPT1.cpcdis   = false;                             // Counter disable when RC compare.
	WAVEFORM_OPT1.cpcstop  = false;                             // Counter clock stopped with RC compare.

	WAVEFORM_OPT1.burst    = false;                             // Burst signal selection.
	WAVEFORM_OPT1.clki     = false;                             // Clock inversion.
	WAVEFORM_OPT1.tcclks   = TC_CLOCK_SOURCE_TC3;               // Internal source clock 3, connected to fPBA / 8.
	
	
	WAVEFORM_OPT2.channel  = 2;									// Channel selection.

	WAVEFORM_OPT2.bswtrg   = TC_EVT_EFFECT_NOOP;                // Software trigger effect on TIOB.
	WAVEFORM_OPT2.beevt    = TC_EVT_EFFECT_NOOP;                // External event effect on TIOB.
	WAVEFORM_OPT2.bcpc     = TC_EVT_EFFECT_NOOP;                // RC compare effect on TIOB.
	WAVEFORM_OPT2.bcpb     = TC_EVT_EFFECT_NOOP;                // RB compare effect on TIOB.

	WAVEFORM_OPT2.aswtrg   = TC_EVT_EFFECT_NOOP;               // Software trigger effect on TIOA.
	WAVEFORM_OPT2.aeevt    = TC_EVT_EFFECT_NOOP;               // External event effect on TIOA.
	WAVEFORM_OPT2.acpc     = TC_EVT_EFFECT_CLEAR;              // RC compare effect on TIOA: toggle.
	WAVEFORM_OPT2.acpa     = TC_EVT_EFFECT_SET;                // RA compare effect on TIOA: toggle (other possibilities are none; set and clear).

	WAVEFORM_OPT2.wavsel   = TC_WAVEFORM_SEL_UP_MODE_RC_TRIGGER;// Waveform selection: Up mode with automatic trigger(reset) on RC compare.
	WAVEFORM_OPT2.enetrg   = false;                             // External event trigger enable.
	WAVEFORM_OPT2.eevt     = 1;                                 // External event selection.
	WAVEFORM_OPT2.eevtedg  = TC_SEL_NO_EDGE;                    // External event edge selection.
	WAVEFORM_OPT2.cpcdis   = false;                             // Counter disable when RC compare.
	WAVEFORM_OPT2.cpcstop  = false;                             // Counter clock stopped with RC compare.

	WAVEFORM_OPT2.burst    = false;                             // Burst signal selection.
	WAVEFORM_OPT2.clki     = false;                             // Clock inversion.
	WAVEFORM_OPT2.tcclks   = TC_CLOCK_SOURCE_TC3;               // Internal source clock 3, connected to fPBA / 8.	
	
	
	
	
	tc_init_waveform(&AVR32_TC0, &WAVEFORM_OPT0);
	tc_write_rc(&AVR32_TC0,0,30000);
	tc_write_ra(&AVR32_TC0,0,27900);
	tc_write_rb(&AVR32_TC0,0,27900);
	tc_start(&AVR32_TC0,0);

	
	tc_init_waveform(&AVR32_TC0, &WAVEFORM_OPT1);
	tc_write_rc(&AVR32_TC0,1,30000);
	tc_write_ra(&AVR32_TC0,1,27900);
	tc_write_rb(&AVR32_TC0,1,27900);
	tc_start(&AVR32_TC0,1);
	
	tc_init_waveform(&AVR32_TC0, &WAVEFORM_OPT2);
	tc_write_rc(&AVR32_TC0,2,30000);
	tc_write_ra(&AVR32_TC0,2,27900);
	tc_write_rb(&AVR32_TC0,2,27900);
	tc_start(&AVR32_TC0,2);	
	
}



//North Motor, TC0-B0
void Motors::motor1_update(int percent, float pid_pitch, float pid_yaw, float pid_alt)
//void Motors::motor1_update(float percent_manual, float RPSin_N) 
{
	int r;
	float k;
	k = (2000-1050)*0.01*(percent)+1050 - pid_pitch-pid_yaw + pid_alt;

// 	float percent_added = (RPSin_N-27.77)/1.389;
// 	
// 	float percent_turn = percent_manual + percent_added;
	
// 	if(percent_turn >= 100){
// 		percent_turn = 100;
// 	}
// 	
// 	if(percent_turn < 0){
// 		percent_turn = 0;
// 	}
// 
// 	
// 	k = (2000-1050)*0.01*(percent_turn)+1050;
// 	
// 	hal.uartB->print("North: ");
// 	hal.uartB->println(percent_turn);	
	
	r = (30000 - 2*k);
	
// 	if(r < 26100){
// 		r = 26100;
// 	}
	
	if(r <= 27900 && r >= 26000){
		tc_write_rb(&AVR32_TC0,0,r);
	}
	else if(r < 26000)
	tc_write_rb(&AVR32_TC0,0,26000);
	else if(r >27900)
	tc_write_rb(&AVR32_TC0,0,27900);
}

//East Motor
void Motors::motor2_update(int percent, float pid_roll, float pid_yaw, float pid_alt)
//void Motors::motor2_update(float percent_manual, float RPSin_E)
{
	int r;
	float k;
	k = (2000-1050)*0.01*(percent)+1050 + pid_roll +pid_yaw + pid_alt;
	
// 	float percent_added = (RPSin_E-27.77)/1.389;
// 	k = (2000-1050)*0.01*(percent_manual + percent_added)+1050;
	
// 	hal.uartB->print("East: ");
// 	hal.uartB->println(k);
	r = (30000 - 2*k);
// 	if(r < 26100){
// 		r = 26100;
// 	}
	if(r <= 27900 && r >= 26000)
	{
		tc_write_ra(&AVR32_TC0,1,r);
	}
	else if(r < 26000)
	tc_write_ra(&AVR32_TC0,1,26000);
	else if(r > 27900)
	tc_write_ra(&AVR32_TC0,1,27900);
}

//South Motor,TC0-A2
void Motors::motor3_update(int percent, float pid_pitch, float pid_yaw, float pid_alt)
//void Motors::motor3_update(float percent_manual, float RPSin_S)
{
	int r;
	float k;
	k = (2000-1050)*0.01*(percent)+1050 + pid_pitch-pid_yaw + pid_alt;
	
// 	float percent_added = (RPSin_S-27.77)/1.389;
// 	
// 	float percent_turn = percent_manual + percent_added;
// 	
// 	if(percent_turn >= 100){
// 		percent_turn = 100;
// 	}
// 	
// 	if(percent_turn < 0){
// 		percent_turn = 0;
// 	}
// 
// 	
// 	k = (2000-1050)*0.01*(percent_turn)+1050;
// 	
// 
// 	
// 	hal.uartB->print("South: ");
// 	hal.uartB->println(percent_turn);

	r = (30000 - 2*k);
// 	if(r < 26100){
// 		r = 26100;
// 	}
	if(r <= 27900 && r >= 26000)
	{
		tc_write_ra(&AVR32_TC0,2,r);
	}
	else if(r < 26000)
	tc_write_ra(&AVR32_TC1,2,26000);
	else if(r >27900)
	tc_write_ra(&AVR32_TC1,2,27900);
}

//West Motor
void Motors::motor4_update(int percent, float pid_roll, float pid_yaw, float pid_alt)
//void Motors::motor4_update(float percent_manual, float RPSin_S)
{
	int r;
	float k;
	k = (2000-1050)*0.01*(percent)+1050 -pid_roll+pid_yaw + pid_alt;	
	
// 	float percent_added = (RPSin_W-27.77)/1.389;
// 	k = (2000-1050)*0.01*(percent_manual + percent_added)+1050;
	
// 	hal.uartB->print("West: ");
// 	hal.uartB->println(k);
	r = (30000 - 2*k);
// 	if(r < 26100){
// 		r = 26100;
// 	}	

	if(r <= 27900 && r >= 26000)
	{
		tc_write_rb(&AVR32_TC0,1,r);
	}
	else if(r < 26000)
	tc_write_rb(&AVR32_TC0,1,26000);
	else if(r > 27900)
	tc_write_rb(&AVR32_TC0,1,27900);
}

// void Motors::set_throttle(int percent, float pid_pitch, float pid_roll, float pid_yaw, float pid_alt) {
// 	motor1_update(percent, pid_roll, pid_yaw, pid_alt);
// 	motor2_update(percent,pid_pitch, pid_yaw, pid_alt);
// 	motor3_update(percent, pid_roll, pid_yaw, pid_alt);
// 	motor4_update(percent, pid_pitch, pid_yaw, pid_alt);
// }

void Motors::set_throttle(int percent_manual, float RPSin_N, float RPSin_E, float RPSin_S, float RPSin_W) {
// 	motor1_update(percent_manual, RPSin_N);
// 	motor2_update(percent_manual, RPSin_E);
// 	motor3_update(percent_manual, RPSin_S);
// 	motor4_update(percent_manual, RPSin_W);
}

void Motors::kill_motors() {
	
	tc_write_ra(&AVR32_TC0,0,30000);
	tc_write_rb(&AVR32_TC0,0,30000);
	tc_write_ra(&AVR32_TC0,1,30000);
	tc_write_rb(&AVR32_TC0,1,30000);
	tc_write_ra(&AVR32_TC0,2,30000);
	tc_write_rb(&AVR32_TC0,2,30000);

	
}