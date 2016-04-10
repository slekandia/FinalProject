/*
 * Rangefinder.h
 *
 * Created: 12/12/2015 11:38:14 PM
 *  Author: alpombeo
 */ 


#ifndef RANGEFINDER_H_
#define RANGEFINDER_H_

#include "libraries/AP_HAL/AP_HAL.h"
#include "libraries/AP_HAL_Empty/AP_HAL_Empty.h"

#include "libraries/AP_HAL_Empty/AnalogIn.h"

extern "C" {
	#include <asf.h>
};

class Rangefinder{
	public:
	void init();
	void update();
	float getRange_Latest(int chnumber);
	
	float getRange_Average(int chnumber);
	
	float getRange_Ratio(int chnumber);
	
	
	AP_HAL::AnalogSource *_adc_source1;
	AP_HAL::AnalogSource *_adc_source2;
	AP_HAL::AnalogSource *_adc_source3;
	AP_HAL::AnalogSource *_adc_source4;
	AP_HAL::AnalogSource *_adc_source5;
	AP_HAL::AnalogSource *_adc_source6;
	AP_HAL::AnalogSource *_adc_source7;
	AP_HAL::AnalogSource *_adc_source8;
	AP_HAL::AnalogSource *_adc_source9;
	AP_HAL::AnalogSource *_adc_source10;
	AP_HAL::AnalogSource *_adc_source11;
	AP_HAL::AnalogSource *_adc_source12;
	AP_HAL::AnalogSource *_adc_source13;
	AP_HAL::AnalogSource *_adc_source14;
	AP_HAL::AnalogSource *_adc_source15;
	//AP_HAL::AnalogSource *_adc_source16;
	AP_HAL::AnalogSource *_sourcearray [15];
	
};

#endif /* RANGEFINDER_H_ */