/*
 * Rangefinder.cpp
 *
 * Created: 12/12/2015 11:38:27 PM
 *  Author: alpombeo
 */ 

#include "Rangefinder.h"
#include "libraries/AP_HAL_Empty/AnalogIn.h"

extern const AP_HAL::HAL& hal;


void Rangefinder::init()
{
	
 	hal.analogin->init();

	
	_adc_source1 = hal.analogin->channel(1);
	_adc_source2 = hal.analogin->channel(2);
	_adc_source3 = hal.analogin->channel(3);
	_adc_source4 = hal.analogin->channel(4);
	_adc_source5 = hal.analogin->channel(5);
	_adc_source6 = hal.analogin->channel(6);
	_adc_source7 = hal.analogin->channel(7);
	_adc_source8 = hal.analogin->channel(8);
	_adc_source9 = hal.analogin->channel(9);
	_adc_source10 = hal.analogin->channel(10);
	_adc_source11 = hal.analogin->channel(11);
	_adc_source12 = hal.analogin->channel(12);
	_adc_source13 = hal.analogin->channel(13);
	_adc_source14 = hal.analogin->channel(14);
	_adc_source15 = hal.analogin->channel(15);
	//_adc_source16 = hal.analogin->channel(16);
	
	_sourcearray [0] = _adc_source1;
	_sourcearray [1] = _adc_source2;
	_sourcearray [2] = _adc_source3;
	_sourcearray [3] = _adc_source4;
	_sourcearray [4] = _adc_source5;
	_sourcearray [5] = _adc_source6;
	_sourcearray [6] = _adc_source7;
	_sourcearray [7] = _adc_source8;
	_sourcearray [8] = _adc_source9;
	_sourcearray [9] = _adc_source10;
	_sourcearray [10] = _adc_source11;
	_sourcearray [11] = _adc_source12;
	_sourcearray [12] = _adc_source13;
	_sourcearray [13] = _adc_source14;
	_sourcearray [14] = _adc_source15;
	//_sourcearray [15] = _adc_source16;

}

float Rangefinder::getRange_Latest(int chnumber){
	float voltage_reading = _sourcearray[chnumber]->voltage_latest();
	return voltage_reading*259.183673/127;
}

float Rangefinder::getRange_Average(int chnumber){
	float voltage_reading = _sourcearray[chnumber]->voltage_average();
	return voltage_reading*259.183673/127;
}

float Rangefinder::getRange_Ratio(int chnumber){
	float voltage_reading = _sourcearray[chnumber]->voltage_average_ratiometric();
	return voltage_reading*259.183673/127;
}


void Rangefinder::update() {
	hal.analogin->_timer_event();
}