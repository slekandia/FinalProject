
#ifndef __AP_HAL_HAL_H__
#define __AP_HAL_HAL_H__

#include "AP_HAL_Namespace.h"
#include "AnalogIn.h"
#include "UARTDriver.h"

class AP_HAL::HAL {
	public:
	HAL(AP_HAL::UARTDriver* _console,
	AP_HAL::UARTDriver* _uartB, // 1st GPS
	AP_HAL::UARTDriver* _uartE, // 2nd GPS
	AP_HAL::Scheduler*  _scheduler,
	AP_HAL::AnalogIn* _analogin
	)
	:
	console(_console),
	uartB(_uartB),
	uartE(_uartE),
	scheduler(_scheduler),
	analogin(_analogin)
	
	{}

	virtual void init(int argc, char * const argv[]) const = 0;

	AP_HAL::UARTDriver* console;
	AP_HAL::UARTDriver* uartB;
	AP_HAL::UARTDriver* uartE;
	AP_HAL::Scheduler*  scheduler;
	AP_HAL::AnalogIn*   analogin;
	

};

#endif // __AP_HAL_HAL_H__

