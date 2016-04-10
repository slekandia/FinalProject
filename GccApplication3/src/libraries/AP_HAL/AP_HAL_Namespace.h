
#ifndef __AP_HAL_NAMESPACE_H__
#define __AP_HAL_NAMESPACE_H__

#include "string.h"

namespace AP_HAL {

    /* Toplevel pure virtual class Hal.*/
    class HAL;
    /* Toplevel class names for drivers: */
    class UARTDriver;
    class Scheduler;
	class AnalogSource;
	class AnalogIn;
    
 

    /* Utility Classes */
	    class Print;
	    class Stream;
	    class BetterStream;


    /* Typdefs for function pointers (Procedure, Member Procedure) 

       For member functions we use the FastDelegate delegates class
       which allows us to encapculate a member function as a type
     */
    typedef void(*Proc)(void);

    /**
     * Global names for all of the existing SPI devices on all platforms.
     */

}

// macro to hide the details of AP_HAL::MemberProc

#endif // __AP_HAL_NAMESPACE_H__
