
#ifndef __AP_HAL_EMPTY_NAMESPACE_H__
#define __AP_HAL_EMPTY_NAMESPACE_H__

/* While not strictly required, names inside the Empty namespace are prefixed
 * with Empty for clarity. (Some of our users aren't familiar with all of the
 * C++ namespace rules.)
 */

namespace Empty {
	class HAL_Empty;
    class EmptyUARTDriver;
    class EmptyScheduler;
	class EmptyUtil;
	class EmptyAnalogSource;
	class AVRAnalogIn;
	class AnalogSource;

}

#endif // __AP_HAL_EMPTY_NAMESPACE_H__

