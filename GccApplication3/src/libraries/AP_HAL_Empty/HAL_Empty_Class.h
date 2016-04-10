
#ifndef __AP_HAL_EMPTY_CLASS_H__
#define __AP_HAL_EMPTY_CLASS_H__

#include "libraries/AP_HAL/AP_HAL.h"

#include "libraries/AP_HAL_Empty/AP_HAL_Empty_Namespace.h"

class Empty::HAL_Empty : public AP_HAL::HAL {
public:
    HAL_Empty();
    void init(int argc, char * const * argv) const;

};

extern const Empty::HAL_Empty AP_HAL_Empty;

#endif // __AP_HAL_EMPTY_CLASS_H__

