
#ifndef __AP_HAL_Empty_UTIL_H__
#define __AP_HAL_Empty_UTIL_H__

#include "libraries/AP_HAL/AP_HAL.h"
#include "AP_HAL_Empty_Namespace.h"
#include "memcheck.h"

class Empty::EmptyUtil : public AP_HAL::Util {
public:
    bool run_debug_shell(AP_HAL::BetterStream *stream) { return false; }
    uint16_t available_memory(void) { return memcheck_available_memory(); }
};

