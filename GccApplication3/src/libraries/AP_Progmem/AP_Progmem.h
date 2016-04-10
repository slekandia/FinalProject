
#ifndef __AP_PROGMEM_H__
#define __AP_PROGMEM_H__

#include "libraries/AP_HAL/AP_HAL_Boards.h"

#include "libraries/AP_Progmem/AP_Progmem_Identity.h"

#define PROGMEM_STRING(_v, _s)  static const char _v[] PROGMEM = _s

#endif // __AP_PROGMEM_H__

