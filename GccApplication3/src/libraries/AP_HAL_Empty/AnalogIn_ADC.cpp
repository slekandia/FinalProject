#include "AnalogIn.h"

using namespace Empty;

extern const AP_HAL::HAL& hal;
extern "C" {
	#include <asf.h>
};
//bu tamam, gpio'dan ANALOG_INPUT_NONE de?i?tirsen yeter
EmptyAnalogSource::EmptyAnalogSource(uint8_t pin) :
    _sum_count(0),
    _sum(0),
    _last_average(0),
    _pin(ANALOG_INPUT_NONE),
    _stop_pin(ANALOG_INPUT_NONE),
    _settle_time_ms(0)
{
    set_pin(pin);
}

//bu da tamam gpio'dan ANALOG_INPUT_BOARD_VCC de?i?tirsen yeter
float EmptyAnalogSource::read_average() {
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        uint16_t v = (uint16_t) _read_average();
        return 1126400UL / v;
    } else {
        return _read_average();
    }
}
//cli ne cli??? sregi de o de?i?tiriyor san?rsam
float EmptyAnalogSource::read_latest() {
	uint16_t latest = _latest;
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return 1126400UL / latest;
    } else {
        return latest;
    }
}

/*
  return voltage from 0.0 to 5.0V, scaled to Vcc
 */
float EmptyAnalogSource::voltage_average(void)
{
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_average();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
	//ASAGIYI DEGISTIR GERCEK BOARDDA
   // if (vcc_mV < 4000) {
     //   vcc_mV = 4000;
   // } else if (vcc_mV > 6000) {
      //  vcc_mV = 6000;
 //   }
	vcc_mV = 1000;
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

/*
  return voltage from 0.0 to 5.0V, scaled to Vcc
 */
float EmptyAnalogSource::voltage_latest(void)
{
    if (_pin == ANALOG_INPUT_BOARD_VCC) {
        return read_latest() * 0.001f;
    }
    float vcc_mV = hal.analogin->channel(ANALOG_INPUT_BOARD_VCC)->read_average();
    float v = read_latest();
    // constrain Vcc reading so that a bad Vcc doesn't throw off
    // the reading of other sources too badly
    if (vcc_mV < 4000) {
        vcc_mV = 4000;
    } else if (vcc_mV > 6000) {
        vcc_mV = 6000;
    }
	
	vcc_mV = 1000;
    return v * vcc_mV * 9.765625e-7; // 9.765625e-7 = 1.0/(1024*1000)
}

/*
  return voltage from 0.0 to 5.0V, assuming a ratiometric sensor. This
  means the result is really a pseudo-voltage, that assumes the supply
  voltage is exactly 5.0V.
 */
float EmptyAnalogSource::voltage_average_ratiometric(void)
{
    float v = read_average();
    return v * (5.0f / 1023.0f);
}

void EmptyAnalogSource::set_pin(uint8_t pin) {
    if (pin != _pin) {
        _sum = 0;
        _sum_count = 0;
        _last_average = 0;
        _latest = 0;
        _pin = pin;
    }
}

void EmptyAnalogSource::set_stop_pin(uint8_t pin) {
    _stop_pin = pin;
}

void EmptyAnalogSource::set_settle_time(uint16_t settle_time_ms) 
{
    _settle_time_ms = settle_time_ms;
}

/* read_average is called from the normal thread (not an interrupt). */
float EmptyAnalogSource::_read_average() {
    uint16_t sum;
    uint8_t sum_count;

    if (_sum_count == 0) {
        // avoid blocking waiting for new samples
        return _last_average;
    }

    /* Read and clear in a critical section */
    
    sum = _sum;
    sum_count = _sum_count;
    _sum = 0;
    _sum_count = 0;

    float avg = sum / (float) sum_count;

    _last_average = avg;
    return avg;
}


void EmptyAnalogSource::stop_read() {
    
}

bool EmptyAnalogSource::reading_settled() 
{
    if (_settle_time_ms != 0 && (hal.scheduler->millis() - _read_start_time_ms) < _settle_time_ms) {
        return false;
    }
    return true;
}

/* new_sample is called from an interrupt. It always has access to
 *  _sum and _sum_count. Lock out the interrupts briefly with
 * cli/sei to read these variables from outside an interrupt. */
void EmptyAnalogSource::new_sample(int16_t sample) {
    _sum += sample;
    _latest = sample;
    if (_sum_count >= 63) {
        _sum >>= 1;
        _sum_count = 32;
    } else {
        _sum_count++;
    }
}
