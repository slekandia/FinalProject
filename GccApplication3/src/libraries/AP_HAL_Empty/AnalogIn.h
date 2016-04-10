
#ifndef __AP_HAL_EMPTY_ANALOGIN_H__
#define __AP_HAL_EMPTY_ANALOGIN_H__

#include "AP_HAL_Empty.h"
#define AVR_INPUT_MAX_CHANNELS	15
extern "C" {
	#include <asf.h>
};
class Empty::EmptyAnalogSource : public AP_HAL::AnalogSource {
public:
    friend class Empty::AVRAnalogIn;
    /* pin designates the ADC input number, or when == AVR_ANALOG_PIN_VCC,
     * board vcc */
    EmptyAnalogSource(uint8_t pin);

    /* implement AnalogSource virtual api: */
    float read_average();
    float read_latest();
    void set_pin(uint8_t p);
    float voltage_average();
    float voltage_latest();
    float voltage_average_ratiometric();
    void set_stop_pin(uint8_t p);
    void set_settle_time(uint16_t settle_time_ms);    

    /* implementation specific interface: */

    /* new_sample(): called with value of ADC measurments, from interrput */
    void new_sample(int16_t);

    /* setup_read(): called to setup ADC registers for next measurment,
     * from interrupt */
    void setup_read();

    /* stop_read(): called to stop device measurement */
    void stop_read();

    /* reading_settled(): called to check if we have read for long enough */
    bool reading_settled();

    /* read_average: called to calculate and clear the internal average.
     * implements read_average(), unscaled. */
    float _read_average();

    int16_t get_pin() { return _pin; };
private:
    /* following three are used from both an interrupt and normal thread */
    volatile uint8_t _sum_count;
    volatile uint16_t _sum;
    volatile uint16_t _latest;
    float _last_average;

    /* _pin designates the ADC input mux for the sample */
    uint8_t _pin;

    /* _stop_pin designates a digital pin to use for
       enabling/disabling the analog device */
    uint8_t _stop_pin;
    uint16_t _settle_time_ms;
    uint32_t _read_start_time_ms;
};

/* AVRAnalogIn : a concrete class providing the implementations of the 
 * timer event and the AP_HAL::AnalogIn interface */
class Empty::AVRAnalogIn : public AP_HAL::AnalogIn {
public:
    AVRAnalogIn();
	//degistir init 
    //void init(void* ap_hal_scheduler);
	void init();
    AP_HAL::AnalogSource* channel(int16_t n);
    float board_voltage(void);
	//void update_status(EmptyAnalogSource*);

 protected:
	void _timer_event();
	uint16_t getADCValues(int ADC_Ch_Index, adcifa_sequencer_opt_t *seq0_opt, adcifa_sequencer_opt_t *seq1_opt);
    EmptyAnalogSource* _create_channel(int16_t num);
    void _register_channel(EmptyAnalogSource*);
    
    EmptyAnalogSource* _channels[AVR_INPUT_MAX_CHANNELS];
    int16_t _num_channels;
    int _active_channel;
	adcifa_sequencer_opt_t sequancer_opt_0;
	adcifa_sequencer_opt_t sequancer_opt_1;
	
    

private:
    EmptyAnalogSource _vcc;
};

#endif // __AP_HAL_AVR_ANALOG_IN_H__
