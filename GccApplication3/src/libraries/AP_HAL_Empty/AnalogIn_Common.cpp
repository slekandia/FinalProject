/*
 * AnalogIn_Common.cpp
 *
 * Created: 11/3/2015 3:02:50 PM
 *  Author: alpombeo
 */ 

#include "libraries/AP_HAL_Empty/AnalogIn.h"
#include "libraries/AP_HAL/AP_HAL.h"
#define AVR_INPUT_MAX_CHANNELS	16
using namespace Empty;
#define NUMBER_OF_CH_SEQ0	8
#define NUMBER_OF_CH_SEQ1	8
extern const AP_HAL::HAL& hal;
extern "C" {
	#include <asf.h>
	};


/* CHANNEL_READ_REPEAT: how many reads on a channel before using the value.
 * This seems to be determined empirically */


AVRAnalogIn::AVRAnalogIn() :
    _vcc(EmptyAnalogSource(ANALOG_INPUT_BOARD_VCC))
{}

void AVRAnalogIn::init() 
{
    /* Register AVRAnalogIn::_timer_event with the scheduler. */
	//schedular kullan unutma
    //hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&AVRAnalogIn::_timer_event, void));
    /* Register each private channel with AVRAnalogIn. */
    
	
	adcifa_opt_t adc_options;
	adc_options.frequency = 1; /**< Internal ADC Frequency in Hz */
	adc_options.reference_source = ADCIFA_ADCREF1; /**< Reference Source */
	adc_options.sample_and_hold_disable = true; /**< Sample and Hold Selection */
	adc_options.single_sequencer_mode = true; /**< Single Sequencer Mode */
	adc_options.free_running_mode_enable = false; /**< Enable Free Running Mode */
	adc_options.sleep_mode_enable = false; /**< Sleep Mode Selection */
	adc_options.mux_settle_more_time = false; /** Multiplexer Settle Time */
	adc_options.gain_calibration_value = AVR32_FLASHC_FROW_GCAL_OFFSET; /**< Gain Calibration Value NE YAZICA?IMI B?LM?YORUM */
	adc_options.offset_calibration_value = AVR32_FLASHC_FROW_OCAL_OFFSET; /**< Offset Calibration Value NE YAZICA?IMI B?LM?YORUM */
	adc_options.sh0_calibration_value = 0; /**< S/H Gain Calibration Value for Seq0 NE YAZICA?IMI B?LM?YORUM*/
	adc_options.sh1_calibration_value = 0; /**< S/H Gain Calibration Value for Seq1 NE YAZICA?IMI B?LM?YORUM*/
	
	adcifa_sequencer_opt_t sequancer_opt_0;
	sequancer_opt_0.convnb = NUMBER_OF_CH_SEQ0; /**< Number of conversion. */
	sequancer_opt_0.resolution = ADCIFA_SRES_12B; /**< Sequencer resolution. */
	sequancer_opt_0.trigger_selection = ADCIFA_TRGSEL_ITIMER; /**< Trigger selection. */
	sequancer_opt_0.start_of_conversion = ADCIFA_SOCB_ALLSEQ; /**< Start of conversion. */
	sequancer_opt_0.sh_mode = ADCIFA_SH_MODE_STANDARD;
	sequancer_opt_0.half_word_adjustment = ADCIFA_HWLA_NOADJ;
	sequancer_opt_0.software_acknowledge = ADCIFA_SA_NO_EOS_SOFTACK;
	
	adcifa_sequencer_opt_t sequancer_opt_1;
	sequancer_opt_1.convnb = NUMBER_OF_CH_SEQ1; /**< Number of conversion. */
	sequancer_opt_1.resolution = ADCIFA_SRES_12B; /**< Sequencer resolution. */
	sequancer_opt_1.trigger_selection = ADCIFA_TRGSEL_ITIMER; /**< Trigger selection. */
	sequancer_opt_1.start_of_conversion = ADCIFA_SOCB_ALLSEQ; /**< Start of conversion. */
	sequancer_opt_1.sh_mode = ADCIFA_SH_MODE_STANDARD;
	sequancer_opt_1.half_word_adjustment = ADCIFA_HWLA_NOADJ;
	sequancer_opt_1.software_acknowledge = ADCIFA_SA_NO_EOS_SOFTACK;
	
	adcifa_sequencer_conversion_opt_t sequencer_conv_opt_0[NUMBER_OF_CH_SEQ0];
	sequencer_conv_opt_0[0] = {AVR32_ADCIFA_INP_ADCIN0, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[1] = {AVR32_ADCIFA_INP_ADCIN1, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[2] = {AVR32_ADCIFA_INP_ADCIN2, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[3] = {AVR32_ADCIFA_INP_ADCIN3, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[4] = {AVR32_ADCIFA_INP_ADCIN4, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[5] = {AVR32_ADCIFA_INP_ADCIN5, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[6] = {AVR32_ADCIFA_INP_ADCIN6, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_0[7] = {AVR32_ADCIFA_INP_ADCIN7, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
		
	adcifa_sequencer_conversion_opt_t sequencer_conv_opt_1[NUMBER_OF_CH_SEQ1];
	sequencer_conv_opt_1[0] = {AVR32_ADCIFA_INN_ADCIN8, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[1] = {AVR32_ADCIFA_INN_ADCIN9, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[2] = {AVR32_ADCIFA_INN_ADCIN10, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[3] = {AVR32_ADCIFA_INN_ADCIN11, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[4] = {AVR32_ADCIFA_INN_ADCIN12, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[5] = {AVR32_ADCIFA_INN_ADCIN13, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[6] = {AVR32_ADCIFA_INN_ADCIN14, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	sequencer_conv_opt_1[7] = {AVR32_ADCIFA_INN_ADCIN15, AVR32_ADCIFA_INN_GNDANA, ADCIFA_SHG_1};
	
	
	adcifa_window_monitor_opt_t monitor_options;
	monitor_options.mode = ADCIFA_WINDOW_MODE_NONE;
	monitor_options.high_threshold = 0;
	monitor_options.low_threshold = 0;
	monitor_options.source_index = 0;
	
	adcifa_get_calibration_data(&AVR32_ADCIFA , &adc_options);
	uint8_t ADC_CONFIG_RESULT = adcifa_configure(&AVR32_ADCIFA , &adc_options, sysclk_get_pbb_hz());
	//hal.console->println(ADC_CONFIG_RESULT, 2);
	adcifa_calibrate_offset(&AVR32_ADCIFA , &adc_options, sysclk_get_pbb_hz());
	
	
	uint8_t CALIBRATION_RESULT_SEQ1 = adcifa_configure_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ0, &sequancer_opt_0, sequencer_conv_opt_0);
	uint8_t CALIBRATION_RESULT_SEQ2 = adcifa_configure_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ1, &sequancer_opt_1, sequencer_conv_opt_1);
	
	adcifa_configure_window_monitor( &AVR32_ADCIFA, ADCIFA_SEQ0,  &monitor_options);
	adcifa_configure_window_monitor( &AVR32_ADCIFA, ADCIFA_SEQ1,  &monitor_options);
	
	
	//register private channel
	_num_channels = 0;
	_active_channel = 0;
	_register_channel(&_vcc);
	

	
}

EmptyAnalogSource* AVRAnalogIn::_create_channel(int16_t chnum) {
    EmptyAnalogSource *ch = new EmptyAnalogSource(chnum);
    _register_channel(ch);
    return ch;
}



void AVRAnalogIn::_register_channel(EmptyAnalogSource* ch) {
	
    if (_num_channels >= AVR_INPUT_MAX_CHANNELS) {
        for(;;) {
			
            hal.console->print_P(PSTR(
                "Error: AP_HAL_AVR::AVRAnalogIn out of channels\r\n"));
            hal.scheduler->delay(1000);
        }
    }
    _channels[_num_channels] = ch;
  
    _num_channels++;

    if (_num_channels == 1) {
        /* After registering the first channel, we can enable the ADC */
		
		adcifa_start_itimer(&AVR32_ADCIFA, AVR32_ADCIFA_ITIMER_ITMC);
		adcifa_start_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ0);
		adcifa_start_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ1);
    }
}

void AVRAnalogIn:: _timer_event() 
{
    if (_channels[_active_channel]->_pin == ANALOG_INPUT_NONE) {
       _channels[_active_channel]->new_sample(0);
        goto next_channel;
    }

  
  if (_num_channels == 0) {
        /* No channels are registered - nothing to be done. */
        return;
	}

    /* Read the conversion registers. */
    {
		if (_active_channel <= 6){
			int16_t sample =  ADCIFA_read_resx_sequencer_0(_active_channel) ;
			/* Give the active channel a new sample */
			_channels[_active_channel]->new_sample( sample );
			//_active_channel = (_active_channel + 1) % _num_channels;
			
		}
		else if (_active_channel > 6){
			int16_t sample =  ADCIFA_read_resx_sequencer_1(_active_channel - 7) ;
			/* Give the active channel a new sample */
			_channels[_active_channel]->new_sample( sample );
			//_active_channel = (_active_channel + 1) % _num_channels;
			
		}
    }
	
next_channel:
    /* stop the previous channel, if a stop pin is defined */
    _channels[_active_channel]->stop_read();
    /* Move to the next channel */
   _active_channel = (_active_channel + 1) % _num_channels;

}


AP_HAL::AnalogSource* AVRAnalogIn::channel(int16_t ch) 
{
    if (ch == ANALOG_INPUT_BOARD_VCC) {
            return &_vcc;
    } else {
        return _create_channel(ch);
    }
}

/*
  return board voltage in volts
 */
float AVRAnalogIn::board_voltage(void)
{
    return _vcc.voltage_latest();
}



uint16_t AVRAnalogIn::getADCValues (int ADC_Ch_Index, adcifa_sequencer_opt_t *seq0_opt, adcifa_sequencer_opt_t *seq1_opt)
{
	uint16_t Get_Value_result;
	int16_t adc_values_temp [NUMBER_OF_CH_SEQ0];
	if (ADC_Ch_Index <= 6){
		uint16_t Get_Value_result = adcifa_get_values_from_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ0, seq0_opt, adc_values_temp);
		return adc_values_temp[ADC_Ch_Index-1];
	} else {
		uint16_t Get_Value_result = adcifa_get_values_from_sequencer(&AVR32_ADCIFA, ADCIFA_SEQ1, seq1_opt, adc_values_temp);
		return adc_values_temp[ADC_Ch_Index-8];
	}
	return Get_Value_result;
}



