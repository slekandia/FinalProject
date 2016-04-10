
#include "UARTDriver.h"




#define USART_SERIAL_CHAR_LENGTH 8
#define USART_SERIAL_PARITY USART_NO_PARITY
#define USART_SERIAL_STOP_BITS USART_1_STOPBIT

using namespace Empty;
extern const AP_HAL::HAL& hal;

EmptyUARTDriver::EmptyUARTDriver(volatile avr32_usart_t *portNumber)
:
_portNumber(portNumber),
_initialised(false),
_in_timer(false)
 {
	 
 }



void EmptyUARTDriver::begin(uint32_t b) {
    begin (b,0,0);		
}

void EmptyUARTDriver::begin(uint32_t b, uint16_t rxS, uint16_t txS) {
	
	static usart_serial_options_t usart_options;
	usart_options.baudrate = b;
	usart_options.charlength = USART_SERIAL_CHAR_LENGTH;
	usart_options.paritytype = USART_SERIAL_PARITY;
	usart_options.stopbits = USART_SERIAL_STOP_BITS;
	usart_serial_init(_portNumber, &usart_options);
}

void EmptyUARTDriver::try_initialise(void)
{
	
}
	 

void EmptyUARTDriver::end()
{

}

void EmptyUARTDriver::flush() {}
	
bool EmptyUARTDriver::is_initialized() { 
	
	return false;}
	
void EmptyUARTDriver::set_blocking_writes(bool blocking) {}
bool EmptyUARTDriver::tx_pending() { return false; }

/* Empty implementations of Stream virtual methods */
int16_t EmptyUARTDriver::available() 
{ 

}
int16_t EmptyUARTDriver::txspace() {

}
uint8_t EmptyUARTDriver::read() { 
		uint8_t c;
		usart_serial_getchar(_portNumber, &c);
		return c;
}

	/* 
   write one byte to the buffer
 */
size_t EmptyUARTDriver::write(uint8_t c) 
{ 
	usart_serial_putchar(_portNumber,c);
	return 1;
} 



size_t EmptyUARTDriver::write(const uint8_t *buffer, size_t size)
{
	size_t n = 0;
	while (size--) {
		n += write(*buffer++);
	}
	return n;
}

