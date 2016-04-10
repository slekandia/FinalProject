
#ifndef __AP_HAL_EMPTY_UARTDRIVER_H__
#define __AP_HAL_EMPTY_UARTDRIVER_H__

extern "C" {
	#include <asf.h>
	};
#include "AP_HAL_Empty.h"
#include "libraries/AP_HAL/UARTDriver.h"

class Empty::EmptyUARTDriver : public AP_HAL::UARTDriver {
public:
    EmptyUARTDriver(volatile avr32_usart_t*portNumber);
    /* Empty implementations of UARTDriver virtual methods */
    void begin(uint32_t b);
    void begin(uint32_t b, uint16_t rxS, uint16_t txS);
    void end();
    void flush();
    bool is_initialized();
    void set_blocking_writes(bool blocking);
    bool tx_pending();

    /* Empty implementations of Stream virtual methods */
    int16_t available();
    int16_t txspace();
    uint8_t read();

    /* Empty implementations of Print virtual methods */
    size_t write(uint8_t c);
    size_t write(const uint8_t *buffer, size_t size);
	
	//PX4'DAN COPY PASTE
    void _timer_tick(void);

    int _get_fd(void) {
	    return _fd;
    }

    //void set_flow_control(enum flow_control flow_control);
    enum flow_control get_flow_control(void) { return _flow_control; }

 
private:
    volatile avr32_usart_t *_portNumber;
    int _fd;
    uint32_t _baudrate;
    volatile bool _initialised;
    volatile bool _in_timer;
	uint16_t _txS;
	uint16_t _rxS;
    bool _nonblocking_writes;

    // we use in-task ring buffers to reduce the system call cost
    // of ::read() and ::write() in the main loop
    uint8_t *_readbuf;
    uint16_t _readbuf_size;

    // _head is where the next available data is. _tail is where new
    // data is put
    volatile uint16_t _readbuf_head;
    volatile uint16_t _readbuf_tail;

    uint8_t *_writebuf;
    uint16_t _writebuf_size;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;

    int _write_fd(const uint8_t *buf, uint16_t n);
    int _read_fd(uint8_t *buf, uint16_t n);
    uint64_t _last_write_time;

    void try_initialise(void);
    uint32_t _last_initialise_attempt_ms;

    uint32_t _os_write_buffer_size;
    uint32_t _total_read;
    uint32_t _total_written;
	
	void transmit();
	
	enum flow_control _flow_control;
};




#endif // __AP_HAL_EMPTY_UARTDRIVER_H__
