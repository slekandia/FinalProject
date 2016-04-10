/*
 * GPS.cpp
 *
 * Created: 12/2/2015 11:22:15 PM
 *  Author: Mert
 */ 

extern "C" {
	#include <asf.h>
	};
#  define USART_SERIAL_BAUDRATE			9600
#  define USART_SERIAL_CHAR_LENGTH		8
#  define USART_SERIAL_PARITY			USART_NO_PARITY
#  define USART_SERIAL_STOP_BITS		USART_1_STOPBIT

#define USART1_BAUD 38400

#define USART1_RX_PIN AVR32_USART1_RXD_0_0_PIN
#define USART1_RX_FUNCTION AVR32_USART1_RXD_0_0_FUNCTION
#define USART1_TX_PIN AVR32_USART1_TXD_0_0_PIN
#define USART1_TX_FUNCTION AVR32_USART1_TXD_0_0_FUNCTION
class GPS {
public:
	GPS() {
		updated = false;

		};
	float lattitude;
	float longitude;
	  int32_t latitude_fixed, longitude_fixed;
  float latitudeDegrees, longitudeDegrees;

	char time;
	bool updated;
	
	void update(char *array);
	};