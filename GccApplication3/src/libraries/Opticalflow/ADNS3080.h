/*
 * ADNS3080.h
 *
 * Created: 05.02.2016 11:40:27
 *  Author: Metin
 */ 
#ifndef __ADNS3080__
#define __ADNS3080__
extern "C"{
	#include <asf.h>
};
#define ADNS3080_PRODUCT_ID            0x00
#define ADNS3080_MOTION                0x02
#define ADNS3080_DELTA_X               0x03
#define ADNS3080_DELTA_Y               0x04
#define ADNS3080_SQUAL                 0x05
#define ADNS3080_CONFIGURATION_BITS    0x0A
#define ADNS3080_MOTION_CLEAR          0x12
#define ADNS3080_FRAME_CAPTURE         0x13
#define ADNS3080_MOTION_BURST          0x50

// ADNS3080 hardware config
#define ADNS3080_PIXELS_X              30
#define ADNS3080_PIXELS_Y              30

// Id returned by ADNS3080_PRODUCT_ID register
#define ADNS3080_PRODUCT_ID_VALUE      0x17


//Pinout of SPI

#define ADNS3080_SPI                 (&AVR32_SPI1)
#define ADNS3080_SPI_NPCS            (0)
#define ADNS3080_SPI_SCK_PIN         AVR32_SPI1_SCK_1_PIN
#define ADNS3080_SPI_SCK_FUNCTION    AVR32_SPI1_SCK_1_FUNCTION
#define ADNS3080_SPI_MISO_PIN        AVR32_SPI1_MISO_1_PIN
#define ADNS3080_SPI_MISO_FUNCTION   AVR32_SPI1_MISO_1_FUNCTION
#define ADNS3080_SPI_MOSI_PIN        AVR32_SPI1_MOSI_1_PIN
#define ADNS3080_SPI_MOSI_FUNCTION   AVR32_SPI1_MOSI_1_FUNCTION
#define ADNS3080_SPI_NPCS0_PIN       AVR32_SPI1_NPCS_0_2_PIN  
#define ADNS3080_SPI_NPCS0_FUNCTION  AVR32_SPI1_NPCS_0_2_FUNCTION


static void init_resources_ADNS3080(){
		//reset pin for software reset Wireless 4th pin is used
		//gpio_configure_pin(49,GPIO_DIR_OUTPUT);
		static const gpio_map_t J1920_SPI0_GPIO_MAP =
		{
			{ADNS3080_SPI_SCK_PIN,         ADNS3080_SPI_SCK_FUNCTION         },  // SPI Clock.
			{ADNS3080_SPI_MISO_PIN,         ADNS3080_SPI_MISO_FUNCTION        },  // MISO.
			{ ADNS3080_SPI_MOSI_PIN,         ADNS3080_SPI_MOSI_FUNCTION       },  // MOSI.
			{ ADNS3080_SPI_NPCS0_PIN,			ADNS3080_SPI_NPCS0_FUNCTION      },
			//#define AT45DBX_ENABLE_NPCS_PIN(npsc, unused) \
			//{AT45DBX_SPI_NPCS##NPCS##_PIN, AT45DBX_SPI_NPCS##NPCS##_FUNCTION},  // Chip Select NPCS.
			//MREPEAT(AT45DBX_MEM_CNT, AT45DBX_ENABLE_NPCS_PIN, ~)
			//#undef AT45DBX_ENABLE_NPCS_PIN
		};
		gpio_enable_module(J1920_SPI0_GPIO_MAP,
		sizeof(J1920_SPI0_GPIO_MAP) / sizeof(J1920_SPI0_GPIO_MAP[0]));	
		spi_options_t spiOptions;

		spiOptions.reg          = ADNS3080_SPI_NPCS;
		spiOptions.baudrate     = 1000000;
		spiOptions.bits         = 8;
		spiOptions.spck_delay   = 0;
		spiOptions.trans_delay  = 0;
		spiOptions.stay_act     = 1;
		spiOptions.spi_mode     = 0;
		spiOptions.modfdis      = 1;
			

		while(spi_initMaster(ADNS3080_SPI,&spiOptions)!=STATUS_OK);
			//Setup configuration for chip connected to CS1
		while(spi_setupChipReg(ADNS3080_SPI,&spiOptions,sysclk_get_pba_hz())!=STATUS_OK);
			//Allow the module to transfer data
			spi_enable(ADNS3080_SPI);
		
		
		
};

static void dummy_write(){
uint16_t txdata=0x00;
spi_put(ADNS3080_SPI,txdata);
//Wait for a complete transmission
while(!spi_is_tx_empty(ADNS3080_SPI));
}

static void ADNS3080_reset(){
	gpio_toggle_pin(49);
	cpu_delay_ms(10,FOSC0);
	gpio_toggle_pin(49);
	cpu_delay_ms(500,FOSC0);
}
static void ADNS3080_Write(uint8_t reg, uint8_t data){
	  spi_selectChip(ADNS3080_SPI,0);
	  while(!spi_is_tx_ready(ADNS3080_SPI));
	  spi_put(ADNS3080_SPI,reg | 0x80); // Indicate write operation
	  while(!spi_is_tx_empty(ADNS3080_SPI));
	  cpu_delay_us(75,FOSC0); // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
	  spi_put(ADNS3080_SPI,data); // Write data
	  while(!spi_is_tx_empty(ADNS3080_SPI));
      spi_unselectChip(ADNS3080_SPI,0);
}
void ADNS3080_Read(uint8_t reg, uint16_t data) {
	uint16_t dumdata;
	spi_selectChip(ADNS3080_SPI,0);
	while(!spi_is_tx_ready(ADNS3080_SPI));
	spi_put(ADNS3080_SPI,reg); // Send register address
	while(!spi_is_tx_empty(ADNS3080_SPI));
	delay_ms(0.5);
	/*cpu_delay_us(50,FOSC0);*/ // Wait minimum 75 us in case writing to Motion or Motion_Burst registers
	spi_read(ADNS3080_SPI); // Write data to a buffer
	spi_unselectChip(ADNS3080_SPI,0);
}

#endif