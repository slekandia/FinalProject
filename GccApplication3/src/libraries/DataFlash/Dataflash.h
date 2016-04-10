#ifndef dataflash_h
#define dataflash_h
/*
 * Dataflash.h
 *
 * Created: 30.01.2016 16:44:43
 *  Author: met
 */ 
extern "C"{
#include <asf.h>
#include "at45dbx.h"
#include "conf_at45dbx.h"
};

static void at45dbx_resources_init(){
	
	spi_options_t spiOptions;
	
	spiOptions.reg          = AT45DBX_SPI_FIRST_NPCS;   // Defined in conf_at45dbx.h.
	spiOptions.baudrate     = AT45DBX_SPI_MASTER_SPEED; // Defined in conf_at45dbx.h.
	spiOptions.bits         = AT45DBX_SPI_BITS;        // Defined in conf_at45dbx.h.
	spiOptions.spck_delay   = 0;
	spiOptions.trans_delay  = 0;
	spiOptions.stay_act     = 1;
	spiOptions.spi_mode     = 0;
	spiOptions.modfdis      = 1;
	
	spi_initMaster(AT45DBX_SPI, &spiOptions);
	spi_selectionMode(AT45DBX_SPI, 0, 0, 0);
	spi_enable(AT45DBX_SPI);
	at45dbx_init(spiOptions, FOSC0);
	
};
static U8 dataflash_read(U8 sector){
	U8 j = 0;
	if (at45dbx_read_open(sector) == true)
	{
	j = at45dbx_read_byte();
	at45dbx_read_close();
	}
	return j;
};
static void dataflash_write(U8 sector, U8 data){
	if (at45dbx_write_open(sector) == true)
	{
	at45dbx_write_byte(data);
	at45dbx_write_close();
	}
};
static void dataflash_multiple_write(U8 sector_beginning, U8* data, U8 length){
	for (int i = 0; i<length; i=i+1)
	{
		dataflash_write(sector_beginning+i,data[i]);
	}
};
static void dataflash_multiple_read(U8 sector_beginning, U8 length, U8* c){
	
	for (int i = 0; i<length; i=i+1)
	{
		c[i] = dataflash_read(sector_beginning+i);
	}
	
};


static void cast_32_8_array(int32_t array_2_cast,uint8_t* result){
	result[0] = (array_2_cast);
	result[1] = (array_2_cast) >> 8;
	result[2] = (array_2_cast) >> 16;
	result[3] = (array_2_cast) >> 24;
};

static int32_t cast_8_32_array(uint8_t* array_2_cast){
	int32_t result;
	return result = ((int32_t)array_2_cast[3] << 24)+((int32_t)array_2_cast[2] << 16)+((int32_t)array_2_cast[1] <<8)+((int32_t)array_2_cast[0]);
};

static void hard_bias_2_dataflash(uint8_t start, float* mag_Hard_Bias){
	uint8_t result[4];
	cast_32_8_array(mag_Hard_Bias[0],result);
	dataflash_multiple_write(start,result,4);
	cast_32_8_array(mag_Hard_Bias[1],result);
	dataflash_multiple_write(start+4,result,4);
	cast_32_8_array(mag_Hard_Bias[2],result);
	dataflash_multiple_write(start+8,result,4);
};
#endif