/*
 * GPS.cpp
 *
 * Created: 12/2/2015 11:20:58 PM
 *  Author: Mert
 */ 
#include "libraries/GPS/GPS.h"
#include "libraries/AP_HAL_Empty/UARTDriver.h"
extern "C" {
#include <stdio.h>
#include <string.h>
};
extern const AP_HAL::HAL& hal;

void GPS::update(char *array) {
	 if (strstr(array, "$GPRMC")) {
		 // found RMC
		 char *p = array;

		 // get time
		 int32_t degree;
		 long minutes;
		 char degreebuff[10];
		 // parse out latitude
		 p = strchr(p, ',')+1;
		 if (',' != *p)
		 {
			 strncpy(degreebuff, p, 2);
			 p += 2;
			 degreebuff[2] = '\0';
			 long degree = atol(degreebuff) * 10000000;
			 strncpy(degreebuff, p, 2); // minutes
			 p += 3; // skip decimal point
			 strncpy(degreebuff + 2, p, 4);
			 degreebuff[6] = '\0';
			 long minutes = 50 * atol(degreebuff) / 3;
			 latitude_fixed = degree + minutes;
			 lattitude = degree / 100000 + minutes * 0.000006F;
			 latitudeDegrees = (lattitude-100*int(lattitude/100))/60.0;
			 latitudeDegrees += int(lattitude/100);
		 }
		 p = strchr(p, ',')+1;

		 
		 // parse out longitude
		 p = strchr(p, ',')+1;
		 if (',' != *p)
		 {
			 strncpy(degreebuff, p, 3);
			 p += 3;
			 degreebuff[3] = '\0';
			 degree = atol(degreebuff) * 10000000;
			 strncpy(degreebuff, p, 2); // minutes
			 p += 3; // skip decimal point
			 strncpy(degreebuff + 2, p, 4);
			 degreebuff[6] = '\0';
			 minutes = 50 * atol(degreebuff) / 3;
			 longitude_fixed = degree + minutes;
			 longitude = degree / 100000 + minutes * 0.000006F;
			 longitudeDegrees = (longitude-100*int(longitude/100))/60.0;
			 longitudeDegrees += int(longitude/100);
		 }
		 

		 // we dont parse the remaining, yet!
// 	bool poncik = true;
// 	uint8_t i = 0;
// 	for (i = 0; i < 255 & poncik; i++) {
// 		if(array[i] == '$') {
// 			if(array[i+1] == 'G')
// 				if(array[i+2] == 'P')
// 					if(array[i+3] == 'R')
// 						if(array[i+4] == 'M')
// 							if(array[i+5] == 'C')
// 								poncik = false;
// 		}
// 	}
// 	
// 	if(!poncik & (i < 222)) {
// 		lattitude = (array[i+16]-48)*10+(array[i+17]-48) +  ((array[i+18]-48)*10 + (array[i+19]-48) + (array[i+21]-48)*0.1+(array[i+22]-48)*0.01)*0.0166667;
// 		longitude = (array[i+27]-48)*100 + (array[i+28]-48)*10 + (array[i+29]-48) + ((array[i+30]-48)*10 + (array[i+31]-48) + (array[i+32]-48)*0.1+(array[i+33]-48)*0.01)*0.0166667;
// 	}
// 	
	
// 	 updated = false;
// 	 uint8_t c,asd;
// 	float convert;
// 	float convert2;
// 	float convert3;
// 	float convert4;
// 	int a = 0;
// 	float b = 0;
// 	float d = 0;
// 	hal.uartB->read();
// 	hal.console->println("inside");
//  	while(hal.uartB->read() != '$');
// 	while(hal.uartB->read() != 'G');
// 	while(hal.uartB->read() != 'P');
// 	while(hal.uartB->read() != 'R');
// 	while(hal.uartB->read() != 'M');
// 	while(hal.uartB->read() != 'C');
// 	while(hal.uartB->read() != ',');
// 	
// 	for(a = 0;a<13;a++){
// 		asd = hal.uartB->read();
// 	}			
// 	
// 	convert = (hal.uartB->read()-48)*10;
// 	convert2 = hal.uartB->read()-48;
// 	lattitude = convert+convert2;
// 	
// 	//minute conversion
// 	convert = (hal.uartB->read()-48)*10;
// 	convert2 = (hal.uartB->read()-48);
// 	lattitude = lattitude + (convert+convert2)*(0.01666667);
// 
// 	//skip the "."
// 	asd = hal.uartB->read();
// 	
// 	//second conversion
// 	convert = (hal.uartB->read()-48)*0.1;
// 	convert2 = (hal.uartB->read()-48)*0.01;
// 	convert3 = (hal.uartB->read()-48)*0.001;
// 	convert4 = (hal.uartB->read()-48)*0.0001;
// 	d = convert+convert2+convert3+convert4;
// 	lattitude = lattitude + d*0.01666667;
// 	
// 	
// 	
// 	for(a = 0;a<4;a++){
// 		asd = hal.uartB->read();
// 	}
// 	
// 	convert = (hal.uartB->read()-48)*10;
// 	convert2 = hal.uartB->read()-48;
// 	longitude = convert+convert2;
// 	
// 	//minute conversion
// 	convert = (hal.uartB->read()-48)*10;
// 	convert2 = (hal.uartB->read()-48);
// 	longitude = longitude + (convert+convert2)*(0.01666667);
// 
// 	//skip the "."
// 	asd = hal.uartB->read();
// 	
// 	//second conversion
// 	convert = (hal.uartB->read()-48)*0.1;
// 	convert2 = (hal.uartB->read()-48)*0.01;
// 	convert3 = (hal.uartB->read()-48)*0.001;
// 	convert4 = (hal.uartB->read()-48)*0.0001;
// 	d = convert+convert2+convert3+convert4;
// 	longitude = longitude + d*0.01666667;
}
}

