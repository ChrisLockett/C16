
//
//	sample_buffer.c
//
//	this file has functions for manipulating sample buffers
// 	for the DataLoggerXX program that runs on the rabbit3100
//	conected to the CVO RIM board
//
//  
#include "rabbit.h"
#include "sample_buffer.h"
#include "spi.h"

//use sing ended inputs for adc readings
#define SINGLEENDEDINPUTS

//	defines for the Chip Select
//	Allows for faster setting and clearing CS then function calls.
#define CS_HIGH() oute(PBDR,srPBDR|0x08)
#define CS_LOW() oute(PBDR,srPBDR&=0x7f)


//	int wasSent(OneSecBuff *buf);
//
//	returns 1 if the buffer was read/sent
//	returns 0 if the buffer was not sent read/sent
int wasSent(OneSecBuff *buf){
	if(buf->flag & 0x01){ //bit 0 of flag in the buffer
		return 1;
	}else {
		return 0;
	}
}

//	int markSent(OneSecBuff *buf);
//
//	marks the buffer as sent
void markSent(OneSecBuff *buf){
	buf->flag |= 0x01;
}


//	void InitSampleBuffer(OneSecBuff *buff);
//
//	Setup the one second sample buffer that hold the samples taken
void InitSampleBuffer(OneSecBuff *buff){
    //set buffer state
	buff->flag = 0;
	//set buffer indexes
	buff->ADC0_index 	= 0;
	buff->ADC1_index 	= 0;
	buff->ADC2_index 	= 0;
	buff->ADC3_index 	= 0;
	buff->ADC4_index 	= 0;
	buff->ADC5_index 	= 0;
	buff->ADC6_index 	= 0;
	buff->ADC7_index 	= 0;
	buff->Digital_index = 0;
	buff->GPS_index 	= 0;
}

//	void clearBuffer(OneSecBuff *buff);
//
//	clears the buffer by seting all of the buffers to zero and
//	clearing the flag var
void _nearcall clearBuffer(OneSecBuff *buff){
    //set buffer state
	buff->flag = 0;
	//set buffer indexes
	buff->ADC0_index 	= 0;
	buff->ADC1_index 	= 0;
	buff->ADC2_index 	= 0;
	buff->ADC3_index 	= 0;
	buff->ADC4_index 	= 0;
	buff->ADC5_index 	= 0;
	buff->ADC6_index 	= 0;
	buff->ADC7_index 	= 0;
	buff->Digital_index = 0;
	buff->GPS_index 	= 0;
}


//	unsigned int _nearcall readOneSampleADC(int chan);
//
//	Reads a sample from the ADC on the sepecified channel
unsigned int _nearcall readOneSampleADC(int chan){

	char adc_cmd[1];
	char adc_reading[3];
	unsigned int adc_sample;
	//unsigned int isample;


	switch (chan) {

#ifdef SINGLEENDEDINPUTS
	//  ( SE, chan 0, )
	case 0: 	adc_cmd[0] = '\x86'; 	break;
	//  ( SE, chan 1, )
	case 1:		adc_cmd[0] = '\xc6'; 	break;
	//  ( SE, chan 2, )
	case 2:		adc_cmd[0] = '\x96';	break;
	//  ( SE, chan 3, )
	case 3:		adc_cmd[0] = '\xd6';	break;
	//  ( SE, chan 4, e)
	case 4:		adc_cmd[0] = '\xa6';  	break;
	//  ( SE, chan 5, e)
	case 5:		adc_cmd[0] = '\xe6';  	break;
	//  ( SE, chan 6, )
	case 6:		adc_cmd[0] = '\xb6';  	break;
	//  ( SE, chan 7, )
	case 7:		adc_cmd[0] = '\xf6';  	break;
#endif


#ifdef DIFFERENTIALINPUTS
	//  ( DI, chan 0, )
	case 0: 	adc_cmd[0] = '\x82'; 	break;
	//  ( DI, chan 1, )
	case 1:		adc_cmd[0] = '\x92'; 	break;
	//  ( DI, chan 2, )
	case 2:		adc_cmd[0] = '\xa2';	break;
	//  ( DI, chan 3, )
	case 3:		adc_cmd[0] = '\xb2';	break;
	//  ( DI, chan 4, )
	case 4:		adc_cmd[0] = '\xc2';  	break;
	//  ( DI, chan 5, )
	case 5:		adc_cmd[0] = '\xd2';  	break;
	//  ( DI, chan 6, )
	case 6:		adc_cmd[0] = '\xe2';  	break;
	//  ( DI, chan 7, )
	case 7:		adc_cmd[0] = '\xf2';  	break;
#endif

	//return zero if chan is not in [0, 7] (ERROR)
	default:	return 0;
   	}


	//  Start ADC conversion
	CS_LOW(); //chip select low
	SPIWrite(adc_cmd, 1); //start the ADC
	CS_HIGH(); // chip select high

	//wait for the end of the conversion at least 8 microsecs

	//no delay seems to be needed

	//read 3 bytes from the spi
	CS_LOW(); // chip select set low
	SPIRead(adc_reading, 3); // read 3 bytes from the SPI
	CS_HIGH(); // Chip select set high

	//the ADC returns the 16bit reading in 3 bytes
	// 7 most significant bits in byte 0 (1st)
	// 8 next most significant bits in byte 1 (2nd)
	// the least significant bit in byte 2 (3rd)
	adc_sample = adc_reading[0] << 9;
	adc_sample = adc_sample + (adc_reading[1] << 1);
	adc_sample = adc_sample + (adc_reading[2] >> 7);
	return adc_sample;
}

//	unsigned int _nearcall readADC(int chan);
//
//	takes a reading from the ADC buy taking a number
//  of samples(SAMPLESPERREADING) and reporting the average
#define SAMPLESPERREADING 4
unsigned int _nearcall readADC(int chan){
	unsigned int reading;
	unsigned long val;   //32 bit long
	int i;

    //get an average of the number of samples
	val = 0;
	for(i =0; i < SAMPLESPERREADING; ++i){
		val += readOneSampleADC(chan);
	}
	reading = val / SAMPLESPERREADING;

	return reading;
}

//	void ADCInit(void);
//
//	Sets up the SPI and gets the ADC ready to be sampled
//	Must be called befor any call to readADC()
void ADCInit(void){
	// set up the SPI interface
	SPIinit();
	// set up chip select
	oute(PBDDR,srPBDDR|=0x80);	// bit 7 is output
	oute(PBDR,srPBDR|=0x80);	// bit 7 is high
}
