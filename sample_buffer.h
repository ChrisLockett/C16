/*
 *	sample_buffer.h
 *
 * 	This file has functions for manipulating sample buffers
 * 	for the DataLoggerXX program that runs on the rabbit3100
 *	conected to the CVO RIM board.
 *
 *  
 */

/*
 *	DEFINE statements
 */
#define MAX_SENSOR_RATE 300	//maximum sample rate in S/s
#define MAX_GPS_BUFFER	1024	//maximum size of a GPS message to be stored

//#define DIFFERENTIALINPUTS //use diferental inputs on the ADC

// define to use single ended inputs in the readADC function
#define SINGLEENDEDINPUTS


/*
 *	OneSecBuff
 *
 * 	A OneSecBuff is a buffer for holding the sensor data from all of the
 *	sensors for one second.
 */
typedef struct OneSecBuff{
	//State
	char 	flag;						//flag
    									//bit0 is set when the buffer
    									//has been sent/handled
	//Buffers
	int		ADC0[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 0
	int		ADC1[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 1
	int		ADC2[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 2
	int		ADC3[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 3
	int 	ADC4[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 4
	int		ADC5[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 5
	int		ADC6[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 6
	int		ADC7[MAX_SENSOR_RATE + 2];		//buffer for ADC channel 7
	int		Digital[MAX_SENSOR_RATE + 2];	//buffer to hold all samples from digital inputs
    char	GPS[MAX_GPS_BUFFER]; 		//buffer to hold all messages sent by the GPS

	//Buffer indexes
	unsigned int ADC0_index;			//index into ADC0
    unsigned int ADC1_index;			//index into ADC1
	unsigned int ADC2_index;			//index into ADC2
	unsigned int ADC3_index;			//index into ADC3
	unsigned int ADC4_index;			//index into ADC4
	unsigned int ADC5_index;			//index into ADC5
	unsigned int ADC6_index;			//index into ADC6
	unsigned int ADC7_index;			//index into ADC7
	unsigned int Digital_index;			//index into Digital
	unsigned int GPS_index;				//index into GPS
} OneSecBuff;

/*
 *	void markSent(OneSecBuff *buf);
 *
 *	marks the buffer as sent
 */
void markSent(OneSecBuff *buf);

/*
 *	void clearBuffer(OneSecBuff *buff);
 *
 *	clears the buffer by seting all of the buffer indexes to zero and
 *	clearing the buffer flags
 */
void clearBuffer(OneSecBuff *buff);

/*
 *	int wasSent(OneSecBuff *buf);
 *
 *	returns 1 if the buffer was read/sent
 *	returns 0 if the buffer was not sent read/sent
 */
int wasSent(OneSecBuff *buf);

/*
 *	void InitSampleBuffer(OneSecBuff *buf);
 *
 *	Setup the one second sample buffer that hold the samples taken.
 */
void InitSampleBuffer(OneSecBuff *buf);


/*
 *	unsigned int _nearcall readADC(int chan);
 *
 *	Reads a sample from the ADC on the sepecified channel.
 */
unsigned int _nearcall readADC(int chan);

/*
 *	void ADCInit(void);
 *
 *	Sets up the SPI and gets the ADC ready to be ssampled
 *	Must be called befor any call to readADC().
 */
void ADCInit(void);
