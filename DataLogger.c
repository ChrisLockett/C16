/*
 *	DataLogger main file

 *
 *  CGL :)
 */

#define VERSION "104"



#include <stdio.h>
#include <rabbit.h>
#include <dcdefs.h>
#include <time.h>
#include <string.h>
#include <stdlib.h>
#include "CoExec/co_exec.h"
#include "spi.h"
#include "sample_buffer.h"
#include "user_block.h"

// number of time slices that are avalable
// the station sends it's data at the start of
// it's time slice which is the station number mod
// the number of time slices.
#define TIMESLICES 8

// this is the number of sec the systme will
// keep logging or sending data after not receiving
// a valid GPZDA message from the GPS system still needs the one sec time pulse
#define MAXTIMEAGE 3600  // must be up dated at least every hr

//defines for the ADC
#define CS_HIGH() oute(PBDR,srPBDR|0x80)
#define CS_LOW() oute(PBDR,srPBDR&=0x7f)

// define to allow debug output
//#define DEBUGOUTPUT


// define to stop the board from reseting due to inaction
//#define NORESET

// startup baudrate the baudrate for the inital startup of the system
#define SBAUDRATE  19200l

#define GPSBAUDRATE 9600l

#define NUM_TASKS 3
TASK tasks[NUM_TASKS];

/*
 *	GLOBALS
 */

//
// configuration variables ans defaults
//

//serial number / sation number
int serailNumber = 0;

//time slot
int timeSlot = 0;

//int	timeset = 0;	// set when time was read sucessfuly from the GPS
int	timeage	= 10000;	// number of sec sence the time was last set
						// initaly set to 10000(too old)


//sample rate
int samplerate = 50;

//data baud rate
int dataBaudrate = 1;

//GPS baud rate
int GPSBaudrate = 1;

//scan data rate
int scandatarate = 1;

//types of messages to send
int sendType0  = 0;
int sendType4  = 1;
int sendType8  = 1;
int sendType12 = 1;

//channels to send in type 12
int sendCh0 = 1;
int sendCh1 = 1;
int sendCh2 = 1;
int sendCh3 = 0;
int sendCh4 = 0;
int sendCh5 = 0;
int sendCh6 = 0;
int sendCh7 = 0;

//selects wether a chennel is single ended or diferental
//
// curently not used
int differentalCh0 = 0;
int differentalCh1 = 0;
int differentalCh2 = 0;
int differentalCh3 = 0;
int differentalCh4 = 0;
int differentalCh5 = 0;
int differentalCh6 = 0;
int differentalCh7 = 0;

//
// internal state variables
//
int datasent;

//buffers from serial ports
char GPSRx[512];
char GPSTx[32];

char serFRx[32];
char serFTx[1024];

/*
 * Two Sample Buffers one to read samples into
 * and one to send samples from, buffers switch
 * every second and the samples from the last second
 * are built into messages that are then sent.
 */
OneSecBuff	sb1;
OneSecBuff	sb2;

//pointers to the two sample buffers
OneSecBuff	*curentSB = &sb1;
OneSecBuff	*lastSB	  = &sb2;

/*
 *	FUNCTION PROTOTYPES
 */
void readConfig(void);

void _nearcall task1(void);
void _nearcall task2(void);

int sendRIMtype0(int Y, int Month, int D, int H, int M, int S);  // seismic old type
int sendRIMtype2(char *message,int msize); // informational message type
int sendRIMtype4(int Y, int Month, int D, int H, int M, int S);  // GPS
int sendRIMtype8(int Y, int Month, int D, int H, int M, int S);  // low data rate scan
int sendRIMtype12(int Y, int Month, int D, int H,int M, int S);  // siesmin new type
int sendRIMtype12chan(int Y, int Month, int D, int H,int M, int S, int chan);

void RTCsetTime(int Y, int Month, int D, int H,	int M, int S, int m);
struct tm * far RTCtime(void);

void serFwrite(char buf[], int bufsize);

void _fastint near _nostep GPSisr(void);
void setSampleRate(int rate);
void near _nostep TimerBRoutine(void);



/*
 *	void main(void)
 *
 *	sets all peramiters and starts all tasks
 */
void main(void) {
    int inaction = 0; 				// inaction counter
	//setup the watch dog timer(WDT).
	WDT_ENABLE();                   // enable the WDT
	WDT_2S();    					// set the timer for 2 sec
    //set the clock rate
	enableClockDoubler( 1 );		//turn on the clock doubler

    ipset0();

	init_tasks(NUM_TASKS,4096*3);	// initalize the tasks
	startTimer(8,NULL,1);			// setup the ms timer

#ifdef DEBUGOUTPUT
	//print a greeting
	printf("System starting\n");
#endif
    ADCInit();						// setup the ADC to take samples

    datasent = 0;					// no data was sent

	// Initalize the sample buffers
    InitSampleBuffer(curentSB);
	InitSampleBuffer(lastSB);


    // setup  the inital startup baudrate on serial port F to 115200
    SerialInitF(SBAUDRATE,SER_8BITS|SER_USE_C, 1, serFRx,
		sizeof serFRx, serFTx, sizeof serFTx);

	readConfig();

	setSampleRate(samplerate);

	//Set the output data rate of PORT F (DATA OUT to manifold)
	if(dataBaudrate == 1) {
		//set the baud rate to 19200
		SerialInitF(19200l,SER_8BITS|SER_USE_C, 1, serFRx,
			sizeof serFRx, serFTx, sizeof serFTx);
	}else if(dataBaudrate == 2){
		//set the baud rate to 38400
		SerialInitF(38400l,SER_8BITS|SER_USE_C, 1, serFRx,
			sizeof serFRx, serFTx, sizeof serFTx);
	}else if(dataBaudrate == 3){
		//set the baud rate to 57600
		SerialInitF(57600l,SER_8BITS|SER_USE_C, 1, serFRx,
			sizeof serFRx, serFTx, sizeof serFTx);
	}else if(dataBaudrate == 4){
		//set the baud rate to 115200
		SerialInitF(115200l,SER_8BITS|SER_USE_C, 1, serFRx,
			sizeof serFRx, serFTx, sizeof serFTx);
	}

	//Set the baud rate of PORT E (GPS data rate)
	if(GPSBaudrate == 1) {
		//set the baud rate to 9600
		SerialInitE(9600l,SER_8BITS|SER_USE_C, 1, GPSRx,
			sizeof GPSRx, GPSTx, sizeof GPSTx);
	}else if(GPSBaudrate == 2){
        //set the baud rate to 19200
		SerialInitE(19200l,SER_8BITS|SER_USE_C, 1, GPSRx,
			sizeof GPSRx, GPSTx, sizeof GPSTx);
	}else if(GPSBaudrate == 3){
        //set the baud rate to 115200
		SerialInitE(115200l,SER_8BITS|SER_USE_C, 1, GPSRx,
			sizeof GPSRx, GPSTx, sizeof GPSTx);
	}

	// set GPSisr
	setInterruptHandler(INT_INT1,GPSisr);
	//outi(I1CR,srI1CR=0x9);			// interupt on rising edge, level 1, use 0x5 for the falling edge
	outi(I1CR,srI1CR=0x5);				// interupt on falling edge, level 1, use 0x9 for rising edge

	// make and start a task1
	create_task(task1,NULL,4096,0);

	// make and start a task2
	create_task(task2,NULL,4096,0);

	// this loop only updates the switch state and checks if
	// data was recently sent in a message
	while(1) {
        WDT_2S();				// reset the watch dog timer
        sleep(10);

#ifdef NORESET
		inaction = 0;			//stops the WDT from resetting the system
#endif
		if(datasent) {			// If data was sent
			datasent = 0;		// data was sent reset datasent
			inaction = 0;		// counter reset
		}else {
			inaction++;			// nothing is done so increment the inaction counter
		}
        if(inaction % 10000 == 9999) {
        	sendRIMtype2("ALIVE",5);
#ifdef DEBUGOUTPUT
			printf("system inactive inaction = %d\n",inaction);
#endif
		}
		if(inaction == 20000) {
#ifdef DEBUGOUTPUT
			printf("system inactive\n");
#endif
		}
		if(inaction > 30000) {
#ifdef DEBUGOUTPUT
			printf("system is being reset\n");
#endif
			sleep(4000);
		}
	} // end of while(1)
} // end of main()




/*
 *  void readFromGPS(void)
 *
 *  Reads data from the Serail port E into the curent sample buffer
 */
void readFromGPS(void) {
	char c;
	while(SerialRecvCountE() > 0) {
			c = SerialGetcE();
			// add the char to the GPS buffer atomicly by disabling interrupts
			curentSB->GPS[curentSB->GPS_index] = c;	// add char to the buffer
            curentSB->GPS_index++;					// increment the index
			curentSB->GPS[curentSB->GPS_index] = 0;	// keep GPS buffer a valid string
	}
}

/*
 *   void setRTCfromLastSB(void)
 * Trys to set the RTC from the data in the last SB
 *
 */
void setRTCfromLastSB(void) {
	int i;
	int Y;
	int Month;
	int D;
	int H;
	int M;
	int S;
	int m;

	//changed (may 28,2014) check for the last GPZDA message
	// check that the GPS has enough new data to contain a GPZDA message
	// then try to parse the time out of the GPZDA message
    for(i = lastSB->GPS_index; i > 0; --i) {
		if( lastSB->GPS_index > 18) {
			//format of GPZDA Message from the UBLOX GPS(S has 4 seg digits)
			if( sscanf((lastSB->GPS+i),"GPZDA,%2d%2d%2d.%2d,%d,%d,%d",&H,&M,&S,&m,&D,&Month,&Y) == 7) {
				m = m / 100;   //Two dec. places
				timeage = 0;    // the time is now new
                RTCsetTime(Y, Month - 1, D, H, M, S ,m);
           	//format of the GPZDA Message from the Trimble GPS(S has 5 seg digits)
			}else if(sscanf((lastSB->GPS+i),"GPZDA,%2d%2d%2d.%3d,%d,%d,%d",&H,&M,&S,&m,&D,&Month,&Y) == 7) {
               	timeage = 0;    // the time is now new
				m = m / 1000;  // Three dec. places
                RTCsetTime(Y, Month - 1, D, H, M, S ,m);
			}
		}
	}
}


/*
 *	void task1(void)
 *
 *	GPSMessageReceiver
 */
void _nearcall task1(void) {
	int i;
    while(1){
    	sleep(1);
		//read the data from port E to the curent GPS buffer
		readFromGPS();
		//sets the RTC
        if(!wasSent(lastSB)) {
#ifdef DEBUGOUTPUT
        	printf("!!!!!!!!!!!!!!PULSE!!!!!!!!!!!!!!!!!\n");
#endif
        	setRTCfromLastSB();
            //sleep for 900 ms but check gps buffer for new data
			//every 100 ms
            for(i = 0; i < 9; ++i){
                readFromGPS(); 
            	sleep(100);
			}
		}
	}
}


/*
 *	int task2(int val)
 *
 *	RIM MessageBuilder
 *
 *	determines the start time of the last second and
 *	builds the sample data into messages that are sent out.
 */
void _nearcall task2(void) {
	int Y;      		// Year
	int Month;			// Month
	int D;				// Day
	int H;				// Hour
	int M;      		// Min
	int S;      		// Sec

	//int lastS = 11;

	timeage	= 10000;	// number of sec sence the time was last set
						// initaly set to 10000(too old)

	while(1){ 			// loop forever
        sleep(1);

       	// if the timeset was not set to 1 and timeage is less then MAXTIMEAGE
		// use the RTC
        //if(!wasSent(lastSB) && (timeage < MAXTIMEAGE) && (lastS != RTCtime()->tm_sec)) {
        if(!wasSent(lastSB) && (timeage < MAXTIMEAGE)) {
			sleep(60); // allow the clock to be set by the gps if there is a clock ajustment

			// delay to prevent the stations from sending data at the same time this devides
			// the second into TIMESLICES parts
			// 950 is used instead of 1000 to allow for a little more white space at the end of the second
			// to prevent some data from being lost due to the sample buffers being switched.
			sleep( (900 / TIMESLICES) * (timeSlot % TIMESLICES) );

	       	Y     = RTCtime()->tm_year + 1900;
           	//changed to keep rtc from rolling over to the next year
           	//Month =	RTCtime()->tm_mon;
            Month =	RTCtime()->tm_mon + 1;
           	D     = RTCtime()->tm_mday;
           	H     = RTCtime()->tm_hour;
           	M     = RTCtime()->tm_min;
			//S =	lastS = RTCtime()->tm_sec;
            S     =	RTCtime()->tm_sec;
			++timeage;	// increment timeage

			// send a RIM type 0 message?  // RIM type 0 is no longer used
            if(sendType0 ) {
				WDT_2S();
            	sendRIMtype0(Y, Month, D, H, M, S); // try sending type0 message
			}

			//changed to send the gps buffer when ever there is a GPS message
			//this should allow for greater compatability with GPS receivers.
            //if(timeset && sendType4 && timeage == 1) {
			if(sendType4 && (lastSB->GPS_index > 0)) {
				WDT_2S();
            	sendRIMtype4(Y, Month, D, H, M, S); // try sending type4 (GPS) messages
			}

            if(sendType8 ) {
				WDT_2S();
				//check to see if it is time to send type 8 message at reduced data rate
                if(scandatarate == 1) {  // every second
            		sendRIMtype8(Y, Month, D, H, M, S); // try sending type8 (8 channel scan) messages
                }
				if((scandatarate == 2) && (S % 10 == 0)) { //every 10 seconds
                	sendRIMtype8(Y, Month, D, H, M, S); // try sending type8 (8 channel scan) messages
				}
				if((scandatarate == 3) && (S % 60 == 0)) { //every 60 seconds
                	sendRIMtype8(Y, Month, D, H, M, S); // try sending type8 (8 channel scan) messages
				}
				if((scandatarate == 4) && (S % 60 == 0) && (M % 10 == 0)) { //ever 10 min
                	sendRIMtype8(Y, Month, D, H, M, S); // try sending type8 (8 channel scan) messages
				}
			}

			if(sendType12 ) {
				WDT_2S();
            	sendRIMtype12(Y, Month, D, H, M, S); // try sending type8 (8 channel scan) messages
			}

			// prevents this sample block from being sent again
			markSent(lastSB);
    	}// end if
	}// end while(1)
}
/*
 * FUNCTIONS
 */

/*
 *  int far sendRIMtype0(int Y, int Month, int D, int H,
 *				int M, int S)
 *
 *	builds and sends a RIM type 0 message
 *	message is built out of lastSB
 */
int sendRIMtype0(int Y, int Month, int D, int H,
				int M, int S) {
	int i;
	int j;
	char *cp;								// char pointer
    char mbuff[ 37 + 2 * MAX_SENSOR_RATE ];	// size of the header and
											// maximum size of the data
	char timebuff[20];
    float sr;				// to hold sample rate as a float

	//build a RIM type 0 message
   	i = 0;					// index into message being built
	//Delimiter
	memcpy(mbuff+i, "\r\nSOP", 5);
	i += 5;
	//Message type
	mbuff[i++] = 0;			// message type 0
	//Serial number
	cp = &serailNumber;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//Sample rate
    sr = (float)samplerate; // convert to float
	cp = &sr;				// get a void pointer to sr
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	mbuff[i++] = cp[2];
	mbuff[i++] = cp[3];
	//Sample count
	cp = &(lastSB->ADC0_index);
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	// Time string
    sprintf(timebuff,"%d%0.2d%0.2d%0.2d%0.2d%0.2d.000",Y,Month,D,H,M,S);
    memcpy(mbuff+i,timebuff,18);	//copy the time buffer into mbuff
	i += 18;
	//samples
	for(j = 0; j < lastSB->ADC0_index; ++j) {
       	//send each sample
		cp = &(lastSB->ADC0[j]);
		mbuff[i++] = cp[0];
		mbuff[i++] = cp[1];
	}
	memcpy(mbuff+i,"EOP\r\n", 5);
	i += 5;
    //send message
	serFwrite(mbuff,i);
	//message sent
	datasent = 1;			// signal that a message was sent
#ifdef DEBUGOUTPUT
	printf("message type 0 sent %d bytes at %s\n",i,timebuff);
#endif
	return 1;
}

/*
 *  int far sendRIMtype2(char *message,int msize);
 *
 *	builds and sends a RIM type 2 message
 *  to send untimed informational messages
 *	max 128 bytes

 */
int sendRIMtype2(char *message,int msize) {
	int i;
	int s;
	char *cp;
    char mbuff[ 13 + 129];	// header size and


	//build a RIM type 2 message
   	i = 0;					// index into message being built
	//Delimiter
	memcpy(mbuff+i, "\r\nSOP", 5);
	i += 5;
	//Message type
	mbuff[i++] = 2;			// message type 0
	//Serial number
	cp = &serailNumber;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	memcpy(mbuff+i,message,msize);
	i = i + msize;

	memcpy(mbuff+i,"EOP\r\n", 5);
	i += 5;
    //send message
	serFwrite(mbuff,i);
	//message sent
	datasent = 1;			// signal that a message was sent
#ifdef DEBUGOUTPUT
	printf("message type 2 sent %d bytes \n",i);
#endif
	return 1;
}


/*
 *  int sendRIMtype4(int Y, int Month, int D, int H,
 *				int M, int S)
 *
 *	Sends the GPS data that was received from the GPS
 *	removes the GPZDA message
 */
int sendRIMtype4(int Y, int Month, int D, int H,
				int M, int S) {
	int i;
	int j;
	int gpStart;
	int dsize;
	int l;
	char *cp;
	char timebuff[20];
	char mbuff[33 + MAX_GPS_BUFFER];

	// Build a RIM type 4 message(GPS)
   	i = 0;					// Index into message being built
	// Delimiter
	memcpy(mbuff+i, "\r\nSOP", 5);
	i += 5;
	// Message type
	mbuff[i++] = 4;			// Message type 4

	// Station number
	cp = &serailNumber;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	// Start time
    sprintf(timebuff,"%d%0.2d%0.2d%0.2d%0.2d%0.2d.000",Y,Month,D,H,M,S);
    memcpy(mbuff+i,timebuff,18);	// copy the time buffer into mbuff
	i += 18;

	//always use the whole gps buffer 1/22/2014
    dsize = lastSB->GPS_index;
	cp = &dsize;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	for(j=0; j < lastSB->GPS_index ;++j){
    	mbuff[i++] = lastSB->GPS[j];
	}

#ifdef DEBUGOUTPUT
    printf("\n");
#endif

    memcpy(mbuff+i,"EOP\r\n", 5);
	i += 5;

	// Send message
	serFwrite(mbuff,i);

	mbuff[i] = 0;

	// Message sent
	datasent = 1;		// Signal that a message was sent

#ifdef DEBUGOUTPUT
	printf("%s",mbuff);
	printf("message type 4 sent %d bytes at %s",i,timebuff);
#endif

	return 1;
}

/*
 *  int sendRIMtype8(int Y, int Month, int D, int H,
 *				int M, int S)
 *
 *	Sends a near instant scan of all 8 ADC channels
 */
int sendRIMtype8(int Y, int Month, int D, int H,
				int M, int S) {

	char mbuff[ 37 + 2 * 8 ];	// size of the header and data
	char timebuff[20];
    char *cp;
	int i;

	unsigned int ch0;
	unsigned int ch1;
	unsigned int ch2;
	unsigned int ch3;
	unsigned int ch4;
	unsigned int ch5;
	unsigned int ch6;
	unsigned int ch7;

	//read the samples as are this second
    EnterCritical();
	ch0 = readADC(0);
	ch1 = readADC(1);
	ch2 = readADC(2);
	ch3 = readADC(3);
	ch4 = readADC(4);
	ch5 = readADC(5);
	ch6 = readADC(6);
	ch7 = readADC(7);
	ExitCritical();

   	i = 0;					// Index into message being built

	// Delimiter "Start Of Packet!"
	memcpy(mbuff+i, "\r\nSOP", 5);
	i += 5;

	// Message type "type 8"
	mbuff[i++] = 8;			// Message type 8

	// Station number
	cp = &serailNumber;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	// Start time
    sprintf(timebuff,"%d%0.2d%0.2d%0.2d%0.2d%0.2d.000",Y,Month,D,H,M,S);
    memcpy(mbuff+i,timebuff,18);	// copy the time buffer into mbuff
	i += 18;

	//ch0
    cp = &ch0;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//ch1
    cp = &ch1;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//ch2
    cp = &ch2;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//ch3
    cp = &ch3;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

    //ch4
    cp = &ch4;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//ch5
    cp = &ch5;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

    //ch6
    cp = &ch6;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];

	//ch7
    cp = &ch7;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
    cp = &ch0;

    memcpy(mbuff+i,"EOP\r\n", 5);
	i += 5;

	// Send the message
	serFwrite(mbuff,i);

    datasent = 1; // set datasent to true

#ifdef DEBUGOUTPUT
	printf("message type 8 sent %d bytes at %s\n",i,timebuff);
#endif

	return 1;
}


/*
 *  int far sendRIMtype12(int Y, int Month, int D, int H,
 *				int M, int S)
 *
 *	builds and sends a RIM type 0 message
 *	message is built out of lastSB
 */
int sendRIMtype12(int Y, int Month, int D, int H,
				int M, int S) {
    //ch0
	if(sendCh0){
		sendRIMtype12chan(Y,Month,D,H,M,S,0);
	}
	//ch1
    if(sendCh1){
		sendRIMtype12chan(Y,Month,D,H,M,S,1);
	}
	//ch2
	if(sendCh2){
		sendRIMtype12chan(Y,Month,D,H,M,S,2);
	}
	//ch3
	if(sendCh3){
		sendRIMtype12chan(Y,Month,D,H,M,S,3);
	}
	//ch4
	if(sendCh4){
		sendRIMtype12chan(Y,Month,D,H,M,S,4);
	}
	//ch5
	if(sendCh5){
		sendRIMtype12chan(Y,Month,D,H,M,S,5);
	}
	//ch6
	if(sendCh6){
		sendRIMtype12chan(Y,Month,D,H,M,S,6);
	}
	//ch7
	if(sendCh7){
		sendRIMtype12chan(Y,Month,D,H,M,S,7);
	}

    return 1;
}

/*
 *  int sendRIMtype12chan(int Y, int Month, int D, int H,
 *				int M, int S, int chan)
 *
 *	builds and sends a RIM type 12 message
 *	message is built out of lastSB
 */
int sendRIMtype12chan(int Y, int Month, int D, int H, int M, int S, int chan) {
	int i;
	int j;
	char *cp;								// char pointer
    char mbuff[ 37 + 2 * MAX_SENSOR_RATE ];	// size of the header and
											// maximum size of the data
	char timebuff[20];
    float sr;				// to hold sample rate as a float

	int index;
	int *samples;

	switch(chan) {
    	case 0:
    		index = lastSB->ADC0_index;
    		samples = lastSB->ADC0;
    		break;
    	case 1:
            index = lastSB->ADC1_index;
    		samples = lastSB->ADC1;
    		break;
    	case 2:
            index = lastSB->ADC2_index;
    		samples = lastSB->ADC2;
    		break;
    	case 3:
            index = lastSB->ADC3_index;
    		samples = lastSB->ADC3;
    		break;
    	case 4:
            index = lastSB->ADC4_index;
    		samples = lastSB->ADC4;
    		break;
    	case 5:
            index = lastSB->ADC5_index;
    		samples = lastSB->ADC5;
    		break;
    	case 6:
            index = lastSB->ADC6_index;
    		samples = lastSB->ADC6;
    		break;
    	case 7:
    		index = lastSB->ADC7_index;
    		samples = lastSB->ADC7;
    		break;
    	default:
    		return 0;
	}



	//build a RIM type 12 message
   	i = 0;					// index into message being built
	//Delimiter
	memcpy(mbuff+i, "\r\nSOP", 5);
	i += 5;
	//Message type
	mbuff[i++] = 12;			// message type 12
	//Serial number
	cp = &serailNumber;
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	//Channel number
    cp = &chan;
    mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	//Sample rate
    sr = (float)samplerate; // convert to float
	cp = &sr;				// get a void pointer to sr
	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	mbuff[i++] = cp[2];
	mbuff[i++] = cp[3];

	//Sample count
	cp = &index;

	mbuff[i++] = cp[0];
	mbuff[i++] = cp[1];
	// Time string
    sprintf(timebuff,"%d%0.2d%0.2d%0.2d%0.2d%0.2d.000",Y,Month,D,H,M,S);
    memcpy(mbuff+i,timebuff,18);	//copy the time buffer into mbuff
	i += 18;
	//samples
	for(j = 0; j < index; ++j) {
       	//send each sample
		cp = &(samples[j]);
		mbuff[i++] = cp[0];
		mbuff[i++] = cp[1];
	}
	memcpy(mbuff+i,"EOP\r\n", 5);
	i += 5;
    //send message
	serFwrite(mbuff,i);
	//message sent
	datasent = 1;			// signal that a message was sent
#ifdef DEBUGOUTPUT
	printf("message type 12 sent %d bytes at %s\n",i,timebuff);
#endif
	return 1;
}

/*
 *	void serFwrite(char *buf, int bufsize);
 *
 *	writes the buffer out serial port F
 */
void serFwrite(char buf[], int bufsize) {
	int i;
	for(i = 0; i < bufsize; ++i) {
		SerialPutcF(buf[i]);
	}

}

/*
 *  void setSampleRate(int rate);
 *
 *  sets the sample rate and setsup the timer B ISR
 */
void setSampleRate(int rate) {
    unsigned long timerBdelay;
	samplerate = rate;
	//check if rate is to fast
	if(rate > MAX_SENSOR_RATE){
#ifdef DEBUGOUTPUT
        printf("Sample rate %d is to fast\n",rate);
#endif
    	rate = MAX_SENSOR_RATE;
	}

	//convert rate to delay time
    //1,000,000uS = 1 S
	timerBdelay = (long) 1000001 / (long) samplerate;

    //setup timer B to interupt every timerBdelay uS
	//interupt level 2
    TimerBInit( timerBdelay, 2);
#ifdef DEBUGOUTPUT
    printf("Sample rate set to %d S/s\n",rate);
#endif
}

/*  struct tm * far RTCtime(void)
 *
 *  reads the time from the real Time Clock and
 *	returns a tm
 *
 */
struct tm * far RTCtime(void) {
	time_t now;
	now = readRTC(0);
#ifdef DEBUGOUTPUT
    printf("RTC read       = %d\n",now);
#endif
	return( gmtime(&now) );
}

/*	void RTCsetTime(int Y, int Month, int D, int H,
 *			int M, int S)
 *
 *	sets the Real Time Clock
 *
 */
void RTCsetTime(int Y, int Month, int D, int H,
			int M, int S, int m) {

	struct tm now;
	struct tm dbug_t_pre;
	struct tm dbug_t_post;
	time_t now_t;

	time_t pre_t;
	time_t post_t;
	time_t reset_t;

	//fill out the time structure
	now.tm_year = Y - 1900;
	now.tm_mon = Month;
	now.tm_mday = D;
	now.tm_hour = H;
	now.tm_min = M;
	now.tm_sec = S;

	//make a time object
	now_t = mktime(&now);

	//if m > 98 add a second
    if(m > 98) {
		now_t++;
	}

	pre_t = readRTC(0);

    // OCTOBER 9, 2014
	//This is hopfully the majic bullet that solves the off by just one second problem
	// SET THE TIME RESETTING All 6 Bytes of the RTC
	outi(RTCCR,0x40); //arm RTC for a reset
	outi(RTCCR,0x80); //reset the RTC
    reset_t = readRTC(0);
	writeRTC(now_t);  //set the RTC to the curennt time

    post_t = readRTC(0);
#ifdef DEBUGOUTPUT
    //time debug info
	printf("\n");
	printf("RTC pre-set    = %d\n",pre_t);
	printf("RTC post reset = %d\n",reset_t);
	printf("RTC set to     = %d\n",now_t);
	printf("RTC post-set   = %d\n",post_t);
#endif

	//all done :)
}

/*
 * 	Configuration menu system
 */

/*
 *	void serFwriteStr(char *buf);
 *
 *	writes the buffer out serial port F
 */
void serFwriteStr(char buff[]) {
	int i;
	int maxsize = 256;
#ifdef DEBUGOUTPUT
	printf("%s",buff);
#endif
	for(i = 0; i < maxsize; ++i){
    	if(buff[i] == 0) {
    		return;
    	}
    	SerialPutcF(buff[i]);
	}
}

int getInput() {
	char in;
	char buff[30];
	int i = 0;
	int ret;
    while(1){
        WDT_2S();					//reset the watchdog
    	if(SerialRecvCountF() > 0){
    		in = SerialGetcF();
            buff[i]     = in;
			buff[i + 1] = 0;
			SerialPutcF(buff[i]);
			++i;
    	}
    	if( in == 13)   //check for ENTER
         	break;
        if( i > 29)
			break;
		sleep(10);
	}
	if(sscanf(buff,"%d",&ret)){
		return ret;
	}else {
    	return -1;
	}
}

void writeOutput(char buff[]) {
	serFwriteStr(buff);
}

int checkminmax(int input, int min, int max){

	if( input > max || input < min ) {
    	writeOutput("\r\nInput out of bounds\r\n");
		return 0;
	}else {
    	return 1;
	}
}


//prints the main menu
int configurationMenu(void) {
	int selection = 0;
    int userin;
	char buff[255];
	while(1){


		writeOutput("\r\n---------------------------------------------------------\r\n");
        writeOutput("\r\n                         ");
        writeOutput(VERSION);
        writeOutput("\r\n");
		writeOutput("                          MAIN MENU\n");
		writeOutput("\r\n");
		writeOutput("---------------------------------------------------------\r\n");
        sprintf(buff,"     1.  Station Number - %d\r\n",serailNumber);
		writeOutput(buff);
		sprintf(buff,"     2.  Time Slot      - %d\r\n",timeSlot);
        writeOutput(buff);
		sprintf(buff,"     3.  Sample Rate    - %d\r\n",samplerate);
        writeOutput(buff);
		sprintf(buff,"     4.  BaudRate       - %d - ",dataBaudrate);
        writeOutput(buff);
		if(dataBaudrate == 1){
        	writeOutput("19,200\r\n");
		}else if(dataBaudrate == 2){
         	writeOutput("38,400\r\n");
		}else if(dataBaudrate == 3){
         	writeOutput("57,600\r\n");
		}else if(dataBaudrate == 4){
         	writeOutput("115,200\r\n");
		}else {
        	writeOutput("\r\n");
		}
		sprintf(buff,"     5.  GPS BaudRate   - %d - ",GPSBaudrate);
        writeOutput(buff);
		if(GPSBaudrate == 1){
        	writeOutput("9600\r\n");
		}else if(GPSBaudrate == 2){
         	writeOutput("19,200\r\n");
		}else if(GPSBaudrate == 3){
         	writeOutput("115,200\r\n");
		}else {
        	writeOutput("\r\n");
		}
		sprintf(buff,"     6.  Scan Data Rate - %d - ",scandatarate);
        writeOutput(buff);
		if(scandatarate == 1){
        	writeOutput("every second\r\n");
		}else if(scandatarate == 2){
         	writeOutput("every 10 seconds\r\n");
		}else if(scandatarate == 3){
         	writeOutput("every minute\r\n");
		}else if(scandatarate == 4){
         	writeOutput("every 10 minutes\r\n");
		}else {
        	writeOutput("\r\n");
		}
		writeOutput("     7.  Seismic Menu\r\n");
	    writeOutput("     8.  Exit\r\n");
		writeOutput("---------------------------------------------------------\r\n");
		writeOutput(">");

		selection = getInput();

    	if(selection == 1) {
        	writeOutput("\r\nEnter new Station Number [0 - 255] >");
            userin = getInput();
			if(checkminmax(userin,0,255)) {
            	serailNumber = userin;
            	setConfiguration();
            }
        	writeOutput("\r\n");
    	}

       	if(selection == 2) {
        	writeOutput("\r\nEnter new Time Slot [0 - 7] >");
        	userin = getInput();
        	if(checkminmax(userin,0,TIMESLICES-1)) {
            	timeSlot = userin;
            	setConfiguration();
            }
			writeOutput("\r\n");
       	}

       	if(selection == 3) {
        	writeOutput("\r\nEnter new Sample Rate [1 - 300] >");
			userin = getInput();
			if(checkminmax(userin,1,300)){
            	samplerate = userin;
            	setConfiguration();
			}
			writeOutput("\r\n");
       	}

       	if(selection == 4) {
            writeOutput("\r\n1. 19200");
            writeOutput("\r\n2. 38400");
			writeOutput("\r\n3. 57600");
            writeOutput("\r\n4. 115200");
        	writeOutput("\r\nEnter new baudrate [1 - 4] >");
			userin = getInput();
			if(checkminmax(userin,1,4)) {
            	dataBaudrate = userin;
            	setConfiguration();
			}
			writeOutput("\r\n");
       	}

		if(selection == 5) {
            writeOutput("\r\n1. 9600");
            writeOutput("\r\n2. 19200");
			writeOutput("\r\n3. 115200");
        	writeOutput("\r\nEnter new GPS baudrate [1 - 3] >");
			userin = getInput();
			if(checkminmax(userin,1,3)) {
            	GPSBaudrate = userin;
            	setConfiguration();
			}
			writeOutput("\r\n");
       	}


        if(selection == 6) {
            writeOutput("\r\n1. every second");
            writeOutput("\r\n2. every 10 seconds");
			writeOutput("\r\n3. every minute");
            writeOutput("\r\n4. every 10 minutes");
        	writeOutput("\r\nEnter new rate [1 - 4] >");
			userin = getInput();
			if(checkminmax(userin,1,4)) {
            	scandatarate = userin;
            	setConfiguration();
			}
			writeOutput("\r\n");
       	}

       	if(selection == 7) {
        	analogMenu();
       	}

       	if(selection == 8) {
			return 1;
       	}
	}

}

//prints the siesmic menu
int analogMenu(void) {

    int selection = 0;
    int userin;
    while (1) {
		writeOutput("                      SEISMIC MENU\r\n");
		writeOutput("\r\n");

		writeOutput("     0.  Channel 0 - ");
		if(sendCh0){
	    	writeOutput("ON ");
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

	    writeOutput("     1.  Channel 1 - ");
		if(sendCh1){
	    	writeOutput("ON ") ;
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     2.  Channel 2 - ");
		if(sendCh2){
	    	writeOutput("ON ");
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     3.  Channel 3 - ");
		if(sendCh3){
	   	 	writeOutput("ON ");
	   	 	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     4.  Channel 4 - ");
		if(sendCh4){
	    	writeOutput("ON ");
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     5.  Channel 5 - ");
		if(sendCh5){
	    	writeOutput("ON ");
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     6.  Channel 6 - ");
		if(sendCh6){
	    	writeOutput("ON ");
	    	writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     7.  Channel 7 - ");
		if(sendCh7){
	    	writeOutput("ON ");
            writeOutput("\r\n");
		}else {
			writeOutput("OFF\r\n");
		}

		writeOutput("     8.  Return to Main menu\r\n");
		writeOutput("> ");

	    selection = getInput();

		if(selection == 0){
        	if (sendCh0) {
        		sendCh0 = 0;
        	} else {
				sendCh0 = 1;
        	}
			setConfiguration();
		}

		if(selection == 1){
        	if (sendCh1) {
        		sendCh1 = 0;
        	} else {
				sendCh1 = 1;
        	}
			setConfiguration();
		}

		if(selection == 2){
        	if (sendCh2) {
        		sendCh2 = 0;
        	} else {
				sendCh2 = 1;
        	}
			setConfiguration();
		}

        if(selection == 3){
        	if (sendCh3) {
        		sendCh3 = 0;
        	} else {
				sendCh3 = 1;
        	}
			setConfiguration();
		}

		if(selection == 4){
        	if (sendCh4) {
        		sendCh4 = 0;
        	} else {
				sendCh4 = 1;
        	}
			setConfiguration();
		}

		if(selection == 5){
        	if (sendCh5) {
        		sendCh5 = 0;
        	} else {
				sendCh5 = 1;
        	}
			setConfiguration();
		}

		if(selection == 6){
        	if (sendCh6) {
        		sendCh6 = 0;
        	} else {
				sendCh6 = 1;
        	}
			setConfiguration();
		}

		if(selection == 7){
        	if (sendCh7) {
        		sendCh7 = 0;
        	} else {
				sendCh7 = 1;
        	}
			setConfiguration();
		}

		if(selection == 8){
        	return 1;
    	}
    }
}


// set default configuration
//
void setDefualtConfiguration(void) {

	int ret;

	serailNumber    = 0;
	differentalCh3  = 0;
	differentalCh4  = 0;
	differentalCh5  = 0;
	differentalCh6  = 0;
	differentalCh7  = 0;

	ret = setConfiguration();
}

// **************************************************************************
//
// reads the settings from flash
//
// **************************************************************************
//
int readConfiguration(void) {

    int ret;
    user_block_s working_ub;

	ret = readUserBlock((void *)&working_ub, 0x0000, sizeof(working_ub));

    serailNumber 	= working_ub.stationnumber;
	timeSlot        = working_ub.timeslot;
    samplerate      = working_ub.samplerate;
    dataBaudrate    = working_ub.baudrate;
	GPSBaudrate     = working_ub.GPSbaudrate;
	scandatarate    = working_ub.scanrate;


    sendType4       = working_ub.sendGPS;
	sendType8       = working_ub.sendScan;
	sendType12      = working_ub.sendSeismic;

    sendCh0         = working_ub.sendCH0;
	differentalCh0   = working_ub.differentalCH0;
   	sendCh1         = working_ub.sendCH1;
	differentalCh1   = working_ub.differentalCH1;
    sendCh2         = working_ub.sendCH2;
	differentalCh2   = working_ub.differentalCH2;
    sendCh3         = working_ub.sendCH3;
	differentalCh3   = working_ub.differentalCH3;
    sendCh4         = working_ub.sendCH4;
	differentalCh4   = working_ub.differentalCH4;
    sendCh5         = working_ub.sendCH5;
	differentalCh5   = working_ub.differentalCH5;
	sendCh6         = working_ub.sendCH6;
	differentalCh6   = working_ub.differentalCH6;
    sendCh7         = working_ub.sendCH7;
	differentalCh7   = working_ub.differentalCH7;


	//check if the setting make sence if not set to default values
    if(timeSlot > TIMESLICES || timeSlot < 0) {
    	setDefualtConfiguration();
	}
    if(samplerate > 300 || samplerate < 0) {
    	setDefualtConfiguration();
	}
    if(dataBaudrate > 4 || samplerate < 1) {
    	setDefualtConfiguration();
	}


	return ret;
}


// **************************************************************************
//
// writes the settings to flash
//
// **************************************************************************
//
int setConfiguration(void) {
    int offset;
	char *cptr1, *cptr2;
    user_block_s working_ub;

	working_ub.stationnumber  = serailNumber;
    working_ub.timeslot       = timeSlot;
    working_ub.samplerate     = samplerate;
	working_ub.baudrate		  = dataBaudrate;
    working_ub.GPSbaudrate	  = GPSBaudrate;
	working_ub.scanrate       = scandatarate;

	working_ub.sendGPS        = sendType4;
	working_ub.sendScan       = sendType8;
	working_ub.sendSeismic    = sendType12;

	working_ub.sendCH0        = sendCh0;
	working_ub.differentalCH0  = differentalCh0;
   	working_ub.sendCH1        = sendCh1;
	working_ub.differentalCH1  = differentalCh1;
    working_ub.sendCH2        = sendCh2;
	working_ub.differentalCH2  = differentalCh2;
    working_ub.sendCH3        = sendCh3;
	working_ub.differentalCH3  = differentalCh3;
    working_ub.sendCH4        = sendCh4;
	working_ub.differentalCH4  = differentalCh4;
    working_ub.sendCH5        = sendCh5;
	working_ub.differentalCH5  = differentalCh5;
	working_ub.sendCH6        = sendCh6;
	working_ub.differentalCH6  = differentalCh6;
    working_ub.sendCH7        = sendCh7;
	working_ub.differentalCH7  = differentalCh7;


	cptr1 = (char *)&working_ub;        // get address of start of user block
    cptr2 = &working_ub.stationnumber;  // get address of stationnumber element
    offset = cptr2-cptr1;               // offset into user block structure
                                        // of where to start writing at.

	return(writeUserBlock(offset, (void *)cptr2, (sizeof(working_ub)-offset)));
}


int c_received(void){
	char c;
	int t;
	int reationtime;
    reationtime = 10;
    writeOutput("\r\n");
	for(t = reationtime; t > 0; --t){
		writeOutput(".");
        WDT_2S();
		sleep(1000);   //wait 1 S
		// check if a c was received
		while(SerialRecvCountF() > 0) {
			c = SerialGetcF();
			SerialPutcF(c);
			if(c == 'c')
				return 1;
		}
	}
	//no c was not received in 2 S so return 0
	return 0;
}


/*
 *
 */
void sendStartupMessage(void) {
	//send the message
    writeOutput("Press \"c\" to enter configuration mode\r\n");
}

/*
 *
 */
void sendSysStartmessage(void) {
	//send the message
	writeOutput("System starting\r\n");
    //delay for chars to be printed
    WDT_2S();
	sleep(1000);   //wait 1 S
}

/*
 *
 */
void readConfig(void){
	readConfiguration();
    sendStartupMessage();
#ifdef DEBUGOUTPUT
	printf("checking for c\n");
#endif

	if(c_received()){
		configurationMenu();

#ifdef DEBUGOUTPUT
		printf("c pressed\n");
#endif

	}else {

#ifdef DEBUGOUTPUT
		printf("c not pressed\n");
#endif

	}
	sendSysStartmessage();
}


/************************************************************************
 *
 *	ISRs
 *
 ***********************************************************************/

/*
 *	void _fastint near _nostep GPSisr(void)
 *
 *	GPS timing pulse ISR
 *
 *	This ISR handles the one second time tick put out by the gps
 */
void _fastint near _nostep GPSisr(void) {
    OneSecBuff	*swap;
	//clear lastSB
   	clearBuffer(lastSB);
	//swap curentSB and lastSB (pointers)
    swap = curentSB;
    curentSB = lastSB;
	lastSB = swap;
}

/*
 *	void _near _nostep TimerBRoutine(void)
 *
 *	Sampling isr
 *
 *	This isr takes samples into the curent sample buffer when trigered
 *	by timer B.
 */
void _near _nostep TimerBRoutine(void) {

	// this is a inline of TakeADCSamples(void)
	// take a sample from each ADC channel

	// With the new analog front end (FEN board) that kelly and Pat made
	// The signals are super duper  good

	if(sendCh0) {
	    //read Channel 0
		curentSB->ADC0[curentSB->ADC0_index] = readADC(0);
		curentSB->ADC0_index++;
		if(curentSB->ADC0_index > MAX_SENSOR_RATE) {
			curentSB->ADC0_index = 0;	//clear the buffer
		}
	}

	if(sendCh1) {
		//read Channel 1
	    curentSB->ADC1[curentSB->ADC1_index] = readADC(1);
		curentSB->ADC1_index++;
		if(curentSB->ADC1_index > MAX_SENSOR_RATE) {
			curentSB->ADC1_index = 0;	//clear the buffer
		}
	}

    if(sendCh2) {
		//read Channel 2
	    curentSB->ADC2[curentSB->ADC2_index] = readADC(2);
		curentSB->ADC2_index++;
		if(curentSB->ADC2_index > MAX_SENSOR_RATE) {
			curentSB->ADC2_index = 0;	//clear the buffer
		}
	}


    if(sendCh3)  {
		//read Channel 3
	    curentSB->ADC3[curentSB->ADC3_index] = readADC(3);
		curentSB->ADC3_index++;
		if(curentSB->ADC3_index > MAX_SENSOR_RATE) {
				curentSB->ADC3_index = 0;	//clear the buffer
		}
	}

	if(sendCh4) {
	    //read Channel 4
	    curentSB->ADC4[curentSB->ADC4_index] = readADC(4);
		curentSB->ADC4_index++;
		if(curentSB->ADC4_index > MAX_SENSOR_RATE) {
			curentSB->ADC4_index = 0;	//clear the buffer
		}
	}

	if(sendCh5) {
	    //read Channel 5
	    curentSB->ADC5[curentSB->ADC5_index] = readADC(5);
		curentSB->ADC5_index++;
		if(curentSB->ADC5_index > MAX_SENSOR_RATE) {
			curentSB->ADC5_index = 0;	//clear the buffer
		}
	}

    if(sendCh6) {
		//read Channel 6
	    curentSB->ADC6[curentSB->ADC6_index] = readADC(6);
		curentSB->ADC6_index++;
		if(curentSB->ADC6_index > MAX_SENSOR_RATE) {
			curentSB->ADC6_index = 0;	//clear the buffer
		}
	}

	if(sendCh7) {
		//read Channel 7
	    curentSB->ADC7[curentSB->ADC7_index] = readADC(7);
		curentSB->ADC7_index++;
		if(curentSB->ADC7_index > MAX_SENSOR_RATE) {
			curentSB->ADC7_index = 0;	//clear the buffer
		}
	}
}
