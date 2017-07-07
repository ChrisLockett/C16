/*
    userBlock   - Supports Dyanmic C user block functions providing this
    functionality on top of Softools FlashBlock functions.

    For Softools Control Cross-C ANSI C Compiler.
    Copyright (c) 1992-2005 Softools, Inc.  All rights reserved.
*/

// Defines the User Block data structure
//
#ifndef __USERBLOCK_H
#define __USERBLOCK_H


// This User Block example is based on using a 512k flash with 4k sectors.
// Put the 2 controlling defined constants at the top of this file so they
// can easily be found and changed.

//=============================================================================
// Define USER_BLOCK_AT_7E000 to define the user block at address 0x7e000.
// To define the user block at address 0x7f000 then comment out the line below
// that defines "USER_BLOCK_AT_7E000"
//
#define USER_BLOCK_AT_7E000



//=============================================================================
// When USER_BLOCK_AT_7E000 is not defined then the User block is at 0x7f000.
// This is the last 4k block of flash and this 4k block is also used by Z-World
// to define their System ID block.
// Define PRESERVE_ZID to preserve the Z-World ID block.
// To not preserve the ZID block, comment out the line below defining
// PRESERVE_ZID
//
#define PRESERVE_ZID



//=============================================================================
// define the size of available user data based on USER_BLOCK_AT_7E000 and
// PRESERVE_ZID
// If you want to define a smaller block then change BLOCK_SIZE
#define BLOCK_SIZE      0x1000  // size of block

#define UB_DATA_OFFSET  6       // offset to start of user data
#define UB_HDR_SIZE     6       // size of 3 int's for crc, marker, and size
#define ZID_SIZE        0x100   // allow 256 bytes for the Z-World ID block
                                // ZID could currently take 163 bytes so let's
                                // leave room for ZID expansion.

#ifdef USER_BLOCK_AT_7E000
    #define UB_SIZE (BLOCK_SIZE - UB_HDR_SIZE)
#else
    #ifdef PRESERVE_ZID
        #define UB_SIZE (BLOCK_SIZE - UB_HDR_SIZE-ZID_SIZE-sizeof(FB_Prefix))
    #else
        #define UB_SIZE (BLOCK_SIZE - UB_HDR_SIZE)
    #endif
#endif

//=============================================================================
// Function prototypes for functions in User_Block.c that are
// not in the SCRabbit.lib.
int initUserBlock(void);
int validateUserBlock(void);
int calcCRC(int, void *, int);


typedef struct _user_block_s
{                                   // the first 3 elements must always exist
    unsigned int    crc;            // 2 byte CRC value, Must be first element
    unsigned int    marker;         // 2 byte marker of 0x55aa
    unsigned int    size;           // size in bytes of the user block inluding
                                    // these first 3 elements

    //configuration elements to be stored in nonvolitile flash

	//station information
    unsigned char	stationnumber;
	unsigned char 	timeslot;
    unsigned int	samplerate;
	unsigned char   scanrate;

	//data to send
	unsigned char 	sendGPS;
	unsigned char	sendScan;
	unsigned char	sendSeismic;

	//baud rate
	unsigned char baudrate;
	unsigned char GPSbaudrate;

	//siesmic data to send
    unsigned char	sendCH0;
	unsigned char	differentalCH0;
   	unsigned char	sendCH1;
	unsigned char	differentalCH1;
    unsigned char	sendCH2;
	unsigned char	differentalCH2;
    unsigned char	sendCH3;
	unsigned char	differentalCH3;
    unsigned char	sendCH4;
	unsigned char	differentalCH4;
    unsigned char	sendCH5;
	unsigned char	differentalCH5;
	unsigned char	sendCH6;
	unsigned char	differentalCH6;
    unsigned char	sendCH7;
	unsigned char	differentalCH7;

} user_block_s;


//	Prototypes for compatible function support
int		readUserBlock(void *dest, unsigned addr, unsigned numbytes);
int		writeUserBlock(unsigned addr, void *source, unsigned numbytes);

#endif  // #ifndef __USERBLOCK_H
