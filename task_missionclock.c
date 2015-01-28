/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_5sec.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2009-11-02 00:45:07-08 $

******************************************************************************/
#include "main.h"
#include "task_missionclock.h"

// Pumpkin CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"
#include "csk_sd.h"
#include "thin_usr.h"

//extern void CMDS (char a[], char * saveName);
#define MAX_ASCII_ARRAY	300

extern unsigned char i2c_read_ack(void);
extern unsigned char i2c_read_nack(void);
extern char send_i2c_byte(int data);
extern void reset_i2c_bus(void);
extern void i2c_restart(void);
extern void i2c_start(void);


static char asciiA[MAX_ASCII_ARRAY];	// Maximum size of asciified array
// -----------------------------------------------------
// I don't know where else to put these utilities -MAS

/* Converts an array of characters into an ascii-fied hex array. In other
 * words, 0x04 becomes "04 ".
 *
 * aLen is the original length of the array
 * returns: a char array of nearly 3 times the original length, with the full ASCII set
 *   Note that it will NOT have the last trailing space!
 */
char* asciifiedArray(char *a, int aLen) {
	if (aLen >= MAX_ASCII_ARRAY) {
		aLen = MAX_ASCII_ARRAY;
	} else if (aLen < 0) {
		aLen = 0;
	}
    sprintf(asciiA, "%02X", a[0]);
    int i;
    for (i=1; i < aLen; i++) {
        sprintf(asciiA, "%s %02hhX", asciiA, a[i]);
    }
    return(asciiA);
}

char* asciifiedArrayNoSpace(char *a, int aLen) {
	if (aLen >= MAX_ASCII_ARRAY) {
		aLen = MAX_ASCII_ARRAY;
	} else if (aLen < 0) {
		aLen = 0;
	}
    sprintf(asciiA, "%02X", a[0]);
    int i;
    for (i=1; i < aLen; i++) {
        sprintf(asciiA, "%s%02hhX", asciiA, a[i]);
    }
    return(asciiA);
}

/* Converts an array of characters into an ascii-fied hex array of
 * length 3 characters each (i.e., %03X)
 *
 * aLen is the original length of the array
 * returns: a char array of nearly 4 times the original length, with the full ASCII set
 *   Note that it will NOT have the last trailing space!
 */
char* asciified3Array(unsigned int *a, int aLen) {
	if (aLen >= MAX_ASCII_ARRAY) {
		aLen = MAX_ASCII_ARRAY;
	} else if (aLen < 0) {
		aLen = 0;
	}
    sprintf(asciiA, "%03X", a[0]);
    int i;
    for (i=1; i < aLen; i++) {
        sprintf(asciiA, "%s %03X", asciiA, a[i]);
    }
    return(asciiA);
}

char* asciified3ArrayNoSpace(unsigned int *a, int aLen) {
	if (aLen >= MAX_ASCII_ARRAY) {
		aLen = MAX_ASCII_ARRAY;
	} else if (aLen < 0) {
		aLen = 0;
	}
    sprintf(asciiA, "%03X", a[0]);
    int i;
    for (i=1; i < aLen; i++) {
        sprintf(asciiA, "%s%03X", asciiA, a[i]);
    }
    return(asciiA);
}
//--------------------------------------------------------

void timeStamp(char* aString) {
		char a[21];
		unsigned char outpt[8];
		long i;

		//Grab time registers from RTC.
		i2c_start();
		send_i2c_byte((0x68<<1));
		send_i2c_byte(0x00);
		i2c_restart();
		send_i2c_byte((0x68<<1)+1);
		for(i=0;i<7;i++) outpt[i]=i2c_read_ack();
		outpt[7]=i2c_read_nack();
		reset_i2c_bus();
		for(i=0;i<100;i++) Nop();

		//Zero out unknown bits.
		outpt[1]=outpt[1]&0b01111111;
		outpt[2]=outpt[2]&0b01111111;
		outpt[3]=outpt[3]&0b00111111;
		//outpt[4] will be ignored.
		outpt[5]=outpt[5]&0b00111111;
		outpt[6]=outpt[6]&0b00011111;

		sprintf(a," %02X:%02X:%02X.%02X %02X/%02X/20%02X",outpt[3],outpt[2],outpt[1],outpt[0],outpt[5],outpt[6],outpt[7]);
		strcat(aString,a);
		//sprintf(aString,"%s%s",aString,a);
}

static unsigned long long missionclock;
unsigned long long getMissionClock() {
	return missionclock;
}

void getMissionClockString(char* a) {
    unsigned long long mc=missionclock;
    if(mc>=999999999) mc=999999999;
    sprintf(a,"%09llu",mc);
}

void task_missionclock(void) {
//	static unsigned long long secsBeforeDeployment=45*60; //45minutes by 60 seconds.

	missionclock=0;
	//while(!(OSReadBinSem(BINSEM_EJECTED_P))) {
	//	OS_Delay(100); 
	//	missionclock++;
	//}
//	while((!(OSReadBinSem(BINSEM_DEPLOYED_P))) && missionclock<secsBeforeDeployment) {
//		OS_Delay(95);
//		missionclock++;
//	}

//	if(!(OSReadBinSem(BINSEM_DEPLOYED_P))) {
		//Replaced burn circuit with a globally callable signal.
/*		OSSignalBinSem(BINSEM_BURNCIRCUIT_P);
		OS_Delay(250);OS_Delay(250);OS_Delay(250);OS_Delay(250);
		OS_Delay(250);OS_Delay(250); //Wait for the burn circuit to finish.
		missionclock=missionclock+15;
	
		OSSignalBinSem(BINSEM_DEPLOYED_P);
		csk_uart0_puts("Satellite is Deployed\r\n");
		F_FILE * Deployed_Ejected_SDSave = f_open(STATE_FILE,"r+");
		//Record Deployed on SD-Card
		//In file: 1=>just deployed, 2=>just ejected, 3=>both deployed and ejected true.
		if(Deployed_Ejected_SDSave) {
			f_seek(Deployed_Ejected_SDSave,0,SEEK_END);
			unsigned long _eof=f_tell(Deployed_Ejected_SDSave);
			f_seek(Deployed_Ejected_SDSave,0,SEEK_SET);
			if(_eof>0) {
				char SD_SAVESTATE[1];
				f_read(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
				f_seek(Deployed_Ejected_SDSave,0,SEEK_SET);
				if(SD_SAVESTATE[0]==2) {
					SD_SAVESTATE[0]=3; //3=>Deployed and Ejected.
					f_write(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
				}
				if(SD_SAVESTATE[0]!=3) {
					SD_SAVESTATE[0]=1; //1=>Just Deployed.
					f_write(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
				}
			}
			else {
				char SD_SAVESTATE[1];
				SD_SAVESTATE[0]=1; //1=>Just Deployed.
				f_write(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
			}
			f_close(Deployed_Ejected_SDSave);
		}
	}
*/
	while(1) {
		OS_Delay(100);
		missionclock++;
	}
} /* task_missionclock() */


