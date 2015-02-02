/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND, 
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.
!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!I2C--Not 5sec~~~~~~~~~~~~~~~~~~~~~~~
$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\task_5sec.c,v $
$Author: aek $
$Revision: 3.1 $
$Date: 2009-11-02 00:45:07-08 $

******************************************************************************/
#include "main.h"
#include "task_externalcmds.h"

// Pumpkin CubeSat Kit headers 
#include "csk_io.h"
#include "csk_uart.h"
#define CSK_EFFS_THIN_LIB 1
#include "csk_sd.h"
#include "thin_usr.h"

// Pumpkin Salvo headers
#include "salvo.h"

/** Our headers and defined vars **/
#include "rascal.h"

// defines POSE_BOEING from rascal.h
pose POSE_BOEING;

// Other tasks

extern char * i2c_SnR(int address, char sending[], int numSend, int numRec, char charArrayOfAppropriateSize[], int asciiOrHex);
//extern void i2c_sendz(char sending[]);
//extern char * i2c_readz(int num, char charArrayOfAppropriateSize[], int asciiOrHex);
extern char send_i2c_byte(int data);
extern void i2c_start(void);
extern void i2c_restart(void);
extern void reset_i2c_bus(void);
extern unsigned char i2c_read_ack(void);
extern unsigned char i2c_read_nack(void);
extern void Vand(char a[], int numSent, int numRet);
extern void VandHex(char a[], int numSent, int numRet);
extern void VUC(char a[], int numSend, char returnArray[], int numRet);
extern void VUCHex(char a[], int numSend, char returnArray[], int numRet);
extern char HeTrans255Str(char* inpt);
extern char HeTrans255(char* inpt, int n);
extern void HeCkSum(char* buffer, int n);
//extern char HeTrans255(char* inpt, int n);
extern unsigned long long getMissionClock();
extern void deleteSchedule(int num);
extern void getSchedule(int num, char* a);
extern void setHeSaveData3(char* a);
extern void setHeDefaultPowerLevel(unsigned char pl);
extern void setHeHighPowerLevel(unsigned char pl);
//extern void setBeacon2_5SecInterval(unsigned char num);
extern void commandHeStandardConfig();
extern char* getHeConfig();
extern void commandHeNoBeacons();
extern void setBeaconFrameIntervals(unsigned int *nums);

unsigned int makeHex(char char1, char char2) {
	int total=0;
	if (char1==49) total=total+1*16;
	if (char1==50) total=total+2*16;
	if (char1==51) total=total+3*16;
	if (char1==52) total=total+4*16;
	if (char1==53) total=total+5*16;
	if (char1==54) total=total+6*16;
	if (char1==55) total=total+7*16;
	if (char1==56) total=total+8*16;
	if (char1==57) total=total+9*16;
	if (char1==65) total=total+10*16;
	if (char1==66) total=total+11*16;
	if (char1==67) total=total+12*16;
	if (char1==68) total=total+13*16;
	if (char1==69) total=total+14*16;
	if (char1==70) total=total+15*16;
	if (char2==49) total=total+1;
	if (char2==50) total=total+2;
	if (char2==51) total=total+3;
	if (char2==52) total=total+4;
	if (char2==53) total=total+5;
	if (char2==54) total=total+6;
	if (char2==55) total=total+7;
	if (char2==56) total=total+8;
	if (char2==57) total=total+9;
	if (char2==65) total=total+10;
	if (char2==66) total=total+11;
	if (char2==67) total=total+12;
	if (char2==68) total=total+13;
	if (char2==69) total=total+14;
	if (char2==70) total=total+15;
	return total;
}

void BroadcastOrSave(char a[], char * fName){
	if (!fName) {
//		HeTrans255Str(a);
	}
	else {
		F_FILE * file = f_open(fName, "a");
		f_write(a, 1, strlen(a), file);
		f_close(file);
	}
}
void BroadcastOrSaveN(char a[], char * fName, int num){
	if (!fName) {
//		HeTrans255(a,num);
	}
	else {
		F_FILE * file = f_open(fName, "a");
		f_write(a, 1, num, file);
		f_close(file);
	}
}

//static int ZEROCLOCKINT=0;
//static int IRONMANINT=0;

void CMDS(char cmd[], char * saveName) {
//	OSSignalBinSem(BINSEM_RAISEPOWERLEVEL_P); 
	static char a[256];
    memcpy(a, cmd, 256);
    csk_uart0_puts("task_externalcmds:\t");
	csk_uart0_puts(a);
	csk_uart0_puts("\r\n");
	char tmp[400]; 
	int I;
	for(I=0;I<1000;I++) Nop();
	if (a[0]=='\r' || a[0]=='\n' || a[0]==0) { 
		return;
	}
    
    
    // Start command handling

    // Thruster request!
    if (a[0]=='P' && a[1]=='R' && a[2]=='P') { //if a (beings with PRP!!!)
      int BEGIN = 3; // beginning index position of thruster command
      int END = 10;  // ending index position of thruster command
      
      for (BEGIN;BEGIN<=END;BEGIN++){ // sanity check to ensure all binary bits
        if ((a[BEGIN] != '0') && (a[BEGIN] != '1')){
          csk_uart0_puts("Invalid PRP command!\r\n");
          return;
        }
      }
      //csk_uart0_puts("Got here!\r\n");
      OSSignalMsg(MSG_PRPTONAV_P,(OStypeMsgP) (a));  
      return;
    }
	//At this point, no command has been recognized, as it would have returned if it had been.
	char tmps[80];
	sprintf(tmps, "%s{%s}", COMMAND_NO_JOY, a);
	csk_uart0_puts(tmps);
	BroadcastOrSave(tmps, saveName);
//	HeTrans255Str(a);
}

void task_externalcmds(void) {
    char a[256];
    
    //task_nav msg test
  	//static char a[256]="PRP11111111500";

	static unsigned int i=0;
	
    // defines and inits POSE_BOEING
	POSE_BOEING.x=0.0;
    POSE_BOEING.y=0.0;
    POSE_BOEING.z=0.0;
    POSE_BOEING.q1=0.0;
    POSE_BOEING.q2=0.0;
    POSE_BOEING.q3=0.0;
    POSE_BOEING.q4=0.0;
	


	while(1) {
        // test for POSE_BOEING
        /** char str[40];
        csk_uart0_puts("\r\nPOSE_BOEING from CMDS:\r\n");
        sprintf(str, "%d", POSE_BOEING.x++);
        csk_uart0_puts("\r\nEND POSE_BOEING\r\n");
        csk_uart0_puts(str);
		END POSE_BOEING TEST **/

        OS_Delay(250);
 		int waitTmp=1; //ENTER: Will wait until something is received, and store it in a.
		while(waitTmp) {
			OS_Delay(20);
			waitTmp=1;
			//strcpy(a,"");
			i=0;
			while(csk_uart0_count() && i<256) {
				//sprintf(a,"%s%c",a,csk_uart0_getchar());
				a[i]=csk_uart0_getchar();
				waitTmp=0;
				i++;
			}
			a[i]=0;
		}  //END: Will wait until something is received, and store it in a.
		
        CMDS(a, 0);
       
        
	}
} /* task_externalcmds() */


