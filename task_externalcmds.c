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
#define MAIN_OUTGOING_FLAG "RASCALSLUGND"
#define BEACON_BEGIN_FLAG "SLUBCN03xxyyzz"
#define BEACON_END_FLAG "BEAEND"

// Pumpkin Salvo headers
#include "salvo.h"

/** Our headers and defined vars **/
#include "rascal.h"

// defines POSE_BOEING from rascal.h
pose POSE_BOEING;

// defines outgoing serial frame
static char outgoing[258];

// defines and inits beacon flags
//static const char main_start_end[] = "RASCALSLUGND";
//static const char beacon_begin[] = "SLUBCN03xxyyzz";
//static const char beacon_end[] = "BEAEND";



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

// init THRUST_ENABLE_FLAG to be DISABLED
THRUST_ENABLE_FLAG = DISABLED;

// init STATUS-FLAG to be IDLE
STATUS_FLAG = IDLE;

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

// function to take recieved serial frame and check for start and end frame
// also handles escape chars in message data
char* recv_frame(char msg[], int frame_size){
  
  //de-stuffed message
  char dmsg[DATA_SIZE];
		
  // byte stuffed character signature
  char ESC_FLAG[]={0x7d, 0x5e};
  char ESC_ESC[]={0x7d, 0x5d};
  int counter = 0;

  // discards frame if not correct size or header/end aren't in the right places
  if ((frame_size != FRAME_SIZE) || (msg[0] != FLAG) || (msg[FRAME_SIZE-1] != FLAG)){
    //   printf("Discarded message\n");
        //   printf("%x\n", msg[0]);
        //   printf("%x\n", msg[FRAME_SIZE-1]);
    msg[0] = '\0';
    //dmsg = NULL;
    return (NULL);
  } else {
    // looks for byte stuffed characters in msg
    int j = 0;	//	int counter for 'for' loop below -->
    for (j = 0; j < FRAME_SIZE; j++){
      // byte stuffed character check for ESC_FLAG
	  if ((msg[j] == ESC_FLAG[0]) && (msg[j+1] == ESC_FLAG[1])){
	    // writes escaped character to de-stuffed message
	    dmsg[j] = 0x7e;
              // printf("De-stuffed ESC_FLAG\n");
	  }
	  // byte stuffed character check for ESC_ESC
	  else if ((msg[j] == ESC_ESC[0]) && (msg[j+1] == ESC_ESC[1])){
	    // writes escaped character to de-stuffed message
        dmsg[j] = 0x7d;
        // printf("De-stuffed ESC_ESC\n");
	  }
	  // drops start flag from de-stuffed message
	  else if (msg[j] == FLAG){
        counter++;
        continue;
      }      
	  // else copies char to de-stuffed message
      else{
	    dmsg[j-counter] = msg[j];
	  }
    }
  
  return dmsg;
}

}// end recv_frame

// This function takes and processes command sequences and passes them off accordingly
void CMDS(const char *a, char * saveName) {
//	OSSignalBinSem(BINSEM_RAISEPOWERLEVEL_P); 
	
    //csk_uart0_puts("task_externalcmds:\t");
//	char tmp[400]; 
	
    //static char tmps[256];
    //strcpy(tmps, *a);

    /* Why is this Nop here??? */
    //int I;
	//for(I=0;I<1000;I++) Nop();

	if (a[0]=='\r' || a[0]=='\n' || a[0]==0) { return; }
    
    // Start command handling
    // TODO: handle both slu headers SLUGRNRASCAL and SLUBCN03xxyyzz
    if (a[0]=='S' && a[1]=='L' && a[2]=='U' && a[6]=='R' && a[7]=='A' && a[8]=='S' && a[DATA_SIZE - 12] == 'S' && a[DATA_SIZE - 11]=='L' && a[DATA_SIZE - 10]=='U' && a[DATA_SIZE - 6]=='R' && a[DATA_SIZE - 5]=='A' && a[DATA_SIZE - 4]=='S') { // if "SLUGNDRASCAL" header is found process ground commands from list (see Max)
      // Thruster request!
      if (a[12]=='P' && a[13]=='R' && a[14]=='P') { //if a (beings with PRP!!!)
        int BEGIN = 15; // beginning index position of thruster command
        int END = 22;  // ending index position of thruster command
      
        for (BEGIN;BEGIN<=END;BEGIN++){ // sanity check to ensure all binary bits
          if ((a[BEGIN] != '0') && (a[BEGIN] != '1')){
            csk_uart0_puts("Invalid PRP command!\r\n");
            return;
          }
        }
        static char cmd[DATA_SIZE];
        memcpy(cmd, a, DATA_SIZE*sizeof(char));
        //csk_uart0_puts(cmd);
	    //csk_uart0_puts("\r\n");
        OSSignalMsg(MSG_PRPTONAV_P,(OStypeMsgP) (cmd));  // passes command to task_nav as it's "listening" for this
        return;
      } // End - 'PRP' command

      else if (a[12]=='S' && a[13]=='T') { // Check for "STATUS" command header
        csk_uart0_puts("STATUS OUTPUT\n");
        return;
      }
      else if (a[12]=='S' && a[13]=='P') { // Check for "SPRT" command header
        csk_uart0_puts("SPRT OUTPUT\n");
        return;
      }
      else if (a[12]=='G') { // Check for "GO2" command header
        csk_uart0_puts("GO2 OUTPUT\n");
        return;
      }
      else if (a[12]=='P' && a[13]=='O' && a[16]=='I') { // Check for "POSEIMG" command header
        csk_uart0_puts("POSEIMG OUTPUT\n");
        return;
      }
      else if (a[12]=='P' && a[13]=='O' && a[16]=='E') { // Check for "POSEEST" command header
        csk_uart0_puts("POSEEST OUTPUT\n");
        return;
      }
      else if (a[12]=='T' && a[18]=='I') { // Check for "THRUSTINFO" command header
        csk_uart0_puts("THRUSTINFO OUTPUT\n");
        return;
      }
      else if (a[12]=='T' && a[18]=='_' && a[19]=='E') { // Check for "THRUST_ENABLE" command header
        csk_uart0_puts("THRUSTERS ENABLED\n");
        THRUST_ENABLE_FLAG = ENABLED;
        return;
      }
      else if (a[12]=='T' && a[18]=='_' && a[19]=='D') { // Check for "THRUST_DISABLE" command header
        csk_uart0_puts("THRUSTERS DISABLED\n");
        THRUST_ENABLE_FLAG = DISABLED;
        return;
      }
      else if (a[12]=='I' && a[13]=='M' && a[14]=='A' && a[15] == 'G' && a[16] == 'E' && a[17] == 'R') { // Check for "IMAGER" command header
        csk_uart0_puts("IMAGE REQUEST\n");
        static char cmd[DATA_SIZE];
        memcpy(cmd, a, DATA_SIZE*sizeof(char));
        //csk_uart0_puts(cmd);
	    //csk_uart0_puts("\r\n");
        OSSignalMsg(MSG_IMAGER_P,(OStypeMsgP) (cmd));  // passes command to task_nav as it's "listening" for this
        return;
       
      }
      
     }
     else
     {
       long int fixedquat[4];
       char conversionarray[4];
       int i, j;
       for (i = 0; i < 4; i++)
       {
         for (j = 0; j < 4; j++)
         {
           conversionarray[j] = a[12+j];
         }
         fixedquat[i] = (long int)conversionarray;
         Nop();
       }
     }
    
	//At this point, no command has been recognized, as it would have returned if it had been.
	//char tmps[80];
	sprintf(a, "%s{%s}", COMMAND_NO_JOY, a);
	csk_uart0_puts(a);
	BroadcastOrSave(a, saveName);
    return;
} // End - CMDS

void task_externalcmds(void) {

  static unsigned int i=0;
  sprintf(outgoing, "%c%s%s", FLAG, MAIN_OUTGOING_FLAG, BEACON_BEGIN_FLAG); 
  sprintf((outgoing + (239*sizeof(char))), "%s%s%c", BEACON_END_FLAG, MAIN_OUTGOING_FLAG, FLAG);
  static char a[258]; // holds received command
  static char* dmsg;
    
  //task_nav msg test
 //static char a[256]="PRP11111111500";

  _Q16 x;

  // defines and inits POSE_BOEING
  POSE_BOEING.q1 = 0.0045;
  POSE_BOEING.q2 = -0.0022;
  POSE_BOEING.q3 = 0.0029;
  POSE_BOEING.q4 = 1.000;
  POSE_BOEING.q1dot = 0.0;
  POSE_BOEING.q2dot = 0.0;
  POSE_BOEING.q3dot = 0.0;
  POSE_BOEING.q4dot = 0.0;
  POSE_BOEING.xi = 0.0;
  POSE_BOEING.yi = 0.0;
  POSE_BOEING.zi = 0.0;
  POSE_BOEING.xidot = 0.0;
  POSE_BOEING.yidot = 0.0;
  POSE_BOEING.zidot = 0.0;

	while(1) {
        // test for POSE_BOEING
        /** char str[40];
        csk_uart0_puts("\r\nPOSE_BOEING from CMDS:\r\n");
        sprintf(str, "%d", POSE_BOEING.xi++);
        csk_uart0_puts("\r\nEND POSE_BOEING\r\n");
        csk_uart0_puts(str);
		END POSE_BOEING TEST **/
        OS_Delay(250);
 		int waitTmp=1; //ENTER: Will wait until something is received, and store it in a.
		while(waitTmp) {
			OS_Delay(20);
			waitTmp=1;
            dmsg = NULL;
			//strcpy(a,"");
			i=0;
			while(csk_uart0_count() && i<258) {
				//sprintf(a,"%s%c",a,csk_uart0_getchar());
				a[i]=csk_uart0_getchar();
				waitTmp=0;
				i++;
			}
            if (a[0] == 0x7e && a[257] == 0x7e) {
              //csk_uart0_puts("I'm in sanity check!");
              //dmsg = recv_frame(a, FRAME_SIZE);
              dmsg = recv_frame(a, FRAME_SIZE);
			}
            a[0]=0;
		}  //END: Will wait until something is received, and store it in a.
        Nop();
        
        char tmp[256] = {0xff};
        
/*
        sprintf(tmp, &dmsg);
        csk_uart0_puts("This is &dmsg: \r\n");
        csk_uart0_puts(tmp);
*/  
        if(dmsg != NULL) {
            //char str[300];
            //for(i = 0; i < 300; i++){
              //sprintf(str, "%c", a[i]);
            //}
            //csk_uart0_puts(str);
	        // Begins full propulsion tank purge - for shipment and ground testing!
	        if (dmsg[0]=='P' && dmsg[1]=='U' && dmsg[2]=='R' && dmsg[3]=='G' && dmsg[4]=='E' && dmsg[5]=='O' && dmsg[6]=='N') { // if dmsg is PURGEON
	          // turns ON S1 and S2 solenoids AND ALL thrusters
	         /* csk_uart0_puts("BEGIN - Full Propulsion Tank Purge!\r\n");
	          csk_io22_high(); csk_uart0_puts("S1 ON!\r\n");
	          csk_io20_high(); csk_uart0_puts("S2 ON!\r\n");
	          OS_Delay(10);
	          csk_io23_high(); csk_uart0_puts("A ON!\r\n");
	          //csk_io18_high(); csk_uart0_puts("B ON!\r\n");
	          csk_io17_high(); csk_uart0_puts("C ON!\r\n");
	          //csk_io21_high(); csk_uart0_puts("D ON!\r\n");
	          csk_io19_high(); csk_uart0_puts("E ON!\r\n");
	          //csk_io16_high(); csk_uart0_puts("F ON!\r\n");*/
	          
	        } // End 'PURGEON'
	
	        // Ends full propulsion tank purge - for shipment and ground testing!
	        else if (dmsg[0]=='P' && dmsg[1]=='U' && dmsg[2]=='R' && dmsg[3]=='G' && dmsg[4]=='E' && dmsg[5]=='O' && dmsg[6]=='F' && dmsg[7]=='F') { // if dmg is PURGEOFF
	        
	        // turns OFF solenoids and thrusters
	      /*  csk_io22_low(); csk_uart0_puts("S1 OFF!\r\n");
	        csk_io20_low(); csk_uart0_puts("S2 OFF!\r\n");
	        OS_Delay(200);
	        csk_io23_low(); csk_uart0_puts("A OFF!\r\n");
	        //csk_io18_low(); csk_uart0_puts("B OFF!\r\n");
	        csk_io17_low(); csk_uart0_puts("C OFF!\r\n");
	        //csk_io21_low(); csk_uart0_puts("D OFF!\r\n");
	        csk_io19_low(); csk_uart0_puts("E OFF!\r\n");
	        //csk_io16_low(); csk_uart0_puts("F OFF!\r\n");
	        csk_uart0_puts("END - Full Propulsion Tank Purge!\r\n");*/
	        
	          
	        } // End 'PURGEOFF'
	      
            char b[256];
            memcpy(b, dmsg, DATA_SIZE*sizeof(char));
	        CMDS(&b, 0);
	        
        }
	 }
} /* task_externalcmds() */


