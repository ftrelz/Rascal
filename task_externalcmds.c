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
#define MAIN_OUTGOING_FLAG "RASCALSLUGND"  // size=12
#define BEACON_BEGIN_FLAG "SLUBCN03xxyyzz" // size=14
#define BEACON_END_FLAG "BEAEND"           // size=6

// Pumpkin Salvo headers
#include "salvo.h"

/** Our headers and defined vars **/
#include "rascal.h"

// defines POSE_BOEING from rascal.h
pose POSE_BOEING;

// defines outgoing serial frame
static char outgoing[FRAME_SIZE];

// made external to access tx_frame from other tasks
extern void tx_frame(char msg[], int msg_size);

// Other tasks
/*
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
*/
// init THRUST_ENABLE_FLAG to be DISABLED
int THRUST_ENABLE_FLAG = DISABLED;

// init STATUS-FLAG to be IDLE
int STATUS_FLAG = IDLE;

/*
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
*/

// function to take recieved serial frame and check for start and end frame
// also handles escape chars in message data
char* rx_frame(char msg[], int frame_size){
  
  //de-stuffed message
  static char dmsg[DATA_SIZE];
		
  // byte stuffed character signature
  char ESC_FLAG[]={0x7d, 0x5e};
  char ESC_ESC[]={0x7d, 0x5d};
  int counter = 0;

  // discards frame if not correct size or header/end aren't in the right places
  if ((frame_size != FRAME_SIZE) || (msg[0] != FLAG) || (msg[FRAME_SIZE-1] != FLAG)){
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
	    dmsg[j-counter] = 0x7e;
        j++;
        counter++;
	  }
	  // byte stuffed character check for ESC_ESC
	  else if ((msg[j] == ESC_ESC[0]) && (msg[j+1] == ESC_ESC[1])){
	    // writes escaped character to de-stuffed message
        dmsg[j-counter] = 0x7d;
        j++;
        counter++;
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
}// end rx_frame

// ONLY function to prepare/pack and messaages send to Messasge Queue (which is transmitted in main of task_externalcmds below
// REMEMBER: HAVE ONLY ONE EGRESS POINT!!!
void tx_frame(char msg[], int msg_size){		
  // byte stuffed character signature
  char ESC_FLAG[]={0x7d, 0x5e};
  char ESC_ESC[]={0x7d, 0x5d};
  int msg_len = strlen(msg);
  
  sprintf(outgoing, "%c%s%s", FLAG, MAIN_OUTGOING_FLAG, BEACON_BEGIN_FLAG);
  int i =0;
  int HEADER_LENGTH = 27; // index at 0
  // If either Boeing Serial Frame flag is in the message data or their escape character (0x7E) is found they must be replaced
  // with ESC_FLAG and ESC_ESC respectively
  for (i; i < msg_len; i++) {
    if (msg[i] == FLAG ) {
      outgoing[i+HEADER_LENGTH] = ESC_FLAG[0];
      outgoing[i+HEADER_LENGTH+1] = ESC_FLAG[1];
      i++;
      msg_size++;
    }
    else if (msg[i] == ESC_FLAG[0]) {
      outgoing[i+HEADER_LENGTH] = ESC_ESC[0];
      outgoing[i+HEADER_LENGTH+1] = ESC_ESC[1];
      i++;
      msg_size++;
    }
    else {
      outgoing[i+HEADER_LENGTH] = msg[i];
    }
  }
  
  sprintf(&outgoing[i+HEADER_LENGTH], "%s%s", BEACON_END_FLAG, MAIN_OUTGOING_FLAG);
  outgoing[257] = FLAG;

  OSSignalMsgQ(MSGQ_TX_P, (OStypeMsgP) outgoing);
	  
     
}// end tx_frame

// This function takes and processes command sequences and passes them off accordingly
void CMDS(const char *a, char * saveName) {

	if (a[0]=='\r' || a[0]=='\n' || a[0]==0) { return; }

    char cmd_tmp[200]; // holds outgoing messages (then sent to tx_frame() at end of CMDS)

    // Start command handling
    // TODO: handle both slu headers SLUGRNRASCAL and SLUBCN03xxyyzz
    if (a[0]=='S' && a[1]=='L' && a[2]=='U' && a[6]=='R' && a[7]=='A' && a[8]=='S' && a[DATA_SIZE - 12] == 'S' && a[DATA_SIZE - 11]=='L' && a[DATA_SIZE - 10]=='U' && a[DATA_SIZE - 6]=='R' && a[DATA_SIZE - 5]=='A' && a[DATA_SIZE - 4]=='S') { // if "SLUGNDRASCAL" header is found process ground commands from list (see Max)
      // Thruster request!
      if (a[12]=='P' && a[13]=='R' && a[14]=='P') { //if a (beings with PRP!!!)
        int BEGIN = 15; // beginning index position of thruster command
        int END = 22;  // ending index position of thruster command
      
        for (BEGIN;BEGIN<=END;BEGIN++){ // sanity check to ensure all binary (semaphore) bits
          if ((a[BEGIN] != '0') && (a[BEGIN] != '1')){
            sprintf(cmd_tmp, "Invalid PRP command!");
          }
        }
        static char cmd[DATA_SIZE];
        memcpy(cmd, a, DATA_SIZE*sizeof(char));
       
        OSSignalMsg(MSG_PRPTONAV_P,(OStypeMsgP) (cmd));  // passes command to task_nav as it's "listening" for this
        return;
      } // End - 'PRP' command

      else if (a[12]=='S' && a[13]=='T' && a[14]=='A' && a[15]=='T' && a[16]=='U' && a[17]=='S') { // Check for "STATUS" command header
        // in meters - Monty Python fans rejoice!!!
        int CHEESE_SHOP = 3; // distance error when RPO = 10m
        int PET_SHOP = 10; // distance error when RPO = 100m
        int error = POSE_EST.yi - POSE_DESIRED.yi; // using correct Body coordinates (not Matlab coords) -- 20151009 DJU
         
		if (POSE_DESIRED.yi == 10) {
		  if (error > CHEESE_SHOP) {
		    sprintf(cmd_tmp, "Status: 01");
		  } else if (error <= CHEESE_SHOP && error >= 0) {
		    sprintf(cmd_tmp, "Status: 00");
		  } else {
		    sprintf(cmd_tmp, "Status: 02");
		  }
		} else if (POSE_DESIRED.yi == 100) {
		  if (error > PET_SHOP) {
		    sprintf(cmd_tmp, "Status: 01");
		  } else if (error <= PET_SHOP && error >= 0) {
		    sprintf(cmd_tmp, "Status: 00");
		  } else {
		    sprintf(cmd_tmp, "Status: 02");
		  }
		}
        
      }
      else if (a[12]=='S' && a[13]=='P' && a[14]=='R' && a[15]=='T') { // Check for "SPRT" command header
        static char cmd[DATA_SIZE];
		memcpy(cmd, a, DATA_SIZE*sizeof(char));
        OSSignalMsg(MSG_PRPTONAV_P,(OStypeMsgP) (cmd));  // passes command to task_nav as it's "listening" for this
        return;
      }
      else if (a[12]=='G' && a[13]=='O' && a[14]=='2') { // Check for "GO2" command header
		char temp[4];
		memcpy(temp, (a+15), 3);
	    temp[3] = NULL;

        // y and x coordinates here are switched because of the error in the matlab corrdinate frame (according to Mary)
		// the x coordinate of the go2 command is assigned to the y coordinate of pose_desired and vice versa
        // becasue all estimator calculations were implemented in the matlab coordinate frame
	    POSE_DESIRED.yi = (float)atoi(temp);
		memcpy(temp, (a+18), 3);
	    temp[3] = NULL;
	    POSE_DESIRED.xi = (float)atoi(temp);
		memcpy(temp, (a+21), 3);
	    temp[3] = NULL;
	    POSE_DESIRED.zi = (float)atoi(temp);
       
      }
      else if (a[12]=='P' && a[13]=='O' && a[14]=='S'&& a[15]=='E' && a[16]=='I' && a[17]=='M'&& a[18]=='G') { // Check for "POSEIMG" command header
		sprintf(cmd_tmp, "IMG%03d%03d%03d", (int)POSE_IMG.xi, (int)POSE_IMG.yi, (int)POSE_IMG.zi);
      
      }
      else if (a[12]=='P' && a[13]=='O' && a[14]=='S'&& a[15]=='E' && a[16]=='E' && a[17]=='S'&& a[18]=='T' ) { // Check for "POSEEST" command header
		sprintf(cmd_tmp, "EST%03d%03d%03d", (int)POSE_EST.xi, (int)POSE_EST.yi, (int)POSE_EST.zi);
       
      }
      else if (a[12]=='T' && a[13]=='H' && a[14]=='R'&& a[15]=='U' && a[16]=='S' && a[17]=='T'&& a[18]=='I' && a[19]=='N' && a[20]=='F'&& a[21]=='O') { // Check for "THRUSTINFO" command header
		sprintf(cmd_tmp, "INFO%01d%01d%01d%01d%01d%01d%01d%01d", THRUSTER_INFO.thruster_Azminus, THRUSTER_INFO.thruster_Byplus,
																THRUSTER_INFO.thruster_Cxminus, THRUSTER_INFO.thruster_Dzplus,
																THRUSTER_INFO.thruster_Eyminus, THRUSTER_INFO.thruster_Fxplus);
      
      }
      else if (a[12]=='T' && a[13]=='H' && a[14]=='R'&& a[15]=='U' && a[16]=='S' && a[17]=='T'&& a[18]=='_' && a[19]=='E' && a[20]=='N'&& a[21]=='A' && a[22]=='B' && a[23]=='L' && a[24]=='E') { // Check for "THRUST_ENABLE" command header
		sprintf(cmd_tmp, "THRUSTERS ENABLED");
        THRUST_ENABLE_FLAG = ENABLED;
     
      }
      else if (a[12]=='T' && a[13]=='H' && a[14]=='R'&& a[15]=='U' && a[16]=='S' && a[17]=='T'&& a[18]=='_' && a[19]=='D' && a[20]=='I'&& a[21]=='S' && a[22]=='A' && a[23]=='B' && a[24]=='L' && a[25]=='E') { // Check for "THRUST_DISABLE" command header
        sprintf(cmd_tmp, "THRUSTERS DISABLED");
        THRUST_ENABLE_FLAG = DISABLED;
      
      }
      else if (a[12]=='I' && a[13]=='M' && a[14]=='A' && a[15] == 'G' && a[16] == 'E' && a[17] == 'R') { // Check for "IMAGER" command header
        static char cmd[DATA_SIZE];
        memcpy(cmd, a, DATA_SIZE*sizeof(char));
      
        OSSignalMsg(MSG_IMAGER_P,(OStypeMsgP) (cmd));  // passes command to task_nav as it's "listening" for this
       
        sprintf(cmd_tmp, "IMAGE REQUEST");
      }
     } // end if "SLUGNDRASCAL" header is found
     else {
    	//At this point, no command has been recognized, as it would have returned if it had been.
    	sprintf(cmd_tmp, "%s{%s}", COMMAND_NO_JOY, a);
    	//BroadcastOrSave(a, saveName);
     } // end else

     tx_frame(cmd_tmp, TX_MSG_SIZE);
     return;
   
} // End - CMDS

void task_externalcmds(void) {

  static unsigned int i=0;
  //sprintf(outgoing, "%c%s%s", FLAG, MAIN_OUTGOING_FLAG, BEACON_BEGIN_FLAG); 
  //sprintf((outgoing + (239*sizeof(char))), "%s%s%c", BEACON_END_FLAG, MAIN_OUTGOING_FLAG, FLAG);
  static char a[258]; // holds received command
  static char* dmsg;
  static OStypeMsgP msgP;
  char* tx_tmp;
  //task_nav msg test
 
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
      
        OS_Delay(250);
 		int waitTmp=1; //ENTER: Will wait until something is received, and store it in a.
		while(waitTmp) {
			OS_Delay(20);
            
            /** ONLY EGRESS POINT! KEEP IT THAT WAY! -- 20151014 DJU **/
            // checks Message Queue and if something, transmit it char by char (to not to stop on 0x00's within the message)
			while ((msgP = OSTryMsgQ(MSGQ_TX_P))) {
              tx_tmp = (char*)msgP;
              for (i = 0; i < FRAME_SIZE; i++) 	
                csk_uart0_putchar(tx_tmp[i]);
			}

			waitTmp=1;
            dmsg = NULL;
			
			i=0;
			while(csk_uart0_count() && i<258) {
				a[i]=csk_uart0_getchar();
				waitTmp=0;
				i++;
			}
            if (a[0] == 0x7e && a[257] == 0x7e) {   
              dmsg = rx_frame(a, FRAME_SIZE);
			}
            a[0]=0;
		}  //END: Will wait until something is received, and store it in a.
        
        if(dmsg != NULL) {
            
	        // Begins full propulsion tank purge - for shipment and ground testing!
	        if (dmsg[0]=='P' && dmsg[1]=='U' && dmsg[2]=='R' && dmsg[3]=='G' && dmsg[4]=='E' && dmsg[5]=='O' && dmsg[6]=='N') { // if dmsg is PURGEON
	          // turns ON S1 and S2 solenoids AND ALL thrusters
	          csk_uart0_puts("BEGIN - Full Propulsion Tank Purge!");
	          csk_io27_high(); csk_uart0_puts("S1 ON!");
	          csk_io30_high(); csk_uart0_puts("S2 ON!");
	          OS_Delay(10);
	          csk_io26_high(); csk_uart0_puts("A ON!");
	          //csk_io18_high(); csk_uart0_puts("B ON!");
	          csk_io24_high(); csk_uart0_puts("C ON!");
	          //csk_io21_high(); csk_uart0_puts("D ON!");
	          csk_io31_high(); csk_uart0_puts("E ON!");
	          //csk_io16_high(); csk_uart0_puts("F ON!");
	          
	        } // End 'PURGEON'
	
	        // Ends full propulsion tank purge - for shipment and ground testing!
	        else if (dmsg[0]=='P' && dmsg[1]=='U' && dmsg[2]=='R' && dmsg[3]=='G' && dmsg[4]=='E' && dmsg[5]=='O' && dmsg[6]=='F' && dmsg[7]=='F') { // if dmg is PURGEOFF
	        // turns OFF solenoids and thrusters
	        csk_io27_low(); csk_uart0_puts("S1 OFF!");
	        csk_io30_low(); csk_uart0_puts("S2 OFF!");
	        OS_Delay(200);
	        csk_io26_low(); csk_uart0_puts("A OFF!");
	        //csk_io18_low(); csk_uart0_puts("B OFF!");
	        csk_io24_low(); csk_uart0_puts("C OFF!");
	        //csk_io21_low(); csk_uart0_puts("D OFF!");
	        csk_io31_low(); csk_uart0_puts("E OFF!");
	        //csk_io16_low(); csk_uart0_puts("F OFF!");
	        csk_uart0_puts("END - Full Propulsion Tank Purge!");
	        
	          
	        } // End 'PURGEOFF'
	      
            char b[DATA_SIZE];
            memcpy(b, dmsg, DATA_SIZE*sizeof(char));
	        CMDS(&b, 0);
	        
        } // end if(dmsg != NULL)
	 } // end while(1)
} /* task_externalcmds() */


