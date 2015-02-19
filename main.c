/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\main.c,v $
$Author: aek $
$Revision: 3.2 $
$Date: 2010-03-28 19:40:15-08 $

******************************************************************************/
#include "events.h"
#include "init.h"
#include "main.h"
#include "tasks.h"
#include "task_I2C.h"
#include "task_externalcmds.h"
#include "task_missionclock.h"
#include "task_estimator.h"
#include "task_nav.h"
#include "task_imager.h"

// Pumpkin Salvo headers
#include "salvo.h"
#include "csk_io.h"
#include "csk_sd.h"
#include "thin_usr.h"


csk_status_t csk_status;
//char strTmp[80];                  // usable to anyone who wants it ...


/******************************************************************************
****                                                                       ****
**                                                                           **
main()

Conventional Salvo main(), with peripheral initialization, Salvo 
initialization, and the Salvo scheduler.

**                                                                           **
****                                                                       ****
******************************************************************************/
int main() {

  csk_io42_low();
  
  // Power for FPGA/Imager board! Make sure this gets turned on NOW!
  csk_io40_high();  

  // Do target-specific (e.g., clocks, UARTs) and general 
  //  (e.g., CSK IO) initialization.
  init();
                              
  // Initialize Salvo RTOS.
  OSInit();

	/*1=Startup critical
	* 2=COMs
	* 3=Abstract data collection
	* 4=Permanent schedules
	* 6=Burn circuit--wait for call
	* 7=Ejection wait on RTC
	* 8=I2C init
	* 9=Debugging/does nothing
	*/
  // Create tasks -- MAKE SURE THAT OSTASKS in salvocfg.h IS LARGE ENOUGH!
  OSCreateTask(task_missionclock,	   TASK_MISSIONCLOCK_P,		 1);
  OSCreateTask(task_externalcmds,      TASK_EXTERNALCMDS_P,      4);
//  OSCreateTask(task_burnCircuit,	   TASK_BURNCIRCUIT_P,		 6);
  OSCreateTask(task_I2C,               TASK_I2C_P,               8);
  OSCreateTask(task_nav,               TASK_NAV_P,               1);
  OSCreateTask(task_estimator,         TASK_ESTIMATOR_P,         3);
  OSCreateTask(task_imager,            TASK_IMAGER_P,            5);

  // Create events.
//  OSCreateBinSem(RSRC_USB_MHX_IF_P, 1);    // Initially available
  OSCreateSem(SEM_CMD_CHAR_P,       0);    // No chars received yet
//  OSCreateSem(SEM_PRP_BURNTIME_P,   0);    // Used to wait beyond max timeout in NAV, if needed
//  OSCreateBinSem(BINSEM_HEON_P,0); //This is set in task_externalcmdsMHX.
//  OSCreateBinSem(BINSEM_DEPLOYED_P,   0);
//  OSCreateBinSem(BINSEM_EJECTED_P,   0); 
//  OSCreateBinSem(BINSEM_BURNCIRCUIT_P,   0); 
//  OSCreateBinSem(BINSEM_RAISEPOWERLEVEL_P, 0);
//  OSCreateBinSem(BINSEM_CLEAR_TO_SEND_P, 0);
//  OSCreateBinSem(BINSEM_SEND_BEACON_P, 0);
//  OSCreateBinSem(BINSEM_VUC_TURN_ON_P, 0);
//  OSCreateBinSem(BINSEM_VUC_TURN_OFF_P, 0);
//    OSCreateMsg(MSG_SDR_P, (OStypeMsgP) 0);
//    OSCreateMsg(MSG_HETOSDCARD_P, (OStypeMsgP) 0);
//    OSCreateMsg(MSG_GETLINES_P,(OStypeMsgP) 0);
//    OSCreateMsg(MSG_EDITCMDSCH_P,(OStypeMsgP) 0);
  // Create message queues
//  OSgltypeMsgQP MsqQBuff[SIZEOF_HE_MSGQ];
//  OSCreateMsgQ(MSGQ_HETX_P, MQCBP_HETX_P, MsqQBuff, SIZEOF_HE_MSGQ);
  OSCreateMsg(MSG_PRPTONAV_P, (OStypeMsgP) 0); // PRP externalcmd to NAV (thruster request)    
  OSCreateMSG(MSG_QUATERNIONTONAV_P, (OStypeMsgP), 0); //Quaternion info from cmds to NAV

	//Init SD-Card
	csk_sd_pwr_on();
	csk_sd_open();
	f_initvolume();

	//In file: 1=>just deployed, 2=>just ejected, 3=>both deployed and ejected true.
	F_FILE * Deployed_Ejected_SDSave = f_open(STATE_FILE,"r");
	if(Deployed_Ejected_SDSave) {
			/*
			f_close(Deployed_Ejected_SDSave);
			Deployed_Ejected_SDSave = f_open("DEPEJEC","w");
			f_seek(Deployed_Ejected_SDSave,0,SEEK_SET);
			char tmp[1]={3};
			f_write(tmp,1,1,Deployed_Ejected_SDSave);
			f_close(Deployed_Ejected_SDSave);
			Deployed_Ejected_SDSave = f_open("DEPEJEC","r");*/
			/*f_close(Deployed_Ejected_SDSave);
			F_FILE* file=f_open("names","w");
			char tmp[]="Maria Barna\r\nDan Dillon\r\nJim Dreas\r\nWesley Gardner\r\nBryant Gaume\r\nRichard Henry\r\nJoe Kirwen\r\nSteve Massey\r\nTom Moline\r\nTyler Olson\r\nManu Posso\r\nPhillip Reyes\r\nNate Richard\r\nAllison Roland\r\nBob Urberger\r\nDenana Vehab";
			f_write(tmp,1,strlen(tmp),file);
			f_close(file);
			Deployed_Ejected_SDSave = f_open("DEPEJEC","r");*/
		f_seek(Deployed_Ejected_SDSave,0,SEEK_END);
		unsigned long _eof=f_tell(Deployed_Ejected_SDSave);
		f_seek(Deployed_Ejected_SDSave,0,SEEK_SET);
		if(_eof>0) {
			char SD_SAVESTATE[1];
			f_read(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
			if(SD_SAVESTATE[0]==1) {
//				OSSignalBinSem(BINSEM_DEPLOYED_P);
			}
			if(SD_SAVESTATE[0]==2) {
//				OSSignalBinSem(BINSEM_EJECTED_P);
			}
			if(SD_SAVESTATE[0]==3) {
//				OSSignalBinSem(BINSEM_DEPLOYED_P);
//				OSSignalBinSem(BINSEM_EJECTED_P);
			}
		}
		f_close(Deployed_Ejected_SDSave);
	}
	else { //Create New File.
		Deployed_Ejected_SDSave = f_open(STATE_FILE,"w");
		char SD_SAVESTATE[1];
		SD_SAVESTATE[0]=0;
		f_write(SD_SAVESTATE,1,1,Deployed_Ejected_SDSave);
		f_close(Deployed_Ejected_SDSave);
	}

  // Enable interrupts (enables UART tx & rx).
  __enable_interrupt();

  // Multitask.
  while (1) {
    OSSched();
  } /* while */
  
} /* main() */
