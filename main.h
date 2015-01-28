/******************************************************************************
(C) Copyright Pumpkin, Inc. All Rights Reserved.

This file may be distributed under the terms of the License
Agreement provided with this software.

THIS FILE IS PROVIDED AS IS WITH NO WARRANTY OF ANY KIND,
INCLUDING THE WARRANTY OF DESIGN, MERCHANTABILITY AND
FITNESS FOR A PARTICULAR PURPOSE.

$Source: C:\\RCS\\D\\Pumpkin\\CubeSatKit\\Example\\all\\all\\CubeSatKit_Dev_Board\\Test\\Test1\\main.h,v $
$Author: aek $
$Revision: 3.5 $
$Date: 2010-02-13 13:17:58-08 $

******************************************************************************/
#ifndef __main_h
#define __main_h


// Symbols, etc.
#define STR_APP_NAME          "test\\test1 application"
#define STR_VERSION           "Built on " __DATE__ " at " __TIME__ 
#define STR_WARNING           "WARNING: Use 'z' command with caution -- refer to project's abstract.txt"
#define STR_BAUD_RATE         "9600"
#define STR_1TAB              "\t"
#define STR_2TABS             "\t\t"
#define STR_CRLF              "\r\n"
#define OS_VERSION            1
#define SC_ID 				  3        // ID for SLU/Rascal: 3
#define STATE_FILE            "DEPEJEC"
  
// Macros for user-readable messages.
#define user_debug_msg(x)     csk_uart0_msg_ts(x)
#define data_debug_msg(x)     csk_uart1_msg_ts(x)

// Other macros.
#define LOOP_HERE()            do { ; } while (1)

// Target-specific symbols.
#define SYSTEM_TIMER_RELOAD    328
#define NOP                    _NOP()

// Function declarations.
extern void init_devices(void);

// Structure declarations  
typedef struct {
  unsigned int usb_present:1;
  unsigned int usb_connected:1;
  unsigned int mhx_connected:1;
  unsigned int MCLKOutEnabled:2;
  unsigned int exercise_io_running:1;
} csk_status_t;

// Extern variable declarations.
extern csk_status_t csk_status;
//extern char strTmp[];
//static int I2CSPEED; 			//This is set in task_i2c.
#define MSG_PRPTONAV_P            OSECBP(1)
//#define BINSEM_COP_ARG_P   	  	  OSECBP(5)
//#define MSG_GETLINES_P   		  OSECBP(10)
//#define MSG_HETOSDCARD_P   		  OSECBP(11)
//#define BINSEM_HEON_P			  OSECBP(3)
//#define BINSEM_DEPLOYED_P   	  OSECBP(4)
//#define BINSEM_EJECTED_P 	  	  OSECBP(5)
//#define BINSEM_BURNCIRCUIT_P 	  OSECBP(6)
//#define MSG_SDR_P			 	  OSECBP(7)
//#define BINSEM_RAISEPOWERLEVEL_P  OSECBP(8)
//#define BINSEM_CLEAR_TO_SEND_P    OSECBP(9)
//#define BINSEM_SEND_BEACON_P	  OSECBP(10)
//#define BINSEM_VUC_TURN_ON_P	  OSECBP(11)
//#define BINSEM_VUC_TURN_OFF_P	  OSECBP(12)
//#define MSG_EDITCMDSCH_P   		  OSECBP(13)
//#define MSGQ_HETX_P               OSECBP(14)
//#define MQCBP_HETX_P               OSMQCBP(1)

#endif /* __main_h */






