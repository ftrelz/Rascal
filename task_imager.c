#include "main.h"
#include "task_imager.h"
#include <math.h>

//Pumpking CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"

// Our headers
#include "rascal.h"

// define POSE_IMG
pose POSE_IMG;

void task_imager(void) {
  
  //csk_io40_high();
  // inits POSE_IMG
  POSE_IMG.q1 = 0.0;
  POSE_IMG.q2 = 0.0;
  POSE_IMG.q3 = 0.0;
  POSE_IMG.q4 = 0.0;
  POSE_IMG.q1dot = 0.0;
  POSE_IMG.q2dot = 0.0;
  POSE_IMG.q3dot = 0.0;
  POSE_IMG.q4dot = 0.0;
  POSE_IMG.xi = 0.0000;
  POSE_IMG.yi = 35.000;
  POSE_IMG.zi = 2.3693;
  POSE_IMG.xidot = 0.0009;
  POSE_IMG.yidot = 0.0176;
  POSE_IMG.zidot = -0.0018;
  
  static char a[200];
  static unsigned int i=0;

 
  while(1) {
    OS_Delay(250);
 /*   
     char* test='AB';
    csk_uart0_puts(test);
    csk_uart0_puts("\r\n");
    csk_uart2_puts(test);
*/
    //char tmp[50];
    //sprintf(tmp, "I'm in task_imager!\r\n");
   // csk_uart0_puts(tmp);
    
    
//    csk_uart0_puts(csk_uart2_getchar());
/*
    int waitTmp=1; //ENTER: Will wait until something is received, and store it in a.
		while(waitTmp) {
			OS_Delay(20);
             waitTmp=1;
			//strcpy(a,"");
			i=0;
			while(csk_uart2_count() && i<258) {
                csk_uart0_puts("I'm in uart2_count!");
				//sprintf(a,"%s%c",a,csk_uart0_getchar());
				a[i]=csk_uart2_getchar();
				waitTmp=0;
				i++;
			}
  } 
  csk_uart0_puts(a);
 */
  } // end while(1)
} // end task_imager(void)
