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
  
  // inits POSE_IMG
  POSE_IMG.q1 = 0.0;
  POSE_IMG.q2 = 0.0;
  POSE_IMG.q3 = 0.0;
  POSE_IMG.q4 = 0.0;
  POSE_IMG.q1dot = 0.0;
  POSE_IMG.q2dot = 0.0;
  POSE_IMG.q3dot = 0.0;
  POSE_IMG.q4dot = 0.0;
  POSE_IMG.xi = 0.0;
  POSE_IMG.yi = 0.0;
  POSE_IMG.zi = 0.0;
  POSE_IMG.xidot = 0.0;
  POSE_IMG.yidot = 0.0;
  POSE_IMG.zidot = 0.0;
  
  while(1) {
    OS_Delay(250);
    

    char tmp[50];
    sprintf(tmp, "I'm in task_imager!\r\n");
    csk_uart0_puts(tmp);


  } // end while(1)
} // end task_imager(void)
