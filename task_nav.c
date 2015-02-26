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
#include "task_nav.h"
#include "task_estimator.h"
#include <math.h>

// Pumpkin CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"

// Our headers
#include "rascal.h"

#define NEWTONS	0.001

//velocity vDesire;
//velocity rSense_I;
//velocity v;
parameters params;
pose POSE_DESIRED;
thrusterinfo THRUSTER_INFO;
float C_ItoB[3][3];
static float BThrust[3][11];
//static float deltaVB[3][11];
typedef struct _three_by_eleven {
	float data[3][11];
} three_by_eleven;

typedef struct _three_by_three {
  float data[3][3];
} three_by_three;

// used in yHoldPotential
int sign(float x) {
  if(x > 0) return 1;
  else if(x == 0) return 0;
  else return -1;
}

void q2dc(three_by_three *C_ItoB)
{
    int i, j;
  // used to multiply array values by struct values
  float poseValues[3][3];
  poseValues[0][0] = POSE_BOEING.q1 * POSE_BOEING.q1;
  poseValues[0][1] = POSE_BOEING.q2 * POSE_BOEING.q1;
  poseValues[0][2] = POSE_BOEING.q3 * POSE_BOEING.q1;
  poseValues[1][0] = POSE_BOEING.q1 * POSE_BOEING.q2;
  poseValues[1][1] = POSE_BOEING.q2 * POSE_BOEING.q2;
  poseValues[1][2] = POSE_BOEING.q3 * POSE_BOEING.q2;
  poseValues[2][0] = POSE_BOEING.q3 * POSE_BOEING.q1;
  poseValues[2][1] = POSE_BOEING.q3 * POSE_BOEING.q2;
  poseValues[2][2] = POSE_BOEING.q3 * POSE_BOEING.q3;
  // find in q2dc line 13, identity matrix
  const float eye[3][3] = {{1.000, 0.000, 0.000},
                           {0.000, 1.000, 0.000},
                           {0.000, 0.000, 1.000}};
  // find in q2dc line 12
  const float Q[3][3] = {{0, -POSE_BOEING.q3, POSE_BOEING.q2},
                         {POSE_BOEING.q3, 0, -POSE_BOEING.q1},
                         {-POSE_BOEING.q2, POSE_BOEING.q1, 0}};
  float ans[3][3];
  // first term of calculation on line 13 of q2dc, term before +
  float firstTerm[3][3];
  // middle term of calculation on line 13 of q2dc, term between + and -
  float middleTerm[3][3];
  for(i = 0; i < 3; i++)
  {
    for(j = 0; j < 3; j++)
    {
      firstTerm[i][j] = (2.0 * (POSE_BOEING.q4 * POSE_BOEING.q4) - 1) * eye[i][j];
      middleTerm[i][j] = 2.0 * poseValues[i][j]; 
      ans[i][j] = firstTerm[i][j] + middleTerm[i][j] - ((2*POSE_BOEING.q4)*Q[i][j]);
      C_ItoB->data[i][j] = ans[i][j];
    }
  }
}

/*
// Calculates inital values for dqdt
// dqdt = qdot(q(:,i), omega_BI(:,i));
// q = POSE_EST_q_values
// qdot.m in matlab
void output_dqdt(float p_dqdt[], float p_omega_BI[], float POSE_EST_q_values[])
{
  float omega_matrix[4][4] = {{0.000, p_omega_BI[2], -p_omega_BI[1], p_omega_BI[0]},
                              {-p_omega_BI[2], 0.000, p_omega_BI[0], p_omega_BI[1]},
                              {p_omega_BI[1], -p_omega_BI[0], 0.000, p_omega_BI[2]},
                              {-p_omega_BI[0], -p_omega_BI[1], -p_omega_BI[2], 0.000}};

  int c, d;
  float sum;
 
    for ( c = 0 ; c < 4 ; c++ ) {
      for ( d = 0 ; d < 4 ; d++ ){
        sum = sum + (0.5 * omega_matrix[c][d] * POSE_EST_q_values[d]);
      }

      p_dqdt[c] = sum;
      sum = 0; 
    }
}


// "Returns" updated q
// need to figure out expm() from matlab code
// see: q(:,i+1) = qdot(q(:,i), omega_BI(:,i+1), dt);
// qdot.m in matlab
void output_q(float POSE_EST_q_values[], float p_omega_BI[])
{
  float omega_matrix[4][4] = {{0.000, p_omega_BI[2], -p_omega_BI[1], p_omega_BI[0]},
                              {-p_omega_BI[2], 0.000, p_omega_BI[0], p_omega_BI[1]},
                              {p_omega_BI[1], -p_omega_BI[0], 0.000, p_omega_BI[2]},
                              {-p_omega_BI[0], -p_omega_BI[1], -p_omega_BI[2], 0.000}};

  float temp_q_values[4];

  // need to figure out exmp(omega_matrix) for this part
  // expm(0.5*omega_matrix*time)*q; found in qdot.m
  // then use multiplication loop below to multiply q * result of expm

  int c, d;
  float sum;
 
    for ( c = 0 ; c < 4 ; c++ ) {
      for ( d = 0 ; d < 4 ; d++ ){
        sum = sum + (0.5 * omega_matrix[c][d] * POSE_EST_q_values[d]);
      }

      temp_q_values[c] = sum;
      sum = 0; 
    }
  
  // assigns new q values into POSE
  POSE_EST.q1 = temp_q_values[0];
  POSE_EST.q2 = temp_q_values[1];
  POSE_EST.q3 = temp_q_values[2];
  POSE_EST.q4 = temp_q_values[3];
}
*/

// function used in select thruster to calculate vErr_B
// -C_ItoB * (POSE_DESIRE - POSE_IMG)
void matrix_mul_C_ItoBxPOSE(float matrix1[][3], float multiply[])
{
  int c, d;

  // used to multiply array values by struct values
  float POSE_values[3];
  float sum;
  sum = 0;

  POSE_values[0] = POSE_DESIRED.xidot - POSE_IMG.xidot;
  POSE_values[1] = POSE_DESIRED.yidot - POSE_IMG.yidot;
  POSE_values[2] = POSE_DESIRED.zidot - POSE_IMG.zidot;

    for ( c = 0 ; c < 3 ; c++ ) {
      for ( d = 0 ; d < 3 ; d++ ){
        sum = sum + (-matrix1[c][d] * POSE_values[d]);
      }  
 
      multiply[c] = sum;
      sum = 0; 
    }
}


void matrix_mul_vErr_BxdeltaVB(float matrix1[], float matrix2[][11], float p_score[], int p_j)
{
  int c;
  float sum;
  sum = 0;
  
  for ( c = 0 ; c < 3 ; c++ ) {
    sum += matrix1[c] * matrix2[p_j][c];
  }
  sum *=2;
  for (c = 0; c < 3; c++) {
     p_score[c] = sum + powf(matrix2[p_j][c], 2.00);
  }
  
}


void yHoldPotential(parameters *params) {
  float yError = POSE_IMG.yi - POSE_DESIRED.yi;
  float O = (2/PI)*(atanf(yError/5));
  float xDesire = params->xCruise * O;

  POSE_DESIRED.xidot = -(POSE_IMG.xi - xDesire) / 250;
  POSE_DESIRED.yidot = -(3/2) * params->w * POSE_IMG.xi;

  if(fabsf(POSE_DESIRED.zi) > 0.001) {
    if((fabsf(POSE_IMG.zi) / POSE_DESIRED.zi) > 1) {
      POSE_DESIRED.zidot = -.01 * sign(POSE_IMG.zi);
    } else {
      float t_phiz = sign(POSE_IMG.zidot) * (1/params->w) * acosf(POSE_IMG.zi) / POSE_DESIRED.zi;
      POSE_DESIRED.zidot = params->w * POSE_DESIRED.zi * sinf(params->w * t_phiz);
    }
  }
}

int selectThruster(three_by_eleven *deltaVB, three_by_three *C_ItoB) {
  int numThrustOptions = 11;
  int c, d;
  float score[3];
  float bestScore[3];
  //yHoldPotential(&params);

/* Begin yHoldPotential */
  float yError = POSE_IMG.yi - POSE_DESIRED.yi;
  float O = (2/PI)*(atanf(yError/5));
  float xDesire = params.xCruise * O;

  POSE_DESIRED.xidot = -(POSE_IMG.xi - xDesire) / 250;
  POSE_DESIRED.yidot = -(3.0/2.0) * params.w * POSE_IMG.xi;

  if(fabsf(POSE_DESIRED.zi) > 0.001) {
    if((fabsf(POSE_IMG.zi) / POSE_DESIRED.zi) > 1) {
      POSE_DESIRED.zidot = -.01 * sign(POSE_IMG.zi);
    } else {
      float t_phiz = sign(POSE_IMG.zidot) * (1/params.w) * acosf(POSE_IMG.zi) / POSE_DESIRED.zi;
      POSE_DESIRED.zidot = params.w * POSE_DESIRED.zi * sinf(params.w * t_phiz);
    }
  }
/* End yHoldPotential */

  float vErr_B[3];

  //matrix_mul_C_ItoBxPOSE(C_ItoB, vErr_B);
/* Begin matrix_mul_C_ItoBxPOSE */
  // used to multiply array values by struct values
  float POSE_values[3];
  float sum;
  sum = 0;

  POSE_values[0] = POSE_DESIRED.xidot - POSE_IMG.xidot;
  POSE_values[1] = POSE_DESIRED.yidot - POSE_IMG.yidot;
  POSE_values[2] = POSE_DESIRED.zidot - POSE_IMG.zidot;

    for ( c = 0 ; c < 3 ; c++ ) {
      for ( d = 0 ; d < 3 ; d++ ){
        sum = sum + (-C_ItoB->data[c][d] * POSE_values[d]);
      }  
 
      vErr_B[c] = sum;
      sum = 0; 
    }
/* End matrix_mul_C_ItoBxPOSE */

  int bestThruster = 0; // no thrust option
  bestScore[0] = -params.verror; // score if we dont thrust
  bestScore[1] = -params.verror;
  bestScore[2] = -params.verror;
  int j;
  for(j = 1; j < numThrustOptions; j++) {

    //matrix_mul_vErr_BxdeltaVB(vErr_B, deltaVB, score, j);
/* Begin matrix_mul_vErr_BxdeltaVB */
  
  sum = 0;
  
  for ( c = 0 ; c < 3 ; c++ ) {
    sum += vErr_B[c] * deltaVB->data[c][j];
  }
  sum *=2;
  for (c = 0; c < 3; c++) {
     score[c] = sum + powf(deltaVB->data[c][j], 2.00);
  }
/* End matrix_mul_vErr_BxdeltaVB */
    
    if ((score[0] < bestScore[0]) && (score[1] < bestScore[1]) && (score[2] < bestScore[2])) {
      bestThruster = j;
      bestScore[0] = score[0];
      bestScore[1] = score[1];
      bestScore[2] = score[2];
    } 
    
  }
  
  return bestThruster; 
}


void task_nav(void) {
  // might need to change these to POSE_EST values
  /*
  rSense_I.x = 0.0033;
  rSense_I.y = -0.0027;
  rSense_I.z = 0.0;
  v.x = 0.0016;
  v.y = -0.0027;
  v.z = 0.0;
  */

 // defines and inits POSE_DESIRED
  POSE_DESIRED.q1 = 0.0;
  POSE_DESIRED.q2 = 0.0;
  POSE_DESIRED.q3 = 0.0;
  POSE_DESIRED.q4 = 0.0;
  POSE_DESIRED.q1dot = 0.0;
  POSE_DESIRED.q2dot = 0.0;
  POSE_DESIRED.q3dot = 0.0;
  POSE_DESIRED.q4dot = 0.0;
  POSE_DESIRED.xi = 0.0;
  POSE_DESIRED.yi = 100.0;
  POSE_DESIRED.zi = 0.0;
  POSE_DESIRED.xidot = 0.0;
  POSE_DESIRED.yidot = 0.0;
  POSE_DESIRED.zidot = 0.0;

  params.w = 0.0012;  /* (2*pi)/orbitPeriod in rad/s */
  params.verror = 0.00000225; /* 0.0015^2 in m/s */
  params.xdes = 0.0;
  params.ydes = 10.0;
  params.zdes = 0.0;
  params.xCruise = 10.0; /* next orbit location (orbit transfer location) */

// defines and inits THRUSTER_INFO
  THRUSTER_INFO.thruster_Azminus = 0;
  THRUSTER_INFO.Azminustime = 0;
  THRUSTER_INFO.thruster_Bxminus = 0;
  THRUSTER_INFO.Bxminustime = 0;
  THRUSTER_INFO.thruster_Cyminus = 0;
  THRUSTER_INFO.Cyminustime = 0;
  THRUSTER_INFO.thruster_Dzplus = 0;
  THRUSTER_INFO.Dzplustime = 0;
  THRUSTER_INFO.thruster_Explus = 0;
  THRUSTER_INFO.Explustime = 0;
  THRUSTER_INFO.thruster_Fyplus = 0;
  THRUSTER_INFO.Fyplustime = 0;

static float BThrust[3][11] = {{0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 1.0000, -1.0000, -1.0000},
                               {0.0000, 0.0000, 0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 1.0000, -1.0000, 1.0000, -1.0000},
                               {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 0.0000, 0.0000}};

                // Thrust Options:    0        1       2        3       4      5       6        7       8        9        10    BODY FRAME COORDINATES
static float tmp_deltaVB[3][11] = {{0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, 0.0003, -0.0003, -0.0003}, //     X-axis
                                   {0.0000, 0.0000, 0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0003, -0.0003, 0.0003, -0.0003}, //     Y-axis
                                   {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0000, 0.0000}};  //     Z-axis

static three_by_eleven deltaVB;

memcpy(deltaVB.data, tmp_deltaVB, sizeof(float[3][11]));

// value for i=2 in Matlab sim

/*
static float tmp_C_ItoB[3][3] = {{-0.6733, 0.7070, 0.2164},
                            {-0.7208, -0.6929, 0.0209},
                            {0.1648, -0.1419, 0.9761}};


static float tmp_C_ItoB[3][3] = {{1.0000, -0.0058, -0.0069},
                            {0.0059, 1.0000, 0.0076},
                            {0.0068, -0.0076, 0.9999}};
*/


static float tmp_C_ItoB[3][3] = {{0.0000, 1.000, 0.0000},
                            {-1.0000, 0.0000, 0.0000},
                            {0.0000, 0.0000, 1.0000}};


static three_by_three C_ItoB;

memcpy(C_ItoB.data, tmp_C_ItoB, sizeof(float[3][3]));
  
// all possible thruster combinations
static float tmp_BThrust[3][11] = {{0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, 1.0000, -1.0000, -1.0000},
                               {0.0000, 0.0000, 0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 1.0000, -1.0000, 1.0000, -1.0000},
                               {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000, -1.0000, 0.0000, 0.0000, 0.0000, 0.0000}};

static three_by_eleven BThrust_1;

memcpy(BThrust_1.data, tmp_BThrust, sizeof(float[3][11]));

/*
  static float deltaVB_row0[] = {0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, 0.0003, -0.0003, -0.0003};
  static float deltaVB_row1[] = {0.0000, 0.0000, 0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0003, -0.0003, 0.0003, -0.0003};
  static float deltaVB_row2[] = {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0003, -0.0003, 0.0000, 0.0000, 0.0000, 0.0000};
  static float *deltaVB[3];
  deltaVB[0] = deltaVB_row0;
  deltaVB[1] = deltaVB_row1;
  deltaVB[2] = deltaVB_row2;
*/

  static int thrusterOption;
  static int i, j;
  static char* prpCMD; //static char to receive "message" from external_cmds
  //static char prpCMD[DATA_SIZE];

  while(1) {
    OS_Delay(250);
    if (THRUST_ENABLE_FLAG == ENABLED) {
	    // Looks for non-zero values in POSE_DESIRED to begin movement towards secondary spacecraft
	    if (POSE_DESIRED.xi != 0 || POSE_DESIRED.yi != 0 || POSE_DESIRED.zi != 0) {
	      thrusterOption = selectThruster(&deltaVB, &C_ItoB);
	      q2dc(&C_ItoB);
	      // char tmp[20];
	      //sprintf(tmp, "This is x: %d\r\n", thrusterOption);
	      //csk_uart0_puts(tmp);
	/*
	      for(i = 0; i < 3; i++) {
	        for(j = 0; j < 3; j++) {
	          sprintf(tmp, "%f ", C_ItoB.data[i][j]);
	          csk_uart0_puts(tmp);
	        }
	      }
	*/
	      if (thrusterOption >= 1 && thrusterOption <=10) {
	 /*
	        csk_uart0_puts("Thrusters ON!\r\n");
	        csk_io22_high(); csk_uart0_puts("S1 ON!\r\n");
	        csk_io20_high(); csk_uart0_puts("S2 ON!\r\n");
	        
	        OS_Delay(220);  //delay of about 2s to pressurize veins - per Bryant
	        
	        switch(thrusterOption) {
	          case 1:
	            csk_io16_high(); csk_uart0_puts("F ON! (+X-Body axis)\r\n");
	            break;
	          case 2:
	            csk_io17_high(); csk_uart0_puts("C ON! (-X-Body axis)\r\n");
	            break;
	          case 3:
	            csk_io18_high(); csk_uart0_puts("B ON! (+Y-Body axis)\r\n");
	            break;
	          case 4:
	            csk_io19_high(); csk_uart0_puts("E ON! (-Y-Body axis)\r\n");
	            break;
	          case 5:
	            csk_io21_high(); csk_uart0_puts("D ON! (+Z-Body axis)\r\n");
	            break;
	          case 6:
	            csk_io23_high(); csk_uart0_puts("A ON! (-Z-Body axis)\r\n");
	            break;
	          case 7:
	            csk_io16_high(); csk_uart0_puts("F ON! (+X-Body axis)\r\n");
	            csk_io18_high(); csk_uart0_puts("B ON! (+Y-Body axis)\r\n");
	            break;
	          case 8:
	            csk_io16_high(); csk_uart0_puts("F ON! (+X-Body axis)\r\n");
	            csk_io19_high(); csk_uart0_puts("E ON! (-Y-Body axis)\r\n");
	            break;
	          case 9:
	            csk_io17_high(); csk_uart0_puts("C ON! (-X-Body axis)\r\n");
	            csk_io18_high(); csk_uart0_puts("B ON! (+Y-Body axis)\r\n");
	            break;
	          case 10:
	            csk_io17_high(); csk_uart0_puts("C ON! (-X-Body axis)\r\n");
	            csk_io19_high(); csk_uart0_puts("E ON! (-Y-Body axis)\r\n");
	            break;
	        }
	        
	        OS_Delay(10); // fire thrusters for 100ms
	
	         // turn off main solenoids
	         csk_io22_low(); csk_uart0_puts("S1 OFF!\r\n");
	         OS_Delay(100);
	         csk_io20_low(); csk_uart0_puts("S2 OFF!\r\n");
	
	         // turn off thrusters
	         switch(thrusterOption) {
	          case 1:
	            csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	            break;
	          case 2:
	            csk_io17_low(); csk_uart0_puts("C OFf! (-X-Body axis)\r\n");
	            break;
	          case 3:
	            csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	            break;
	          case 4:
	            csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	            break;
	          case 5:
	            csk_io21_low(); csk_uart0_puts("D OFF! (+Z-Body axis)\r\n");
	            break;
	          case 6:
	            csk_io23_low(); csk_uart0_puts("A OFF! (-Z-Body axis)\r\n");
	            break;
	          case 7:
	            csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	            csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	            break;
	          case 8:
	            csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	            csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	            break;
	          case 9:
	            csk_io17_low(); csk_uart0_puts("C OFF! (-X-Body axis)\r\n");
	            csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	            break;
	          case 10:
	            csk_io17_low(); csk_uart0_puts("C OFF! (-X-Body axis)\r\n");
	            csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	            break;
	        }
	         
	        
	        if (thrusterOption == 1) { csk_io16_high(); csk_uart0_puts("F ON!\r\n"); }
	        else if (thrusterOption == 2) { csk_io17_high(); csk_uart0_puts("C ON!\r\n"); }
	        else if (thrusterOption == 3) { csk_io18_high(); csk_uart0_puts("B ON!\r\n"); }
	        else if
	        */
	      }
	    }
	
	
	      if(OSReadMsg(MSG_PRPTONAV_P)) {
	        prpCMD=((char*)(OSTryMsg(MSG_PRPTONAV_P)));
	        //memcpy(prpCMD, OSTryMsg(MSG_PRPTONAV_P), DATA_SIZE*sizeof(char));
	        char tmp[150];
	        
	        // Start message verification (for sanity (AND pointer) check)
	        if (prpCMD[0]=='P' && prpCMD[1]=='R' && prpCMD[2]=='P') { //if a (beings with PRP!!!) -- sanity check
	          // Thruster request!
	          /* Expects PRP12ABCDEFXXX,
	             PRP = propulsion command block (gonna send a message to NAV)
	             12ABCDEF = 0 or 1 (binary) and represent the thrusters labeled on Rascal as shown below.
	             XXX = number of deciseconds to fire thrusters
	           Thruster  GPIO
	           --------  ----
	                 A   IO.23
	            2 = S1   IO.22
	                 D   IO.21
	            4 = S2   IO.20
	                 E   IO.19
	                 B   IO.18
	                 C   IO.17
	                 F   IO.16
	          */      
	          // extracts burn time from prpCMD = last 3 digits of prpCMD
	          int x, y, z, burn_time_ds;
	          static int burn_time = 0;
	          //static int MAX_TIMEOUT = 255; // because OSBYTES_OF_DELAYS = 1 (1 Byte)
	          static int TIMEOUT_MULTIPLE = 0;
	          x = (prpCMD[11] - '0') * 100;
	          y = (prpCMD[12] - '0') * 10;
	          z = prpCMD[13] - '0';
	          burn_time_ds = (x+y+z);    // time in deciseconds
	          burn_time = burn_time_ds*10; // time in system ticks: 1 system tick = 10ms (@100Hz)
	          while (burn_time > 255) {
	            TIMEOUT_MULTIPLE++;
	            burn_time -= 255;
	          }
	          sprintf(tmp, "Burn time is: %d deciseconds\r\n", burn_time_ds);
	          csk_uart0_puts(tmp);
	          sprintf(tmp, "Burn time is: %d system ticks  TIMEOUT_MULTIPLE: %d\r\n", burn_time, TIMEOUT_MULTIPLE);
	          csk_uart0_puts(tmp);
	
	       
	        
	          /** 21 Nov 2014 - DJU
	           ** The sequence this is wired and set to initiate is on purpose as for some
	           ** reason only 3 GPIOs can be turned on at once with the header board being used
	           ** to connect to the thrusters. Thrusters S1 and S2 will not work together on IO23 and IO22.
	           ** Unsure as to why but will note it for further research.
	          **/
	          // turns ON appropriate thruster(s)
	          csk_uart0_puts("Thrusters ON!\r\n");
	          if (prpCMD[4] == '1') {csk_io22_high(); csk_uart0_puts("S1 ON!\r\n");}
	          if (prpCMD[6] == '1') {csk_io20_high(); csk_uart0_puts("S2 ON!\r\n");}
	          
	          OS_Delay(200);  //delay of about 2s to pressurize veins - per Bryant
	
	          if (prpCMD[3] == '1') {csk_io23_high(); csk_uart0_puts("A ON!\r\n");}
	          if (prpCMD[8] == '1') {csk_io18_high(); csk_uart0_puts("B ON!\r\n");}
	          if (prpCMD[9] == '1') {csk_io17_high(); csk_uart0_puts("C ON!\r\n");}
	          if (prpCMD[5] == '1') {csk_io21_high(); csk_uart0_puts("D ON!\r\n");}
	          if (prpCMD[7] == '1') {csk_io19_high(); csk_uart0_puts("E ON!\r\n");}
	          if (prpCMD[10]== '1') {csk_io16_high(); csk_uart0_puts("F ON!\r\n");}
	        
	          // delay for length of burn_time
	          // from pg 211 of Salvo RTOS pdf on how to wait longer than maximum timeout
	          if (TIMEOUT_MULTIPLE > 0) {
	            static int i;
	            for (i = 0; i < TIMEOUT_MULTIPLE; i++) {
	              //OS_WaitSem(SEM_PRP_BURNTIME_P, MAX_TIMEOUT);
	              OS_Delay(255); 
	            }
	          }
	          OS_Delay(burn_time); // time left after loop
	          
	          // turns OFF appropriate thruster(s)
	          if (prpCMD[4] == '1') {csk_io22_low(); csk_uart0_puts("S1 OFF!\r\n");}
	          
	          OS_Delay(250); // delay of 2.5s to shut S1 valve - per Bryant
	          if (prpCMD[6] == '1') {csk_io20_low(); csk_uart0_puts("S2 OFF!\r\n");}
	
	          if (prpCMD[3] == '1') {csk_io23_low(); csk_uart0_puts("A OFF!\r\n");}
	          if (prpCMD[8] == '1') {csk_io18_low(); csk_uart0_puts("B OFF!\r\n");}
	          if (prpCMD[9] == '1') {csk_io17_low(); csk_uart0_puts("C OFF!\r\n");}
	          if (prpCMD[5] == '1') {csk_io21_low(); csk_uart0_puts("D OFF!\r\n");}
	          if (prpCMD[7] == '1') {csk_io19_low(); csk_uart0_puts("E OFF!\r\n");}
	          if (prpCMD[10]== '1') {csk_io16_low(); csk_uart0_puts("F OFF!\r\n");}
	        
	          csk_uart0_puts("Thrusters OFF!\r\n");
	
	          /** tests for passing around POSEs **/
	          /**
	          sprintf(tmp, "This is POSE_DESIRED.xi: %d\r\n", POSE_DESIRED.xi++);
	          csk_uart0_puts(tmp);
	          sprintf(tmp, "This is POSE_EST.xi: %d\r\n", POSE_EST.xi);
	          csk_uart0_puts(tmp);
	          **/
	          } else {
	          char tmps[80];
		      sprintf(tmps, "Command failed in task_nav: %s", prpCMD);
		      csk_uart0_puts(tmps);
	          }
	        }
	
	    /** COMMENTED OUT TO GET NAV FUNCTIONAL
		// Just some constant stuff for the calculations
			const int m = 3; // kg
			const int dt = 1; // seconds
			const int velError = 0.00000225; 
			int omega = (2*3.14159)/(60*90); 
			const int xCruise = 10; 	
			BThrust = {0*T(:,1) T T(:,1)+T(:,3) T(:,1)+T(:,4) T(:,2)+T(:,3) T(:,2)+T(:,4)};
			deltaVB = BThrust*(dt/m);
		// Get position and velocity (from task_estimator)
			// postion (name posSensorI (x,y,z))
			// velocity (name velSensorI (x,y,z)
		
		// Get Desired position coordinates sent to the code from SLUGND	
			extern int xposDes; 
			extern int yposDes;
			extern int zposDes;
		
		// Compute desired velocity (and velocity error) [yHoldPotential.m] -- This defines the error, rotationa; velocity, 
		// and orbit transfer locationa nd then uses teh desired position to give the desired velocity values
	
			yError = yposSensorI - yposDes; //sensor value is what is coming from the estimator
			turnTime = (2/pi)*atan(yError/5); //disctates how fast the orbit transfer location is reached
			xDesire = xCruise*turnTime
	
			//Driving the velocity values to the accurate value:
			xvelDes = -(xposSensorI-xDesire)/250; //this controls how "smooth" the motion to the next value is
			yvelDes = -(3/2)*omega*xposSensorI;
	
			if(abs(zposDes > 0.001)) {
				if(abs(zposSensorI/zposDes > 1)) {
					zvelDes = -.01*sign(zposSensorI);
				}
				else {
					t_phiz = sign(zvelSensorI*(1/omega)*acos(zposSensorI/zposDes);
					zvelDes = omega*zposDes*sin(omega*t_phiz);
				}	
			}			
	
		// Score the various thruster options [selectthruster.m]
			int T[3][6] = 0.001*{1, -1, 0,  0, 0,  0,
				       0,  0, 1, -1, 0,  0,
			    	   0,  0, 0,  0, 1, -1};  
			velocityErrorB = xvelDes
			bestThruster = 1; //No thrust option	
			const int BestScore = -velError;
				for (j=2:ThrustOptions) {
					score = 2*(velocityErrorB*deltaVB(:,j)+deltaVB(:,j).^2;
					if (score<BestScore) {
						bestThruster = j;
						BestScore = score;
					}
				}
	
		// Choose best thruster(s) -- BEST SCORE -- and operate thruster(s) [Satmotion3dDeliverable1 lines 107 - 116]
	 
		//This involves some 12x3 matricies, so I will attack this at another time.
		
	    END COMMENT **/
    } 
   }// END while(1)
} 
