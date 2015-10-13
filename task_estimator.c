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
#include "task_estimator.h"
#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <math.h>
#include <libq.h>

// Pumpkin CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"

// Pumpkin Salvo headers
#include "salvo.h"

// Our headers
#include "rascal.h"

//define pi for calcualtions
//#define PI 3.14159265359

// defines POSE_EST from rascal.h
pose POSE_EST;
citob C_ItoB;

/*
float estimator(pose POSE_EST_PREVIOUS, thrusterChoice, pose POSE_IMG, pose POSE_BOEING) {
  //TBD
}
*/

  
  // find a new way
  // holds q values of POSE_EST for purposes of matrix multiplication
  // static float POSE_EST_q_values[4] = {POSE_EST.q1, POSE_EST.q2, POSE_EST.q3, POSE_EST.q4};
  

//void q2dc(three_by_three *C_ItoB)
void q2dc()
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
      C_ItoB.data[i][j] = ans[i][j];
    }
  }
} // end q2dc()

// tbd: deep copy values from POSE_EST into s_hat_prev
void set_s_hat_previous(float p_s_hat_prev[6]) {
  p_s_hat_prev[0] = POSE_EST.xi;
  p_s_hat_prev[1] = POSE_EST.yi;
  p_s_hat_prev[2] = POSE_EST.zi;
  p_s_hat_prev[3] = POSE_EST.xidot;
  p_s_hat_prev[4] = POSE_EST.yidot;
  p_s_hat_prev[5] = POSE_EST.zidot;
}


/* Multiplies L (6x3) matrix with (3x6) Cd matrix. Result is LxCd (6x6) matrix */
void matrix_mul_LxCd(float matrix1[][3], int rows1, int col1, float matrix2[][6], int rows2, int col2, float multiply[][6])
{
  int c, d, k;
  float sum;
 
    for ( c = 0 ; c < rows1 ; c++ )
    {
      for ( d = 0 ; d < col2 ; d++ )
      {
        for ( k = 0 ; k < rows2 ; k++ )
        {
          sum = sum + matrix1[c][k] * matrix2[k][d];
        }
 
        multiply[c][d] = sum;
        sum = 0;
      }
    }
}

// Calculates Bd_I by using C_ItoB and Bd, with Bd being 
// from Matlab: Bd_I = [C_ItoB, zeros(3,3); zeros(3,3), C_ItoB]*Bd;
// Bd_I = 6*11 array
// Bd = 6*11  
void calculate_Bd_I(float p_Bd_I[][11], float p_Bd[][11]) {

  // [C_ItoB, zeros(3,3); zeros(3,3), C_ItoB]  <-- from Matlab
  float tmp_matrix[6][6] = { { C_ItoB.data[0][0], C_ItoB.data[0][1], C_ItoB.data[0][2], 0.0000, 0.0000, 0.0000 }, 
                         { C_ItoB.data[1][0], C_ItoB.data[1][1], C_ItoB.data[1][1], 0.0000, 0.0000, 0.0000 },
                         { C_ItoB.data[2][0], C_ItoB.data[2][1], C_ItoB.data[2][2], 0.0000, 0.0000, 0.0000 },
                         { 0.0000, 0.0000, 0.0000, C_ItoB.data[0][0], C_ItoB.data[0][1], C_ItoB.data[0][2] },
                         { 0.0000, 0.0000, 0.0000, C_ItoB.data[1][0], C_ItoB.data[1][1], C_ItoB.data[1][1] },
                         { 0.0000, 0.0000, 0.0000, C_ItoB.data[2][0], C_ItoB.data[2][1], C_ItoB.data[2][2] }};

  int c, d, k;
  int rows1 = 6;
  int rows2 = 6;
  int col2 = 11;
  float sum;
  
  for ( c = 0 ; c < rows1 ; c++ )
    {
      for ( d = 0 ; d < col2 ; d++ )
      {
        for ( k = 0 ; k < rows2 ; k++ )
        {
          sum = sum + tmp_matrix[c][k] * p_Bd[k][d];
        }
 
        p_Bd_I[c][d] = sum;
        sum = 0;
      }
    }
} // end matrix_mul_Bd_I

// From matlab rascalEstimator.m: Bd_I*thrusterChoice
void matrix_mul_Bd_Ixthrusterchoice(float matrix1[][11], int rows1, int col1, float multiply[6])
{
  // matrix2 is the B_Thrust analog from the MatLab code
  // all matrix2 values are result of thruster combination selection times .001 and are measured in Newtons
  float matrix2[3][1];
  int rows2 = 3, col2 = 1;
  // matrix1 = Bd_I (6x11)
  // matrix2 = thrusterChoice
  int c, d, k;
  float sum;


	switch(THRUSTER_INFO.thrusterOption) {
	  case 0:
	    matrix2[0][0] = 0.0;
		matrix2[1][0] = 0.0;
		matrix2[2][0] = 0.0;
	    break;
	  case 1:
		matrix2[0][0] = 0.001;
		matrix2[1][0] = 0.0;
		matrix2[2][0] = 0.0;
	    //csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,0,0,0,0,0,0,1,thrustTime, thrusterOption);
	    break;
	  case 2:
		matrix2[0][0] = -0.001;
		matrix2[1][0] = 0.0;
		matrix2[2][0] = 0.0;
	    //csk_io17_low(); csk_uart0_puts("C OFf! (-X-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,1,thrustTime,0,0,0,0,0,0, thrusterOption);
	    break;
	  case 3:
		matrix2[0][0] = 0.0;
		matrix2[1][0] = 0.001;
		matrix2[2][0] = 0.0;
	    //csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,1,thrustTime,0,0,0,0,0,0,0,0, thrusterOption);
	    break;
	  case 4:
		matrix2[0][0] = 0.0;
		matrix2[1][0] = -0.001;
		matrix2[2][0] = 0.0;
	    //csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,0,0,0,0,1,thrustTime,0,0, thrusterOption);
	    break;
	  case 5:
		matrix2[0][0] = 0.0;
		matrix2[1][0] = 0.0;
		matrix2[2][0] = 0.001;
	    //csk_io21_low(); csk_uart0_puts("D OFF! (+Z-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,0,0,1,thrustTime,0,0,0,0, thrusterOption);
	    break;
	  case 6:
		matrix2[0][0] = 0.0;
		matrix2[1][0] = 0.0;
		matrix2[2][0] = -0.001;
	    //csk_io23_low(); csk_uart0_puts("A OFF! (-Z-Body axis)\r\n");
	    //updateTHRUSTER_INFO(1,thrustTime,0,0,0,0,0,0,0,0,0,0, thrusterOption);
	    break;
	  case 7:
		matrix2[0][0] = 0.001;
		matrix2[1][0] = 0.001;
		matrix2[2][0] = 0.0;
	    //csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	    //csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,1,thrustTime,0,0,0,0,0,0,1,thrustTime, thrusterOption);
	    break;
	  case 8:
		matrix2[0][0] = 0.001;
		matrix2[1][0] = -0.001;
		matrix2[2][0] = 0.0;
	    //csk_io16_low(); csk_uart0_puts("F OFF! (+X-Body axis)\r\n");
	    //csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,0,0,0,0,1,thrustTime,1,thrustTime, thrusterOption);
	    break;
	  case 9:
		matrix2[0][0] = -0.001;
		matrix2[1][0] = 0.001;
		matrix2[2][0] = 0.0;
	    //csk_io17_low(); csk_uart0_puts("C OFF! (-X-Body axis)\r\n");
	    //csk_io18_low(); csk_uart0_puts("B OFF! (+Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,1,thrustTime,1,thrustTime,0,0,0,0,0,0, thrusterOption);
	    break;
	  case 10:
		matrix2[0][0] = -0.001;
		matrix2[1][0] = -0.001;
		matrix2[2][0] = 0.0;
	    //csk_io17_low(); csk_uart0_puts("C OFF! (-X-Body axis)\r\n");
	    //csk_io19_low(); csk_uart0_puts("E OFF! (-Y-Body axis)\r\n");
	    //updateTHRUSTER_INFO(0,0,0,0,1,thrustTime,0,0,1,thrustTime,0,0, thrusterOption);
	    break;
	}
 
    for ( c = 0 ; c < rows1 ; c++ )
    {
      for ( d = 0 ; d < col2 ; d++ )
      {
        for ( k = 0 ; k < rows2 ; k++ )
        {
          sum = sum + matrix1[c][k] * matrix2[k][d];
        }
 
        multiply[c] = sum;
        sum = 0;
      }
    }
}

/* result of: {Ad - (L * Cd) * s_hat_prev] - it's multiplying first part (6x6 matrix)
   with s_hat_prev (6 element array)
   modifies this 6 element array: Ad_LxCdxs_hat_prev 
   This is not a direct matrix to matrix multiplication because of second array being 6 elements */
void matrix_mul_s_hat_prev(float matrix1[][6], float matrix2[], float multiply[])
{
  // Referenced arrays:
  // matrix1 = Ad_LxCd
  // matrix2 = s_hat_prev
  // matrix3 = Ad_LxCdxs_hat_prev
  
  int c, d;
  float sum;
 
    for ( c = 0 ; c < 6 ; c++ ) {
      for ( d = 0 ; d < 6 ; d++ ){
        sum = sum + matrix1[c][d] * matrix2[d];
      }  
 
      multiply[c] = sum;
      sum = 0; 
    }
}

//void matrix_mul_Lxy(float matrix1[][3], int rows1, int col1, float matrix2[][1], int rows2, int col2, float multiply[][1])
void matrix_mul_Lxy(float matrix1[][3], int rows1, int col1, float multiply[6])
{
  // POSE_IMG position values (xi, yi, zi)
  float matrix2[3][1];
  matrix2[0][0] = POSE_IMG.xi;
  matrix2[1][0] = POSE_IMG.yi;
  matrix2[2][0] = POSE_IMG.zi;
  int c, d, k;
  float sum;
 
    for ( c = 0 ; c < rows1 ; c++ )
    {
      for ( d = 0 ; d < 1 ; d++ )
      {
        for ( k = 0 ; k < 3 ; k++ )
        {
          sum = sum + matrix1[c][k] * matrix2[k][d];
        }
 
        multiply[c] = sum;
        sum = 0;
      }
    }
}

/* subtracts two 6x6 matrices and stores result in 3rd matrix */
void matrix_sub(float matrix1[][6], float matrix2[][6], float answer[][6])
{
  int i, j;
  for(i = 0; i < 6; i++)
  {
    for(j = 0; j < 6; j++)
    {
      answer[i][j] = matrix1[i][j] - matrix2[i][j];
    }
  }
}

/* Matlab code: 
     s_hat = (Ad - L*Cd)*s_hat_prev + Bd_I*thrusterChoice + L*y
     inputs are: (6x1), (6x1), & (6x1) matrices, actually 6 element arrays
     output is : (6x1) matrix -- actually 6 element array
*/
void matrix_add_s_hat(float matrix1[6], float matrix2[6], float matrix3[6], float answer[6])
{
  int i;
  for(i = 0; i < 6; i++)
  {
    //new s_hat (6x1) 6 element array
	//matrix1 = Ad_LxCdxs_hat_prev
	//matrix2 = Bd_Ixthrusterchoice
	//matrix3 = Lxy
	//answer = s_hat (THIS IS POSE_EST!!!!!!!)
    answer[i] = matrix1[i] + matrix2[i] + matrix3[i];
  }
} // end matrix_add_s_hat 

char * int2bin(long i)
{
  size_t bits = sizeof(long) * CHAR_BIT;

  char str[33];
  if(!str) return NULL;
  str[bits] = 0;

  // type punning because signed shift is implementation-defined
  unsigned u = *(unsigned *)&i;
  for(; bits--; u >>= 1)
    str[bits] = u & 1 ? '1' : '0';

  return str;
}

void task_estimator(void) {

  unsigned long x;
  char* ans;
  unsigned long boeing_quat = 4294859922, quat_float = 0;

  if (boeing_quat & 2147483648) {  
      ans = int2bin(boeing_quat);
	  //csk_uart0_puts("%s\n", ans);
	  csk_uart0_puts(ans);
	  x = boeing_quat;
	  quat_float = x & 3221225472;
	  quat_float = quat_float >> 14;
      quat_float = quat_float | 0xFFFe0000;
	  x = x & 1073741823;
	  x = x >> 14;
	  quat_float = x | quat_float;
  } else {
	  ans = int2bin(boeing_quat);
	  //csk_uart0_puts("%s\n", ans);
	  csk_uart0_puts(ans);
	  x = boeing_quat;
	  quat_float = x & 3221225472;
	  quat_float = quat_float >> 14;
	  x = x & 1073741823;
	  x = x >> 14;
	  quat_float = x | quat_float;
  }  

  quat_float++;
  //ans = int2bin(quat_float);
  
  _Q16 fpTest, anothertest;
  fpTest = (_Q16)quat_float;
  float test;
  test = _itofQ16(fpTest);
  anothertest = _Q16ftoi(-0.0001);
  x = sizeof(float);

  Nop(); Nop(); Nop();
  //csk_uart0_puts("%s\n%ld\n", ans, quat_float);
  //csk_uart0_puts(ans);
  //csk_uart0_puts("\n");
  //sprintf(ans, "%ld\n", quat_float);
  //csk_uart0_puts(ans);

  
  // defines and inits POSE_EST
  //intial values for quaternion part of POSE_EST taken from line 84 of Sat3DDeliverable
  POSE_EST.q1 = 0.000;
  POSE_EST.q2 = 0.000;
  POSE_EST.q3 = 0.7071;
  POSE_EST.q4 = 0.7071;
  POSE_EST.q1dot = 0.0;
  POSE_EST.q2dot = 0.0;
  POSE_EST.q3dot = 0.0;
  POSE_EST.q4dot = 0.0;
  POSE_EST.xi=0.0;
  POSE_EST.yi=0.0;
  POSE_EST.zi=0.0;
  POSE_EST.xidot = 0.0;
  POSE_EST.yidot = 0.0;
  POSE_EST.zidot = 0.0;
  
  // defines static vars used for linear orbit matrices
  //static float n, dt;
  //n = 2*PI/95/60;
  //dt = 0.1;
  //static float xi, xdoti, yi, ydoti, zi, zdoti, xi1, xdoti1, yi1, ydoti1, zi1, zdoti1;
  //xi = 0.0;
  //yi = 0.0;
  //zi = 0.0;
 
  static float Ad[6][6] = { {1.0000, 0.0000, 0.0000, 1.0000, 0.0012, 0.0000}, 
                         {0.0000, 1.0000, 0.0000, -0.0012, 1.0000, 0.0000},
                         {0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 1.0000},
                         {0.0000, 0.0000, 0.0000, 1.0000, 0.0023, 0.0000},
                         {0.0000, 0.0000, 0.0000, -0.0023, 1.0000, 0.0000},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000}};

 
  static float Bd[6][11] = { {0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0000, 0.0000, 0.0005, 0.0005, -0.0005, -0.0005}, 
                         {0.0000, 0.0000, 0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0005, -0.0005, 0.0005, -0.0005},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0000, 0.0000},
                         {0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0010, -0.0010, -0.0010},
                         {0.0000, 0.0000, 0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0010, -0.0010, 0.0010, -0.0010},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0000, 0.0000}};
  
  static float Bd_I[6][11];

  static int rowsCd = 3;
  static int colsCd = 6;
  static float Cd[3][6] = {{1, 0, 0, 0, 0, 0},
                           {0, 1, 0, 0, 0, 0},
                           {0, 0, 1, 0, 0, 0}};
  
  

  static int rowsL = 6;
  static int colsL = 3;
  static float L[6][3] = { {0.0700, 0.0001, 0.0000}, 
                         {-0.0001, 0.0699, 0.0000},
                         {0.0000, 0.0000, 0.0531},
                         {0.0024, 0.0001, 0.0000},
                         {-0.0001, 0.0024, 0.0000},
                         {0.0000, 0.0000, 0.0014}};
  

  

  // s_hat 0-2 = POSE_EST position values (in I coords)
  // s_hat 3-5 = POSE_EST velocity values (in I coords)
  static float s_hat[6] = {0, 0, 0, 0, 0, 0};  //also POSE_EST
  static float s_hat_prev[6] = {0, 0, 0, 0, 0, 0};

  static float LxCd[6][6], Ad_LxCd[6][6], Ad_LxCdxs_hat_prev[6], Bd_Ixthrusterchoice[6], Lxy[6];

  // value for i=2 in Matlab sim

  /*
  static float tmp_C_ItoB[3][3] = {{-0.6733, 0.7070, 0.2164},
                                   {-0.7208, -0.6929, 0.0209},
                                   {0.1648, -0.1419, 0.9761}};


  static float tmp_C_ItoB[3][3] = {{1.0000, -0.0058, -0.0069},
                                   {0.0059, 1.0000, 0.0076},
                                   {0.0068, -0.0076, 0.9999}};
  */


  /*C_ItoB.data = {{0.0000, 1.000, 0.0000},
                   {-1.0000, 0.0000, 0.0000},
                   {0.0000, 0.0000, 1.0000}};*/
  C_ItoB.data[0][0] = 0.0000;
  C_ItoB.data[0][1] = 1.0000;
  C_ItoB.data[0][2] = 0.0000;
  C_ItoB.data[1][0] = -1.0000;
  C_ItoB.data[1][1] = 0.0000;
  C_ItoB.data[1][2] = 0.0000;
  C_ItoB.data[2][0] = 0.0000;
  C_ItoB.data[2][1] = 0.0000;
  C_ItoB.data[2][2] = 1.0000;


  calculate_Bd_I(Bd_I, Bd); // Bd_I = [C_ItoB, zeros(3,3); zeros(3,3), C_ItoB]*Bd;  <-- from Matlab

  /* s_hat calculations are ran here before while(1) in order to calculate from initial values */
  /** 
      These next function calls come from the RascalEstimator.m Matlab code
      to calculate s_hat, which is POSE_EST, like so:
      s_hat = (Ad - L*Cd)*s_hat_prev + Bd_I*thrusterChoice + L*y;
  **/

  /****** begin s_hat (POSE_EST) calculations ******/
  set_s_hat_previous(s_hat_prev); // deep copies POSE_EST into s_hat_prev
  matrix_mul_LxCd(L, rowsL, colsL, Cd, rowsCd, colsCd, LxCd);
  matrix_sub(Ad, LxCd, Ad_LxCd);
  matrix_mul_s_hat_prev(Ad_LxCd, s_hat_prev, Ad_LxCdxs_hat_prev);
  matrix_mul_Bd_Ixthrusterchoice(Bd_I, 6, 6, Bd_Ixthrusterchoice);
  matrix_mul_Lxy(L, rowsL, colsL, Lxy);
  matrix_add_s_hat(Ad_LxCdxs_hat_prev, Bd_Ixthrusterchoice, Lxy, s_hat);
  /****** end s_hat (POSE_EST) calculations) ******/
  // q2dc() is LAST in ESTIMATOR calculation realm - per Matlab and Fred
  q2dc();

  while(1) {
    OS_Delay(50);

    calculate_Bd_I(Bd_I, Bd); // Bd_I = [C_ItoB, zeros(3,3); zeros(3,3), C_ItoB]*Bd;  <-- from Matlab 

    /** 
      These next function calls come from the RascalEstimator.m Matlab code
      to calculate s_hat, which is POSE_EST, like so:
      s_hat = (Ad - L*Cd)*s_hat_prev + Bd_I*thrusterChoice + L*y;
    **/

    /****** begin s_hat (POSE_EST) calculations ******/
    set_s_hat_previous(s_hat_prev); // deep copies POSE_EST into s_hat_prev
    matrix_mul_LxCd(L, rowsL, colsL, Cd, rowsCd, colsCd, LxCd);
    matrix_sub(Ad, LxCd, Ad_LxCd);
    matrix_mul_s_hat_prev(Ad_LxCd, s_hat_prev, Ad_LxCdxs_hat_prev);
    matrix_mul_Bd_Ixthrusterchoice(Bd_I, 6, 11, Bd_Ixthrusterchoice);
    matrix_mul_Lxy(L, rowsL, colsL, Lxy);
    matrix_add_s_hat(Ad_LxCdxs_hat_prev, Bd_Ixthrusterchoice, Lxy, s_hat);
    /****** end s_hat (POSE_EST) calculations) ******/
	// q2dc() is LAST in ESTIMATOR calculation realm - per Matlab and Fred
    q2dc();
    
    /** Tests for POSE**/
/*
    char str[100];
    csk_uart0_puts("\r\nPOSE_BOEING from EST:\r\n");
    //sprintf(str, "%d,%d,%d,%d,%d,%d,%d\r\n", POSE_EST.xi++, POSE_EST.yi++, POSE_EST.zi++,POSE_EST.q1++,POSE_EST.q2++,POSE_EST.q3++,POSE_EST.q4++);
    sprintf(str, "%f,%f,%f,%f,%f,%f,%f\r\n", POSE_BOEING.xi++, POSE_BOEING.yi++, POSE_BOEING.zi++,POSE_BOEING.q1++,POSE_BOEING.q2++,POSE_BOEING.q3++,POSE_BOEING.q4++);
    //sprintf(str, "%d   %d", POSE_EST.xi,POSE_BOEING.xi);
    csk_uart0_puts(str);
    csk_uart0_puts("END POSE_EST\r\n");
    //csk_uart0_puts(STR_CRLF "POSE_BOEING.xi: ");
    //csk_uart0_puts((char *)POSE_EST.xi);
*/    
    /**END Tests for POSE **/

/*
    //FROM MATLAB CODE -variables declared as floats
	xi1 = xi + dt*xdoti ;
	yi1 = yi + dt*ydoti ;
	zi1 = zi + dt*zdoti ;
	xdoti1 = (3*n*n*dt)*xi + xdoti + 2*n*dt*ydoti ;
	ydoti1 = -2*n*dt*xdoti + ydoti ;
	zdoti1 = -n*n*dt*zi + zdoti ;

	xi=xi1;
	yi=yi1;
	zi=zi1;
	xdoti=xdoti1;
	ydoti=ydoti1;
	zdoti=zdoti1;
*/
	// Ask (?) Boeing for quaternion (need to figure out how this is implemented)

	// Propagate position and velocity states 


	// Listen for BINSEM (?) of image updates  -talk with Nick/Bob/Swartwout about this
	
	// And fix the estimation accordingly (MAS will help)

	// Make sure to have velocity and position in BODY coordinates
	// available to other tasks

  } /* while */
} /* task_estimator() */


