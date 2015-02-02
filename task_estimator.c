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
#include <math.h>

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
pose POSE_EST_PREV;

/*
float estimator(pose POSE_EST_PREVIOUS, thrusterChoice, pose POSE_IMG, pose POSE_BOEING) {
  //TBD
}
*/
  static int rowsL = 6;
  static int colsL = 3;
  static float L[6][3] = { {0.0700, 0.0001, 0.0000}, 
                         {-0.0001, 0.0699, 0.0000},
                         {0.0000, 0.0000, 0.0531},
                         {0.0024, 0.0001, 0.0000},
                         {-0.0001, 0.0024, 0.0000},
                         {0.0000, 0.0000, 0.0014}};

  static int rowsAd = 6;
  static int colsAd = 6;
  static float Ad[6][6] = { {1.0000, 0.0000, 0.0000, 1.0000, 0.0012, 0.0000}, 
                         {0.0000, 1.0000, 0.0000, -0.0012, 1.0000, 0.0000},
                         {0.0000, 0.0000, 1.0000, 0.0000, 0.0000, 1.0000},
                         {0.0000, 0.0000, 0.0000, 1.0000, 0.0023, 0.0000},
                         {0.0000, 0.0000, 0.0000, -0.0023, 1.0000, 0.0000},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 1.0000}};

  static int rowsBd = 6;
  static int colsBd = 11;
  static float Bd[6][11] = { {0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0000, 0.0000, 0.0005, 0.0005, -0.0005, -0.0005}, 
                         {0.0000, 0.0000, 0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0005, -0.0005, 0.0005, -0.0005},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0005, -0.0005, 0.0000, 0.0000, 0.0000, 0.0000},
                         {0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, 0.0010, -0.0010, -0.0010},
                         {0.0000, 0.0000, 0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0010, -0.0010, 0.0010, -0.0010},
                         {0.0000, 0.0000, 0.0000, 0.0000, 0.0000, 0.0010, -0.0010, 0.0000, 0.0000, 0.0000, 0.0000}};

  static int rowsCd = 3;
  static int colsCd = 6;
  static float Cd[3][6] = {{1, 0, 0, 0, 0, 0},
                         {0, 1, 0, 0, 0, 0},
                         {0, 0, 1, 0, 0, 0}};

  static int rows_s_hat_prev = 6;
  static int cols_s_hat_prev = 1;
  static float s_hat[6] = {0, 0, 0, 0, 0, 0};  //also POSE_EST
  static float s_hat_previous[6] = {0, 0, 0, 0, 0, 0};
  
  // holds q values of POSE_EST for purposes of matrix multiplication
  static float POSE_EST_q_values[4] = {POSE_EST.q1, POSE_EST.q2, POSE_EST.q3, POSE_EST.q4};
  

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

void matrix_mul_Bd_Ixthrusterchoice(float matrix1[][11], int rows1, int col1, float matrix2[][1], int rows2, int col2, float multiply[][1])
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

/* result of: {Ad - (L * Cd) * s_hat_prev] - it's multiplying first part (6x6 matrix)
   with s_hat_prev (6 element array)
   modifies this 6 element array: Ad_LxCdxs_hat_prev 
   This is not a direct matrix to matrix multiplication because of second array being 6 elements */
void matrix_mul_s_hat_prev(float matrix1[][6], float matrix2[], float multiply[])
{
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

// to be completed, Lxy not determined
void matrix_mul_Lxy(float matrix1[][3], int rows1, int col1, float matrix2[][1], int rows2, int col2, float multiply[][1])
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
     [s_hat = (Ad - L*Cd)*s_hat_prev + Bd_I*thrusterChoice + L*y]
     inputs are: (6x1), (6x1), & (6x1) matrices, actually 6 element arrays
     output is : (6x1) matrix -- actually 6 element array
*/
void matrix_add_s_hat(float matrix1[6], float matrix2[6], float matrix3[6], float answer[6])
{
  int i;
  for(i = 0; i < 6; i++)
  {
    //new s_hat (6x1) 6 element array  
    answer[i] = matrix1[i] + matrix2[i] + matrix3[i];
  }
}

void task_estimator(void) {
  
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
  POSE_EST.x=0.0;
  POSE_EST.y=0.0;
  POSE_EST.z=0.0;
  POSE_EST.xdot = 0.0;
  POSE_EST.ydot = 0.0;
  POSE_EST.zdot = 0.0;
  
  // defines static vars used for linear orbit matrices
  static float n, dt;
  n = 2*PI/95/60;
  dt = 0.1;
  static float xi, xdoti, yi, ydoti, zi, zdoti, xi1, xdoti1, yi1, ydoti1, zi1, zdoti1;
  xi = 0.0;
  yi = 0.0;
  zi = 0.0;

  static float LxCd[6][6], Ad_LxCd[6][6], Ad_LxCdxs_hat_prev[6], Bd_Ixthrusterchoice[6], Lxy[6], s_hat_prev[6];

  matrix_mul_LxCd(L, rowsL, colsL, Cd, rowsCd, colsCd, LxCd);
  matrix_sub(Ad, LxCd, Ad_LxCd);
  matrix_mul_s_hat_prev(Ad_LxCd, s_hat_prev, Ad_LxCdxs_hat_prev);
  
  //to be completed: need thrusterchoice, L, and y
  matrix_add_s_hat(Ad_LxCdxs_hat_prev, Bd_Ixthrusterchoice, Lxy, s_hat);

  while(1) {
    // POSE_EST test
    //OS_Delay(100);
    //POSE_EST.x++;
    //POSE_BOEING.x++;
    
    OS_Delay(50);
    
    // inits s_hat_prev with values for testing
    int i;
    for (i=0; i < 6; i++){
      s_hat_prev[i] = 1.0;
    }
    
    matrix_mul_LxCd(L, rowsL, colsL, Cd, rowsCd, colsCd, LxCd);
    matrix_sub(Ad, LxCd, Ad_LxCd);
    matrix_mul_s_hat_prev(Ad_LxCd, s_hat_prev, Ad_LxCdxs_hat_prev);

    /** Tests for POSE**/
/*
    char str[100];
    csk_uart0_puts("\r\nPOSE_BOEING from EST:\r\n");
    //sprintf(str, "%d,%d,%d,%d,%d,%d,%d\r\n", POSE_EST.x++, POSE_EST.y++, POSE_EST.z++,POSE_EST.q1++,POSE_EST.q2++,POSE_EST.q3++,POSE_EST.q4++);
    sprintf(str, "%f,%f,%f,%f,%f,%f,%f\r\n", POSE_BOEING.x++, POSE_BOEING.y++, POSE_BOEING.z++,POSE_BOEING.q1++,POSE_BOEING.q2++,POSE_BOEING.q3++,POSE_BOEING.q4++);
    //sprintf(str, "%d   %d", POSE_EST.x,POSE_BOEING.x);
    csk_uart0_puts(str);
    csk_uart0_puts("END POSE_EST\r\n");
    //csk_uart0_puts(STR_CRLF "POSE_BOEING.x: ");
    //csk_uart0_puts((char *)POSE_EST.x);
*/    
    /**END Tests for POSE **/

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

	// Ask (?) Boeing for quaternion (need to figure out how this is implemented)

	// Propagate position and velocity states 


	// Listen for BINSEM (?) of image updates  -talk with Nick/Bob/Swartwout about this
	
	// And fix the estimation accordingly (MAS will help)

	// Make sure to have velocity and position in BODY coordinates
	// available to other tasks

  } /* while */
} /* task_estimator() */


