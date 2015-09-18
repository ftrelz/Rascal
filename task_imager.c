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

// the two arrays that are constant and will be initialized once below
// TODO: calcualte initial values and add them in main function
static float f;
static float s[4][4];
static float A[4];

/*
// creates structs to hold x and y coords of each resistor in image frame
// (two structs will be used: one to hold x coords and one for y coords)
typedef struct {
  float r1;
  float r2;
  float r3;
  float r4;
} resistor_coords;
*/

void dc2q(float Cic[][3], float return_matrix[4])
{
  // Create q matrix based off Cic values
  // static float q[4];
  return_matrix[0] = 0.5 * sqrtf(1 + Cic[0][0] - Cic[1][1] - Cic[2][2]);
  return_matrix[1] = 0.5 * sqrtf(1 - Cic[0][0] + Cic[1][1] - Cic[2][2]);
  return_matrix[2] = 0.5 * sqrtf(1 - Cic[0][0] - Cic[1][1] + Cic[2][2]);
  return_matrix[3] = 0.5 * sqrtf(1 + Cic[0][0] + Cic[1][1] + Cic[2][2]);

  static float max_q_value;
  static int max_q_value_index;

  // Find max value in q vector and store it along with its index
  if(return_matrix[1] >= return_matrix[0])
  {
    max_q_value = return_matrix[1];
    max_q_value_index = 1;
  }
  else if(return_matrix[2] > max_q_value)
  {
    max_q_value = return_matrix[2];
    max_q_value_index = 2;
  }
  else if(return_matrix[3] > max_q_value)
  {
    max_q_value = return_matrix[3];
    max_q_value_index = 3;
  }
  else if(return_matrix[0] >= max_q_value)
  {
    max_q_value = return_matrix[0];
    max_q_value_index = 0;
  }

  // Now assign other three parameters based off the earlier max
  // Calculations translated from dc2q in updated nav sims matlab files
  if(max_q_value_index == 3)
  {
    return_matrix[0] = ((Cic[1][2] - Cic[2][1]) / 4.0) / return_matrix[3];
    return_matrix[1] = ((Cic[2][0] - Cic[0][2]) / 4.0) / return_matrix[3];
    return_matrix[2] = ((Cic[0][1] - Cic[1][0]) / 4.0) / return_matrix[3];
  }
  else if(max_q_value_index == 0)
  {
    return_matrix[1] = ((Cic[0][0] + Cic[1][0]) / 4.0) / return_matrix[0];
    return_matrix[2] = ((Cic[0][2] + Cic[2][0]) / 4.0) / return_matrix[0];
    return_matrix[3] = ((Cic[1][2] - Cic[2][1]) / 4.0) / return_matrix[0];
  }
  else if(max_q_value_index == 1)
  {
    return_matrix[2] = ((Cic[1][2] + Cic[2][1]) / 4.0) / return_matrix[1];
    return_matrix[0] = ((Cic[1][0] + Cic[0][1]) / 4.0) / return_matrix[1];
    return_matrix[3] = ((Cic[2][0] - Cic[0][2]) / 4.0) / return_matrix[1];
  }
  else if(max_q_value_index == 2)
  {
    return_matrix[0] = ((Cic[2][0] + Cic[0][2]) / 4.0) / return_matrix[2];
    return_matrix[1] = ((Cic[2][1] + Cic[1][2]) / 4.0) / return_matrix[2];
    return_matrix[3] = ((Cic[0][1] - Cic[1][0]) / 4.0) / return_matrix[2];
  }
}

// these arrays will hold x and y coords of each resistor in image frame
static float x[4];
static float y[4];

// this array holds resistor coordinates on the pcb in the image frame
static float pAll_i [3][4];

// tetraNav takes resistor location data (resistor_coords structs) from thermal image and updates POSE_IMG (does not return anything as the matlab code does)
// with values called camera_i, camera_quaternion, and camera_angles
void tetraNav(float pAll_i[][4])
{
  /*
    Step 0: build s-matrix
    Step 1: find A
    Note: previous two step's calculations will be done by hand and then the values will be hard coded in as they stay constant no matter what
    
    Step 2: find B
    Step 3: find C
    Step 4: find H
    Step 5: NOT NEEDED
    Note: value determined from step 5 ('f' in matlab code) is the focal length of the camera which is a known constant therefore all code
    from step 5 is superfluous, the same can be said for the value cotf

    Step 6: Compute the length of the tetrahedral sides in the F-frame (gives four element array 'F')
    Step 7: Compute the average length of one side in the C-frame (camera frame)
    Note: step 7 computes two values: dSum and dCount which then provide our first value for array d, step 8 then computes the rest of d's values

    Step 8: Compute the other tetrahedral sides which are proportional to the first value of the d array
    Step 9: Calculate pAll_c using 'd', 'x', 'y', and f (position of resistors on pcb from reference of camera frame)
    Step 10: Build pAll_zero (conversion matrix made with pAll_c values)
    Step 11: Compute rotation matrix (Cic, comes from pAll_zero and pAll_i)
    Step 12: find camera_angles (computed using calues from Cic) (and update POSE_IMG?)
    Step 13: find camera_i (also comes from Cic) (and update POSE_IMG?)
    Step 14: find camera_quaternion (dc2q(Cic)) (and update POSE_IMG?)
    Step 15: ???
    Step 16: Profit!!!
    ( •_•)
    ( •_•)>¬¦-¦
    (¬¦_¦)
  */

  static float B[4];
  static float C[4][4];
  static float Hsq[4][4];
  static float d[4];
  static float dCount = 0;
  
  // array used to hold the length of each tetrahedral side in the 'F' frame
  static float F[4];
  
  // these arrays hold resistor coordinates on the pcb in their respective frames
  static float pAll_c [3][4];	// camera frame
  static float pAll_zero [3][4];	// conversion matrix

  static int i, j, dSum = 0;

  /************** STEP 2 **************/
  B[0] = x[0] * (y[2] - y[1]) + y[0] * (x[1] - x[2]) + y[1] * x[2] - x[1] * y[2];
  B[1] = x[0] * (y[3] - y[1]) + y[0] * (x[1] - x[3]) + y[1] * x[3] - x[1] * y[3];
  B[2] = x[0] * (y[3] - y[2]) + y[0] * (x[2] - x[3]) + y[2] * x[3] - x[2] * y[3];
  B[3] = x[1] * (y[3] - y[2]) + y[1] * (x[2] - x[3]) + y[2] * x[3] - x[2] * y[3];

  /************** STEP 3 **************/
  C[0][1] = (B[2]/A[2]) * (A[3]/B[3]);
  C[0][2] = (B[1]/A[1]) * (A[3]/B[3]);
  C[0][3] = (B[0]/A[0]) * (A[3]/B[3]);
  C[1][2] = (B[1]/A[1]) * (A[2]/B[2]);
  C[1][3] = (B[0]/A[0]) * (A[2]/B[2]);
  C[2][3] = (B[0]/A[0]) * (A[1]/B[1]);
  for (i = 0; i < 4 ; i++)
  {
    for (j = (i + 1); j < 4 ; j++)
    {
      C[j][i] = 1.0 / C[i][j];
    }  
  }

  /************** STEP 4 **************/
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 4; j++)
    {
      Hsq[i][j] = powf((x[i] - C[i][j] * x[j]), 2.00) + powf((y[i] - C[i][j] * y[j]), 2.00);
    }  
  }

  /************** STEP 5 **************/
    /*
      NOT NEEDED
    */

  /************** STEP 6 **************/
  for (i = 0; i < 4; i++)
  {
    F[i] = sqrtf(powf(x[i], 2.0) + powf(y[i], 2.0) + powf(f, 2.0));
  }

  /************** STEP 7 **************/
  for (i = 1; i < 4; i++)
  {
    dSum = dSum + ((s[0][i] * F[0])/(sqrtf(Hsq[0][i] + (powf(f, 2.0) * powf(1-C[0][i], 2.0)))));
    dCount++;
  }
  d[0] = dSum/dCount;

  /************** STEP 8 **************/
  for (i = 1; i < 4; i++)
  {
    d[i] = ((C[0][i] * F[i]) / F[0]) * d[0];
  }

  /************** STEP 9 **************/
  /*Matlab statement equates to:
    
               d(0) * x(0) / F(0)  d(1) * x(1) / F(1)  d(2) * x(2) / F(2)  d(3) * x(3) / F(3)
    pAll_c = [ d(0) * y(0) / F(0)  d(1) * y(1) / F(1)  d(2) * y(2) / F(2)  d(3) * y(3) / F(3) ]
               d(0) * f / F(0)     d(1) * f / F(1)     d(2) * f / F(2)     d(3) * f / F(3)
    
  */
  for (i = 0; i < 4; i++)
  {
    pAll_c[0][i] = d[i] * x[i] / F[i];
    pAll_c[1][i] = d[i] * y[i] / F[i];
    pAll_c[2][i] = d[i] * f / F[i];
  }

  /************** STEP 10 **************/
  /*Matlab statement equates to:
    
                  pAll_c[0][0]-pAll_c[0][0]  pAll_c[0][1]-pAll_c[0][0]  pAll_c[0][2]-pAll_c[0][0]  pAll_c[0][3]-pAll_c[0][0]
    pAll_zero = [ pAll_c[1][0]-pAll_c[1][0]  pAll_c[1][1]-pAll_c[1][0]  pAll_c[1][2]-pAll_c[1][0]  pAll_c[1][3]-pAll_c[1][0] ]
                  pAll_c[2][0]-pAll_c[2][0]  pAll_c[2][1]-pAll_c[2][0]  pAll_c[2][2]-pAll_c[2][0]  pAll_c[2][3]-pAll_c[2][0]
    
  */
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 4; j++)
    {
      pAll_zero[i][j] = pAll_c[i][j] - pAll_c[i][0];
    }
  }

  /************** STEP 11 **************/
  /*
    Matlab calculation: Cic = -pAll_zero * pAll_i(1:2,:)' * inv(pAll_i(1:2,:) * pAll_i(1:2,:)')

    This calculation is broken down into two parts, hence the array names part1 and part2.
    part1 is the result of the equation -pAll_zero * pAll_i(1:2,:)' which is the first half of the calculation for Cic
    part2 is the result of the equation inv(pAll_i(1:2,:) * pAll_i(1:2,:)') which is the second half of the equation
  */

  // -pAll_zero
  static float negpAll_zero[3][4];
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 4; j++)
    {
      negpAll_zero[i][j] = -pAll_zero[i][j];
    }
  }

  // pAll_i(1:2,:)'
  static float invpAll_i12[4][2];
  for (i = 0; i < 4; i++)
  {
    for (j = 0; j < 2; j++)
    {
      invpAll_i12[i][j] = pAll_i[j][i];
    }
  }

  // -pAll_zero * pAll_i(1:2,:)'
  static float part1[3][2];
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 2; j++)
    {
      part1[i][j] = (negpAll_zero[i][0] * invpAll_i12[0][j]) + (negpAll_zero[i][1] * invpAll_i12[1][j])
                     + (negpAll_zero[i][2] * invpAll_i12[2][j]) + (negpAll_zero[i][3] * invpAll_i12[3][j]);
    }
  }

  // pAll_i(1:2,:) * pAll_i(1:2,:)'
  static float A[2][2];
  A[0][0] = (pAll_i[0][0] * invpAll_i12[0][0]) + (pAll_i[0][1] * invpAll_i12[0][1]) + (pAll_i[0][2] * invpAll_i12[0][2]) + (pAll_i[0][3] * invpAll_i12[0][3]);
  A[0][1] = (pAll_i[0][0] * invpAll_i12[0][1]) + (pAll_i[0][1] * invpAll_i12[1][1]) + (pAll_i[0][2] * invpAll_i12[1][2]) + (pAll_i[0][3] * invpAll_i12[1][3]);
  A[1][0] = (pAll_i[0][0] * invpAll_i12[0][1]) + (pAll_i[0][1] * invpAll_i12[1][1]) + (pAll_i[0][2] * invpAll_i12[1][2]) + (pAll_i[0][3] * invpAll_i12[1][3]);
  A[1][1] = (pAll_i[1][0] * invpAll_i12[1][0]) + (pAll_i[1][1] * invpAll_i12[1][1]) + (pAll_i[1][2] * invpAll_i12[1][2]) + (pAll_i[1][3] * invpAll_i12[1][3]);

  // inverse matrix of A is the result of part 2 of the calculation: (Ainv = inv(pAll_i(1:2,:) * pAll_i(1:2,:)'))
  static float part2[2][2];
  static float A_determinant;
  A_determinant = 1.0 / ((A[0][0] * A[1][1]) - (A[0][1] * A[1][0]));

  // These calculations equate to A^(-1) = (1/det(A)) * [d, -b;-c, a]
  // with A = [a, b;c, d]
  part2[0][0] = A_determinant * A[1][1];
  part2[0][1] = A_determinant * (-A[0][1]);
  part2[1][0] = A_determinant * (-A[1][0]);
  part2[1][1] = A_determinant * A[0][0];

  static float Cic[3][3];
  for (i = 0; i < 3; i++)
  {
    for (j = 0; j < 2; j++)
    {
      Cic[i][j] = (part1[i][0] * part2[0][j]) + (part1[i][1] * part2[1][j]);
    }
  }

  // Next, we find the cross product of the first two columns of Cic and put the resulting vector in the third column of Cic
  /*
    
                          |     i           j           k       |
    Cic(:,1) X Cic(:,2) = |  Cic[0][0]   Cic[1][0]   Cic[2][0]  |
                          |  Cic[0][1]   Cic[1][1]   Cic[2][1]  |


                 |  Cic[1][0]   Cic[2][0]  |
    Cic[0][2] =  |                         |
                 |  Cic[1][1]   Cic[2][1]  |


                 |  Cic[0][0]   Cic[2][0]  |
    Cic[1][2] =  |                         |
                 |  Cic[0][1]   Cic[2][1]  |


                 |  Cic[0][0]   Cic[1][0]  |
    Cic[2][2] =  |                         |
                 |  Cic[0][1]   Cic[1][1]  |

  */
  Cic[0][2] = ((Cic[1][0] * Cic[2][1]) - (Cic[2][0] * Cic[1][1]));
  Cic[1][2] = ((Cic[2][0] * Cic[0][1]) - (Cic[0][0] * Cic[2][1]));
  Cic[2][2] = ((Cic[0][0] * Cic[1][1]) - (Cic[1][0] * Cic[0][1]));



  // unit Function: treats 3x3 Cic array as 3 separate column vectors and changes Cic to hold unit vectors of original column vectors
  float magnitude[3];

  // Find each vector's magnitude
  for (i = 0; i < 3; i++)
  {
      magnitude[i] = sqrtf(powf(Cic[0][i], 2.00) + powf(Cic[1][i], 2.00) + powf(Cic[2][i], 2.00));
  }

  // Now calculate unit vectors
  for (i = 0; i < 3; i++)
  {
    if ((int)magnitude[i] != 0)
    {
      Cic[0][i] = Cic[0][i] / magnitude[i];
      Cic[1][i] = Cic[1][i] / magnitude[i];
      Cic[2][i] = Cic[2][i] / magnitude[i];
    }
  }

  /************** STEP 12 **************/
  /*
    For testing purposes only, not needed in actual implementation.
    Matlab code:  camera_angles(1) = atan(Cic(2,3)/Cic(3,3));
                  camera_angles(2) = -asin(Cic(1,3));
                  camera_angles(1) = atan(Cic(1,2)/Cic(1,1));
    Possibly POSE_IMG values?
  */

  //static float camera_angles[3];
  //camera_angles[0] = atanf(Cic[1][2] / Cic[2][2]);
  //camera_angles[1] = -asinf(Cic[0][2]);
  //camera_angles[2] = atanf(Cic[0][1] / Cic[0][0]);

  /************** STEP 13 **************/
  /*
    This calculation flips Cic about its diagonal and then multiplies the flipped matrix by the first column of pAll_c (pAll_c(:,1))
  */
  static float camera_i[3];
  for (i = 0; i < 3; i++)
  {
    camera_i[i] = (Cic[0][i] * pAll_c[0][0]) + (Cic[1][i] * pAll_c[1][0]) + (Cic[2][i] * pAll_c[2][0]);
  }

  /************** STEP 14 **************/
  static float camera_quaternion[4];
  
  // This function call assigns results of dc2q to camera_quaternion
  dc2q(Cic, camera_quaternion);
}

void task_imager(void) {
  
  csk_io40_high();
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
  
  //static char image[500];
  static unsigned int a[512];
  static unsigned long int i=0;
  static unsigned int j=0;
  

  static char* imagerCMD; //static char to receive "message" from external_cmds (imager cmd)
  F_FILE* FULL_IMAGE_SDSave;
 
  while(1) {
    OS_Delay(250);
    

     //static const char test[2]={'A','B'};
   // static const char A = 'A';
   // static const char B = 'B';
    //static const char* AB = "AB";
    //csk_uart0_puts(test);
    //csk_uart0_puts("\r\n");
    //csk_uart2_puts(AB);

     if(OSReadMsg(MSG_IMAGER_P)) {
	   imagerCMD=((char*)(OSTryMsg(MSG_IMAGER_P)));
	   static char tmp[150];
       sprintf(tmp, "I'm in task_imager!\r\n");
       csk_uart0_puts(tmp);
	        
	   // Start message verification (for sanity (AND pointer) check)
	  if (imagerCMD[12]=='I' && imagerCMD[13]=='M' && imagerCMD[14]=='A' && imagerCMD[15] == 'G' && imagerCMD[16] == 'E' && imagerCMD[17] == 'R') { //if a (beings with IMAGER!!!) -- sanity check
        if (imagerCMD[18]=='_' && imagerCMD[19]=='F' && imagerCMD[20]=='U' && imagerCMD[21]=='L' && imagerCMD[22]=='L') { //if a has the flag FULL
           sprintf(tmp, "I'm in FULL!\r\n");
           csk_uart0_puts(tmp);
           //sprintf(tmp, "%c", 0xAB);
           //tmp[0] = 0xAB;
		   //tmp[1] = 0;
           //while(1){ csk_uart2_putchar(0xAB);  }
           //csk_uart2_puts(tmp);
           csk_uart2_putchar(0xAB); 
           csk_uart0_putchar(0xAB);
        }
        else if (imagerCMD[18]=='_' && imagerCMD[19]=='P' && imagerCMD[20]=='A' && imagerCMD[21]=='R' && imagerCMD[22]=='T' && imagerCMD[23]=='I' && imagerCMD[24]=='A' && imagerCMD[25]=='L' ){//if a has the flag PARTIAL
          csk_uart2_putchar(0xCD);
          sprintf(tmp, "I'm in PARTIAL!\r\n");
          csk_uart0_puts(tmp);
        } else if (imagerCMD[18] == '_' && imagerCMD[19] == 'T' && imagerCMD[20] == 'E' && imagerCMD[21] == 'S' && imagerCMD[22] == 'T') {
          sprintf(tmp, "I'm in TEST!");
          csk_uart0_puts(tmp);

          //sprintf(tmp, "%c%c%c%c%c%c%c%c%c%c", 0x6e, 0x00, 0x00, 0x0b, 0x00, 0x00, 0x2f, 0x4a, 0x00, 0x00);
		  Nop();Nop();Nop();

/*
		  csk_uart2_putchar(0x6e);//6e
		  csk_uart2_putchar(0x00);//00
		  csk_uart2_putchar(0x00);//00
		  csk_uart2_putchar(0x0b);//0b
		  csk_uart2_putchar(0x00);//00
		  csk_uart2_putchar(0x00);//00
		  csk_uart2_putchar(0x2f);//2f
		  csk_uart2_putchar(0x4a);//4a
		  csk_uart2_putchar(0x00);//00
		  csk_uart2_putchar(0x00);//00
*/

          csk_uart2_puts("test");
          Nop();Nop();Nop();

         /* i = 0;
          while(csk_uart2_count()) {
	        tmp[i] = csk_uart2_getchar();
            i++;
          }
          Nop();Nop();Nop();
          csk_uart0_puts(tmp);
          Nop();Nop();Nop();*/
        }
       
//    csk_uart0_puts(csk_uart2_getchar());

      int waitTmp=1; //ENTER: Will wait until something is received, and store it in a.
      while(waitTmp) {
		  OS_Delay(20);
          waitTmp=1;
		  //strcpy(a,"");
          static int rx_count_flag=1;
          int rx_count;
		  i=0;
          for (i = 0; i < 1000; i++)
          {
            Nop();
          } 
		while(csk_uart2_count() && i<162) {
        //while(csk_uart2_count()  && i<300000) {
          if(rx_count_flag) {
            //sprintf(tmp, "UART COUNT: %d\r\n", csk_uart2_count());
            //csk_uart0_puts(tmp);
            rx_count_flag=0;
            //FULL_IMAGE_SDSave = f_open("FULLIMG", "w");
          }

          //csk_uart0_puts("I'm in uart2_count!");
		  //sprintf(a,"%s%c",a,csk_uart2_getchar());
          //sprintf(a, csk_uart2_getchar());
          //csk_uart0_putchar(a));
          j = 0;
          while(csk_uart2_count() && j < 512) {
		    a[j]=csk_uart2_getchar();
          //j = csk_uart2_getchar();
            char temp[150];
            sprintf(temp, "%x ", a);
            csk_uart0_puts(temp);
            //csk_uart0_putchar(a[i]);
            j++;
          }

          //f_write(a, 1, sizeof(a), FULL_IMAGE_SDSave);
          //f_write(j, 1, sizeof(j), FULL_IMAGE_SDSave);
          
          i++;
             //sprintf(temp, "%d\n", i);
          //csk_uart0_puts(temp);
          //sprintf(tmp, "i= %d", i);
          //csk_uart0_puts(tmp);
          //OS_Delay(20);
         unsigned long int s;
         //s = csk_uart2_count();
         //f_seek(FULL_IMAGE_SDSave, 0, F_SEEK_END);
         s = f_tell(FULL_IMAGE_SDSave);
         sprintf(tmp, "%lu\n", s);
         csk_uart0_puts(tmp);

	     } // end while(csk_uart2_count() && i<162)

         waitTmp = 0;
         f_close(FULL_IMAGE_SDSave);
       }
       csk_uart0_puts("Im out of while(waitTmp)!!");
       //int s;
       //s = f_filelength("IMAGEF");
       //sprintf(tmp, "%ul", s);
       //csk_uart0_puts(tmp);
      //sprintf(tmp, "size of a[512]: %d", sizeof(a));
      //csk_uart0_puts(tmp);
      //sprintf(image, a); 
      //csk_uart0_puts(image);
        //i=0;
        //while(a) {
        //  csk_uart0_putchar(a[i]);
        //  i++;
        //}


      }// end message verification
    }// end osread_msg
  } // end while(1)
} // end task_imager(void)
