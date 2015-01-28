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
#include "task_5sec.h"

// Pumpkin CubeSat Kit headers
#include "csk_io.h"
#include "csk_uart.h"
#include "csk_sd.h"
#include "thin_usr.h"

// Pumpkin Salvo headers
#include "salvo.h"

extern void HeCkSum(char* buffer, int n);
extern char HeTrans255Str(char* inpt);
extern unsigned char i2c_read_ack(void);
extern unsigned char i2c_read_nack(void);
extern char send_i2c_byte(int data);
extern void reset_i2c_bus(void);
extern void i2c_restart(void);
extern void i2c_start(void);
extern void CMDS(char a[], char * saveName);

// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// ---------- NOT USED
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------
// -----------------------------------------


/******************************************************************************
****                                                                       ****
**                                                                           **
task_5sec()

Simple task that runs every 5s.

**                                                                           **
****                                                                       ****
******************************************************************************/
void task_5sec(void) {
//	while (1) {
		// This block will help you positively identify which pins are the ones labeled IO <0-3>
		
		/*char s[300];
		sprintf(s,"%d-%d",8,(PORTG&BIT9)!=0);
		sprintf(s,"%s %d-%d",s,9,(PORTG&BIT8)!=0);
		sprintf(s,"%s %d-%d",s,10,(PORTG&BIT7)!=0);
		sprintf(s,"%s %d-%d",s,11,(PORTG&BIT6)!=0);
		sprintf(s,"%s %d-%d",s,12,(PORTD&BIT15)!=0);
		sprintf(s,"%s %d-%d",s,13,(PORTD&BIT14)!=0);
		sprintf(s,"%s %d-%d",s,16,(PORTD&BIT8)!=0);
		sprintf(s,"%s %d-%d",s,17,(PORTD&BIT3)!=0);
		sprintf(s,"%s %d-%d",s,18,(PORTD&BIT12)!=0);
		sprintf(s,"%s %d-%d",s,19,(PORTD&BIT4)!=0);
		sprintf(s,"%s %d-%d",s,20,(PORTA&BIT2)!=0);
		sprintf(s,"%s %d-%d",s,21,(PORTA&BIT3)!=0);
		sprintf(s,"%s %d-%d",s,22,(PORTE&BIT6)!=0);
		sprintf(s,"%s %d-%d",s,23,(PORTE&BIT7)!=0);
		sprintf(s,"%s %d-%d",s,26,(PORTG&BIT1)!=0);
		sprintf(s,"%s %d-%d",s,27,(PORTG&BIT0)!=0);
		sprintf(s,"%s %d-%d",s,28,(PORTA&BIT14)!=0);
		sprintf(s,"%s %d-%d",s,29,(PORTA&BIT15)!=0);
		sprintf(s,"%s %d-%d",s,30,(PORTE&BIT8)!=0);
		sprintf(s,"%s %d-%d",s,31,(PORTE&BIT9)!=0);
		sprintf(s,"%s %d-%d",s,33,(PORTB&BIT4)!=0);
		sprintf(s,"%s %d-%d",s,34,(PORTB&BIT3)!=0);
		sprintf(s,"%s %d-%d",s,35,(PORTB&BIT2)!=0);
		sprintf(s,"%s %d-%d",s,36,(PORTB&BIT1)!=0);
		sprintf(s,"%s %d-%d",s,37,(PORTB&BIT0)!=0);
		sprintf(s,"%s %d-%d",s,38,(PORTB&BIT6)!=0);
		sprintf(s,"%s %d-%d",s,39,(PORTB&BIT7)!=0);
		sprintf(s,"%s %d-%d",s,40,(PORTB&BIT8)!=0);
		sprintf(s,"%s %d-%d",s,41,(PORTB&BIT9)!=0);
		sprintf(s,"%s %d-%d",s,42,(PORTB&BIT10)!=0);
		sprintf(s,"%s %d-%d",s,43,(PORTB&BIT11)!=0);
		sprintf(s,"%s %d-%d\r\n",s,47,(PORTB&BIT15)!=0);
		csk_uart0_puts(s);
		OS_Delay(1);*/


		/*csk_io8_high();  OS_Delay(30); 
		csk_io8_low();  OS_Delay(30); 
		csk_io9_high();  OS_Delay(30); 
		csk_io9_low();  OS_Delay(30); 
		csk_io10_high();  OS_Delay(30); 
		csk_io10_low();  OS_Delay(30); 
		csk_io11_high();  OS_Delay(30); 
		csk_io11_low();  OS_Delay(30); 

		csk_io12_high();  OS_Delay(30); 
		csk_io12_low();  OS_Delay(30); 
		csk_io13_high();  OS_Delay(30); 
		csk_io13_low();  OS_Delay(30); 

		csk_io16_high();  OS_Delay(30); 
		csk_io16_low();  OS_Delay(30); 

		csk_io18_high();  OS_Delay(30); 
		csk_io18_low();  OS_Delay(30); 
		csk_io19_high();  OS_Delay(30); 
		csk_io19_low();  OS_Delay(30); 

		csk_io20_high();  OS_Delay(30); 
		csk_io20_low();  OS_Delay(30); 
		csk_io21_high();  OS_Delay(30); 
		csk_io21_low();  OS_Delay(30); 
		csk_io22_high();  OS_Delay(30); 
		csk_io22_low();  OS_Delay(30); 
		csk_io23_high();  OS_Delay(30); 
		csk_io23_low();  OS_Delay(30); 

		csk_io26_high();  OS_Delay(30); 
		csk_io26_low();  OS_Delay(30); 
		csk_io27_high();  OS_Delay(30); 
		csk_io27_low();  OS_Delay(30); 

		csk_io28_high();  OS_Delay(30); 
		csk_io28_low();  OS_Delay(30); 
		csk_io29_high();  OS_Delay(30); 
		csk_io29_low();  OS_Delay(30); 
		csk_io30_high();  OS_Delay(30); 
		csk_io30_low();  OS_Delay(30); 
		csk_io31_high();  OS_Delay(30); 
		csk_io31_low();  OS_Delay(30); 

		csk_io33_high();  OS_Delay(30); 
		csk_io33_low();  OS_Delay(30); 
		csk_io34_high();  OS_Delay(30); 
		csk_io34_low();  OS_Delay(30); 
		csk_io35_high();  OS_Delay(30); 
		csk_io35_low();  OS_Delay(30); 

		csk_io36_high();  OS_Delay(30); 
		csk_io36_low();  OS_Delay(30); 
		csk_io37_high();  OS_Delay(30); 
		csk_io37_low();  OS_Delay(30); 
		csk_io38_high();  OS_Delay(30); 
		csk_io38_low();  OS_Delay(30); 
		csk_io39_high();  OS_Delay(30); 
		csk_io39_low();  OS_Delay(30); 

		csk_io40_high();  OS_Delay(30); 
		csk_io40_low();  OS_Delay(30); 
		csk_io41_high();  OS_Delay(30); 
		csk_io41_low();  OS_Delay(30); 
		csk_io42_high();  OS_Delay(30); 
		csk_io42_low();  OS_Delay(30); 
		csk_io43_high();  OS_Delay(30); 
		csk_io43_low();  OS_Delay(30); 

		csk_io46_high();  OS_Delay(30); 
		csk_io46_low();  OS_Delay(30); 
		csk_io47_high();  OS_Delay(30); 
		csk_io47_low();  OS_Delay(30); */

	//}
  while (1) {
    OS_Delay(250);
	char outpt[100];
	strcpy(outpt,"\r\ntask_2.5sec\t\tRan again");
	timeStamp(outpt);
	int tmp=0;
	if(OSReadBinSem(BINSEM_DEPLOYED_P)) tmp=1;
	if(OSReadBinSem(BINSEM_EJECTED_P)) tmp=tmp+2;
	sprintf(outpt,"%s State-%d",outpt,tmp);
	strcat(outpt,"\r\n");
	csk_uart0_puts(outpt); //Debug

  } /* while */
} /* task_5sec() */
