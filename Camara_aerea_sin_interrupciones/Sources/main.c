/* ###################################################################
**     Filename    : main.c
**     Project     : Camara_aerea_interrupciones
**     Processor   : MCF51QE128CLK
**     Version     : Driver 01.00
**     Compiler    : CodeWarrior ColdFireV1 C Compiler
**     Date/Time   : 2018-07-03, 20:38, # CodeGen: 0
**     Abstract    :
**         Main module.
**         This module contains user's application code.
**     Settings    :
**     Contents    :
**         No public methods
**
** ###################################################################*/
/*!
** @file main.c
** @version 01.00
** @brief
**         Main module.
**         This module contains user's application code.
*/         
/*!
**  @addtogroup main_module main module documentation
**  @{
*/         
/* MODULE main */


/* Including needed modules to compile this module/procedure */
#include "Cpu.h"
#include "Events.h"
#include "AS1.h"
#include "PWM1.h"
#include "PWM2.h"
#include "Bit1.h"
#include "Bit2.h"
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Math.h"

int i = 0;
int l = 0;
int r = 0;
int state = 0;
unsigned char err;
unsigned char leftMotor[3];
unsigned char rightMotor[3];
int leftPWM,rightPWM;
/* User includes (#include below this line is not maintained by Processor Expert) */

void main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  /* For example: for(;;) { } */ 
  Bit1_PutVal(0);
  Bit2_PutVal(0);
  PWM1_Disable();
  PWM2_Disable();
  for(;;){
	  switch(state){
	  	  case 0:
	  		  do{
	  			  err = AS1_RecvChar(&character);
	  		  }while(err == ERR_RXEMPTY);
	  		  
	  		  if(character == 'L'){
	  			  state = 1;
	  		  }
	  		  
	  		  else{
	  			  state = 0;
	  		  }
	  		  
	  		  break;
	  		  
	  	  case 1:
	  		  l = 0;
	  		  while(TRUE){
	  			do{
	  				err = AS1_RecvChar(&character);
	  			}while(err == ERR_RXEMPTY);
	  			
	  			if(character == 'R'){
	  				break;
	  			}
	  			leftMotor[l] = character;
	  			l++;
	  		  }
	  		  state = 2;
	  		  break;
	  	  case 2:
	  		  r = 0;
			while(TRUE){
				do{
					err = AS1_RecvChar(&character);
				}while(err == ERR_RXEMPTY);
				
				if(character == 13){
					break;
				}
				
				rightMotor[r] = character;
				r++;
			  }
			  state = 3;
			  break;
	  	  case 3:
	  		  if(leftMotor[0] == 'd'){
	  			PWM2_Disable();
	  		  }
	  		  
	  		  else{
				  leftPWM = asciiConvertion(leftMotor,l);
				  PWM2_Enable();
				  PWM2_SetDutyUS(leftPWM);
			  }
	  		  
	  		  if(rightMotor[0] == 'd'){
	  			  PWM1_Disable();
	  	  	  }
	  		  
	  		  else{
				  rightPWM = asciiConvertion(rightMotor,r);
				  PWM1_Enable();
				  PWM1_SetDutyUS(rightPWM);
	  		  }
	  
	  		  AS1_ClearRxBuf();
	  		  state = 0;
	  		  break;
	  		  
	  } 
	 }
  

  /*** Don't write any code pass this line, or it will be deleted during code generation. ***/
  /*** Processor Expert end of main routine. DON'T MODIFY THIS CODE!!! ***/
  for(;;){}
  /*** Processor Expert end of main routine. DON'T WRITE CODE BELOW!!! ***/
} /*** End of main routine. DO NOT MODIFY THIS TEXT!!! ***/

/* END main */
/*!
** @}
*/
/*
** ###################################################################
**
**     This file was created by Processor Expert 10.3 [05.09]
**     for the Freescale ColdFireV1 series of microcontrollers.
**
** ###################################################################
*/
