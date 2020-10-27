/* ###################################################################
**     Filename    : main.c
**     Project     : Comunicacion_Serial_Camara
**     Processor   : MCF51QE128CLK
**     Version     : Driver 01.00
**     Compiler    : CodeWarrior ColdFireV1 C Compiler
**     Date/Time   : 2018-05-09, 14:26, # CodeGen: 0
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
#include "AS2.h"
#include "AD1.h"
#include "TI1.h"
#include "Bit1.h"
#include "Bit2.h"
#include "PWM1.h"
#include "PWM2.h"
/* Include shared modules, which are used for whole project */
#include "PE_Types.h"
#include "PE_Error.h"
#include "PE_Const.h"
#include "IO_Map.h"
#include "Math.h"

#define resetCam 0
#define sendCam 1
#define receiveCam 2
#define convertion 3
#define sharp 4
#define pwm 5

//float kp = 0.834;
int mref = 40;
int velocidad = 5; //6
int velocidad_pwm;
int state = resetCam;
int i,a,f,k,error;
int j;
int n;
int w;
int vuelta;
int mx_anterior;
int pwmf = 1;
//float velocidad_pwm;
unsigned char arrayConvertion[3];
int mx, my;
unsigned short ADC1;
unsigned int d;

unsigned char bsr[1] = {13};
unsigned char meanColor[4] = {'G','M',13};
unsigned char centroid[6] = {'M','M',' ','0',13};
unsigned char trackColor[21] = {'T','C',' ','9','0',' ','1','8','2',' ','9',' ','5','9',' ','1','3',' ','5','1',13};
//unsigned char trackColor[23] = {'T','C',' ','1','2','0',' ','1','8','0',' ','2','0',' ','6','0',' ','1','5',' ','2','5',13};
//unsigned char trackColor[22] = {'T','C',' ','1','3','0',' ','1','8','2',' ','9',' ','5','9',' ','1','3',' ','5','1',13};
//unsigned char trackColor[21] = {'T','C',' ','3','8',' ','1','8','2',' ','9',' ','5','9',' ','1','3',' ','5','1',13};
unsigned char trackWindow[4] = {'T','W',13};
unsigned char capture[3] = {'D','F',13};
unsigned char reset[4] = {13,'R','S',13};
unsigned char LedOn[5] = {'L','1',' ', '1',13};
unsigned char LedOff[5] = {'L','1',' ', '0',13};
unsigned char command[1] = {13};
unsigned char caracter;
unsigned char array[500];
unsigned char err;


void sendWord(unsigned char sendArray[],int size,int f){
	if(f==0){
		for(w=0; w<size; w++)
	   {
			do{
				err = AS2_SendChar(sendArray[w]);
			}while(err != ERR_OK);
	   }
	}
	
	if(f==1){
		for(w=0; w<size; w++)
	   {
			do{
				err = AS1_SendChar(sendArray[w]);
			}while(err != ERR_OK);
	   }
	}
}

int asciiConvertion(unsigned char arrayConvertion[], int size){
	int convertionNumber = 0;
	for(n=0;n<size;n++){
		convertionNumber += (arrayConvertion[n]-48)*pow(10,size-n-1);
	}
	return convertionNumber;
}

move(int d, int mx)
{

	if(pwmf == 1){
		PWM1_Enable();
		PWM2_Enable();
		Bit1_PutVal(0);
		Bit2_PutVal(0);
		PWM1_SetDutyUS(500);
		PWM2_SetDutyUS(500);
		pwmf = 0;
	}

	
	if(d<40 && d>14)	
	{
	
		PWM1_Enable();
		PWM2_Enable();
		Bit1_PutVal(0);
		Bit2_PutVal(0);
		PWM1_SetDutyUS(500);
		
		//Si mx es mayor que 40 se gira a la izquierda
		if (mx>40) // PWM2 debe ir mas lento => Debe ser mayor el valor en el PWM
		{
			error = mx - mref ;
			velocidad_pwm = error*velocidad;
			velocidad_pwm= 500-velocidad_pwm;
			PWM2_SetDutyUS(velocidad_pwm);
			mx_anterior = mx;
		}
		else
		{	
			//Si mx es menor que 40 se gira a la derecha 
			if(mx>0 && mx<=40) //PWM2 debe ir mas rapido => 
			{
			error = mref - mx;
			velocidad_pwm = error*velocidad;
			velocidad_pwm= 500-velocidad_pwm;
			PWM2_SetDutyUS(velocidad_pwm);
			mx_anterior=mx;
			}
			if(mx==0)
			{
					//PWM2_SetDutyUS(600);
					//PWM1_Disable();
				if(mx_anterior>0 && mx_anterior<=40)
				{
					PWM2_SetDutyUS(600);
					PWM1_Disable();
				}
				else if(mx_anterior>40)
				{
					PWM1_SetDutyUS(600);
					PWM2_Disable();
				}
			}			
		}
	}
	
	else 
	{
		if(d>8 && d<14)
	{
		PWM1_Disable();
		PWM2_Disable();
	}
		if(d<8){
			PWM1_Enable();
			PWM2_Enable();
			Bit1_PutVal(1);
			Bit2_PutVal(1);
			PWM1_SetDutyUS(600);
			PWM2_SetDutyUS(600);
		}
}

/*
	
	if(d<40)
	{
		if(d>8 && d<12)
		{
			PWM1_Disable();
			PWM2_Disable();
		}
		else
		{
			if (d<8)
			{
				Bit1_PutVal(1);
				Bit2_PutVal(1);
				PWM1_SetDutyUS(500);
				PWM2_SetDutyUS(400);
			}
			
			else
			{
				if(mx >= 0 || mx<10){
					/*PWM1_Disable();
					PWM2_Disable();
					
					PWM2_SetDutyUS(600);
					PWM1_Disable();
				}
				
				if(mx >= 10 && mx < 30)
				{
					//Gira a la derecha
					Bit1_PutVal(0);
					Bit2_PutVal(0);
					PWM1_SetDutyUS(500);//negro-rojo ---envia 300
					//Condicional para que vaya recto
					PWM2_SetDutyUS(500);//---envia 500
					
					
				}
				
				if(mx > 65 && mx < 95)
				{	//Gira a la izquierda visto desde la perspectiva del carrito
					Bit1_PutVal(0);
					Bit2_PutVal(0);				
					PWM1_SetDutyUS(400); //--logica negada envia 500
					//Cpu_Delay100US(10000);
					PWM2_SetDutyUS(520); //blanco-verde---envia 400
					
					
				}
				
				if(mx <= 65 && mx >= 30)
				{
					Bit1_PutVal(0);
					Bit2_PutVal(0);		
					PWM1_SetDutyUS(400); //envia 490
					//Cpu_Delay100US(10000);
					PWM2_SetDutyUS(400);  
				} 
				
				if(mx >95){
					PWM1_Disable();
					PWM2_Disable();
				}
			}			
			
		} 
		
	}
	else
	{
		PWM1_Disable();
		PWM2_Disable();
	}
*/
	
	
};
/* User includes (#include below this line is not maintained by Processor Expert) */

void main(void)
{
  /* Write your local variable definition here */

  /*** Processor Expert internal initialization. DON'T REMOVE THIS CODE!!! ***/
  PE_low_level_init();

	   for(;;)
	   {   
		   switch(state)
		   {
		   case resetCam:
			   //Cpu_Delay100US(20000);
			   sendWord(reset,sizeof(reset),0);   
			   //Cpu_Delay100US(500);
			   state = sendCam;
			   break;
		  
		   case sendCam:
	
			   //sendWord(LedOn,sizeof(LedOn),0);
			   //sendWord(capture,sizeof(capture),0);
			   
			   //Cpu_Delay100US(2000);
			   
			   //sendWord(meanColor,sizeof(meanColor),0);
			   
			   //sendWord(centroid,sizeof(centroid),0);
			   
			   Cpu_Delay100US(500);
			   
			   sendWord(trackColor,sizeof(trackColor),0);
			   //sendWord(trackWindow,sizeof(trackWindow),0);
			   
			   Cpu_Delay100US(500);
			   state = receiveCam;
			   break;
			   

			   
		   case receiveCam:	
			   
			   j = 0;
			   
			   while(TRUE){
				   AS2_RecvChar(&caracter);
				   array[j]=caracter;
				   j++;
				   if(caracter==13 || j==30){
					   if(j<15){
						   break;
					   }
					   sendWord(array,j,1);
					   if(caracter != 13){
						   sendWord(bsr,sizeof(bsr),1);
					   }
					   break;   		   
				   }  
			   }
			   
			   Cpu_Delay100US(500); 
			   state = convertion;
			   break;
			   
		   case convertion:
			   for(i=0;i<j;i++){
				   if(array[i] == 'M')
				   {
					   continue;
				   }
			   	   
			   	   if(array[i] == ' ' && array[i+1] == 'M')
			   	   {
			   		   continue;
			   	   }
			   	   
			   	   if(array[i] == ' ' && array[i+1] != 'M')
			   	   {
			   		   i++;
			   		   f = 0;
			   		   do{
			   			   arrayConvertion[f] = array[i];
			   			   f++;
			   			   i++;
			   		   }while(array[i] != ' ');
			   		   
			   		   mx = asciiConvertion(arrayConvertion, f);
			   		   
			   		  /*i++;
			   		   f = 0;
			   		   do{
						   arrayConvertion[f] = array[i];
						   f++;
						   i++;
			   		   }while(array[i] != ' ');
			   		   
			   		   my = asciiConvertion(arrayConvertion,f);*/
			   		  
			   		   break;
			   	   }
			   }
			   
			   state = sharp;
			   break;
			   
		   case sharp:
		   
			   AD1_MeasureChan(TRUE,0);
			   AD1_GetChanValue16(0,&ADC1);
			   ADC1=ADC1>>4;
			   
			   d = 51.404*exp(-0.001*ADC1);
			   
			   state = pwm;
			   //Cpu_Delay100US(500);
		   	   break;
		   	   
		   case pwm:
			   
			   move(d,mx);
			   
			   state = resetCam;
			   break;
		   } 
	 } 
		  
	   
  /*** End of Processor Expert internal initialization.                    ***/

  /* Write your code here */
  
  	  	
  /* For example: for(;;) { } */

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