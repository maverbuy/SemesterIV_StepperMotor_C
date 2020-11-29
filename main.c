//*******************************************************************************************
//main.c
//
//Embedded Systems Software - Lab #4
//Modified by: Mitch Verbuyst
//						  Feb 16, 2018
//
//
//main.c and LED.c were provided as a starting point for this lab.  LCD.c and LCD.h were imported
//from a previous lab in order to incorporate the LCD/Joystick for the control of the stepper 
//motor.
//
//
//main.c provides overall control of the program...waiting for input from joystick and calling
//the appropriate function as defined in LED.c
//
//
//*******************************************************************************************

//****************************Libraries*************************//
#include "stm32l476xx.h"
#include "SysClock.h"
#include "LED.h"
#include "LCD.h"


//***************************Prototypes*************************//
void System_Clock_Init(void);


//****************************main.c****************************//
//Provides control of the program
//Inputs: None
//Outputs: None
//**************************************************************//
int main(void){

	System_Clock_Init(); // Switch System Clock = 80 MHz
	Step_Init(); //initialize the ports for stepper motor
	LCD_Initialization(); //initialize the LCD (not used) and joystick
	LCD_Clear(); // clear the LCD

	//loop forever, checking for inputs from joystick
	while(1){
	
		//center pressed, rotate clockwise halfwave stepping
		if((GPIOA->IDR & 0x1) == 0x1){
			HalfWave();
			while ((GPIOA->IDR &0x1) != 0x00);
		}
	
		//left pressed, rotate counterclockwise halfwave stepping
		if((GPIOA->IDR & 0x2) == 0x2){
			HalfWaveR();
			while ((GPIOA->IDR &0x2) != 0x00);
		}
		
		//right pressed, rotate counterclockwise fullwave stepping
		if((GPIOA->IDR & 0x4) == 0x4){
			FullWaveR();
			while ((GPIOA->IDR &0x4) != 0x00);
		}
		
		//down pressed, rotate clockwise fullwave stepping
		if((GPIOA->IDR & 0x20) == 0x20){
			FullWave();
			while ((GPIOA->IDR &0x20) != 0x00);
		}		
	}
}

