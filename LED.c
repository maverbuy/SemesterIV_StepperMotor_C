//*******************************************************************************************
//LED.c
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
//LED.c Functions are as follows:
//
//Joystick center: Rotate clockwise HalfStep
//Joystick Left: Rotate Counterclockwise HalfStep
//Joystick Right: Rotate Counterclockwise FullStep
//Joystick Down: Rotate clockwise FullStep
//
//PB2, PB3, PB6, PB7 are used for the control of the stepper motor.
//GPIOA (0,1,2,3,5) is used for the joystick.
//
//Clock speed: 80 MHz
//
//*******************************************************************************************


//**************************Libraries***********************//
#include "LED.h"
#include "LCD.h"

//**************************Constants***********************//
#define MODE_MASK 0xFFFF0F0F
#define MODE_VAL 0x00005050
#define TYPE_MASK 0xFF33
#define TYPE_VAL 0x0000


//******************************************************************************************
// User LEDs: 
//   LD4 Red = PB2    LD5 Green = PE8
// Note: The Green LED is yellow on my board.
//       PE8 is also the TIM1_CH1N for ADC Triggers.
//******************************************************************************************
void LED_Init(void){
	/* Enable GPIOs clock */ 	
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOBEN | RCC_AHB2ENR_GPIOEEN;
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// LD4 Red = PB2
	///////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOB->MODER = ~(3U<<(2*2));  
	GPIOB->MODER |= 1U<<(2*2);      //  Output(01)
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOB->OSPEEDR &= ~(3U<<(2*2));
	GPIOB->OSPEEDR |=   3U<<(2*2);  // High speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOB->OTYPER &= ~(1U<<2);       // Push-pull
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOB->PUPDR   &= ~(3U<<(2*2));  // No pull-up, no pull-down
	
	///////////////////////////////////////////////////////////////////////////////////////////////
	// LD5 Green = PE8
	///////////////////////////////////////////////////////////////////////////////////////////////
	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOE->MODER = ~(3U<<(2*8));  
	GPIOE->MODER |= 1U<<(2*8);      //  Output(01)
	
	// GPIO Speed: Low speed (00), Medium speed (01), Fast speed (10), High speed (11)
	GPIOE->OSPEEDR &= ~(3U<<(2*8));
	GPIOE->OSPEEDR |=   3U<<(2*8);  // High speed
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOE->OTYPER &= ~(1U<<8);       // Push-pull
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOE->PUPDR   &= ~(3U<<(2*8));  // No pull-up, no pull-down
}

//******************************************************************************************
// Turn Red LED On
//******************************************************************************************
void Red_LED_On(void){
	GPIOB->ODR |= GPIO_ODR_ODR_2;
}

//******************************************************************************************
// Turn Red LED Off
//******************************************************************************************
void Red_LED_Off(void){
	GPIOB->ODR &= ~GPIO_ODR_ODR_2;
}

//******************************************************************************************
// Toggle Red LED 
//******************************************************************************************
void Red_LED_Toggle(void){
	GPIOB->ODR ^= GPIO_ODR_ODR_2;
}

//******************************************************************************************
// Turn Green LED On
//******************************************************************************************
void Green_LED_On(void){
	GPIOE->ODR |= GPIO_ODR_ODR_8;
}

//******************************************************************************************
// Turn Green LED Off
//******************************************************************************************
void Green_LED_Off(void){
	GPIOE->ODR &= ~GPIO_ODR_ODR_8;
}

//******************************************************************************************
// Toggle Green LED
//******************************************************************************************
void Green_LED_Toggle(void){
	GPIOE->ODR ^= GPIO_ODR_ODR_8;
}


//********************************Step_Init*************************//
//
//Initialize the appropriate pins for GPIO B for the control of the 
//stepper motor
//
//Inputs: None
//Outputs: None
//
//*****************************************************************//
void Step_Init(void){
	
		/* Enable GPIO B clock */ 	
	RCC->AHB2ENR |=   RCC_AHB2ENR_GPIOBEN;

	// GPIO Mode: Input(00, reset), Output(01), AlterFunc(10), Analog(11, reset)
	GPIOB->MODER &= MODE_MASK;
	GPIOB->MODER |= MODE_VAL;  //01 output
	
	// GPIO Output Type: Output push-pull (0, reset), Output open drain (1) 
	GPIOB->OTYPER &= TYPE_MASK;
	GPIOB->OTYPER |= TYPE_VAL; //0 push pull
	
	// GPIO Push-Pull: No pull-up, pull-down (00), Pull-up (01), Pull-down (10), Reserved (11)
	GPIOB->PUPDR &= MODE_MASK;
	GPIOB->PUPDR |= 0x00000000; //no pull up, no pull down
	
}


//********************************Halfwave*************************//
//
//Step the motor clockwise using halfsteps
//
//Inputs: None
//Outputs: None
//
//*****************************************************************//
void HalfWave(void){
	
	unsigned char HalfStep[8] = {0x9, 0x8, 0xA, 0x2, 0x6, 0x4, 0x5, 0x1};
	uint8_t An, Bn, B, A;
	int i,j,k;
	
	
	for (j = 0; j < 4096/8; j++){
		for (i = 0; i< 8; i++){
			for (k = 0; k < 12300; k++){;} //delay
				
				//Set the proper windings
				A = (HalfStep[i] & 0x8) >> 3;
				An = (HalfStep[i] & 0x4) >> 2;
				B = (HalfStep[i] & 0x2) >> 1;
				Bn = HalfStep[i] & 0x1;
				
				//Shift winding outputs to register
				GPIOB->ODR &= 0xFF33;
				GPIOB->ODR |= (A << 2);
				GPIOB->ODR |= (An << 3);
				GPIOB->ODR |= (B << 6);
				GPIOB->ODR |= (Bn << 7);
				
				
			}
		}
}

//********************************FullWave*************************//
//
//Step the motor clockwise using full steps
//
//Inputs: None
//Outputs: None
//
//*****************************************************************//
void FullWave(void){
	
	unsigned char FullStep[4] = {0x9, 0xA, 0x6, 0x5};
	uint8_t An, Bn, A, B;
	int i,j,k;
	
		for (j = 0; j < 2048/4; j++){
			for (i = 0; i< 4; i++){
				for (k = 0; k < 27000; k++){;} //delay
				
				//Set the proper windings
				A = (FullStep[i] & 0x8) >> 3;
				An = (FullStep[i] & 0x4) >> 2;
				B = (FullStep[i] & 0x2) >> 1;
				Bn = FullStep[i] & 0x1;
				
				//Shift winding outputs to register
				GPIOB->ODR &= 0xFF33;
				GPIOB->ODR |= (A << 2);
				GPIOB->ODR |= (An << 3);
				GPIOB->ODR |= (B << 6);
				GPIOB->ODR |= (Bn << 7);	
				
			}
		}
}

//********************************HalfWaveR*************************//
//
//Step the motor counterclockwise using halfsteps
//
//Inputs: None
//Outputs: None
//
//*****************************************************************//
void HalfWaveR(void){
	
	unsigned char HalfStep[8] = {0x1, 0x5, 0x4, 0x6, 0x2, 0xA, 0x8, 0x9};
	uint8_t An, Bn, B, A;
	int i,j,k;
	
	for (j = 0; j < 4096/8; j++){			
		for (i = 0; i< 8; i++){
			for (k = 0; k < 12300; k++){;} //delay
				
				//Set the proper windings
				A = (HalfStep[i] & 0x8) >> 3;
				An = (HalfStep[i] & 0x4) >> 2;
				B = (HalfStep[i] & 0x2) >> 1;
				Bn = HalfStep[i] & 0x1;
				
				//Shift winding outputs to register
				GPIOB->ODR &= 0xFF33;
				GPIOB->ODR |= (A << 2);
				GPIOB->ODR |= (An << 3);
				GPIOB->ODR |= (B << 6);
				GPIOB->ODR |= (Bn << 7);
				
				
			}
		}
}

//********************************FullWaveR*************************//
//
//Step the motor counterclockwise using full steps
//
//Inputs: None
//Outputs: None
//
//*****************************************************************//
void FullWaveR(void){
	
	unsigned char FullStep[4] = {0x5, 0x6, 0xA, 0x9};
	uint8_t An, Bn, A, B;
	int i,j,k;
	
		for (j = 0; j < 2048/4; j++){
			for (i = 0; i< 4; i++){
				for (k = 0; k < 27000; k++){;} //delay
				
				//Set the proper windings
				A = (FullStep[i] & 0x8) >> 3;
				An = (FullStep[i] & 0x4) >> 2;
				B = (FullStep[i] & 0x2) >> 1;
				Bn = FullStep[i] & 0x1;
				
				//Shift winding outputs to register
				GPIOB->ODR &= 0xFF33;
				GPIOB->ODR |= (A << 2);
				GPIOB->ODR |= (An << 3);
				GPIOB->ODR |= (B << 6);
				GPIOB->ODR |= (Bn << 7);	
				
			}
		}
}
