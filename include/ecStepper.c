/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-02  	
* @brief   Embedded Controller - ecStepper.c
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecStepper.h"
#include "ecSysTick.h"
#include "ecGPIO.h"

// A B A` B`
//State number 
#define S0 0
#define S1 1
#define S2 2
#define S3 3
#define S4 4
#define S5 5
#define S6 6
#define S7 7


// Stepper Motor function
uint32_t direction = 1; 
uint32_t step_delay = 100; 
uint32_t step_per_rev = 64*32;
	 

// Stepper Motor variable
volatile Stepper_t myStepper; 


//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

// A B A` B`
State_full_t FSM_full[4] = {  
 {0b1001,{S1,S3}},           //state S0
 {0b1100,{S2,S0}},           //state S1
 {0b0110,{S3,S1}},           //state S2
 {0b0011,{S0,S2}}            //state S3
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_half_t;

// A B A` B`
State_half_t FSM_half[8] = { 
 {0b1001,{S1,S7}},           //state S0
 {0b1000,{S2,S0}},           //state S1
 {0b1100,{S3,S1}},           //state S2
 {0b0100,{S4,S2}},           //state S3
 {0b0110,{S5,S3}},           //state S4
 {0b0010,{S6,S4}},           //state S5
 {0b0011,{S7,S5}},           //state S6
 {0b0001,{S0,S6}}            //state S7
};


//Stepper motor  initlization
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4){
	 
   //  GPIO Digital Out Initiation
	 //A
	 myStepper.port1 = port1;
   myStepper.pin1  = pin1;
	 //B
	 myStepper.port2 = port2;
   myStepper.pin2  = pin2;
	 //A`
	 myStepper.port3 = port3;
   myStepper.pin3  = pin3;
	 //B`
	 myStepper.port4 = port4;
   myStepper.pin4  = pin4;
	
	
	//  GPIO Digital Out Initiation
	// No pull-up Pull-down , Push-Pull, Fast	
	// Port1,Pin1 ~ Port4,Pin4
	GPIO_init(port1, pin1, OUTPUT);       // calls RCC_GPIOA_enable()	and motor A mode -> output     
	GPIO_init(port2, pin2, OUTPUT);       // calls RCC_GPIOA_enable()	and motor B mode -> output  
	GPIO_init(port3, pin3, OUTPUT);       // calls RCC_GPIOA_enable()	and motor A` mode -> output  
	GPIO_init(port4, pin4, OUTPUT);       // calls RCC_GPIOA_enable()	and motor B` mode -> output  
  
	GPIO_pupdr(port1, pin1, EC_NOPUPD);   // motor A pupdr -> No pull up and pull down
	GPIO_pupdr(port2, pin2, EC_NOPUPD);   // motor B pupdr -> No pull up and pull down
	GPIO_pupdr(port3, pin3, EC_NOPUPD);   // motor A` pupdr -> No pull up and pull down
	GPIO_pupdr(port4, pin4, EC_NOPUPD);   // motor B` pupdr -> No pull up and pull down
	 
	GPIO_otype(port1, pin1, PP);          // motor A otype -> push-pull
	GPIO_otype(port2, pin2, PP);          // motor B otype -> push-pull
	GPIO_otype(port3, pin3, PP);          // motor A` otype -> push-pull
	GPIO_otype(port4, pin4, PP);          // motor B` otype -> push-pull
	
	GPIO_ospeed(port1, pin1,SFAST);       // motor A ospeed -> Fast speed
  GPIO_ospeed(port2, pin2,SFAST);       // motor B ospeed -> Fast speed
	GPIO_ospeed(port3, pin3,SFAST);       // motor A` ospeed -> Fast speed
	GPIO_ospeed(port4, pin4,SFAST);       // motor B` ospeed -> Fast speed
}


// output stepper motor pin with state(mode: FULL,HALF)
void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
			GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>3 & 1));		
  		GPIO_write(myStepper.port2, myStepper.pin2,(FSM_full[state].out>>2 & 1));	
     	GPIO_write(myStepper.port3, myStepper.pin3,(FSM_full[state].out>>1 & 1));	
      GPIO_write(myStepper.port4, myStepper.pin4,(FSM_full[state].out>>0 & 1));	 

			}	 
		 else if (mode ==HALF){    // HALF mode
			GPIO_write(myStepper.port1, myStepper.pin1,(FSM_half[state].out>>3 & 1));		
  		GPIO_write(myStepper.port2, myStepper.pin2,(FSM_half[state].out>>2 & 1));	
     	GPIO_write(myStepper.port3, myStepper.pin3,(FSM_half[state].out>>1 & 1));	
      GPIO_write(myStepper.port4, myStepper.pin4,(FSM_half[state].out>>0 & 1));	 
			}
}

// convert rpm to msec
void Stepper_setSpeed (long whatSpeed){               // rpm
		step_delay = 	(60000/(step_per_rev*whatSpeed));   // Convert rpm to milli sec
}
// stepper motor opperation
void Stepper_step(int steps, int direction,int mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){     // run for step size
				if (mode == FULL) delay_ms(step_delay);               // delay full
				else if(mode == HALF) delay_ms(step_delay/2);         // delay Half
		 
		    if (mode == FULL) 		 												
						state = FSM_full[state].next[direction];          // state = next state
				else if (mode == HALF) 
						state = FSM_half[state].next[direction];          // state = next state
				
				Stepper_pinOut(state, mode);
   }
}

// stepper motor stop
void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			GPIO_write(myStepper.port1, myStepper.pin1,0);
	    GPIO_write(myStepper.port2, myStepper.pin2,0);
	    GPIO_write(myStepper.port3, myStepper.pin3,0);
	    GPIO_write(myStepper.port4, myStepper.pin4,0);
}

