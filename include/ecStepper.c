#include "stm32f4xx.h"
#include "ecStepper.h"


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

State_full_t FSM_full[4] = {  
 {0b1001,{S1,S3}},           //state S0
 {0b1100,{S2,S0}},           //state S1
 {0b0110,{S3,S1}},           //state S2
 {0b0011,{S0,S2}}            //state S3
};

//HALF stepping sequence
typedef struct {
	uint8_t out;
  uint32_t next[8];
} State_half_t;

State_half_t FSM_half[8] = { 
 {0b1001,{S1,S7}},           //state S0
 {0b1000,{S2,S6}},           //state S1
 {0b1100,{S3,S5}},           //state S2
 {0b0100,{S4,S4}},           //state S3
 {0b0110,{S5,S3}},           //state S4
 {0b0010,{S6,S2}},           //state S5
 {0b0011,{S7,S1}},           //state S6
 {0b0001,{S0,S0}}            //state S7
};



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

void Stepper_pinOut (uint32_t state, int mode){
	
	   if (mode == FULL){         // FULL mode
			GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>3 & 1));		
  		GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>2 & 1));	
     	GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>1 & 1));	
      GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>0 & 1));	 

			}	 
		 else if (mode ==HALF){    // HALF mode
			GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>3 & 1));		
  		GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>2 & 1));	
     	GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>1 & 1));	
      GPIO_write(myStepper.port1, myStepper.pin1,(FSM_full[state].out>>0 & 1));	 
			}
}


void Stepper_setSpeed (long whatSpeed){      // rppm
		step_delay = 	(60000/(2048*whatSpeed));//YOUR CODE   // Convert rpm to milli sec
}
/*
	 int step_number = 0;
	 myStepper._step_num = steps;
	 int state_number = 0;
	 int max_step = 3;
	 if (mode == HALF) max_step = 7;
	 
	
	 for(;myStepper._step_num>0;myStepper._step_num--){ // run for step size
				//step(2048);// YOUR CODE                                  // delay (step_delay); 
				
		    if(direction) step_number++;                  // + direction step number++
				else steps number--;                                  // - direction step number--
				
				if(step_number > max_step)// YOUR CODE                   								//  step_number must be 0 to max_step
		    step_number=// YOUR CODE 
				
		 
		    if (mode == FULL) 		 												
						state_numer=___________// YOUR CODE       // state_number = 0 to 3 for FULL step mode
				else if (mode == HALF) 
						state_numer=___________// YOUR CODE       // state_number = 0 to 7 for HALF step mode					
				
				Stepper_pinOut(state_number, mode);
   }
*/


void Stepper_step(int steps, int direction,int mode){
	 uint32_t state = 0;
	 myStepper._step_num = steps;

	 for(; myStepper._step_num > 0; myStepper._step_num--){ // run for step size
				// YOUR CODE                                  		// delay (step_delay); 
					 
		    if (mode == FULL) 		 												
						state = FSM_full[steps].next[steps];// YOUR CODE       // state = next state
				else if (mode == HALF) 
						state = FSM_half[steps].next[steps];// YOUR CODE       // state = next state
				
				Stepper_pinOut(state, mode);
   }
}


void Stepper_stop (void){ 
     
    	myStepper._step_num = 0;    
			// All pins(Port1~4, Pin1~4) set as DigitalOut '0'
			 	// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
				// YOUR CODE 
}

