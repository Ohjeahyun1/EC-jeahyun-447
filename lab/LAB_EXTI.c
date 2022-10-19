/**
******************************************************************************
* @author	Oh jeahyun
* @Mod		date 2022/10/19
* @brief	Embedded Controller:  LAB_EXTI
*					 - Toggle multiple LEDs by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"

//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
void EXTI15_10_IRQHandler(void);
int state = 0;                     //state lnit


int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// button pressed LEDS toggle
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){ //when button pressed
		LEDs_toggle(state);             //LEDs toggle with state
		state++;                        //state update
		if(state > 3) state=0;          //if state over 3 -> state clear
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
// Initialiization 
void setup(void)
{
	RCC_HSI_init();
	LED_init();                           //4LEDS init,output,Pull-up,medium speed,Push pull
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   //EXTI button PIN -> trigger type(falling),propriority(0)
}

