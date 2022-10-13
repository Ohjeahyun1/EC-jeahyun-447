/**
******************************************************************************
* @author  SSSLAB
* @Mod		 2021-8-30 by YKKIM  	
* @brief   Embedded Controller:  LAB Systick&EXTI with API
*					 - 7 segment
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
void EXTI15_10_IRQHandler(void);
int count = 0;                //count lnit      


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//output sevensegment  
		sevenseg_decode(count % 10);        //not to make over 10
		delay_ms(1000);                     //delay
		count++;                            //count updates
		if (count > 9) count =0;            //count over 10 -> 0
		SysTick_reset();                    //SysTick->VAL = 0
	}
}
//when button pressed seven segment display 0
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){ // when button pressed
		count=9;                        // main count -> 0 7segment display 0
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}


void setup(void)
{
	RCC_PLL_init();
	SysTick_init();                       // SysTick initialization
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   // EXTI button PIN -> trigger type(falling),propriority(0)
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	sevenseg_init();	                    // 7segment init,otype,ospeed,pupdr   
}

