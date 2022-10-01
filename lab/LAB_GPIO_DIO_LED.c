/**
******************************************************************************
* @author	Oh jeahyun
* @Mod		date 2022/10/01
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle LED LD2 by Button B1 pressing
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"


//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	//when button pressed LED toggle
	while(1){
	if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
		bittoggle(GPIOA,LED_PIN);	           // bit toggle function
	}  
	  delay_ms(50);                        //delay for debouncing
	} 
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();
  SysTick_init();	                      // for delay
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable() and LED pin mode -> output
	GPIO_pupdr(GPIOA, LED_PIN, EC_PU);    // GPIOA LED pin pupdr -> pull up
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	GPIO_otype(GPIOA, LED_PIN, PP);       // GPIOA LED pin otype -> push-pull
	GPIO_ospeed(GPIOA,LED_PIN,SMED);      // GPIOA LED pin ospeed -> Medium speed
}