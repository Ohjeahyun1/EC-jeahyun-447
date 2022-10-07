/**
******************************************************************************
* @author	Oh jeahyun
* @Mod		date 2022/10/07
* @brief	Embedded Controller:  LAB Digital In/Out
*					 - Toggle multiple LEDs by Button B1 pressing
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
	unsigned int cnt = 0;
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//sevensegment output
		sevenseg_decode(cnt % 10);                     //not to make over 10
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) cnt++;   //if button pressed 7segment output up(0~9)
		if (cnt > 9) cnt = 0;                          //over 10 -> 0
		delay_ms(50);                                  //delay for debouncing
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	SysTick_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	sevenseg_init();	                    // 7segment init,otype,ospeed,pupdr   
}