/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-19   	
* @brief   Embedded Controller - ecSysTick.c
* 
******************************************************************************
*/

#include "ecSysTick.h"



#define MCU_CLK_PLL 84000000
#define MCU_CLK_HSI 16000000

volatile uint32_t msTicks;

//EC_SYSTEM_CLK
//SysTick initlization PLL -> 1ms
void SysTick_init(void){	
	//  SysTick Control and Status Register
	SysTick->CTRL = 0;											// Disable SysTick IRQ and SysTick Counter

	// Select processor clock
	// 1 = processor clock;  0 = external clock
	SysTick->CTRL |= SysTick_CTRL_CLKSOURCE_Msk;

	// uint32_t MCU_CLK=EC_SYSTEM_CLK
	// SysTick Reload Value Register
	SysTick->LOAD = 84000000 / 1000 - 1;						// 1ms, for HSI PLL = 84MHz.

	// SysTick Current Value Register
	SysTick->VAL = 0;

	// Enables SysTick exception request
	// 1 = counting down to zero asserts the SysTick exception request
	SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
	
	// Enable SysTick IRQ and SysTick Timer
	SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
		
	NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 16
	NVIC_EnableIRQ(SysTick_IRQn);			// Enable interrupt in NVIC
}



void SysTick_Handler(void){
	SysTick_counter();	
}

void SysTick_counter(){
	msTicks++;
}	

//delay input ms
void delay_ms (uint32_t mesc){
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < mesc);
	
	msTicks = 0;
}

//SysTick val -> 0
void SysTick_reset(void)
{
	// SysTick Current Value Register
	SysTick->VAL = 0;
}
// read SysTick val
uint32_t SysTick_val(void) {
	return SysTick->VAL;
}

void SysTick_enable(void) {
		SysTick->CTRL |= 1<<0;			    //SysTick Timer enable
}
void SysTick_disable(void) {
		SysTick->CTRL &= ~1<<0;			    //SysTick Timer disable
}

