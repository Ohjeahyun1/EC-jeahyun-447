/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-13 
* @brief   Embedded Controller:  EC_HAL_for_student_exercise 
* 
******************************************************************************
*/


#include "ecTIM2.h"
#include "ecGPIO.h"

/* Timer Configuration */

void TIM_init(TIM_TypeDef* timerx, uint32_t msec){ 
	
// 1. Enable Timer CLOCK
	if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timerx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(timerx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timerx ==TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(timerx ==TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(timerx ==TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(timerx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	// repeat for TIM4, TIM5, TIM9, TIM11
  // YOUR CODE GOES HERE
	// YOUR CODE GOES HERE
	
	
// 2. Set CNT period
	TIM_period_ms(timerx,msec); 
	
	
// 3. CNT Direction
	timerx->CR1 &= ~(1<<4);					// Upcounter	
	
// 4. Enable Timer Counter
	timerx->CR1 |= TIM_CR1_CEN;		
}


//	Q. Which combination of PSC and ARR for msec unit?
// 	Q. What are the possible range (in sec ?)
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
	// Period usec = 1 to 1000

	// 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
	uint32_t prescaler = _____;
	uint16_t ARRval= (84/(prescaler)*usec);  // 84MHz/1000000 us
	
	TIMx->PSC = ______________;					
	TIMx->ARR = ARRval-1;					
}



void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
	uint32_t prescaler = 8400;
	uint16_t ARRval=(84/(prescaler)*msec*100);  			// 84MHz/1000ms

	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;							
}


// Update Event Interrupt
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec){
// 1. Initialize Timer	
	TIM_init(timerx,msec);
	
// 2. Enable Update Interrupt
	uint32_t IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
	else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
	//else if(timerx ==TIM9)  IRQn_reg = TIM9_IRQn;??
	//else if(timerx ==TIM11)  IRQn_reg = TIM11_IRQn;??
	// repeat for TIM3, TIM4, TIM5, TIM9, TIM11
  // YOUR CODE GOES HERE
	// YOUR CODE GOES HERE
	
	
	NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}



void TIM_INT_enable(TIM_TypeDef* timerx){
	timerx->DIER |= 1<<0;			// Enable Timer Update Interrupt		
}

void TIM_INT_disable(TIM_TypeDef* timerx){
	timerx->DIER &= ~(1<<0);				// Disable Timer Update Interrupt		
}

uint32_t is_UIF(TIM_TypeDef *TIMx){
	return ((TIMx->SR & TIM_SR_UIF) == TIM_SR_UIF);
}

void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~(TIM_SR_UIF);
}
