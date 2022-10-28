/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-28
* @brief   Embedded Controller:  ecTim.c
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecTIM.h"
#include "ecGPIO.h"


/* Timer Configuration */

void TIM_init(TIM_TypeDef* timerx, uint32_t msec,int mode){ 
	
// 1. Enable Timer CLOCK
	if(timerx ==TIM1) RCC->APB2ENR |= RCC_APB2ENR_TIM1EN;
	else if(timerx ==TIM2) RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
	else if(timerx ==TIM3) RCC->APB1ENR |= RCC_APB1ENR_TIM3EN;
	else if(timerx ==TIM4) RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;
	else if(timerx ==TIM5) RCC->APB1ENR |= RCC_APB1ENR_TIM5EN;
	else if(timerx ==TIM9) RCC->APB2ENR |= RCC_APB2ENR_TIM9EN;
	else if(timerx ==TIM10) RCC->APB2ENR |= RCC_APB2ENR_TIM10EN;
	else if(timerx ==TIM11) RCC->APB2ENR |= RCC_APB2ENR_TIM11EN;
	
	
// 2. Set CNT period
	if(mode == 0) TIM_period_ms(timerx,msec); 
	else if(mode == 1) TIM_period_ms_PWM(timerx,msec);
	//TIM_period_test(timerx,msec);
	
// 3. CNT Direction
	timerx->CR1 &= ~TIM_CR1_DIR;					// Upcounter	
	//timerx-> CR1 |= 1<<4;                   // Downcounter
	
// 4. Enable Timer Counter
	timerx->CR1 |= TIM_CR1_CEN;		
}


void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec){   
	// Period usec = 1 to 1000

	// 1us(1MHz, ARR=1) to 65msec (ARR=0xFFFF)
	uint16_t prescaler;
	uint32_t Sys_CLK;                                           //system clock
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) ==RCC_CFGR_SW_PLL)     {   //PLL clock
		Sys_CLK = 84000000;                                       
}  else if((RCC->CFGR & RCC_CFGR_SW_HSI) ==RCC_CFGR_SW_HSI)	 {//HSI clock
		Sys_CLK = 16000000;
}
	
	if(TIMx == TIM2 || TIM == TIM5){
		
  prescaler = (Sys_CLK/1000000);                       //1000000-> 1MHz -> 1us 
	uint32_t ARRval=((Sys_CLK/1000000)/prescaler*usec);  // 84MHz/1000000 us
	TIMx->PSC = prescaler - 1;					
	TIMx->ARR = ARRval-1;	
		
	}else{
	
  prescaler = (Sys_CLK/1000000);                       //1000000-> 1MHz -> 1us 
	uint16_t ARRval=((Sys_CLK/1000000)/prescaler*usec);  // 84MHz/1000000 us	
	TIMx->PSC = prescaler - 1;					
	TIMx->ARR = ARRval-1;	
	}		
}



void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	//16bit 0-65000
	//32bit 0-294,967,295
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
//	uint32_t prescaler = 840; // 10kHz -> 0.1ms
//  uint16_t ARRval= 100; // 84MHz/1000ms  
	uint16_t prescaler;
	uint32_t Sys_CLK;                                           //system clock
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) ==RCC_CFGR_SW_PLL)     {   //PLL clock
		Sys_CLK = 84000000;                                       
}  else if((RCC->CFGR & RCC_CFGR_SW_HSI) ==RCC_CFGR_SW_HSI)	 {//HSI clock
		Sys_CLK = 16000000;
}
	
	if(TIMx == TIM2 || TIM == TIM5){
		
  prescaler = Sys_CLK/10000;                            // PSC = 8400 ->  100ms 
	uint32_t ARRval= ((Sys_CLK/10000)/prescaler*msec*10); // 84MHz/1000ms  
	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;			
		
	}else{
	
  prescaler = Sys_CLK/10000;                            // PSC = 8400 ->  100ms
	uint16_t ARRval=((Sys_CLK/10000)/prescaler*msec*10);  // 84MHz/1000000 us	
	TIMx->PSC = prescaler - 1;					
	TIMx->ARR = ARRval-1;	
	}		
	
					
}

void TIM_period_ms_PWM(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	//16bit 0-65000
	//32bit 0-294,967,295
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
//	uint32_t prescaler = 840; // 10kHz -> 0.1ms
//  uint16_t ARRval= 100; // 84MHz/1000ms  
	
	uint16_t prescaler;
	uint32_t Sys_CLK;                                           //system clock
	
	if((RCC->CFGR & RCC_CFGR_SW_PLL) ==RCC_CFGR_SW_PLL)     {   //PLL clock
		Sys_CLK = 84000000;                                       
}  else if((RCC->CFGR & RCC_CFGR_SW_HSI) ==RCC_CFGR_SW_HSI)	 {//HSI clock
		Sys_CLK = 16000000;
}
	
	if(TIMx == TIM2 || TIM == TIM5){	
  prescaler = (Sys_CLK/100000);                           // PSC = 840 ->  10ms
	uint32_t ARRval= ((Sys_CLK/100000)/prescaler*msec*100); // 84MHz/1000ms  
	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;			
		
	}else{
  prescaler = (Sys_CLK/10000);                           // PSC = 840 ->  10ms
	uint16_t ARRval=((Sys_CLK/10000)/prescaler*msec*100);  // 84MHz/1000ms  
	TIMx->PSC = prescaler - 1;					
	TIMx->ARR = ARRval-1;	
	}				
}

void TIM_period_test(TIM_TypeDef* TIMx, uint32_t msec){ 
	// Period msec = 1 to 6000
	//16bit 0-65000
	//32bit 0-294,967,295
	// 0.1ms(10kHz, ARR=1) to 6.5sec (ARR=0xFFFF)
	uint16_t prescaler = 840; // 10kHz -> 0.1ms
	uint16_t ARRval= (840/prescaler*msec*1000/2); // 84MHz/1000ms  
	TIMx->PSC = prescaler-1;					
	TIMx->ARR = ARRval-1;							
}

// Update Event Interrupt
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec, int mode){
// 1. Initialize Timer	
	TIM_init(timerx,msec,TIM);
	
// 2. Enable Update Interrupt
	TIM_INT_enable(timerx);
	
// 3. Timer IRQn	
	uint32_t IRQn_reg =0;
	if(timerx ==TIM1)       IRQn_reg = TIM1_UP_TIM10_IRQn;
	else if(timerx ==TIM2)  IRQn_reg = TIM2_IRQn;
	else if(timerx ==TIM3)  IRQn_reg = TIM3_IRQn;
	else if(timerx ==TIM4)  IRQn_reg = TIM4_IRQn;
	else if(timerx ==TIM5)  IRQn_reg = TIM5_IRQn;
	else if(timerx ==TIM9)  IRQn_reg = TIM1_BRK_TIM9_IRQn;
  else if(timerx ==TIM10)   IRQn_reg = TIM1_UP_TIM10_IRQn;
  else if(timerx ==TIM11)   IRQn_reg = TIM1_TRG_COM_TIM11_IRQn;
	
	// 4. NVIC setting	
	NVIC_EnableIRQ(IRQn_reg);				
	NVIC_SetPriority(IRQn_reg,2);
}



void TIM_INT_enable(TIM_TypeDef* timerx){
	timerx->DIER |= 1<<0;			// Enable Timer Update Interrupt		
}

void TIM_INT_disable(TIM_TypeDef* timerx){
	timerx->DIER &= ~(1<<0);				// Disable Timer Update Interrupt		
}
// pending 
uint32_t is_UIF(TIM_TypeDef *TIMx){
	return ((TIMx->SR & TIM_SR_UIF) == TIM_SR_UIF);
}

void clear_UIF(TIM_TypeDef *TIMx){
	TIMx->SR &= ~(TIM_SR_UIF);
}

void reset_TIMER(TIM_TypeDef *TIMx){
	TIMx->CNT = 0;
}

