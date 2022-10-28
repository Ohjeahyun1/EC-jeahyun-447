/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-28 	
* @brief   Embedded Controller:  ecPWM.c
* 
******************************************************************************
*/


#include "ecPWM.h"
#include "ecTim.h"
#include "ecGPIO.h"

/* PWM Configuration */

void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin, int DIR,int speed,int msec){
// 0. Match Output Port and Pin for TIMx 	
		pwm->port = port;
		pwm->pin  = pin;
		PWM_pinmap(pwm);
		TIM_TypeDef *TIMx = pwm->timer;
	  
		int CHn = pwm->ch;	

// 1. Initialize GPIO port and pin as AF
		GPIO_init(port, pin, AF);  // AF=2
		GPIO_ospeed(port, pin,speed);  // speed VHIGH=3
	
	
   int AFN = 0;
		if(TIMx ==TIM1) AFN = 1;
	else if(TIMx ==TIM2)  AFN = 1;
	else if(TIMx==TIM3)  AFN = 2;
	else if(TIMx ==TIM4)  AFN = 2;
	else if(TIMx ==TIM5)  AFN = 2;
	else if(TIMx ==TIM9)  AFN = 3;
	else if(TIMx ==TIM10)  AFN = 3;
	else if(TIMx ==TIM11)  AFN = 3;
// 2. Configure GPIO AFR by Pin num.				
	//  AFR[0] for pin: 0~7,     AFR[1] for pin 8~15
	//  AFR=1 for TIM1,TIM2	AFR=2 for TIM3 etc
		port -> AFR[pin/8] &= ~(15<<(4*(pin%8)));
		port -> AFR[pin/8] |= AFN<<(4*(pin%8));
	

			
// 3. Initialize Timer 
		TIM_init(TIMx, msec,PWM);	// msec value.		

// 3-2. Direction of Counter
	if(DIR == 0) 	TIMx->CR1 &= ~(1<<4) ;    // Counting direction: 0 = up-counting, 1 = down-counting
	else if(DIR == 1) TIMx->CR1 |= (1<<4);

			
// 4. Configure Timer Output mode as PWM
	uint32_t ccVal=TIMx->ARR/2;  // default value  CC=ARR/2
	if(CHn == 1){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC1M;                     // Clear ouput compare mode bits for channel 1
		TIMx->CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2; //6<<4												// OC1M = 110 for PWM Mode 1 output on ch1. #define TIM_CCMR1_OC1M_1          (0x2UL << TIM_CCMR1_OC1M_Pos)
		TIMx->CCMR1	|= TIM_CCMR1_OC1PE;                     // Output 1 preload enable (make CCR1 value changable)
		TIMx->CCR1  = ccVal; 																// Output Compare Register for channel 1 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC1P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC1E;												// Enable output for ch1
	}
	else if(CHn == 2){
		TIMx->CCMR1 &= ~TIM_CCMR1_OC2M;                     // Clear ouput compare mode bits for channel 2
		TIMx->CCMR1 |= TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;	// OC1M = 110 for PWM Mode 1 output on ch2
		TIMx->CCMR1	|= TIM_CCMR1_OC2PE;                    	// Output 1 preload enable (make CCR2 value changable)	
		TIMx->CCR2  = ccVal; 																// Output Compare Register for channel 2 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC2P;                       // select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC2E;												// Enable output for ch2
	}
	else if(CHn == 3){
		TIMx->CCMR2 &= ~TIM_CCMR2_OC3M;                     // Clear ouput compare mode bits for channel 3
		TIMx->CCMR2 |= TIM_CCMR2_OC3M_1 | TIM_CCMR2_OC3M_2; // OC1M = 110 for PWM Mode 1 output on ch3
		TIMx->CCMR2	|= TIM_CCMR2_OC3PE;           					// Output 1 preload enable (make CCR3 value changable)	
		TIMx->CCR3  = ccVal; 									   						// Output Compare Register for channel 3 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC3P;                				// select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC3E;									  		// Enable output for ch3
	}
	else if(CHn == 4){
	  TIMx->CCMR2 &= ~TIM_CCMR2_OC4M;                     // Clear ouput compare mode bits for channel 4
		TIMx->CCMR2 |= TIM_CCMR2_OC4M_1 | TIM_CCMR2_OC4M_2; // OC1M = 110 for PWM Mode 1 output on ch4
		TIMx->CCMR2	|= TIM_CCMR2_OC4PE;           					// Output 1 preload enable (make CCR4 value changable)	
		TIMx->CCR4  = ccVal; 									   						// Output Compare Register for channel 4 (default duty ratio = 50%)		
		TIMx->CCER &= ~TIM_CCER_CC4P;                				// select output polarity: active high	
		TIMx->CCER  |= TIM_CCER_CC4E;									  		// Enable output for ch4	
	}	
	
	
	
// 5. Enable Timer Counter
//	if(TIMx == TIM1) TIMx->BDTR |= TIM_BDTR_MOE;					// Main output enable (MOE): 0 = Disable, 1 = Enable	
	TIMx->CR1  |= TIM_CR1_CEN;  													// Enable counter
}

// set PWM period ms
void PWM_period_ms(PWM_t *PWM_pin, uint32_t msec){
	TIM_TypeDef *TIMx = PWM_pin->timer;
	TIM_period_ms_PWM(TIMx,msec);   
}
// set PWM period us
void PWM_period_us(PWM_t *PWM_pin, uint32_t usec){
	TIM_TypeDef *TIMx = PWM_pin->timer;
	TIM_period_us(TIMx, usec); 	
}

// set PWM CCval
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms){ 
	TIM_TypeDef *TIMx = pwm->timer;
	int CHn = pwm->ch;
	uint32_t fsys = 0;
	uint32_t psc=pwm->timer->PSC;
	
	// Check System CLK: PLL or HSI
	if((RCC->CFGR & (3<<0)) == 2)      { fsys = 84000; }  // for msec 84MHz/1000
	else if((RCC->CFGR & (3<<0)) == 0) { fsys = 16000; }
	
	float fclk = fsys/(psc+1);					          // fclk=fsys/(psc+1);
	uint32_t ccval = pulse_width_ms*fclk-1;				// width_ms *fclk - 1;
	
	switch(CHn){
		case 1: TIMx->CCR1 = ccval; break;
		case 2: TIMx->CCR2 = ccval; break;
		case 3: TIMx->CCR3 = ccval; break;
		case 4: TIMx->CCR4 = ccval; break;
		default: break;
	}
}


void PWM_duty(PWM_t *pwm, float duty) {                 //  duty=0 to 1	
		TIM_TypeDef *TIMx = pwm->timer;
	uint32_t arr= pwm->timer->ARR;
	
	  float ccval = (arr+1)*duty-1;    								// (ARR+1)*dutyRatio - 1          
		int CHn = pwm->ch;

		switch(CHn){
			case 1: TIMx->CCR1 = ccval; break;
			case 2: TIMx->CCR2 = ccval; break;
			case 3: TIMx->CCR3 = ccval; break;
			case 4: TIMx->CCR4 = ccval; break;
			default: break;
		}
}


// DO NOT MODIFY HERE
void PWM_pinmap(PWM_t *PWM_pin){
   GPIO_TypeDef *port = PWM_pin->port;
   int pin = PWM_pin->pin;
   
   if(port == GPIOA) {
      switch(pin){
         case 0 : PWM_pin->timer = TIM2; PWM_pin->ch = 1; break;
         case 1 : PWM_pin->timer = TIM2; PWM_pin->ch = 2; break;
         case 5 : PWM_pin->timer = TIM2; PWM_pin->ch = 1; break;
         case 6 : PWM_pin->timer = TIM3; PWM_pin->ch = 1; break;
         //case 7: PWM_pin->timer = TIM1; PWM_pin->ch = 1N; break;
         case 8 : PWM_pin->timer = TIM1; PWM_pin->ch = 1; break;
         case 9 : PWM_pin->timer = TIM1; PWM_pin->ch = 2; break;
         case 10: PWM_pin->timer = TIM1; PWM_pin->ch = 3; break;
         case 15: PWM_pin->timer = TIM2; PWM_pin->ch = 1; break;
         default: break;
      }         
   }
   else if(port == GPIOB) {
      switch(pin){
         //case 0: PWM_pin->timer = TIM1; PWM_pin->ch = 2N; break;
         //case 1: PWM_pin->timer = TIM1; PWM_pin->ch = 3N; break;
         case 3 : PWM_pin->timer = TIM2; PWM_pin->ch = 2; break;
         case 4 : PWM_pin->timer = TIM3; PWM_pin->ch = 1; break;
         case 5 : PWM_pin->timer = TIM3; PWM_pin->ch = 2; break;
         case 6 : PWM_pin->timer = TIM4; PWM_pin->ch = 1; break;
         case 7 : PWM_pin->timer = TIM4; PWM_pin->ch = 2; break;
         case 8 : PWM_pin->timer = TIM4; PWM_pin->ch = 3; break;
         case 9 : PWM_pin->timer = TIM4; PWM_pin->ch = 3; break;
         case 10: PWM_pin->timer = TIM2; PWM_pin->ch = 3; break;
         
         default: break;
      }
   }
   else if(port == GPIOC) {
      switch(pin){
         case 6 : PWM_pin->timer = TIM3; PWM_pin->ch = 1; break;
         case 7 : PWM_pin->timer = TIM3; PWM_pin->ch = 2; break;
         case 8 : PWM_pin->timer = TIM3; PWM_pin->ch = 3; break;
         case 9 : PWM_pin->timer = TIM3; PWM_pin->ch = 4; break;
         
         default: break;
      }
   }
	 // TIM5 needs to be added, if used.
}
