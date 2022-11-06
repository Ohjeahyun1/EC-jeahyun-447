/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-06  	
* @brief   Embedded Controller:  LAB - Timer inputCapture Ultrasonic
*					 
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "math.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecUART.h"
#include "ecSysTIck.h"

uint32_t ovf_cnt = 0;
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

void setup(void);
void TIM2_IRQHandler(void);
//printf distance using Tera Term
//change mm -> cm
int main(void){
	
	setup();
	
	while(1){
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0; 	        // [mm] -> [cm]
		printf("%f [cm]\r\n", distance);                                // print distance
		delay_ms(500);
	}
}

void TIM2_IRQHandler(void){
	if(is_UIF(TIM2)){                                                	// Update interrupt
		ovf_cnt++;												                             	// overflow count
		clear_UIF(TIM2);  							                              	// clear update interrupt flag
	}
	if(is_CCIF(TIM2, 3)){ 								                           	// TIM2_Ch3 (IC3) Capture Flag. Rising Edge Detect
		time1 = TIM2->CCR3;									                            // Capture TimeStart
		clear_CCIF(TIM2, 3);                 	                          // clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM2, 4)){ 								                        // TIM2_Ch3 (IC4) Capture Flag. Falling Edge Detect
		time2 = TIM2->CCR4 ;										                        // Capture TimeEnd
		timeInterval = ((time2-time1)+(0xFFFF+1)*ovf_cnt)/100; 					// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	                          // overflow reset	
		clear_CCIF(TIM2, 4);								                            // clear capture/compare interrupt flag 
	}
}

void setup(){

	RCC_PLL_init();                                                   // 84Mhz system clock
	SysTick_init();                                                   // using sysTick
	UART2_init();                                                     // for printf
  
// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;												                                // PWM1 for trig
	PWM_init(&trig,GPIOA,6,UP,SFAST,PP,EC_NOPUPD,1);			 						// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	                                // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	                                // PWM pulse width of 10us
	
	
// Input Capture configuration -----------------------------------------------------------------------	
	IC_t echo;												                                // Input Capture for echo
	ICAP_init(&echo,GPIOB,10,EC_NOPUPD);    		                      // PB10 as input caputre
	ICAP_counter_us(&echo, 10);   		                                // ICAP counter step time as 10us
	ICAP_setup(&echo, 3, IC_RISE);                                  	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo,4,IC_FALL);                                    	// TIM2_CH3 as IC4 , falling edge detect
	
}
