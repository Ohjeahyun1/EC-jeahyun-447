/**
******************************************************************************
* @author	Oh jeahyun
* @Mod		date 2022/10/28
* @brief	Embedded Controller:  LAB_PWM_RCmotor
*					 - angle 0 to 180 and back to 0 500ms period(moves angle 10) and reset angle 0 when button pressed
* 
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"
#include "ecTIM.h"
#include "ecPWM.h"

//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

PWM_t pwm;

void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM3_IRQHandler(void);
float i = 0;     
int dir = 0;           //direction 0=forward, 1=backward



int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// button pressed reset angle 0 and start over
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){     //when button pressed
	  PWM_duty(&pwm,0.025);               // set motor angle 0
		i = 1.0;                            // reset angel value
		dir = 0;                            // forward
		reset_TIMER(TIM3);                  // reset TIMer value
		clear_pending_EXTI(BUTTON_PIN);     // cleared by writing '1'
	}
}
//Code about motor angle 0 to 180 degree and back angle 0 step of 10 degree
// Timer interrupt 500ms
void TIM3_IRQHandler(void){
	if(is_UIF(TIM3)){	                    //interrupt occur
		if(dir == 0){                       //dir = forward
	PWM_duty(&pwm,0.025+0.05/9.0*i);      // motor angle 0~180
			i++;                              // angle value update
			}else if(dir == 1){               // if dir = backward
		PWM_duty(&pwm,0.125-0.05/9.0*i);    // motor angle 180~0
				i++;                            // angle value update
			}
		if(i>18.0){	                        // not over 180 degree
		dir ^= 1;                           // dir update
		i=1.0;                              // motor angle reset
		}
     clear_UIF(TIM3);                   // clear by writing 0
	}			
}


// Initialization 
void setup(void)
{ 
	RCC_PLL_init();                       // 84Mhz clock
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
  EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   //EXTI button PIN -> trigger type(falling),propriority(0)
	TIM_INT_init(TIM3,500,TIM);           // TIM3 Timer period: 100us -> interrupt 500ms
	PWM_init(&pwm,GPIOA,1,UP,SFAST,1);    // TIM2_CH2(PA1) DOWN clock,FAST,1ms clock 
  PWM_period_ms(&pwm,20);	              // set PWM period 20ms
	GPIO_pupdr(GPIOA,1,EC_PU);            // GPIOA1 pull up
}

