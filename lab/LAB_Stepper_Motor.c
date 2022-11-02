/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-02 	
* @brief   Embedded Controller: Ecstepper motor
*					 
* 
******************************************************************************
*/
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecEXTI.h"
#include "ecSysTick.h"
#include "ecStepper.h"

void setup(void);
void EXTI15_10_IRQHandler(void);
int i = 0;                        //stop flag

//stepper motor operation with 2048 steps, DIR(1 = CW, 0= CCW), MODE(FULL,HALF)
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
	if(i==0)Stepper_step(2048, 0, FULL);  // (Step : 2048, Direction : 0 or 1, Mode : FULL or HALF) // 1=CW, 0 = CCW
	else Stepper_stop();                  // stepper motor stop
	}
}

// when button pin pressed update the flag and stepper motor stop
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)) {
		Stepper_stop();                 // stepper motor stop
		i ^= 1;                         // flag update
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}


// Initialiization 
void setup(void)
{	
	
	RCC_PLL_init();                                 // System Clock = 84MHz
	SysTick_init();                                 // Systick init
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);             // External Interrupt Setting
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);            // GPIOC pin13 initialization
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU);           
	Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
	Stepper_setSpeed(14);                            //  set stepper motor speed max = 14
}


