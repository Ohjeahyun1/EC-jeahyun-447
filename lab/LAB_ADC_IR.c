/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-30 
* @brief   Embedded Controller:  LAB - ADC_IR 
*
******************************************************************************
*/

#include "stm32f4xx.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecUART.h"
#include "ecTIM.h"
#include "ecPWM.h"
#include "ecSysTick.h"
#include "String.h"
#include "ecADC.h"


#define END_CHAR 13
#define MAX_BUF 	10

#define A 0
#define B 1

#define FORWARD 0
#define RIGHT   1
#define LEFT    2


uint32_t IR1, IR2;
int flag = 0;
int seqCHn[16] = {8,9,};

int mode = 1;                                            // DC motor direction flag

float Right = 0.62;    //left
float Left = 0.6;      //right

int DIR_flag = 0;


void setup(void);
void ADC_IRQHandler(void);


uint32_t ovf_cnt = 0;
int k = 0;                                               // distance flag
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;
// DC motor PWM
_Pin dcPwmPin[2] = {
	{GPIOC, 9},                                            // TIM3 Ch4
	{GPIOC, 8}	                                           // TIM3 Ch3
};

PWM_t dcPwm[2];

_Pin dcDirPin[2] = {
	{GPIOB, 8}, {GPIOC, 6}	
};
//Use IR sensors to detect any deviation from the line and follow the line
//If Ultrasonic sensor recognize an obstacle, stop it, and when it disappears, follow the line again
int main(void) {
	// Initialiization --------------------------------------------------------
	setup();
	printf("Hello Nucleo\r\n");

	// Inifinite Loop ----------------------------------------------------------
	while (1){
			if (IR1 > 1000) DIR_flag = RIGHT;                    //when IR1 sensor detected the line RIGHT FLAG
		else if (IR2 > 1000) DIR_flag = LEFT;                  //when IR2 sensor detected the line LEFT FLAG
		else if (IR1 < 999 && IR2 <999) DIR_flag = FORWARD;    //FORWARD FLAG
			
			if (DIR_flag == FORWARD ){                           //FORWARD input
				Right =0.56;                                       //0.62
				Left =0.50;                                        //0.6
				
			}else if(DIR_flag == LEFT ){                         //LEFT input
						Right = 0.4;                                   // 0.4
						Left = 0.8;                                    // 1.0
			}else if(DIR_flag == RIGHT ){                        //RIGHT input
					Right = 0.8;                                     // 1.0
					Left = 0.38;                                     // 0.5
			}
			
	//	printf("IR1: %f \r\n", IR1);
	//  printf("IR2: %f \r\n", IR2);
	//		printf("dirflag : %d \r\n\n",DIR_flag);
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0;  // [mm] -> [cm]
//		printf("distance: %f \r\n", distance);
	//		printf("t1: %f \r\n", time1);
	//		printf("t2: %f \r\n", time2);
//	  printf("K: %f \r\n", k);
		if(distance  < 8.0) {		                               //if obstacle detected
			k = 1;                                               // obstacle flag
							
		}else if(distance > 10.0 && k == 1){                   //when obstacle run out flag clear
			k = 0;
		}
		if(k == 1){                                            // obstacle flag on
			Right = 1.0;                                         // stop the RC motor
			Left  = 1.0;                                         // stop the RC motor
		}
		// change the PWM with the IR sensor flag
    PWM_duty(&dcPwm[A], Right);
	  PWM_duty(&dcPwm[B], Left);
	
	}
		
	
}



// Initialiization 
void setup(void)
{	RCC_PLL_init();
	// USART congfiguration
	USART_init(USART2, 9600);
	SysTick_init();                         //SysTick init
	
	// ADC setting
  ADC_init(GPIOB, 0, TRGO);               //ch8
	ADC_init(GPIOB, 1, TRGO);               //ch9

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);
	
	// ADON, SW Trigger enable
	ADC_start();                            
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;												                                          // PWM1 for trig
	PWM_init(&trig,GPIOA,6,UP,SFAST,PP,EC_NOPUPD,1);			 						          // PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	                                          // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	                                          // PWM pulse width of 10us

	// DC motor init
	PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin,UP,SFAST,PP,EC_PU,1);
	PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin,UP,SFAST,PP,EC_PU,1);
	// DC motor period(1ms)
	PWM_period_ms(&dcPwm[A], 1);
	PWM_period_ms(&dcPwm[B], 1);
  // DC motor output mode, no pull up pulll down, push-pull,High speed
	for (int i = 0; i < 2; i++){
		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);
		GPIO_pupdr(dcDirPin[i].port, dcDirPin[i].pin, EC_PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, PP);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, SHIGH);
	}
	// DC motor
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, mode);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, mode);

	
	// Input Capture configuration -----------------------------------------------------------------------	
		
	IC_t echo;												                                // Input Capture for echo
	ICAP_init(&echo,GPIOB,7 ,EC_NOPUPD);    		                      // PB7 as input caputre
	ICAP_counter_us(&echo, 10);   		                                // ICAP counter step time as 10us
	ICAP_setup(&echo, 2, IC_RISE);                                  	// TIM4_CH2 as IC2 , rising edge detec
	ICAP_setup(&echo,1,IC_FALL);                                      // TIM4_CH1 as IC1 , faling edge detec

	
	
}
/*
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
		time2 = TIM2->CCR4;										                          // Capture TimeEnd
		if((time2-time1)<(TIM2->ARR+1)&(ovf_cnt==1)) ovf_cnt=0;         // if (time2-time1)< ARR+1 make over count 0
		timeInterval = ((time2-time1)+(TIM2->ARR+1)*ovf_cnt)/100; 			// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	                          // overflow reset	
		clear_CCIF(TIM2, 4);								                            // clear capture/compare interrupt flag 
	}
}*/
// Detect the distance using Ultrasonic distance sensor
void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                                                	// Update interrupt
		ovf_cnt++;												                             	// overflow count
		clear_UIF(TIM4);  							                              	// clear update interrupt flag
	}
	if(is_CCIF(TIM4, 2)){ 								                           	// TIM4_Ch2 (IC2) Capture Flag. Rising Edge Detect
		time1 = TIM4->CCR2;									                            // Capture TimeStart
		clear_CCIF(TIM4, 2);                 	                          // clear capture/compare interrupt flag 
	}								                      
	else if(is_CCIF(TIM4, 1)){ 								                        // TIM4_Ch1 (IC1) Capture Flag. Falling Edge Detect
		time2 = TIM4->CCR1;										                          // Capture TimeEnd
		if((time2-time1)<(TIM4->ARR+1)&(ovf_cnt==1)) ovf_cnt=0;         // if (time2-time1)< ARR+1 make over count 0
		timeInterval = ((time2-time1)+(TIM4->ARR+1)*ovf_cnt)/100; 			// Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	                          // overflow reset	
		clear_CCIF(TIM4, 1);								                            // clear capture/compare interrupt flag 
	}
}
// ADC interrupt
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();                                                 //clear adc sr
	}
	
	if(is_ADC_EOC()){                                                  //after finishing sequence
			if (flag==0){
				IR1 = ADC_read();
			}  
			else if (flag==1){
				IR2 = ADC_read();
			}
		flag =! flag;
	}
}
