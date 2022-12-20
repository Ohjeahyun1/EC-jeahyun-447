/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-12-20
* @brief   Embedded Controller:  Final Project main 2 - Auto Parking Car
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

uint8_t pcData = 0;
uint8_t mcu2Data = 0;

uint32_t IR1, IR2;
int flag = 0;
int seqCHn[16] = {8,9,};


int DIR_flag = 0;

//uint8_t buffer[MAX_BUF]={0, };
//int idx = 0;
int bReceive =0;
int mode = 1; 
int park = 0;


float Right = 0.6;
float Left = 0.5;


void setup(void);
void ADC_IRQHandler(void);


uint32_t ovf_cnt = 0;
int k = 0;                                // distance flag
float distance = 0;
float timeInterval = 0;
float time1 = 0;
float time2 = 0;

_Pin dcPwmPin[2] = {
	{GPIOC, 9}, // TIM3 Ch4
	{GPIOC, 8}	// TIM3 Ch3
};

PWM_t dcPwm[2];

_Pin dcDirPin[2] = {
	{GPIOB, 8}, {GPIOC, 6}	
};

int main(void) {
	// Initialiization --------------------------------------------------------
	setup();
	printf("Hello Nucleo\r\n");
	
	// Inifinite Loop ----------------------------------------------------------
	while (1){
		
		
		//USART 1 send command PC to MCU
		if (bReceive == 1){
			// Forward command (Gear D) Backward command (Gear R)
			if (mcu2Data == 'w' ){       // W input
				if(mode == 1){             // Gear Diving
				Right =0.38;        
				Left =0.3;
				}else{                     // Gear Reverse
				Right = 0.7;
				Left = 0.8;
				}
			}else if(mcu2Data == 'd' ){  //d input
      // Right command (Gear D,R) 
				if(mode == 1) 	{          // Gear Diving
						Right = 0.38;
						Left = 1.0;
					}
				else {                     // Gear Reverse
					Right = 1.0;
					Left = 0.5;
				}
			}else if(mcu2Data == 'a' ){  //a input
			// LEFT command (Gear D,R) 
				if(mode == 1) 				{    // Gear Diving
					Right = 1.0;
					Left = 0.2;
				}
				else {                     // Gear Reverse
					Right = 0.3;
					Left = 1.0;
				}
			}else if(mcu2Data == 's' ){  // S input
				// Change Gear command (Gear D <-> R) 
				mode ^= 1;                 // Gear change
				// Change Motor direction
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, mode);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, mode);
				// Change Gear and Car stop
				if(mode == 1){             // Gear Diving
				Right = 1.0;
				Left = 1.0;
				}else {                    // Gear Reverse
				Right = 0.0;
				Left = 0.0;
				}
			} else if(mcu2Data == 'p'){  // P input
				// Auto Paking Gear(Mode) command  
				park = 1;
			}else if(mcu2Data == 'x') {  // X input
				// Car stop command (Gear D,R) and Auto parking stop
				park = 0;
				if(mode == 1){             // Gear Diving
				Right = 1.0;
				Left = 1.0;
				}else {                    // Gear Reverse
				Right = 0.0;
				Left = 0.0;
				}
			}

			bReceive = 0;

		}
		
		if (IR1 > 1000) DIR_flag = RIGHT;                      //when IR1 sensor detected the line out RIGHT FLAG
		else if (IR2 > 1000) DIR_flag = LEFT;                  //when IR2 sensor detected the line out LEFT FLAG
		else if (IR1 < 999 && IR2 <999) DIR_flag = FORWARD;    //FORWARD FLAG
		
		// Auto parking mode
		if(park == 1){			
			if (DIR_flag == FORWARD ){                           //FORWARD input
				Right =0.52;                                       //0.62
				Left =0.5;                                         //0.6
			}else if(DIR_flag == LEFT ){                         //LEFT input
						Right = 0.4;                                   // 0.4
						Left = 0.8;                                    // 1.0
			}else if(DIR_flag == RIGHT ){                        //RIGHT input
					Right = 0.8;                                     // 1.0
					Left = 0.5;                                      // 0.5
			}
		}
	
		distance = (float) timeInterval * 340.0 / 2.0 / 10.0;  // [mm] -> [cm]

	
		if(distance  < 6.0) {		                               //if obstacle detected
			k++;                                                 // obstacle flag
		}else if(distance > 10.0){                             //when obstacle run out flag clear
			k = 0;
		}
		if(k == 1){                                            // obstacle flag on
			if(mode == 1 && park == 0){                          // Car stop when Gear D,P(Auto parking mode)
				Right = 1.0;
				Left = 1.0;
				}
		}
		// change the Motor PWM with command
    PWM_duty(&dcPwm[A], Right);
	  PWM_duty(&dcPwm[B], Left);
				
		
	}
}



// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                                                   // 84Mhz clock                        
	// USART congfiguration
	USART_init(USART2, 9600);                                         // USART2 For veritication
	USART_begin(USART1, GPIOA, 9, GPIOA,10, 9600); 	                  // USART 1 Blue Tooth PA9 - RXD , PA10 - TXD
	// ADC setting
  ADC_init(GPIOB, 0, TRGO);                                         //ch8 IR1 LEFT
	ADC_init(GPIOB, 1, TRGO);                                         //ch9 IR2 RIGHT

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);
	
	// ADON, SW Trigger enable
	ADC_start();
	
		// secrect Door on off switch
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);                              // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU);                             // GPIOC button pin pupdr -> pull up
  EXTI_init(GPIOC,BUTTON_PIN,FALL,0);                               //EXTI button PIN -> trigger type(falling),propriority(0)
  
	
	// PWM configuration ---------------------------------------------------------------------	
	PWM_t trig;												                                // PWM(TIM3_CH1) for trig
	PWM_init(&trig,GPIOA,6,UP,SFAST,PP,EC_NOPUPD,1);			 						// PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	                                // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);   	                                // PWM pulse width of 10us

   
	//Motor speed PWM init 
	PWM_init(&dcPwm[A], dcPwmPin[A].port, dcPwmPin[A].pin,UP,SFAST,PP,EC_PU,1);
	PWM_init(&dcPwm[B], dcPwmPin[B].port, dcPwmPin[B].pin,UP,SFAST,PP,EC_PU,1);
	
	PWM_period_ms(&dcPwm[A], 1);                                      // Motor period 1ms
	PWM_period_ms(&dcPwm[B], 1);                                      // Motor period 1ms
	
	//Motor direction init
	for (int i = 0; i < 2; i++){
		GPIO_init(dcDirPin[i].port, dcDirPin[i].pin, OUTPUT);           
		GPIO_pupdr(dcDirPin[i].port, dcDirPin[i].pin, EC_PD);
		GPIO_otype(dcDirPin[i].port, dcDirPin[i].pin, PP);
		GPIO_ospeed(dcDirPin[i].port, dcDirPin[i].pin, SHIGH);
	}
	
	GPIO_write(dcDirPin[A].port, dcDirPin[A].pin, mode);
	GPIO_write(dcDirPin[B].port, dcDirPin[B].pin, mode);

	
	// Input Capture configuration -----------------------------------------------------------------------	
		
	IC_t echo;												                                // Input Capture for echo
	ICAP_init(&echo,GPIOB,7 ,EC_NOPUPD);    		                      // PB7 as input caputre
	ICAP_counter_us(&echo, 10);   		                                // ICAP counter step time as 10us
	ICAP_setup(&echo, 2, IC_RISE);                                  	// TIM4_CH2 as IC2 , rising edge detec
	ICAP_setup(&echo,1,IC_FALL);                                      // TIM4_CH1 as IC1 , falling edge detec
	
}

void USART1_IRQHandler(){		//USART1 INT 
	if(is_USART_RXNE(USART1)){
		mcu2Data = USART_getc(USART1);			
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &mcu2Data, 1);
		USART_write(USART1, "\r\n", 2);
			bReceive = 1;
	}
}

// Button pressed park mode On
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){     // when button pressed
   park = 1;                            // Park mode On
	 clear_pending_EXTI(BUTTON_PIN);      // cleared by writing '1'
	}
	
}

void USART2_IRQHandler(){		//USART2 INT 
	if(is_USART_RXNE(USART2)){
		pcData = USART_getc(USART2);
		USART_write(USART1, &pcData, 1);	// transmit char to USART1
		
		printf("%c", pcData); 						// echo to sender(pc)
		
		if(pcData == END_CHAR){
			printf("\r\n"); 							// to change line on PC display
		}
	}
}


// Detect the obstacle using Ultrasonic distance sensor
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
// ADC IR sensor
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){       //after finishing sequence
			if (flag==0){
				IR1 = ADC_read();
			}  
			else if (flag==1){
				IR2 = ADC_read();
			}
		flag =! flag;
	}
}