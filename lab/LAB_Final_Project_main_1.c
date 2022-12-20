/**
******************************************************************************
* @author	Oh jeahyun
* @Mod		date 2022/12/20
* @brief	Embedded Controller:  LAB_Final_Project - Comfortable and Safe Smart Home
*					 
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
#include "ecEXTI.h"
#include "String.h"
#include "ecADC.h"

//define the led pin number and button pin number

#define LED_PIN 	5
#define BUTTON_PIN 13

// USART2 : MCU to PC via usb 

uint8_t buf1[4];
uint8_t buf2[4];
uint8_t buf3[4];
uint8_t buf4[4];
uint8_t buf5[4];
uint8_t buf6[4];
uint8_t buf7[4];
uint8_t buf8[4];
uint8_t buf9[4];
uint8_t buf10[4];

// ADC
int seqCHn[16] = {8,9,};
// USART 1 PC - MCU (Blue Tooth)
uint8_t pcData = 0;
uint8_t mcu2Data = 0;

#define END_CHAR 13
#define MAX_BUF 	10
int bReceive =0;

// PWM RC servo Motor
PWM_t Door;
PWM_t Door_s;
PWM_t Parking;
PWM_t Curtain;


void setup(void);
void EXTI15_10_IRQHandler(void);
void TIM4_IRQHandler(void);
void ADC_IRQHandler(void);

//count
int person_count = 1;

// Temperature
float Tem = 0.0;
// Light detect
float Light_detect = 0.0;

// flag
int door = 0;             //door open flag
int parking = 0;          // parking breaker flag
int motion_o = 0;         // motion outside flag
int motion = 0;           // motion time flag
int flag = 0;             // ADC flag
int daynight = 1;         //day = 1,night = 0
int sound = 0;            // sound sensor flag
int security = 0;         // security flag
int alarm_w = 0;          // wake alarm flag
int alarm_s = 0;          // security alarm flag
int in = 0;               // inout flag
int light_co = 0;         // light_co
int TV = 0;               // TV on off flag


// LED
int Light_h = 0;          //Light in the house
int Light_o =	0;          //Light outside the house

// Ultrasonic distance Sensor
uint32_t ovf_cnt = 0;
// for the house area
float distance_h = 0;     //distance inside house
float timeInterval_h = 0;
float time1_h = 0;
float time2_h = 0;

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
//	printf("Hello Nucleo\r\n");
	
	//USART2 excel_DAQ initialize 
	USART_write(USART2,(unsigned char*) "CLEARSHEET\r\n",12);	
	USART_write(USART2,(unsigned char*) "LABEL,Date,Time,Timer,DayNight,Person_in,Sunlight,Visitor,Security,Sound,Light,Temperature,Distance\r\n",105);	
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		if (bReceive == 1){
			if (mcu2Data == 'd' ){             //d input
			door = 1;                          //door open
		  in = 1;                            // person inside
			}else if (mcu2Data == 'e' ){       //e input
			security = 1;                      //security on
			}else if (mcu2Data == 'x' ){       //x input
			security = 0;                      //security off
			}else if (mcu2Data == 'o' ){       //o input
			in = 0;                            //person outside
			}else if (mcu2Data == 'a' ){       //a input
			alarm_w = LOW;                     //alarm off
			}else if (mcu2Data == 'c' ){       //c input
			door = 0;                          //door close
			}
			bReceive = 0;
	}
		
	// distance inside the house
	  distance_h = (float) timeInterval_h * 340.0 / 2.0 / 10.0; 	        // [mm] -> [cm]	
	 

  // motion detected Light outside on
	 motion = GPIO_read(GPIOC,2);
	
	//for increasing the hime of the motion light 
	 if(motion == 4) {
		 motion_o = 10; //motion detected
		 }
	
	//owner inout check
	if(in  == 1){                                      // person inside
	if(daynight == 1){                                 // day mode
			Light_h = HIGH;                                // Light on
		// Depending on distance TV on off
		if (distance_h  < 6.0 ) TV = HIGH;               // TV On
		else TV = LOW;                                   // TV off   
		// Light Too strong 
		if(Light_detect < 500) {
			light_co++;                                       
			if(light_co > 3) PWM_duty(&Curtain,0.025);     // too many sunlight closed curtain
		}else if(Light_detect > 2000){                   // Day -> Night
			PWM_duty(&Curtain,0.025);                      // closed curtain
			Light_h = LOW;                                 // house light off
			alarm_w = LOW;                                 // wake up Alarm off
		  daynight = 0;                                  // night mode
		}else {
			PWM_duty(&Curtain,0.075);                      // opened curtain
			light_co = 0;                                  
		}
	}else{                                             // night mode
	  Light_h = LOW;                                   // House Light off
		PWM_duty(&Curtain,0.025);                        // closed curtain
		PWM_duty(&Door,0.025);                           // closed the door
		if(Light_detect < 800) {                         // Night -> Day
		 PWM_duty(&Curtain,0.075);                       // open the curtain
		 Light_h = HIGH;                                 // house light off
	   alarm_w = HIGH;                                 // Wake up Alarm on
	   daynight = 1;                                   // Day mode 
 		}
	 
	} 
}else{                                               // No person inside
	Light_h = LOW;                                     // Light off
  PWM_duty(&Door,0.025);                             // closed the door
	PWM_duty(&Curtain,0.025);                          // closed curtain
	if (distance_h  < 6.0 && distance_h > 2.0)	person_count++;	           // check stranger in the house
  else person_count = 0;
	if (person_count > 7) security = 1;                // Because of the error(Distance sensor)
}
	
      if(door == 0)		PWM_duty(&Door,0.025);         // closed the door
		else if(door ==1)		PWM_duty(&Door,0.075);       // open the door
  
 //Detect the crack of windown 
  sound = GPIO_read(GPIOA,13);
 // Sound detected security mode on
	if(sound == 0) security++;



// Security Mode
  if(security > 0) {                                 // Security On
		PWM_duty(&Curtain,0.075);                        // open curtain
		PWM_duty(&Door,0.025);                           // closed the door
		alarm_s = HIGH;                                  // Security Alarm on
		Light_o ^=1;                                     // Light outside toggle
	  Light_h = 1;                                     // Light inside On 
		if (in == 1) 		PWM_duty(&Door_s,0.075);         // if person inside open the secret door
	  else  PWM_duty(&Door_s,0.025);                   // if person outside close the secret door
	}else{                                             // Security Off
		PWM_duty(&Door_s,0.025);                         // close the secret door
		alarm_s = LOW;                                   // Security Alarm off
	if (motion_o > 1) Light_o = HIGH;                  // motion detected Light outside On
	else Light_o = LOW;		                             // Light outside On
	}
		// If Tempeature too high Motor start
		if(Tem > 30) {
			if(security == 0){                             // Security Mode off
			if(in == 1) 		GPIO_write(GPIOB,4,HIGH);      // person inside motor start
		  else            GPIO_write(GPIOB,4,LOW);       // person outside motor stop
			}
		}else GPIO_write(GPIOB,4,LOW);                   // Tem < 30 motor stop
	 
		
		
	 // LEDs
	 GPIO_write(GPIOA,7,Light_h);                      // Light inside house
	 GPIO_write(GPIOC,3,Light_o);                      // Light outside house
	 // Alarm
	 GPIO_write(GPIOA,4,alarm_s);                      // security alarm
	 GPIO_write(GPIOC,10,alarm_w);                     // wake up alarm	
	 // TV
	 GPIO_write(GPIOC,11,TV);                          // TV	
		
	 motion_o--;                                       // for motion timer
		
	
	//DayNight,Person_in,Sunlight,Visitor,Security,Sound,Light,Temperature,Distance
	 //USART2 Trasnmit sensor value to server 
		sprintf(buf1, "%d", daynight);                   // Day Night 
		sprintf(buf2, "%d", in);                         // Owner inside outside
		sprintf(buf3, "%f", Light_detect);               // Sunlight
		if (motion == 4) 	  sprintf(buf4, "%d", 1);      // Motion detect outside
		else sprintf(buf4, "%d", motion);
   	sprintf(buf5, "%d", security);                   // Security Mode on off
			if (sound > 0)     sprintf(buf6, "%d", 0);     // Sound sensor
		else sprintf(buf6, "%d", 1);    
    sprintf(buf7, "%d", Light_h);                    // House Light
   	sprintf(buf8, "%f", Tem);	                       // Temperature
    sprintf(buf9, "%f", distance_h);		             // Distance of house
    // 
		USART_write(USART2,(unsigned char*) "DATA,DATE,TIME,TIMER,",21);	// transmit char to USART6
		USART_write(USART2,&buf1,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf2,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf3,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf4,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf5,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf6,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf7,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf8,4);
		USART_write(USART2,(unsigned char*) ",",1);	                      // transmit char to USART6
		USART_write(USART2,&buf9,4);
		USART_write(USART2,(unsigned char*) ",AUTOSCROLL_20\r\n",16);	    // transmit char to USART6		 
	
	  delay_ms(100);
	
      }
    
}

// Secrte Door close when owner escape
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){                  //when button pressed
	 PWM_duty(&Door_s,0.025);                          // if owner outside close the secret door
		in = 0;                                          // owner outside
	 clear_pending_EXTI(BUTTON_PIN);                   // cleared by writing '1'
	}
}


// Initialization 
void setup(void)
{ 
	RCC_PLL_init();                                    // 84Mhz clock
	SysTick_init();                                    // SysTick init
	USART_init(USART2, 9600);                          // USART 2 init (PLX-DAQ)
	USART_begin(USART1, GPIOA, 9, GPIOA,10, 9600); 	   // USART 1 Blue Tooth PA10 - RXD , PA9 - TXD
	
	/*
	// Doorbell switch
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
  EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   //EXTI button PIN -> trigger type(falling),propriority(0)
  */
	
	// ADC setting
  ADC_init(GPIOB, 0, TRGO);                          //ch8 (Temperature)
	ADC_init(GPIOB, 1, TRGO);                          //ch9 (Light intensity sensor)

	// ADC channel sequence setting
	ADC_sequence(2, seqCHn);
	
	// ADON, SW Trigger enable
	ADC_start();
	
	
	
	//EXTI init
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);               // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU);              // GPIOC button pin pupdr -> pull up
  EXTI_init(GPIOC,BUTTON_PIN,FALL,0);                // EXTI button PIN -> trigger type(falling),propriority(0)
	//Motor init
	GPIO_all_init(GPIOB, 4, OUTPUT,EC_PU,PP,SMED);     // motor
	//Alarm init
	GPIO_all_init(GPIOA,4,OUTPUT,EC_PU,PP,SMED);       // security alarm                               
	GPIO_all_init(GPIOC,10,OUTPUT,EC_PU,PP,SMED);      // wake up alarm	
	// TV init
	GPIO_all_init(GPIOC,11,OUTPUT,EC_PU,PP,SMED);      // TV	
	//LEDs init 
	GPIO_all_init(GPIOA,7,OUTPUT,EC_PU,PP,SMED);       // house light
	GPIO_all_init(GPIOC,3,OUTPUT,EC_PU,PP,SMED);       // door light
  // Motion detected input
	GPIO_init(GPIOC, 2, INPUT);                        // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, 2, EC_PU);                       // GPIOC button pin pupdr -> pull up
	// Sound detected digital input
	GPIO_init(GPIOA, 13, INPUT);                       // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOA, 13, EC_PU);                      // GPIOC button pin pupdr -> pull up
	
	// PWM configuration ---------------------------------------------	
	PWM_t trig;												                 // PWM(TIM3_CH1) for trig
	PWM_init(&trig,GPIOA,6,UP,SFAST,PP,EC_NOPUPD,1);   // PA_6: Ultrasonic trig pulse
	PWM_period_us(&trig, 50000);    	                 // PWM of 50ms period. Use period_us()
	PWM_pulsewidth_us(&trig, 10);                      // PWM pulsewidth 10us
	

	// Input Capture configuration(house) ----------------------------
	IC_t echo_h;												               // Input Capture for echo
	ICAP_init(&echo_h,GPIOB,7,EC_NOPUPD);    		       // P7 as input caputre
	ICAP_counter_us(&echo_h, 10);   		               // ICAP counter step time as 10us
	ICAP_setup(&echo_h, 2, IC_RISE);                   // TIM4_CH2 as IC2 , rising edge detec
	ICAP_setup(&echo_h,1,IC_FALL);                     // TIM4_CH1 as IC1 , falling edge detec
	
	
  // PWM RC servo Motor
	PWM_init(&Door,GPIOB,5,UP,SFAST,PP,EC_PU,1);       // TIM3_CH2(PB5) UP clock,FAST,1ms clock 
  PWM_period_ms(&Door,20);	                         // set PWM period 20ms
  PWM_init(&Curtain,GPIOC,8,UP,SFAST,PP,EC_PU,1);    // TIM3_CH3(PC8) UP clock,FAST,1ms clock 
  PWM_period_ms(&Curtain,20);	                       // set PWM period 20ms
	PWM_init(&Door_s,GPIOC,9,UP,SFAST,PP,EC_PU,1);     // TIM3_CH4(PC9) UP clock,FAST,1ms clock 
  PWM_period_ms(&Door_s,20);	                       // set PWM period 20ms
	
}
// USART1 BlueTooth PC -> MCU
void USART1_IRQHandler(){		//USART1 INT 
	if(is_USART_RXNE(USART1)){
		mcu2Data = USART_getc(USART1);			             // PC -> MCU
		USART_write(USART1,(uint8_t*) "BT sent : ", 10);
		USART_write(USART1, &mcu2Data, 1);
		USART_write(USART1, "\r\n", 2);
			bReceive = 1;
	
	}
}
/*
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
*/
// Detect the distance using Ultrasonic distance sensor
void TIM4_IRQHandler(void){
	if(is_UIF(TIM4)){                                                	// Update interrupt
		ovf_cnt++;												                             	// overflow count
		clear_UIF(TIM4);  							                              	// clear update interrupt flag
	}
  if(is_CCIF(TIM4, 2)){ 								                            // TIM4_Ch2 (IC2) Capture Flag. Rising Edge Detect
		time1_h = TIM4->CCR2;									                          // Capture TimeStart
		clear_CCIF(TIM4, 2);
	}else if(is_CCIF(TIM4, 1)){ 								                      // TIM4_Ch1 (IC1) Capture Flag. Falling Edge Detect
		time2_h = TIM4->CCR1;										                        // Capture TimeEnd
		if((time2_h-time1_h)<(TIM4->ARR+1)&(ovf_cnt==1)) ovf_cnt=0;     // if (time2-time1)< ARR+1 make over count 0
		timeInterval_h = ((time2_h-time1_h)+(TIM4->ARR+1)*ovf_cnt)/100; // Total time of echo pulse (10us * counter pulses -> [msec] unit)
		ovf_cnt = 0;                        	                          // overflow reset	
		clear_CCIF(TIM4, 1);								                            // clear capture/compare interrupt flag 
	}
}

// ADC Temperature and Light detect
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){
		clear_ADC_OVR();
	}
	
	if(is_ADC_EOC()){                //after finishing sequence
			if (flag==0){                // Temperature sensor
				Tem = ADC_read()/10.0;
				if(Tem > 60) flag ^= 1;    // for the flag error
			}  
			else if (flag==1){           // Light intensity sensor 
				Light_detect = ADC_read();
			}
		flag =! flag;
	}
}