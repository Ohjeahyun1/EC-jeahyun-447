/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-28  	
* @brief   Embedded Controller - ecGPIO.c
* 
******************************************************************************
*/


#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"

//volatile int EC_SYSCLK=16000000;


//GPID port mode register
// Input(00), Output(01), AlterFunc(10), Analog(11)
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode){
	Port->MODER &= ~(3UL<<(2*pin));       // clear bit
	Port->MODER |= mode<<(2*pin);         // set bit
}

void GPIO_init(GPIO_TypeDef *Port, int pin, int mode){     
	// mode  : Input(0), Output(1), AlterFunc(2), Analog(3)   
	if (Port == GPIOA)
		RCC_GPIOA_enable();
	if (Port == GPIOB)
		RCC_GPIOB_enable();
	if (Port == GPIOC)
		RCC_GPIOC_enable();
  if (Port == GPIOD)
		RCC_GPIOD_enable();
	if (Port == GPIOE)
		RCC_GPIOE_enable();
	GPIO_mode(Port, pin, mode);
	
} 


//write the number of pin on the Port
void GPIO_write(GPIO_TypeDef *Port, int pin,  int output){
	if(output == 0)
	  (Port ->ODR) &=  ~(1UL << pin);     // input 0 -> output 0
	else
		(Port ->ODR) |=  (1UL << pin);      // input 1 -> output 1
			
}
//read the number of pin on the Port  
int  GPIO_read(GPIO_TypeDef *Port, int pin){
	 return (Port->IDR) & (1UL << pin);   // read the port
}

//GPIO port output speed register
// Low speed(00), Medium speed(01), Fast speed(10), High speed(11)
void GPIO_ospeed(GPIO_TypeDef* Port, int pin,  int speed){
	 Port->OSPEEDR &= ~(3UL<<(2*pin));        // clear bit
   Port->OSPEEDR |= speed<<(2*pin);         // set bit
}
//GPIO port output type register
//0: Output push-pull (reset state), 1: Output open-drain
void GPIO_otype(GPIO_TypeDef* Port, int pin,  int type){
	Port->OTYPER &= ~(1UL<<(pin));	          // clear bit
	Port->OTYPER |= type<<(pin);	            // set bit
	//Port->OTYPER &= ~((~type)<<(2*pin));
}
//GPIO port pull-up/pull-down register
//00: No pull-up, pull-down , 01: Pull-up ,10: Pull-down , 11: Reserved
void GPIO_pupdr(GPIO_TypeDef* Port, int pin,  int pupdr){
	  Port->PUPDR &= 	~(3UL<<(2*pin));	      // clear bit
		Port->PUPDR  |=	pupdr<<(2*pin);	        // set bit
}

void bittoggle(GPIO_TypeDef* Port,int pin){
		(Port->ODR) ^= (1UL << pin);       // use XOR Gate
}

//4LEDS output with states
void LEDs_toggle(int state){
	// 4 LEDs HIGH,LOW state definition
	int muled[4][4] ={
		    //row - led1,led2,led3,led4, col- state 
                 //led1,led2,led3,led4		
	                  {1,0,0,0},          //state zero
                    {0,1,0,0},          //state one
                    {0,0,1,0},          //state two
                    {0,0,0,1}           //state three                   
  };  
	    //output of 4 LEDS
	    GPIO_write(GPIOA,LED_PIN,muled[state][0]);
			GPIO_write(GPIOA,6,muled[state][1]);
			GPIO_write(GPIOA,7,muled[state][2]);
			GPIO_write(GPIOB,6,muled[state][3]);
}

//4LEDS init,output,Pull-up,medium speed,Push pull
void LED_init(void){
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	and LED pin mode -> output     
	GPIO_init(GPIOA, 6, OUTPUT);          // calls RCC_GPIOA_enable()	and 6 pin mode -> output
	GPIO_init(GPIOA, 7, OUTPUT);          // calls RCC_GPIOA_enable()	and 7 pin mode -> output
	GPIO_init(GPIOB, 6, OUTPUT);          // calls RCC_GPIOB_enable() and 6 pin mode -> output
  
	GPIO_pupdr(GPIOA, LED_PIN, EC_PU);    // GPIOA pin LED pin pupdr -> pull up
	GPIO_pupdr(GPIOA, 6, EC_PU);          // GPIOA pin 6 pupdr -> pull up
	GPIO_pupdr(GPIOA, 7, EC_PU);          // GPIOA pin 7 pupdr -> pull up
	GPIO_pupdr(GPIOB, 6, EC_PU);          // GPIOB pin 6 pupdr -> pull up
	 
	GPIO_otype(GPIOA, LED_PIN, PP);       // GPIOA LED pin otype -> push-pull
	GPIO_otype(GPIOA, 6, PP);             // GPIOA 6 pin otype -> push-pull
	GPIO_otype(GPIOA, 7, PP);             // GPIOA 7 pin otype -> push-pull
	GPIO_otype(GPIOB, 6, PP);             // GPIOB 6 pin otype -> push-pull
	
	GPIO_ospeed(GPIOA,LED_PIN,SMED);      // GPIOA LED_PIN pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,6,SMED);            // GPIOA 6 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,7,SMED);            // GPIOA 7 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOB,6,SMED);            // GPIOB 6 pin ospeed -> Medium speed
	
}

//4LEDS init,output,Pull-up,medium speed,Push pull
void LED_test_init(void){
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	and LED pin mode -> output  
	GPIO_init(GPIOA, 0, OUTPUT);    // calls RCC_GPIOA_enable()	and LED pin mode -> output     
	GPIO_init(GPIOA, 1, OUTPUT);          // calls RCC_GPIOA_enable()	and 6 pin mode -> output
	GPIO_init(GPIOB, 0, OUTPUT);          // calls RCC_GPIOA_enable()	and 7 pin mode -> output
	GPIO_init(GPIOC, 1, OUTPUT);          // calls RCC_GPIOB_enable() and 6 pin mode -> output
  
	GPIO_pupdr(GPIOA, 0, EC_NOPUPD);    // GPIOA pin LED pin pupdr -> NO pullup and pull down
	GPIO_pupdr(GPIOA, 0, EC_NOPUPD);    // GPIOA pin LED pin pupdr -> NO pullup and pull down
	GPIO_pupdr(GPIOA, 1, EC_NOPUPD);          // GPIOA pin 6 pupdr -> NO pullup and pull down
	GPIO_pupdr(GPIOB, 0, EC_NOPUPD);          // GPIOA pin 7 pupdr -> NO pullup and pull down
	GPIO_pupdr(GPIOC, 1, EC_NOPUPD);          // GPIOB pin 6 pupdr -> NO pullup and pull down
	

	 
	GPIO_otype(GPIOA, LED_PIN, PP);       // GPIOA LED pin otype -> push-pull
	GPIO_otype(GPIOA, 0, PP);       // GPIOA LED pin otype -> push-pull
	GPIO_otype(GPIOA, 1, PP);             // GPIOA 6 pin otype -> push-pull
	GPIO_otype(GPIOB, 0, PP);             // GPIOA 7 pin otype -> push-pull
	GPIO_otype(GPIOC, 1, PP);             // GPIOB 6 pin otype -> push-pull
	
	GPIO_ospeed(GPIOA,LED_PIN,SMED);      // GPIOA LED_PIN pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,0,SMED);      // GPIOA LED_PIN pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,1,SMED);            // GPIOA 6 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOB,0,SMED);            // GPIOA 7 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOC,1,SMED);            // GPIOB 6 pin ospeed -> Medium speed
	
}


// When button pressed Output 7segment(0~9) in order
void sevenseg_decode(int number){
	int seven[11][8] ={
		//row- number , col - a,b,c,d....DP
		               //a,b,c,d,e,f,g,DP
	                  {0,0,0,0,0,0,1,1},          //zero
                    {1,0,0,1,1,1,1,1},          //one
                    {0,0,1,0,0,1,0,1},          //two
                    {0,0,0,0,1,1,0,1},          //three
                    {1,0,0,1,1,0,0,1},          //four
                    {0,1,0,0,1,0,0,1},          //five
                    {0,1,0,0,0,0,0,1},          //six
                    {0,0,0,1,1,1,1,1},          //seven
                    {0,0,0,0,0,0,0,1},          //eight
                    {0,0,0,1,1,0,0,1},          //nine
                    {1,1,1,1,1,1,1,0}           //dot
  };
	//outputs 7segment LEDs				
	GPIO_write(GPIOA,8,seven[number][0]);        // a
	GPIO_write(GPIOB,10,seven[number][1]);       // b
	GPIO_write(GPIOA,7,seven[number][2]);        // c
	GPIO_write(GPIOA,6,seven[number][3]);        // d
	GPIO_write(GPIOA,5,seven[number][4]);        // e
	GPIO_write(GPIOA,9,seven[number][5]);        // f
	GPIO_write(GPIOC,7,seven[number][6]);        // g
	GPIO_write(GPIOB,6,seven[number][7]);        // dp
}

// 7segment init
// output,No pull pull down,medium speed,Push pull
void sevenseg_init(void){
	GPIO_init(GPIOA, 8, OUTPUT);      //a
	GPIO_init(GPIOB, 10,OUTPUT);      //b
	GPIO_init(GPIOA, 7, OUTPUT);      //c
	GPIO_init(GPIOA, 6, OUTPUT);      //d
	GPIO_init(GPIOA, 5, OUTPUT);      //e
  GPIO_init(GPIOA, 9, OUTPUT);      //f
	GPIO_init(GPIOC, 7, OUTPUT);      //g
	GPIO_init(GPIOB, 6, OUTPUT);      //DP
	
	GPIO_pupdr(GPIOA, 8, EC_NOPUPD);  // GPIOA 5 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOB, 10,EC_NOPUPD);  // GPIOB 10 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 7, EC_NOPUPD);  // GPIOA 7 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 6, EC_NOPUPD);  // GPIOA 6 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 5, EC_NOPUPD);  // GPIOA 5 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 9, EC_NOPUPD);  // GPIOA 9 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOC, 7, EC_NOPUPD);  // GPIOC 7 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOB, 6, EC_NOPUPD);  // GPIOB 6 pupdr -> NO pull up Pull down
	
	GPIO_otype(GPIOA, 8, PP);         // GPIOA 5 otype -> push pull
	GPIO_otype(GPIOB, 10,PP);         // GPIOB 10 otype -> push pull
	GPIO_otype(GPIOA, 7, PP);         // GPIOA 7 otype -> push pull
	GPIO_otype(GPIOA, 6, PP);         // GPIOA 6 otype -> push pull
	GPIO_otype(GPIOA, 5, PP);         // GPIOA 5 otype -> push pull
	GPIO_otype(GPIOA, 9, PP);         // GPIOA 9 otype -> push pull
	GPIO_otype(GPIOC, 7, PP);         // GPIOC 7 otype -> push pull
	GPIO_otype(GPIOB, 6, PP);         // GPIOB 6 otype -> push pull
		
	GPIO_ospeed(GPIOA,8,SMED);        // GPIOA 5 ospeed -> Medium speed
	GPIO_ospeed(GPIOB,10,SMED);       // GPIOB 10 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,7,SMED);        // GPIOA 7 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,6,SMED);        // GPIOA 6 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,5,SMED);        // GPIOA 5 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,9,SMED);        // GPIOA 9 ospeed -> Medium speed
	GPIO_ospeed(GPIOC,7,SMED);        // GPIOC 7 ospeed -> Medium speed
	GPIO_ospeed(GPIOB,6,SMED);        // GPIOB 6 ospeed -> Medium speed
	
}

void sevenseg_test_init(void){
	GPIO_init(GPIOB, 9, OUTPUT);      //a
	GPIO_init(GPIOA, 6,OUTPUT);      //b
	GPIO_init(GPIOA, 7, OUTPUT);      //c
	GPIO_init(GPIOB, 6, OUTPUT);      //d
	GPIO_init(GPIOC, 7, OUTPUT);      //e
  GPIO_init(GPIOA, 9, OUTPUT);      //f
	GPIO_init(GPIOA, 8, OUTPUT);      //g
	GPIO_init(GPIOB, 10, OUTPUT);      //DP
	
	GPIO_pupdr(GPIOB, 9, EC_NOPUPD);  // GPIOA 5 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 6,EC_NOPUPD);  // GPIOB 10 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 7, EC_NOPUPD);  // GPIOA 7 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOB, 6, EC_NOPUPD);  // GPIOA 6 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOC, 7, EC_NOPUPD);  // GPIOA 5 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 9, EC_NOPUPD);  // GPIOA 9 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOA, 8, EC_NOPUPD);  // GPIOC 7 pupdr -> NO pull up Pull down
	GPIO_pupdr(GPIOB, 10, EC_NOPUPD);  // GPIOB 6 pupdr -> NO pull up Pull down
	
	GPIO_otype(GPIOB, 9, PP);         // GPIOA 5 otype -> push pull
	GPIO_otype(GPIOA, 6,PP);         // GPIOB 10 otype -> push pull
	GPIO_otype(GPIOA, 7, PP);         // GPIOA 7 otype -> push pull
	GPIO_otype(GPIOB, 6, PP);         // GPIOA 6 otype -> push pull
	GPIO_otype(GPIOC, 7, PP);         // GPIOA 5 otype -> push pull
	GPIO_otype(GPIOA, 9, PP);         // GPIOA 9 otype -> push pull
	GPIO_otype(GPIOA, 8, PP);         // GPIOC 7 otype -> push pull
	GPIO_otype(GPIOB, 10, PP);         // GPIOB 6 otype -> push pull
		
	GPIO_ospeed(GPIOB,9,SMED);        // GPIOA 5 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,6,SMED);       // GPIOB 10 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,7,SMED);        // GPIOA 7 ospeed -> Medium speed
	GPIO_ospeed(GPIOB,6,SMED);        // GPIOA 6 ospeed -> Medium speed
	GPIO_ospeed(GPIOC,7,SMED);        // GPIOA 5 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,9,SMED);        // GPIOA 9 ospeed -> Medium speed
	GPIO_ospeed(GPIOA,8,SMED);        // GPIOC 7 ospeed -> Medium speed
	GPIO_ospeed(GPIOB,10,SMED);        // GPIOB 6 ospeed -> Medium speed
	
}

// When button pressed Output 7segment(0~9) in order
void sevenseg_test_decode(int number){
	int seven[11][8] ={
		//row- number , col - a,b,c,d....DP
		               //a,b,c,d,e,f,g,DP
	                  {0,0,0,0,0,0,1,1},          //zero
                    {1,0,0,1,1,1,1,1},          //one
                    {0,0,1,0,0,1,0,1},          //two
                    {0,0,0,0,1,1,0,1},          //three
                    {1,0,0,1,1,0,0,1},          //four
                    {0,1,0,0,1,0,0,1},          //five
                    {0,1,0,0,0,0,0,1},          //six
                    {0,0,0,1,1,1,1,1},          //seven
                    {0,0,0,0,0,0,0,1},          //eight
                    {0,0,0,1,1,0,0,1},          //nine
                    {0,0,0,0,0,0,1,1}           //dot
  };
	//outputs 7segment LEDs				
	GPIO_write(GPIOB,9,seven[number][0]);        // a
	GPIO_write(GPIOA,6,seven[number][1]);       // b
	GPIO_write(GPIOA,7,seven[number][2]);        // c
	GPIO_write(GPIOB,6,seven[number][3]);        // d
	GPIO_write(GPIOC,7,seven[number][4]);        // e
	GPIO_write(GPIOA,9,seven[number][5]);        // f
	GPIO_write(GPIOA,8,seven[number][6]);        // g
	GPIO_write(GPIOB,10,seven[number][7]);        // dp
}
 //LED Pause
void sevenseg_test_decode1(void){
	GPIO_write(GPIOB,9,0);        // a
	GPIO_write(GPIOA,6,0);       // b
	GPIO_write(GPIOA,7,1);        // c
	GPIO_write(GPIOB,6,1);        // d
	GPIO_write(GPIOC,7,0);        // e
	GPIO_write(GPIOA,9,0);        // f
	GPIO_write(GPIOA,8,0);        // g
	GPIO_write(GPIOB,10,1);        // dp
}

void LED_toggle(void){
		(GPIOA->ODR) ^= (1UL << LED_PIN); 
}

void LEDs_test_down(void){
	    GPIO_write(GPIOA,0,0);
			GPIO_write(GPIOA,1,0);
			GPIO_write(GPIOB,0,0);
			GPIO_write(GPIOC,1,0);
	
}         
void LEDs_test_toggle(int state){
	// 4 LEDs HIGH,LOW state definition
	int muled[16][4] ={
		    //row - led1,led2,led3,led4, col- state 
                 //led1,led2,led3,led4		
	                  {0,0,0,0},          //state zero
                    {0,0,0,1},          //state one
                    {0,0,1,0},          //state two
                    {0,0,1,1},          //state three 
                    {0,1,0,0}, 	        //state four
                    {0,1,0,1},          //state five 
                    {0,1,1,0},          //state six
                    {0,1,1,1},          //state seven
                    {1,0,0,0},          //state 8
                    {1,0,0,1}, 					//state	9	
                    {1,0,1,0},          //state 10
                    {1,0,1,1},          //state 11
                    {1,1,0,0},          //state 12
                    {1,1,0,1},          //state 13 
                    {1,1,1,0}, 					//14 	
                    {1,1,1,1} 					//15 	
  };  
	    //output of 4 LEDS
	    GPIO_write(GPIOA,0,muled[state][0]);
			GPIO_write(GPIOA,1,muled[state][1]);
			GPIO_write(GPIOB,0,muled[state][2]);
			GPIO_write(GPIOC,1,muled[state][3]);
}


//GPIO setting
typedef struct {
	GPIO_TypeDef *port;                //example: GPIO_write(extLED[0].port, extLED[0].pin,....);
  int pin;
} _Pin;


