# LAB: EXTI & SysTick

### LAB: EXTI_SysTick

**Date:** 2022-10-19

**Author/Partner:** 21800447 Jeahyun Oh /  21800805 SeungEung Hwang

**Github:** https://github.com/Ohjeahyun1/EC-jeahyun-447.git

**Demo Video:** https://youtu.be/znmZgr8ZNYg      

## Introduction

In this lab, you are required to create two simple programs: toggling multiple LEDs with a push-button input and displaying the number counting from 0 to 9 at 1 second rate on a 7-segment display.

You must submit

- LAB Report (*.md & *.pdf)
- Zip source files(main*.c, ecRCC.h, ecGPIO.h, ecSysTick.c etc...).
  - Only the source files. Do not submit project files

### Requirement

**Hardware**

- MCU
  - NUCLEO-F411RE
- Actuator
  * 4 LEDs
  * 7-segment display(5101ASR)
- Sensor
  * Button(B1)
- Others:
  - Array resistor (330 ohm)
  - breadboard

**Software**

- Keil uVision, CMSIS, EC_HAL library

## Problem 1: LED Toggle with EXTI Button

A program that toggles multiple LEDs with a push-button input using external interrupt.

### Create HAL library

Declare and Define the following functions in your library. You must

update your header files located in the directory `EC \lib\`.

**ecEXTI.h**

```c
void EXTI_init(GPIO_TypeDef *port, int pin, int trig_type, int priority);  //EXTI port,pin,trigtype,priority init 
void EXTI_enable(uint32_t pin);  // mask in IMR
void EXTI_disable(uint32_t pin);  // unmask in IMR
uint32_t  is_pending_EXTI(uint32_t pin); //EXTI pending check
void clear_pending_EXTI(uint32_t pin); //EXTI pending clear
```



### Procedure

1. Create a new project under the directory `\repos\EC\LAB\LAB_EXTI`

- The project name is “**LAB_EXTI”.**
- Create a new source file named as “**LAB_EXTI.c”**

> You MUST write your name on the source file inside the comment section.

2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**

1. Connect 4 LEDs externally on a breadboard.
2. Toggle LEDs, turning on one LED at a time by pressing the push button.
   - Example: LED0--> LED1--> …LED3--> …LED0….
3. You must use your library function of EXTI.
4. Refer to the [sample code](https://ykkim.gitbook.io/ec/firmware-programming/example-code#button-interrupt)

### Configuration

![image](https://user-images.githubusercontent.com/113822586/192762196-197b8846-398d-4511-b2e6-7de3e4e86b42.png)

### Circuit Diagram

> ![image](https://user-images.githubusercontent.com/113822586/195567605-e9e4a34c-a944-48e1-9d35-254b0d1946b2.png)

### Discussion

1. To detect an external signal we can use two different methods: polling and interrupt. What are the advantages and disadvantages of each approach?

![image](https://user-images.githubusercontent.com/113822586/195570698-b324470b-c7b6-43f2-8dd1-89f37bd9c9ff.png)

1. What would happen if the EXTI interrupt handler does not clear the interrupt pending flags? Check with your code

   When the switch is pressed, the pending flag is not cleared, so it always reacts as if the command was received. Therefore, LEDs turn on sequentially at a very high speed, making it look as if all four LEDs are lit.

   

### Code

Your code goes here: https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/lab/LAB_EXTI.c , https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/include/ecEXTI.c , https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/include/ecGPIO.c

Explain your source code with necessary comments.

**Description with Code**

* Main code

When button pressed LEDS toggle 

```c
//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
void EXTI15_10_IRQHandler(void);
int state = 0;                     //state lnit


int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
}

// button pressed LEDS toggle
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){ //when button pressed
		LEDs_toggle(state);             //LEDs toggle with state
		state++;                        //state update
		if(state > 3) state=0;          //if state over 3 -> state clear
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}
// Initialiization 
void setup(void)
{
	RCC_HSI_init();
	LED_init();                           //4LEDS init,output,Pull-up,medium speed,Push pull
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   //EXTI button PIN -> trigger type(falling),propriority(0)
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
}
```

* 4LEDs initlization

```c
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
```

* 4LEDs toggle function

``` c
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
```



### Results

Experiment images and results

![KakaoTalk_20221013_202052937](https://user-images.githubusercontent.com/113822586/195584346-e6d84817-0c00-47d8-8030-4f6f2ccb3770.jpg)

As the switch is pressed, the state changes and the LEDs turn on sequentially.

Add demo video link:https://youtu.be/znmZgr8ZNYg   

<iframe width="956" height="538" src="https://www.youtube.com/embed/znmZgr8ZNYg" title="Embedded controller - LAB2" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>





## Problem 2: Counting number on 7-Segment

Display the number 0 to 9 on the 7-segment LED at the rate of 1 sec. After displaying up to 9, then it should display ‘0’ and continue counting.

When the button is pressed, the number should be reset ‘0’ and start counting again.

### Create HAL library

Declare and Define the following functions in your library. You must

update your header files located in the directory `EC \lib\`.

**ecSysTick.h**

```c
void SysTick_init(uint32_t msec); //SysTick initlization PLL -> 1ms
void delay_ms(uint32_t msec); //delay input ms
uint32_t SysTick_val(void); // read SysTick val
void SysTick_reset (void); //SysTick val -> 0
void SysTick_enable(void); //SysTick Timer enable
void SysTick_disable (void) //SysTick Timer disable
```



### Procedure

1. Create a new project under the directory

   `\repos\EC\LAB\LAB_EXTI_SysTick`

- The project name is “**LAB_EXTI_SysTick”.**
- Create a new source file named as “**LAB_EXTI_SysTick.c”**

> You MUST write your name on the source file inside the comment section.

2. Include your updated library in `\repos\EC\lib\` to your project.

- **ecGPIO.h, ecGPIO.c**
- **ecRCC.h, ecRCC.c**
- **ecEXTI.h, ecEXTI.c**
- **ecSysTick.h, ecSysTick.c**

1. First, check if every number, 0 to 9, can be displayed properly on the 7-segment.
2. Then, create a code to display the number counting from 0 to 9 and repeat at the rate of 1 second.
3. When the button is pressed, it should start from '0' again. Use EXTI for this reset.
4. Refer to the [sample code](https://ykkim.gitbook.io/ec/stm32-m4-programming/example-code#systick-interrupt)

### Configuration

![image](https://user-images.githubusercontent.com/113822586/194320212-6768c102-2b27-4bda-9a8c-c8c7b4afb863.png)

### Circuit Diagram

![image](https://user-images.githubusercontent.com/113822586/194311937-0366434d-d7d3-4822-8c04-ce6848f67ef6.png)


### Code

Your code goes here: https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/lab/LAB_EXTI_SysTick.c , https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/include/ecSysTick.c , https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/include/ecEXTI.c , https://github.com/Ohjeahyun1/EC-jeahyun-447/blob/a130d04a342e35712214dceeff996eb540113ba1/include/ecGPIO.c

Explain your source code with necessary comments.

* Main code

```c
//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
void EXTI15_10_IRQHandler(void);
int count = 0;                //count lnit      


int main(void) { 
	// Initialiization --------------------------------------------------------
		setup();
	
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//output sevensegment  
		sevenseg_decode(count % 10);        //not to make over 10
		delay_ms(1000);                     //delay
		count++;                            //count updates
		if (count > 9) count =0;            //count over 10 -> 0
		SysTick_reset();                    //SysTick->VAL = 0
	}
}
//when button pressed seven segment display 0
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){ // when button pressed
		count=9;                        // main count -> 0 7segment display 0
		clear_pending_EXTI(BUTTON_PIN); // cleared by writing '1'
	}
}


void setup(void)
{
	RCC_PLL_init();
	SysTick_init();                       // SysTick initialization
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   // EXTI button PIN -> trigger type(falling),propriority(0)
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	sevenseg_init();	                    // 7segment init,otype,ospeed,pupdr   
}
```

* sevenseg_decode function

``` c
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
```

* sevensegment initlization

type(push pull) ,pupdr(No pull up pull down), ospeed(medium)

``` c
// 7segment init,otype,pupdr,ospeed
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
```





### Results

Experiment images and results

![image](https://user-images.githubusercontent.com/113822586/195586639-4043cd95-86b5-45ca-bc90-6ad392cfefb5.png)

It changes in order from 0 to 9 at intervals of 1 second. Pressing the button resets the 7segment LED to 0.

Add demo video link :https://youtu.be/znmZgr8ZNYg   

<iframe width="956" height="538" src="https://www.youtube.com/embed/znmZgr8ZNYg" title="Embedded controller - LAB2" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>



## Reference

Complete list of all references used (github, blog, paper, etc)

https://kkhipp.tistory.com/155



## Troubleshooting

(Option) You can write Troubleshooting section



















