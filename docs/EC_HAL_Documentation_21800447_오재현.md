---
description: EC_HAL API
---


# Embedded Controller - STM32f411 HAL API

Documentation for HAL functions

Written by:   오재현(21800447)

Course:  임베디드컨트롤러

Program: C/C++

IDE/Compiler: Keil uVision 5

OS: WIn10

MCU:  STM32F411RE (Nucleo-64)



---

[TOC]



---





## NUCLEO-F411RE

### PORT connect

![image-20221019134824813](C:\Users\MASTER\AppData\Roaming\Typora\typora-user-images\image-20221019134824813.png)

## GPIO Digital InOut 

### Header File

 `#include "ecGPIO.h"`


```c++
#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

// MODER
#define INPUT  		0x00
#define OUTPUT 		0x01
#define AF     		0x02
#define ANALOG 		0x03

// IDR & ODR
#define HIGH 		1
#define LOW  		0

// PIN
#define LED_PIN 		5
#define BUTTON_PIN 		13

//OSPEED 
#define SLOW 0x00          // Low speed(00)
#define SMED 0x01          // Medium speed(01)
#define SFAST 0x10         // Fast speed(10)
#define SHIGH 0x11         // High speed(11)

//OTYPER 
#define PP 0               // Output push-pull (reset state) (0)
#define OD 1               // Output open-drain(1)

//PUPDR 
#define EC_NOPUPD 0x00     // No pull-up, pull-down(00)
#define EC_PU 0x01         // Pull-up(01)
#define EC_PD 0x10         // Pull-down(10) 
#define EC_RE 0x11         // Reserved(11)


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */
	 
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_write(GPIO_TypeDef *Port, int pin, int output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
```




### GPIO_init\(\)

Initializes GPIO pins with default setting and **Enables** GPIO Clock. Mode: In/Out/AF/Analog

```c++
void GPIO_init(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT(0), OUTPUT(1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_init(GPIOA, 5, OUTPUT);
GPIO_init(GPIOC, 13, INPUT); //GPIO_init(GPIOC, 13, 0);
```



### GPIO_mode\(\)

Configures  GPIO pin modes: In/Out/AF/Analog

```c++
void GPIO_mode(GPIO_TypeDef *Port, int pin, int mode);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **mode**:   INPUT (0), OUTPUT (1),  AF(02), ANALOG (03)

  

**Example code**

```c++
GPIO_mode(GPIOA, 5, OUTPUT);
```



### GPIO_write\(\)

Write the data to GPIO pin: High, Low

```c++
void GPIO_write(GPIO_TypeDef *Port, int pin, int output);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **output**:   LOW(0), HIGH(1)



**Example code**

```c++
GPIO_write(GPIOA, 5, 1);  // GPIOA pin 5-> 1: High
```



### GPIO_read\(\)

Read the data from GPIO pin

```c++
int  GPIO_read(GPIO_TypeDef *Port, int pin);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15



**Example code**

```c++
GPIO_read(GPIOC, 13);  // read GPIOC pin 13
```



### GPIO_ospeed\(\)

Configures  output speed of GPIO pin : Low, Mid, Fast, High

```c++
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **speed**:   LOW_SPEED(0), MID_SPEED(1), FAST_SPEED(2) , HIGH_SPEED(3)

  SLOW,SMED,SFAST,SHIGH



**Example code**

```c++
GPIO_ospeed(GPIOA, 5, SFAST);  // 2: FAST_SPEED
```



### GPIO_otype\(\)

Configures  output type of GPIO pin: Push-Pull / Open-Drain

```c++
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **type**:   PUSH_PULL(0), OPEN_DRAIN(1)

  ​             PP,OD



**Example code**

```c++
GPIO_otype(GPIOA, 5, PP);  // 0: Push-Pull
```



### GPIO_pupdr\(\)

Configures  Pull-up/Pull-down mode of GPIO pin: No Pull-up, Pull-down/ Pull-up/ Pull-down/ Reserved

```c++
void GPIO_pupd(GPIO_TypeDef* Port, int pin, int pupd);
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH

* **pin**:  pin number (int) 0~15

* **pupd**:   NO_PUPD(0), PULL_UP(1), PULL_DOWN(2), RESERVED(3)

  ​             EC_NOPUPD, EC_PU, EC_PD, EC_RE 



**Example code**

```c++
GPIO_pupd(GPIOA, 5, EC_NOPUPD);  // 0: No Pull-up, Pull-down
```



### bittoggle()

Pin toggle function

```c
void bittoggle(GPIO_TypeDef* Port,int pin)
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15

**Example code**

``` c
while(1){
	if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
		bittoggle(GPIOA,LED_PIN);	           // bit toggle function
	}  
	  delay_ms(50);                        //delay for debouncing
	} 
```

### LEDs_toggle()

4 LEDS toggle with state

GPIOA5,6,7,B6

```c
void LEDs_toggle(int state)
```

**Parameters**

* **state:**  0~3 (A5,A6,A7,B6)

**Example code**

``` c
	while(1){
		//output of 4 LEDs according to state
		multled(state);
		if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
				state++;                           //state update
			if(state == 4){                      //There are only 0,1,2,3 states 
			state =0;                            //state reset
		   }
		}
		delay_ms(50);                          //delay for debouncing
	}
```

### LED_init()

4 LEDS toggle init

```c
void LED_init(void)
```

**Parameters**

* **Mode:** output
* **PUPDR:**  Pull-up
* **OTYPE:**  Push pull
* **OSPEED:**  Medium speed

**Example code**

``` c
void setup(void)
{
	LED_init(); 
}
```

### sevenseg_decode()

sevensegment display 0~9 and . with state

GPIOA8,A10,A7,A6,A5,A9,C7,B6

```c
void sevenseg_decode(int number)
```

**Parameters**

* **state:**  0~9

**Example code**

``` c
while(1){
	if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
		bittoggle(GPIOA,LED_PIN);	           // bit toggle function
	}  
	  delay_ms(50);                        //delay for debouncing
	} 
```

### sevenseg_init()

7segment init

GPIOA8,A10,A7,A6,A5,A9,C7,B6

output,No pull up pull down,medium speed,Push pull

```c
void sevenseg_init(void)
```

**Parameters**

* **Mode:** output
* **PUPDR:**  No pull up pull down
* **OTYPE:**  Push pull
* **OSPEED:**  Medium speed

**Example code**

``` c
void setup(void){
sevenseg_init();	 
}
```

### LED_toggle()

LED_Pin(A5) toggle function

```c
void LED_toggle(void)
```

**Example code**

``` c
while(1){
	if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
		LED_toggle();           // bit toggle function
	}  
	  delay_ms(50);                        //delay for debouncing
	} 
```

## ecEXTI.c

### EXTI_init()

EXTI GPIO,pin number, trigger type, priority setting

```c
void EXTI_init(GPIO_TypeDef *Port, int Pin, int trig_type,int priority)
```

**Parameters**

* **Port:**  Port Number,  GPIOA~GPIOH
* **pin**:  pin number (int) 0~15
* **trig_type:** Falling(**FALL**), Rising(**RISE**), and Both(**BOTH**)
* **Priority:** 0-16, smaller number -> Higher priority

**Example code**

```c
EXTI_init(GPIOC,BUTTON_PIN,FALL,0);
```

### EXTI_enable()

EXTI pin interrupt enable

```c
void EXTI_enable(uint32_t pin)
```

**Parameters**

* **pin**:  pin number (int) 0~15

### EXTI_disable()

EXTI pin interrupt disable

```c
void EXTI_disable(uint32_t pin)
```

**Parameters**

* **pin**:  pin number (int) 0~15

### is_pending_EXTI()

Check EXTI pending is ready

must use **clear_pending_EXTI(PIN)**

```c
uint32_t is_pending_EXTI(uint32_t pin)
```

**Parameters**

* **pin**:  pin number (int) 0~15

### clear_pending_EXTI()

Clear EXTI pending 

```c
void clear_pending_EXTI(uint32_t pin)
```

**Parameters**

* **pin**:  pin number (int) 0~15

**Example code**

~~~ c
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){ // when 
		clear_pending_EXTI(BUTTON_PIN); // cleared 
	}
}
~~~



## ecRCC.c

### RCC_HSI_init()

내부 clock 16MHz

```c
void RCC_HSI_init()
```

### RCC_PLL_init()

HSI(16MHz) -> 84MHz 

PLL Clock = [(HSI(16Mhz)/**M** )***N** ]/**P**

현재 세팅: PLL Clock = [(HSI(16Mhz)/8 )*84 ]/2

![image-20221019170555880](C:\Users\MASTER\AppData\Roaming\Typora\typora-user-images\image-20221019170555880.png)

![image-20221019170659626](C:\Users\MASTER\AppData\Roaming\Typora\typora-user-images\image-20221019170659626.png)

### RCC_HSE_init()

외부 clock 8MHz

만들지는 않았음

### RCC_GPOIx_enable()

RCC AHB1 peripheral clock enable

A~E

## ecSysTick.c

### SysTick_init()

PLL 84MHz -> SysTick 1ms setting

NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 16

```c
void SysTick_init(void)
```

**Present setting**

```c
SysTick->LOAD = 84000000 / 1000 - 1;			
//84*10^6/ LOAD = 1000Hz -> 1ms 
NVIC_SetPriority(SysTick_IRQn, 16);		// Set Priority to 16
```

**Example code**

```c
RCC_PLL_init();
SysTick_init();                    // SysTick initialization
```

### SysTick_val()

read SysTick val

```c
uint32_t SysTick_val(void)
```

**Example code**

```c
//시스템 동작 시간 확인용 코드 교재에 있는거 
unsigned int start_time, stop_time, cycle_count;
SysTick->CTRL = 0; // Disable SysTick
SysTick->LOAD = 0xFFFFFFFF; // Set Reload value to maximum
SysTick->VAL = 0; // Clear current value to 0
SysTick->CTRL = 0x5; // Enable SysTick, use processor clock
while(SysTick->VAL != 0); // Wait until SysTick reloaded
start_time = SysTick->VAL; // Get start time
function(); // Execute function to be measured
stop_time = SysTick->VAL; // Get stop time
cycle_count = start_time - stop_time;

//시스템 동작 시간 확인용 코드 
//*********************************************8
uint32_t curTicks = SysTick_val();
function();
uint32_t msTicks = SysTick_val();
uint32_t systemtime = (curTicks-msTicks);
SysTick_reset();
```

### SysTick_reset()

VAL 을 읽어서 시스템의 작동 시간을 잴 때 Val을 reset 할때 사용

```c
void SysTick_reset(void)
```

**Example code**

```c
SysTick_reset();
```

### SysTick_enable()

SysTick Timer enable

```c
void SysTick_enable(void)
```

### SysTick_disable() 

SysTick Timer disable

```c
void SysTick_disable(void)
```

## ecTimer.c

### TIM_INT_init()

현재 세팅: PLL(84MHz) 기준

Enable interrupt, NVIC setting(**2**), 

**inside:** TIM_init(), TIM_period_ms()

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec,int mode)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11
* **ms:**  setting time 한 틱당 얼마의 ms
* **mode:** TIM, PWM

**Example code**

``` c
TIM_INT_init(TIM2,1,TIM) //PLL(84MHz 기준으로 틱 당 1ms upcounter, priority(2)
```

### TIM_init()

현재 세팅

Enable Timer clock,Set CNT period->TIM_period_ms()

CNT Direction(upcounter),Enable Timer counter

```c
void TIM_init(TIM_TypeDef* timerx, uint32_t msec)
```

### TIM_period_us()

HSI와 PLL일 때 둘다 동작하도록 만들어졌음

1Tick 당 몇 us로 할 것인가?

```c
void TIM_period_us(TIM_TypeDef *TIMx, uint32_t usec)
```

**Parameters**

* **prescaler:**  84(현재 수치)       -> 84MHz -> 1MHz (계단의 넓이)
* **ARR:** usec          계단의 개수

**Example code**

```c
TIM_period_us(TIM2, 1) // 틱당 1us
```

### TIM_period_ms()

HSI와 PLL일 때 둘다 동작하도록 만들어졌음

1Tick 당 몇 ms로 할 것인가?

```c
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**

* **prescaler:**  84(현재 수치)       -> 84MHz -> 1MHz (계단의 넓이)
* **ARR:** 10*msec          계단의 개수

**Example code**

```c
TIM_period_ms(TIM2, 1) // 틱당 1ms
```

### TIM_period_ms_PWM()

HSI와 PLL일 때 둘다 동작하도록 만들어졌음

1Tick 당 몇 ms로 할 것인가?

PWM을 위해 psc을 작게 만들었음 -> ARR의 갯수가 많음

```c
void TIM_period_ms_PWM(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**

* **prescaler:**  840(현재 수치)       -> 84MHz -> 100kHz (계단의 넓이)
* **ARR:** 100*msec          계단의 개수

**Example code**

```c
TIM_period_ms(TIM2, 1) // 틱당 1ms
```

### TIM_INT_enable()

TIM interrupt enable

```c
TIM_INT_enable(TIM_TypeDef* timerx)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11

### TIM_INT_disable()

TIM interrupt disable

```c
TIM_INT_disable(TIM_TypeDef* timerx)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11

### is_UIF()

Check TIMER pending is ready

must use **clear_UIF(TIMER)**

```c
is_UIF(TIM_TypeDef *TIMx)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11

### clear_UIF()

Clear TIMER pending 

```c
clear_UIF(TIM_TypeDef *TIMx)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11

### reset_TIMER()

TIMER value를 리셋

```c
reset_TIMER(TIM_TypeDef *TIMx)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11

**Example code**

```c
void EXTI15_10_IRQHandler(void) {  
	if (is_pending_EXTI(BUTTON_PIN)){     //when button pressed
	  PWM_duty(&pwm,0.025);               // set motor angle 0
		i = 1.0;                            // reset angel value
		dir = 0;                            // forward
		reset_TIMER(TIM3);                  // reset TIMer value
		clear_pending_EXTI(BUTTON_PIN);     // cleared by writing '1'
	}
}
```

## ecPWM.c

### PWM_init()

Pin에 따라 채널을 설정해서 PWM을 생성

**inside:** GPIO_init(port, pin, AF);  
		GPIO_ospeed(port, pin,speed);  

​       TIM_init(PWM) // PWM 전용 TIM period

```c
PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin, int DIR,int speed,int msec)
```

**Parameters**

* **pwm:**  typedef struct{
     GPIO_TypeDef *port;
     int pin;
     TIM_TypeDef *timer;
     int ch;
  } PWM_t;
* **port:**  GPIOA~C
* **pin:** 1~15
* **DIR:** UP, DOWN counter
* **Speed:** set GPIO OSPEED SLOW,SMED,SFAST,SHIGH
* **msec:** set PWM TIMer period ms

**Example code**

``` c
PWM_init(&pwm,GPIOA,1,UP,SFAST,1);    // TIM2_CH2(PA1) DOWN clock,FAST,1ms clock 
```

### PWM_period_ms()

PWM의 period setting(ms)

**inside:**TIM_period_ms_PWM(TIMx,msec);  

```c
PWM_period_ms(&pwm,20);	              // set PWM period 20ms
```

**Parameters**

* **pwm:**  typedef struct{
     GPIO_TypeDef *port;
     int pin;
     TIM_TypeDef *timer;
     int ch;
  } PWM_t;
* **msec:** set PWM TIMer period ms

**Example code**

``` c
PWM_init(&pwm,GPIOA,1,UP,SFAST,1);    // TIM2_CH2(PA1) DOWN clock,FAST,1ms clock 
  PWM_period_ms(&pwm,20);	              // set PWM period 20ms
```

### PWM_period_us()

PWM의 period setting(ms)

**inside:**TIM_period_us(TIMx,usec);  

```c
PWM_period_us(&pwm,20);	              // set PWM period 20us
```

**Parameters**

* **pwm:**  typedef struct{
     GPIO_TypeDef *port;
     int pin;
     TIM_TypeDef *timer;
     int ch;
  } PWM_t;
* **usec:** set PWM TIMer period us

### PWM_pulsewidth_ms()

PWM의 pulsewidth setting

```c
PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms)
```

**Parameters**

* **pwm:**  typedef struct{
     GPIO_TypeDef *port;
     int pin;
     TIM_TypeDef *timer;
     int ch;
  } PWM_t;
* **msec:** set TIMer period ms

### PWM_duty()

PWM의 duty 세팅

CCval 값을 바꿔서 duty를 바꿈(0~1)

```c
PWM_duty(PWM_t *pwm, float duty)
```

**Parameters**

* **pwm:**  typedef struct{
     GPIO_TypeDef *port;
     int pin;
     TIM_TypeDef *timer;
     int ch;
  } PWM_t;
* **duty:** set TIMer PWM duty

**Example code**

``` c
PWM_duty(&pwm,0.025+0.05/9.0*i);      // motor angle 0~180
```

## ecStepper.c

### State_full_t FSM_full[4]

FULL mode (A B A' B')

```c
//FULL stepping sequence  - FSM
typedef struct {
	uint8_t out;
  uint32_t next[2];
} State_full_t;

// A B A` B`
State_full_t FSM_full[4] = {  
 {0b1001,{S1,S3}},           //state S0
 {0b1100,{S2,S0}},           //state S1
 {0b0110,{S3,S1}},           //state S2
 {0b0011,{S0,S2}}            //state S3
};
```

**Example code**

```c
GPIO_write(myStepper.port1, myStepper.pin1,(FSM_half[state].out>>3 & 1));		
state = FSM_full[state].next[direction];   
```



### Stepper_init()

stepper motor pin initlization

OUTPUT,No pull up and pull down, push-pull, Fast

```c
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4)
```

**Parameters**

* port: A~D
* pin: 1~13

**Example code**

```c
Stepper_init(GPIOB,10,GPIOB,4,GPIOB,5,GPIOB,3); // Stepper GPIO pin initialization
```

### Stepper_pinOut()

stepper out about state and mode

```c
void Stepper_pinOut (uint32_t state, int mode)
```

**Parameters**

* state: S0~S7(FULL),S0~S3(HALF)
* mode: FULL,HALF

**Example code**

```c
Stepper_pinOut(state, mode);
```

### Stepper_setSpeed ()

convert rpm to msec

```c
void Stepper_setSpeed (long whatSpeed)
```

**Parameters**

* speed: RPM(max 14)

**Example code**

```c
Stepper_setSpeed(14);  
```

### Stepper_step()

 stepper motor opperation

```c
Stepper_step(int steps, int direction,int mode)
```

**Parameters**

* step: stepper motor  step
* direction: CW(1), CCW(0)
* mode: HALF,FULL

**Example code**

```c
Stepper_step(2048, 0, FULL); 
```

### Stepper_stop()

 stepper motor stop

```c
void Stepper_stop (void)
```

**Example code**

```c
Stepper_stop();  
```

## LAB Main code

### GPIO_DIO_LED

Pressed button pin -> LED toggle

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"


//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);
	
int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	//when button pressed LED toggle
	while(1){
	if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
		bittoggle(GPIOA,LED_PIN);	           // bit toggle function
	}  
	  delay_ms(50);                        //delay for debouncing
	} 
}

```

### LAB_GPIO_DIO_multiLED

Pressed button pin  -> 4LEDS toggle

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	int state = 0;                           //state initalization
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//output of 4 LEDs according to state
		multled(state);
		if( GPIO_read(GPIOC,BUTTON_PIN) == 0){ //when button pressed
				state++;                           //state update
			if(state == 4){                      //There are only 0,1,2,3 states 
			state =0;                            //state reset
		   }
		}
		delay_ms(50);                          //delay for debouncing
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();
	SysTick_init();                       // for delay
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	and LED pin mode -> output
	GPIO_init(GPIOA, 6, OUTPUT);          // calls RCC_GPIOA_enable()	and 6 pin mode -> output
	GPIO_init(GPIOA, 7, OUTPUT);          // calls RCC_GPIOA_enable()	and 7 pin mode -> output
	GPIO_init(GPIOB, 6, OUTPUT);          // calls RCC_GPIOB_enable() and 6 pin mode -> output
  
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	GPIO_pupdr(GPIOA, LED_PIN, EC_PU);    // GPIOA LED pin pupdr -> pull up
	GPIO_pupdr(GPIOA, 6, EC_PU);          // GPIOA pin 6 pupdr -> pull up
	GPIO_pupdr(GPIOA, 7, EC_PU);          // GPIOA pin 7 pupdr -> pull up
	GPIO_pupdr(GPIOB, 6, EC_PU);          // GPIOB pin 6 pupdr -> pull up
	 
	GPIO_otype(GPIOA, LED_PIN, PP);       // GPIOA LED pin otype -> push-pull
	GPIO_otype(GPIOA, 6, PP);             // GPIOA 6 pin otype -> push-pull
	GPIO_otype(GPIOA, 7, PP);             // GPIOA 7 pin otype -> push-pull
	GPIO_otype(GPIOB, 6, PP);             // GPIOB 6 pin otype -> push-pull
	
	GPIO_ospeed(GPIOA,LED_PIN,SMED);      // GPIOA LED pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,6,SMED);            // GPIOA 6 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOA,7,SMED);            // GPIOA 7 pin ospeed -> Medium speed
	GPIO_ospeed(GPIOB,6,SMED);            // GPIOB 6 pin ospeed -> Medium speed
}
```

### LAB_GPIO_7segment

Pressed button pin -> 7segment update (0~9)

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecSysTick.h"

//define the led pin number and button pin number
#define LED_PIN 	5
#define BUTTON_PIN 13

void setup(void);


int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	unsigned int cnt = 0;
	// Inifinite Loop ----------------------------------------------------------
	while(1){
		//sevensegment output
		sevenseg_decode(cnt % 10);                     //not to make over 10
		if(GPIO_read(GPIOC, BUTTON_PIN) == 0) cnt++;   //if button pressed 7segment output up(0~9)
		if (cnt > 9) cnt = 0;                          //over 10 -> 0
		delay_ms(50);                                  //delay for debouncing
	}
}


// Initialiization 
void setup(void)
{
	RCC_HSI_init();	
	SysTick_init();
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	sevenseg_init();	                    // 7segment init,otype,ospeed,pupdr   
}
```

### LAB_EXTI

Pressed button pin -> 4LED toggle using EXTI interrupt

```c
#include "stm32f4xx.h"
#include "ecRCC.h"
#include "ecGPIO.h"
#include "ecEXTI.h"

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
	GPIO_init(GPIOC, BUTTON_PIN, INPUT);  // calls RCC_GPIOC_enable() and button pin mode -> input
	GPIO_pupdr(GPIOC, BUTTON_PIN, EC_PU); // GPIOC button pin pupdr -> pull up
	EXTI_init(GPIOC,BUTTON_PIN,FALL,0);   //EXTI button PIN -> trigger type(falling),propriority(0)
}
```

### LAB_PWM_RCmotor

angle 0 to 180 and back to 0 500ms period(moves angle 10) and reset angle 0 when button pressed

```c
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
```

### LAB_Stepper_Motor

2048step의 stepper motor를 CW(1),CCW(0), MODE(HALF,FULL)

speed(RPM) max 14 등을 조절하여 stepper motor를 동작할 수 있다.

```c
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
```

### GPIO_DIO_LED

Using SysTick tick(1ms) -> 1초마다 7segment update(0~9)

Pressed button pin -> 7segment reset(0)

``` c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecSysTick.h"
#include "ecEXTI.h"

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

### TU_TIMER_Interrupt

Using TIMER interrupt(1ms) -> 1s LED toggle

```c
#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecRCC.h"
#include "ecTIM.h"

uint32_t count=0;
uint32_t count1=0;
#define LED_PIN 	5

	
void setup(void);

int main(void) { 
	// Initialiization --------------------------------------------------------
	setup();
	// Inifinite Loop ----------------------------------------------------------
	while(1){}
} 

// Initialiization 
void setup(void)
{	
	RCC_PLL_init();                       // System Clock = 84MHz
	//RCC_PLL_HSE_init();
	GPIO_init(GPIOA, LED_PIN, OUTPUT);    // calls RCC_GPIOA_enable()	
  GPIO_init(GPIOA,6,OUTPUT);
	TIM_INT_init(TIM2,1);
}

void TIM2_IRQHandler(void){
		//Create the code to toggle LED by 1000ms
	if(is_UIF(TIM2)){	
	count++;
		count1++;
		if(count >1000){
			LED_toggle();
			count = 0;
		}
		//if(count1 >2000){
		//	bittoggle(GPIOA,6);
		//	count1 = 0;
	//	}
		clear_UIF(TIM2);    // clear by writing 0
	}
}
```





