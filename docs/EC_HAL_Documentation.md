---
description: EC_HAL API
---


# Embedded Controller - STM32f411 HAL API

Documentation for HAL functions

Written by:   오재현

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
void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec)
```

**Parameters**

* **TIMER:**  TIM1~5,9,10,11
* **ms:**  setting time 한 틱당 얼마의 ms

**Example code**

``` c
TIM_INT_init(TIM2,1) //PLL(84MHz 기준으로 틱 당 1ms upcounter, priority(2)
```

### TIM_init()

현재 세팅

Enable Timer clock,Set CNT period->TIM_period_ms()

CNT Direction(upcounter),Enable Timer counter

```c
void TIM_init(TIM_TypeDef* timerx, uint32_t msec)
```

### TIM_period_ms()

Timer setting (PLL) ms

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



