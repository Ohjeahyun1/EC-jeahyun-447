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

![image-20221111124328208](C:\Users\MASTER\AppData\Roaming\Typora\typora-user-images\image-20221111124328208.png)

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

### ICAP_init()

InputCapture init

GPIO pin 설정, (AF,Push-pull,High)

```c
typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  //int IC number
} IC_t;

void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin,int pupdr)
```

**Parameters**

* **Port:**  GPIOA-C
* **Pin:** 1-15
* **TIMER:**  TIM1~5,9,10,11
* **Ch:**  1-4 (int Timer channel)
* **ICnum:**  1-4

**Example code**

```c
IC_t echo;												                                // Input Capture for echo
	ICAP_init(&echo,GPIOB,10,EC_NOPUPD);    		                      // PB10 as input caputre
	ICAP_counter_us(&echo, 10);   		                                // ICAP counter step time as 10us
	ICAP_setup(&echo, 3, IC_RISE);                                  	// TIM2_CH3 as IC3 , rising edge detect
	ICAP_setup(&echo,4,IC_FALL);                                    	// TIM2_CH3 as IC4 , falling edge detect
	
```

### ICAP_setup()

Input Capture

TI -> IC 

Rising.Falling,Both (IC_RISE,IC_FALL,IC_BOTH)

```c
void ICAP_setup(IC_t *ICx, int ICn, int edge_type)
```

**Example code**

```c
ICAP_setup(&echo, 3, IC_RISE);                                  	// TIM2_CH3 as IC3 , rising edge detect
```

### ICAP_counter_us()

ICAP counter step time as us

```c
void ICAP_counter_us(IC_t *ICx, int usec)
```

**Example code**

```c
ICAP_counter_us(&echo, 10);   		                                // ICAP counter step time as 10us
```

### is_CCIF()

check the flag

```c
uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum)
```

**Parameters**

* ccNum: ch1-4

**Example code**

```c
if(is_CCIF(TIM2, 3))
```

### clear_CCIF()

flag clear

```c
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum)
```

**Parameters**

* ccNum: Ch1-4

**Example code**

```c
clear_CCIF(TIM2, 3);                 	                          // clear capture/compare interrupt flag 
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

## ecUART.c

### UART2_init()

UART2 init (just use pc <->mcu) with bluetooth

```c
void UART2_init()
```

**Example code**

```c
UART2_init();
```

### USART_write();

same code   prinf("c",pcData);

```c
void USART_write(USART_TypeDef* USARTx, uint8_t* buffer, uint32_t nBytes);
```

**Parameters**

* USARTx: USART1-6
* buffer: ex) pcData
* nBytes: 몇개의 문자인지.

**Example code**

```c
USART_write(USART2, &pcData, 1);
```

### USART_delay()

UART2 init (just use pc <->mcu) with bluetooth

```c
void USART_delay(uint32_t us)
```

**Parameters**

* us: delay us time

**Example code**

```c
USART_delay(300);
```

### USART_begin()

USART init 원하는 핀으로

```c
void USART_begin(USART_TypeDef* USARTx, GPIO_TypeDef* GPIO_TX, int pinTX, GPIO_TypeDef* GPIO_RX, int pinRX, int baud);
```

**Parameters**

* USARTx: USART1,2,6
* GPIO_TX: Tx 하려는 GPIOA-C
* pinTx: Tx pin number 0-15
* GPIO_RX: Rx 하려는 GPIOA-C
* pinRx: Rx pin number 0-15
* baud: 통신단위(기본값 9600)

**Example code**

```c
USART_begin(USART1, GPIOA, 9, GPIOA, 10, 9600); 	// PA9 - TXD , PA10 - RXD
```

### USART_begin()

USART init 원하는 USART로 핀 번호는 고정

USART1: Tx ->B6 , Rx ->B3

USART2: Tx ->A2 , Rx ->A3

USART6: Tx ->A11 , Rx ->A12

```c
void USART_init(USART_TypeDef* USARTx, int baud);
```

**Parameters**

* USARTx: USART1,2,6
* baud: 통신단위

**Example code**

```c
USART_init(USART2, 9600);
```

### USART_getc()

USART에서 원하는 값을 가져오는 것

```c
uint8_t USART_getc(USART_TypeDef * USARTx);	
```

**Parameters**

* USARTx: USART1,2,6

**Example code**

```c
mcu2Data = USART_getc(USART1);
pcData = USART_getc(USART2);
```

### is_USART_RXNE()

USART Flag

```c
uint32_t is_USART_RXNE(USART_TypeDef * USARTx);
```

**Parameters**

* USARTx: USART1,2,6

**Example code**

```c
if(is_USART_RXNE(USART1)){
    
}
```

### USARTx_IRQHandler()

USART handler

```c
void USART1_IRQHandler(){
    
}
```

**Example code**

```c
void USART1_IRQHandler(){         //USART1 INT 
	if(is_USART_RXNE(USART1)){
		mcu2Data = USART_getc(USART1);
		printf("received: %c\r\n", mcu2Data);
	}
}


void USART2_IRQHandler(){         //USART2 INT 
	if(is_USART_RXNE(USART2)){
		pcData = USART_getc(USART2);
		USART_write(USART1, &pcData, 1);
		printf("%c", pcData);
		
		if (pcData == END_CHAR)
			printf("\r\n");
	}
}
```

## ecADC.c

### ADC_init()

ADC init

0: SW, 1=TRGO

```c
void ADC_init(GPIO_TypeDef *port, int pin, int trigmode);	
```

**Parameters**

* port: GPIOA-C
* pin: 0-15
* trigmode:     0: SW, 1=TRGO

**Example code**

```c
 ADC_init(GPIOB, 0, TRGO);
```

### ADC_continue()

choose mode(single conversion,continuous conversion mode)

```c
void ADC_continue(int contmode);
```

**Parameters**

* contmode: SW,TRGO

**Example code**

```c
ADC_init(GPIOA,1,SW);
	ADC_continue(CONT);
	ADC_start();
```

### ADC_TRGO()

ADC_init() trigmode가 TGRO일 시에 자동으로 실행

```c
void ADC_TRGO(TIM_TypeDef* TIMx, int msec, int edge);
```

**Parameters**

* Timx: TIM2,3
* msec: Timer sec
* edge: RISE(defalut)

**Example code**

```c
// TRGO Initialize : TIM2, 1msec, RISE edge
	if(trigmode == TRGO) ADC_TRGO(TIM2, 1, RISE);
```

### ADC_sequence()

ADC 순서 정하기

```c
void ADC_sequence(int length, int *seq); 
```

**Parameters**

* length: ADC 채널 갯수
* seq: 채널 

**Example code**

```c
int seqCHn[16] = {8,9,};

ADC_sequence(2, seqCHn);
```

### ADC_start()

ADC start

```c
void ADC_start(void);
```

**Example code**

```c
// ADON, SW Trigger enable
ADC_start();     
```

### is_ADC_EOC()

Regular channel end of conversion

regular channel flag

```c
uint32_t is_ADC_EOC(void);
```

**Example code**

```c
if(is_ADC_EOC()){
    
}
```

### ADC_read()

ADC의 값을 읽기

```c
uint32_t ADC_read(void);
```

**Example code**

```c
if (flag==0){ 
				IR1 = ADC_read();                                            // read ADC IR1
			}  
			else if (flag==1){
				IR2 = ADC_read();                                            // read ADC IR2
			}
```

### is_ADC_OVR()

ADC의 overrun flag 

```c
uint32_t is_ADC_OVR(void);
```

**Example code**

```c
if((is_ADC_OVR())){                                                // ADC over 
		clear_ADC_OVR();                                                 //clear adc sr
	}
```

### clear_ADC_OVR()

ADC의 overrun flag clear

```c
void clear_ADC_OVR(void);
```

**Example code**

```c
	if((is_ADC_OVR())){                                                // ADC over 
		clear_ADC_OVR();                                                 //clear adc sr
	}
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

### LAB_Timer_Inputcapture_Ultrasonic

PWM을 50ms period, pulse width 10us  PA6(Trig)

PB10(echo) 으로 Ultrasonic distance sensor를 사용해서

거리 측정

```c
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

```

### LAB_Final_main1

Smart home

```c
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
```



### LAB_Final_autoparkingcar

Auto parking car

```c
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

### LAB_ADC_IR

IR sensor 로 라인 트레이싱 + distance 측정후 가까우면 정지 -> 물체가 사라지면 다시 트레이싱

```c
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
// read IR sensors
void ADC_IRQHandler(void){
	if((is_ADC_OVR())){                                                // ADC over 
		clear_ADC_OVR();                                                 //clear adc sr
	}
	
	if(is_ADC_EOC()){                                                  //after finishing sequence
			if (flag==0){ 
				IR1 = ADC_read();                                            // read ADC IR1
			}  
			else if (flag==1){
				IR2 = ADC_read();                                            // read ADC IR2
			}
		flag =! flag;
	}
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



