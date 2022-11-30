/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Oh jeahyun
Created          : 05-03-2021
Modified         : 11-11-2022
Language/ver     : C++ in Keil uVision

Description      : Distributed to Students for LAB_GPIO
/----------------------------------------------------------------*/


#include "stm32f411xe.h"
#include "ecRCC.h"

#ifndef __ECGPIO_H
#define __ECGPIO_H

// MODER Input(00), Output(01), AlterFunc(10), Analog(11)
#define INPUT  0x00
#define OUTPUT 0x01
#define AF     0x02
#define ANALOG 0x03

// IDR & ODR
#define HIGH 1
#define LOW  0

// PIN
#define LED_PIN 	5
#define BUTTON_PIN 13

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
void GPIO_write(GPIO_TypeDef *Port, int pin, int Output);
int  GPIO_read(GPIO_TypeDef *Port, int pin);
void GPIO_mode(GPIO_TypeDef* Port, int pin, int mode);
void GPIO_ospeed(GPIO_TypeDef* Port, int pin, int speed);
void GPIO_otype(GPIO_TypeDef* Port, int pin, int type);
void GPIO_pupdr(GPIO_TypeDef* Port, int pin, int pupdr);
void GPIO_all_init(GPIO_TypeDef *Port, int pin, int mode,int pupdr,int otype,int ospeed);
void LED_test_init(void);
void LEDs_test_toggle(int state);
void sevenseg_test_init(void);
void sevenseg_test_decode(int number);
void sevenseg_test_decode1(void);
void LEDs_test_down(void);
void bittoggle(GPIO_TypeDef* Port,int pin);
void sevenseg_init(void);
void sevenseg_decode(int number);
void LED_init(void);
void LEDs_toggle(int state);
void LED_toggle(void);

void LED_Quiz1(void);
void seven_Quiz1(void);
void LEDs_Quiz1(int state);
void sevenseg_Quiz1(int number);

//GPIO setting
typedef struct {
	GPIO_TypeDef *port;                //example: GPIO_write(extLED[0].port, extLED[0].pin,....);
  int pin;
} _Pin;



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
