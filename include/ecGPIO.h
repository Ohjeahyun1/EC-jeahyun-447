/*----------------------------------------------------------------\
@ Embedded Controller by Young-Keun Kim - Handong Global University
Author           : Oh jeahyun
Created          : 05-03-2021
Modified         : 10-01-2022
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

#define HIGH 1
#define LOW  0

#define LED_PIN 	5
#define BUTTON_PIN 13

//OSPEED Low speed(00), Medium speed(01), Fast speed(10), High speed(11)
#define SLOW 0x00
#define SMED 0x01
#define SFAST 0x10
#define SHIGH 0x11

//OTYPER Output push-pull (reset state), 1: Output open-drain
#define PP 0
#define OD 1

//PUPDR 00: No pull-up, pull-down , 01: Pull-up ,10: Pull-down , 11: Reserved
#define EC_NOPUPD 0x00
#define EC_PU 0x01
#define EC_PD 0x10
#define EC_RE 0x11


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
void bittoggle(GPIO_TypeDef* Port,int pin);
int button_pressed(GPIO_TypeDef* Port, int buttonpin);
void multled(int state);	
void sevenseg_init(void);
void sevenseg_decode(int number);


 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
