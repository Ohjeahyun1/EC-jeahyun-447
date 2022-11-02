/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-02 	
* @brief   Embedded Controller - ecStepper.c
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecSysTick.h"
			
#ifndef __EC_STEPPER_H
#define __EC_STEPPER_H

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

//State mode
#define HALF 0
#define FULL 1	 
	 
/* Stepper Motor */
//stepper motor function

typedef struct{
   GPIO_TypeDef *port1;
   int pin1;
	 GPIO_TypeDef *port2;
   int pin2;
	 GPIO_TypeDef *port3;
   int pin3;
	 GPIO_TypeDef *port4;
   int pin4;
	 int _step_num;
} Stepper_t;

// initlization stepper motor pin (output,NO PUPD,pushpull,Fast)
void Stepper_init(GPIO_TypeDef* port1, int pin1, GPIO_TypeDef* port2, int pin2, GPIO_TypeDef* port3, int pin3, GPIO_TypeDef* port4, int pin4);
// convert rpm to msec
void Stepper_setSpeed(long whatSpeed);
// stepper motor opperation
void Stepper_step(int steps, int direction, int mode); 
// stepper motor stop function
void Stepper_stop(void);

#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
