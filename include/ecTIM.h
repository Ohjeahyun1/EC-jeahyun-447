/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-11-06 	
* @brief   Embedded Controller:  ec.Tim.h
* 
******************************************************************************
*/



#ifndef __EC_TIM_H 
#define __EC_TIM_H
#include "stm32f411xe.h"


//MODE (TIM,PWM)
#define TIM   0
#define PWM   1


#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */


/* Timer Configuration */

// Timer Period setup
void TIM_init(TIM_TypeDef *timerx, uint32_t msec,int mode);     //Timer initlization with mode(PWM,TIM)

void TIM_period_ms(TIM_TypeDef* TIMx, uint32_t msec);           //Timer period setting ms
void TIM_period_ms_PWM(TIM_TypeDef* TIMx, uint32_t msec);       //Timer period setting PWM
void TIM_period_ms_PWM_buzzer(TIM_TypeDef* TIMx, uint32_t msec);
void TIM_period_us(TIM_TypeDef* TIMx, uint32_t msec);           //Timer period setting us
void TIM_period_test(TIM_TypeDef* TIMx, uint32_t msec);         //Timer period setting ms for test

// Timer Interrupt setup
void TIM_INT_init(TIM_TypeDef* timerx, uint32_t msec,int mode); //Timer interrupt initlization
void TIM_INT_enable(TIM_TypeDef* timerx);                       //Timer interrupt enable
void TIM_INT_disable(TIM_TypeDef* timerx);                      //Timer interrupt disable

// Timer Interrupt Flag 
uint32_t is_UIF(TIM_TypeDef *TIMx);                             // Check Timer interrupt
void clear_UIF(TIM_TypeDef *TIMx);                              // clear Timer interrupt pending

// Timer value reset
void reset_TIMER(TIM_TypeDef *TIMx);                            //Timer value reset




/* Input Capture*/

// Edge Type
#define IC_RISE 0
#define IC_FALL 1
#define IC_BOTH 2

//Input Capture

typedef struct{
	GPIO_TypeDef *port;
	int pin;   
	TIM_TypeDef *timer;
	int ch;  		//int Timer Channel
	int ICnum;  //int IC number
} IC_t;

void ICAP_init(IC_t *ICx, GPIO_TypeDef *port, int pin,int pupdr); // Initialize input capture mode (default setting)
void ICAP_setup(IC_t *ICx, int IC_number, int edge_type);         // Setup ICn and Edge type
void ICAP_counter_us(IC_t *ICx, int usec);                        // ICAP counter step time as us

uint32_t is_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);              // flag check Timer ch
void clear_CCIF(TIM_TypeDef *TIMx, uint32_t ccNum);               // flag clear Timer ch

void ICAP_pinmap(IC_t *timer_pin);                                //DO NOT MODIFY THIS



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif






