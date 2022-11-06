/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-28 	
* @brief   Embedded Controller:  ecPWM.h
* 
******************************************************************************
*/

#include "stm32f411xe.h"
#include "ecGPIO.h"
#include "ecTIM.h"  			

#ifndef __EC_PWM_H
#define __EC_PWM_H

// DIR UP 0 DOWN 1
#define UP  0x0
#define DOWN 0x1

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

/* PWM STRUCTURE */

typedef struct{
   GPIO_TypeDef *port;
   int pin;
   TIM_TypeDef *timer;
   int ch;
} PWM_t;

/* PWM initialization */

/* PWM Configuration */
void PWM_init(PWM_t *pwm, GPIO_TypeDef *port, int pin,int DIR,int speed,int otype,int pupdr,int msec);


/* PWM PERIOD SETUP */

// allowable range for msec:  1~2,000
void PWM_period_ms(PWM_t *pwm,  uint32_t msec);		
// allowable range for usec:  1~1,000
void PWM_period_us(PWM_t *PWM_pin, uint32_t usec);  


/* DUTY RATIO SETUP */


// High Pulse width in msec
void PWM_pulsewidth_ms(PWM_t *pwm, float pulse_width_ms);
void PWM_pulsewidth_us(PWM_t *pwm, float pulse_width_us);

// Duty ratio 0~1.0
void PWM_duty(PWM_t *pwm, float duty);
// PWM pin map
void PWM_pinmap(PWM_t *PWM_pin);



#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif
