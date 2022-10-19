/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-19   	
* @brief   Embedded Controller - ecSysTick.h
* 
******************************************************************************
*/


#ifndef __EC_SYSTICK_H
#define __EC_SYSTICK_H

#include "stm32f4xx.h"
#include "ecRCC.h"
#include <stdint.h>

#ifdef __cplusplus
 extern "C" {
#endif /* __cplusplus */

void SysTick_init(void);
void SysTick_Handler(void);
void SysTick_counter();
void delay_ms(uint32_t msec);
void SysTick_reset(void);
uint32_t SysTick_val(void);
void buttonpushed(void);
void SysTick_enable(void);
void SysTick_disable(void); 

	 
#ifdef __cplusplus
}
#endif /* __cplusplus */

#endif