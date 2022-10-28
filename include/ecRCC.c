/**
******************************************************************************
* @author  Oh jeahyun
* @Mod		 2022-10-19   	
* @brief   Embedded Controller - ecRCC.c
* 
******************************************************************************
*/


#include "stm32f4xx.h"
#include "ecRCC.h"

volatile int EC_SYSCLK=16000000;

void RCC_HSI_init() {
	// Enable High Speed Internal Clock (HSI = 16 MHz)
  //RCC->CR |= ((uint32_t)RCC_CR_HSION);
	RCC->CR |= 0x00000001U;
	
  // wait until HSI is ready
  //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
	while ( (RCC->CR & 0x00000002U) == 0 ) {;}
	
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW); 								// not essential
  RCC->CFGR |= (uint32_t)RCC_CFGR_SW_HSI; 								//00: HSI16 oscillator used as system clock

	// Wait till HSI is used as system clock source
  while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != 0 );
		   
	//EC_SYSTEM_CLK=16000000;
		//EC_SYSCLK=16000000;
		EC_SYSCLK=16000000;
}

void RCC_HSE_init() {
   
   // Enable High Speed Internal Clock (HSI = 16 MHz)
  //RCC->CR |= ((uint32_t)RCC_CR_HSION); 
   RCC->CR |= 1<<16;                                                                  //1. HSE oscilator ??, OFF:0 ON:1
   
  // wait until HSE is ready
  //while ( (RCC->CR & (uint32_t) RCC_CR_HSIRDY) == 0 ) {;}
   while ( (RCC->CR & (1<<17)) == 0 ) {;}                                  //2. HSERDY? 1? ?????? ????, READY:1, NOT READY:0
   
  // Select HSI as system clock source 
  RCC->CFGR &= (uint32_t)(~RCC_CFGR_SW);                                  //3. System clock switch clear ?? SW1,SW0? 00 ?, ~0x3<<0
  RCC->CFGR |= (uint32_t) 0x1;                                                 //4. HSE? system clock?? ?? SW1,SW0? 01 ?? clear ???? |=1? set

   // Wait till HSI is used as system clock source
  while ((RCC->CFGR & (uint32_t)0x1) != 1 );                            //5. HSE? system clock switch? ??, SWS1,SWS0? 01? ???? ???? (0x1<<0)
         
     //EC_SYSTEM_CLK=16000000;
      //EC_SYSCLK=16000000;
   EC_SYSCLK=8000000;                                                               //6. HSE: 8MHz? ?? ? ? ??.
}

void RCC_PLL_HSE_init() {   
   // To correctly read data from FLASH memory, the number of wait states (LATENCY) 
  // must be correctly programmed according to the frequency of the CPU clock
  // (HCLK) and the supply voltage of the device.      
   FLASH->ACR &= ~FLASH_ACR_LATENCY;                     //1.FLASH memory ???? ??? FLASH->ACR &=~(0x7<<0)  (clear)
   FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;               //2.FLASH memory ???? ??? FLASH->ACR   |=0x2 (2 wait states)
   
   // Enable the External High Speed oscillator (HSE)
   RCC->CR |= RCC_CR_HSEON;                                 //3. HSE oscilator ??, OFF:0 ON:1
   while ( (RCC->CR & (1<<17)) == 0 ) {;}            //4. HSERDY? 1? ?????? ????, READY:1, NOT READY:0
   
   // Disable PLL for configuration
   RCC->CR    &= ~RCC_CR_PLLON;                           //5. PLL? ??? ?? PLLOFF: 0, PLLON: 1, ~(0x1<<24)
   
   // Select clock source to PLL
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC;                //6. PLL? ?? ?? clock source beat ??? ~(0x1<<2)
   RCC->PLLCFGR |= (1<<22);                                  //7. PLL? ?? ?? clock source HSE? ??  (0<<22) HSI:0, HSE:1
   
   // Make PLL as 84 MHz                                                                                                                  //8. PLL? 84MHz? ???
   // f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 8MHz * 84/4 = 168 MHz                              //9. clock input? PLLN??? PLLM ??? 168MHz 
   // f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz                                                                  //10.PLL? ?? clock? PLLP? ????? 84MHz
   RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6;         //PLLN? 50?? 432?? ??            //11.PLLN? 84? ?? clear? set? ?? (RCC->PLLCFGR & ~(0?1<<6)) | 84U << 6;   
   RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 4U ;                //PLLM  2?? 63??      ??            //12.PLLM? 8? ??  clear? set? ?? (RCC->PLLCFGR & ~(0?3<<0)) | 8U ;          
   RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8      //13.PLLP? 2? ??  
    
   
   // Enable PLL after configuration
   RCC->CR   |= RCC_CR_PLLON;                                     //14.RCC->CR |= 1<<24;   PLLON:1, PLLOFF:0
   while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);            //15.while((RCC->CR & (1<<25))>>25 != 0) PLL locked: 1, PLL unlocked: 0 locked???? ????
   
   // Select PLL as system clock
   RCC->CFGR &= ~RCC_CFGR_SW;                                    //16.RCC->CFGR &= ~(3<<0); clear first
   RCC->CFGR |= RCC_CFGR_SW_PLL;                     //17.RCC->CFGR |= 1<<1;      SW1, SW0 00: HSI, 01:HSE, 10:PLL, 11:not applicable 
   
   // Wait until System Clock has been selected               
   while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);            //18.while ((RCC->CFGR & (0x3<<2)) != 8UL) SW1, SW0 00: HSI, 01:HSE, 10:PLL, 11:not applicable
   
   // The maximum frequency of the AHB and APB2 is 100MHz,
   // The maximum frequency of the APB1 is 50 MHz.
   RCC->CFGR &= ~RCC_CFGR_HPRE;     //&=~(15<<4); // AHB prescaler = 1; SYSCLK not divided (84MHz) 0000:not divided, 1000:2????, 1001:4? ???,...1111: 512? ???                            
   RCC->CFGR &= ~RCC_CFGR_PPRE1;    //&=~(7<<10); // APB-high-speed prescaler (APB1) clear first   APB1 0xx: AHB not divided, 100: 2? ???, 101: 4? 110: 8? ???, 111: 16?? ??? 
   RCC->CFGR |=  RCC_CFGR_PPRE1_2;   //|=(4<<10)      // APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)    
   RCC->CFGR &= ~RCC_CFGR_PPRE2;   //&=~(7<<13); // APB high-speed prescaler (APB2) = 1, HCLK not divided   (84MHz) 000: not divided, 100: 2? ???, 101:4? ???,...111:16?? ???
   
   EC_SYSCLK=84000000;
}

void RCC_PLL_init() {	
	// To correctly read data from FLASH memory, the number of wait states (LATENCY)
  // must be correctly programmed according to the frequency of the CPU clock
  // (HCLK) and the supply voltage of the device.		
	FLASH->ACR &= ~FLASH_ACR_LATENCY;
	FLASH->ACR |=  FLASH_ACR_LATENCY_2WS;
		
	// Enable the Internal High Speed oscillator (HSI)
	RCC->CR |= RCC_CR_HSION;
	while((RCC->CR & RCC_CR_HSIRDY) == 0);
	
	// Disable PLL for configuration
	RCC->CR    &= ~RCC_CR_PLLON;
	
	// Select clock source to PLL
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLSRC; 		// Set source for PLL: clear bits
	RCC->PLLCFGR |= RCC_PLLCFGR_PLLSRC_HSI; // Set source for PLL: 0 =HSI, 1 = HSE
	
	// Make PLL as 84 MHz
	// f(VCO clock) = f(PLL clock input) * (PLLN / PLLM) = 16MHz * 84/8 = 168 MHz
	// f(PLL_R) = f(VCO clock) / PLLP = 168MHz/2 = 84MHz
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLN) | 84U << 6;
	RCC->PLLCFGR = (RCC->PLLCFGR & ~RCC_PLLCFGR_PLLM) | 8U ; 
	RCC->PLLCFGR &= ~RCC_PLLCFGR_PLLP;  // 00: PLLP = 2, 01: PLLP = 4, 10: PLLP = 6, 11: PLLP = 8	
	
	
	// Enable PLL after configuration
	RCC->CR   |= RCC_CR_PLLON; 
	while((RCC->CR & RCC_CR_PLLRDY)>>25 != 0);
	
	// Select PLL as system clock
	RCC->CFGR &= ~RCC_CFGR_SW;
	RCC->CFGR |= RCC_CFGR_SW_PLL;
	
	// Wait until System Clock has been selected
	while ((RCC->CFGR & RCC_CFGR_SWS) != 8UL);
	
	// The maximum frequency of the AHB and APB2 is 100MHz,
	// The maximum frequency of the APB1 is 50 MHz.
	RCC->CFGR &= ~RCC_CFGR_HPRE;  		// AHB prescaler = 1; SYSCLK not divided (84MHz)
	RCC->CFGR &= ~RCC_CFGR_PPRE1; 		// APB high-speed prescaler (APB1) = 2, HCLK divided by 2 (42MHz)
	RCC->CFGR |=  RCC_CFGR_PPRE1_2;
	RCC->CFGR &= ~RCC_CFGR_PPRE2; 		// APB high-speed prescaler (APB2) = 1, HCLK not divided	(84MHz)
	
	EC_SYSCLK=84000000;
}


void RCC_GPIOA_enable()
{
	// HSI is used as system clock         
	//RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
}

void RCC_GPIOB_enable()
{
	// HSI is used as system clock         
	//RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

void RCC_GPIOC_enable()
{
	// HSI is used as system clock     
	//RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;
}

void RCC_GPIOD_enable()
{
	// HSI is used as system clock         
	//RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
}

void RCC_GPIOE_enable()
{
	// HSI is used as system clock         
	//RCC_HSI_init();
	// RCC Peripheral Clock Enable Register 
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN;
}