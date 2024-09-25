#ifndef  _SYSTEM_MM32_C_
#define _SYSTEM_MM32_C_
#include "mm32_device.h"


#define SYSCLK_HSI_72MHz  72000000
u32 SystemCoreClock = SYSCLK_HSI_72MHz;
static void SetSysClockTo72_HSI(void);



void SystemInit (void)
{

  //Reset the RCC clock configuration to the default reset state(for debug purpose)
  //Set HSION bit
  RCC->CR |= (u32)0x00000001;

  //Reset SW, HPRE, PPRE1, PPRE2, ADCPRE and MCO bits
  RCC->CFGR &= (u32)0xF8FFC00C;

  //Reset HSEON, CSSON and PLLON bits
  RCC->CR &= (u32)0xFEF6FFFF;

  //Reset HSEBYP bit
  RCC->CR &= (u32)0xFFFBFFFF;

  //Reset PLLSRC, PLLXTPRE, PLLMUL and USBPRE/OTGFSPRE bits
  RCC->CFGR &= (u32)0xFF3CFFFF;
	
  RCC->CR &= (u32)0x008FFFFF;

  //Disable all interrupts and clear pending bits
  RCC->CIR = 0x009F0000;

  //Configure the System clock frequency, HCLK, PCLK2 and PCLK1 prescalers
  //Configure the Flash Latency cycles and enable prefetch buffer
	SetSysClockTo72_HSI();
}

void SetSysClockTo72_HSI(void)
{

  RCC->CR |= RCC_CR_HSION;
  //RCC->CR = RCC_CR_HSION | (0x01U<<20);
  while(!(RCC->CR & RCC_CR_HSIRDY));
  SystemCoreClock         = SYSCLK_HSI_72MHz;
  //Enable Prefetch Buffer
  FLASH->ACR |= FLASH_ACR_PRFTBE;
  //Flash 0 wait state ,bit0~2
  FLASH->ACR &= ~0x07;
  u32 temp = (SystemCoreClock - 1) / 24000000;
  FLASH->ACR |= temp;
  //HCLK = SYSCLK
  RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
  //PCLK2 = HCLK
  RCC->CFGR |= RCC_CFGR_PPRE2_DIV1;

	RCC->CR |= 0x01U<<20;// enable 72mhz HSI (HSI_72M_EN)
	//Select HSI as system clock source
  RCC->CFGR &= ~(RCC_CFGR_SW);
  RCC->CFGR |= RCC_CFGR_SW_PLL;

  //Wait till HSI is used as system clock source
  //while ((RCC->CFGR & (u32)RCC_CFGR_SWS) != (u32)0x08); 
  while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL) {};   
}

#endif
