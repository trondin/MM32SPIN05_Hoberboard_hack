#include "setup.h"
#include "config.h"
#include "util.h"
#include "defines.h"
#include "BLDC_controller.h"           // Model's header file 



extern RT_MODEL *const rtM;

extern P    rtP; 
extern DW   rtDW;             // Observable states 
extern ExtU rtU;              // External inputs 
extern ExtY rtY;              // External outputs 


static int16_t pwm_margin;   // This margin allows to have a window in the PWM signal for 
                             // proper FOC Phase currents measurement 
extern uint8_t ctrlModReq;
volatile int pwmr = 0;

// =========================================================
//  system
// =========================================================
static uint32_t DelayCnt;
uint32_t millis;
extern uint32_t SystemCoreClock;
volatile adc_buf_t adc_buffer;

void delay_Init(void)
{
	millis = 0;
  SysTick_Config(SystemCoreClock / 1000);
  NVIC_SetPriority(SysTick_IRQn, 0x0);
}


void SysTick_Handler(void)  // interruption
{
  if (DelayCnt != 0) DelayCnt--;
	millis++;
}

void delay_ms(uint32_t count)
{
  DelayCnt = count;
  while(DelayCnt != 0);
}

// =========================================================
//  UART
// =========================================================

void UART_Init(void)
{
	RCC->APB2ENR |=RCC_APB2ENR_UART1;
  RCC->AHBENR |= RCC_AHBENR_GPIOB;  

	// PB4 - UART1_RX, PB6 - UART1_TX
  GPIOB->AFRL	&= ~GPIO_AFRL_AFR6;              // AF0 UART_TX
	GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_6_Pos);           
  GPIOB->CRL |= GPIO_CNF_MODE_AF_PP << GPIO_CRL_CNF_MODE_6_Pos;

  GPIOB->AFRL	&= ~GPIO_AFRL_AFR4;	
  GPIOB->AFRL	|= GPIO_AF_MODE3<<GPIO_AFRL_AFR4_Pos;	     // AF3 UART_RX 
	GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_4_Pos);           
  GPIOB->CRL |= GPIO_CNF_MODE_FLOATING << GPIO_CRL_CNF_MODE_4_Pos;

	
  UART1->GCR = 0;
  UART1->CCR = UART_CCR_CHAR_8b;	
	UART1->GCR |= UART_GCR_TX | UART_GCR_RX | UART_GCR_UART;
	uint16_t uartdiv = SystemCoreClock / UART_BAUD;  
	UART1->BRR = uartdiv / 16;   // Mantissa
	UART1->FRA = uartdiv % 16;   // Fraction
	//UART1->IER = 	UART_IER_RX;
	
  NVIC_EnableIRQ(UART1_IRQn);  
}

// =========================================================
//  LED, power, hslls anth other GPIOs
// =========================================================

void GPIO_Init(void)
{
  // LEDs PA15, PA12, PD2, PD3, PB12
  RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB | RCC_AHBENR_GPIOC| RCC_AHBENR_GPIOD;   

  GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_15_Pos);            // PA15
  GPIOA->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_15_Pos;
  GPIOA->BRR  = GPIO_BRR_BR15;                                                // PA15  output low

  GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_12_Pos);            // PA12
  GPIOA->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_12_Pos;
  GPIOA->BRR  = GPIO_BRR_BR12;                                                // PA12  output low	
	
  GPIOD->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_2_Pos);             // PD2
  GPIOD->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_2_Pos;
  GPIOD->BRR  = GPIO_BRR_BR2;                                                 // PD2  output high	                                             

  GPIOD->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_3_Pos);             // PD3
  GPIOD->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_3_Pos;
  GPIOD->BRR  = GPIO_ODR_ODR3;                                                // PD3  output low	       	
	
  GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_12_Pos);             // PB12
  GPIOB->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_12_Pos;
  GPIOB->BRR  = GPIO_BRR_BR12;                                                 // PB12  output low	       	

  // power enable
  GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_2_Pos);             // PB2
  GPIOB->CRL |= GPIO_CNF_MODE_OUT_PP << GPIO_CRL_CNF_MODE_2_Pos;
  GPIOB->BRR  = GPIO_BRR_BR2; 
  // button input
  GPIOB->CRL &= ~(GPIO_CNF_MODE_MASK << GPIO_CRL_CNF_MODE_5_Pos);             // PB5
  GPIOB->CRL |= GPIO_CNF_MODE_FLOATING << GPIO_CRL_CNF_MODE_5_Pos;			

  // buzzer
  GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_9_Pos);             // PB9
  GPIOB->CRH |= GPIO_CNF_MODE_OUT_PP << GPIO_CRH_CNF_MODE_9_Pos;
  GPIOB->BRR  = GPIO_BRR_BR9;  	

  // hall sensors PC15,14,13
  GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_15_Pos);             
  GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_15_Pos;			
  GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);             
  GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_14_Pos;			
  GPIOC->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);             
  GPIOC->CRH |= GPIO_CNF_MODE_FLOATING << GPIO_CRH_CNF_MODE_14_Pos;			    	
}

#define  PWM_RES   (72000000 / 2 / PWM_FREQ)          // = 2250

void TIM1_Init(void) 
{
  RCC->APB2ENR |=  RCC_APB2ENR_TIM1EN;
 	RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB; 

  // PA8,9,10 & PC13,14,15
  // alternative AF2 PWM pullpush output ports
  GPIOA->AFRH	&= ~GPIO_AFRH_AFR8;
  GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR8_Pos;	 
  GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_8_Pos);
  GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_8_Pos;

  GPIOA->AFRH	&= ~GPIO_AFRH_AFR9;
  GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR9_Pos;	   
  GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_9_Pos);
  GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_9_Pos;  

  GPIOA->AFRH	&= ~GPIO_AFRH_AFR10;
  GPIOA->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR10_Pos;	 
  GPIOA->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_10_Pos);
  GPIOA->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_10_Pos;    

  GPIOB->AFRH	&= ~GPIO_AFRH_AFR13;
  GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR13_Pos;	   
  GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_13_Pos);
  GPIOB->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_13_Pos;   
  
  GPIOB->AFRH	&= ~GPIO_AFRH_AFR14;
  GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR14_Pos;	   
  GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_14_Pos);
  GPIOB->CRH |= GPIO_CNF_MODE_AF_PP << GPIO_CRH_CNF_MODE_14_Pos; 

  GPIOB->AFRH	&= ~GPIO_AFRH_AFR15;
  GPIOB->AFRH	|= GPIO_AF_MODE2<<GPIO_AFRH_AFR15_Pos;	     
  GPIOB->CRH &= ~(GPIO_CNF_MODE_MASK << GPIO_CRH_CNF_MODE_15_Pos);
  GPIOB->CRH |= GPIO_CNF_MODE_AF_PP <<GPIO_CRH_CNF_MODE_15_Pos;  

	// -------------
	TIM1->SMCR = 0;
  TIM1->CR1 = TIM_CR1_CMS_CENTERALIGNED1;  // Center-aligned mode 1
  TIM1->PSC = 0;
  TIM1->ARR = PWM_RES;  // 2250	
  TIM1->RCR = 1; 
  // master/slave
  TIM1->CR2 = TIM_CR2_MMS_UPDATE;   // TRGO	
	
  // ConfigChannel 
  TIM1->CCMR1 = TIM_CCMR1_OC1M_PWM1 | TIM_CCMR1_OC2M_PWM1;   
  TIM1->CCMR2 = TIM_CCMR2_OC3M_PWM1;  

  // break
  TIM1->BDTR = TIM_BDTR_OSSR | TIM_BDTR_OSSI | DEAD_TIME;
  // enables
  TIM1->CCER = TIM_CCER_CC1E | TIM_CCER_CC1NE 
	           | TIM_CCER_CC2E | TIM_CCER_CC2NE 
						 | TIM_CCER_CC3E | TIM_CCER_CC3NE;
  TIM1->BDTR |= TIM_BDTR_MOE;
	TIM1->CR2 |= TIM_CR2_OIS1N | TIM_CR2_OIS2N | TIM_CR2_OIS3N;

  // start  
  TIM1->CR1 |= TIM_CR1_CEN; 
}



uint16_t adc_buffer_raw[32];

void ADC1_Init(void)
{
  RCC->APB2ENR |=  RCC_APB2ENR_ADC1EN;	

 	RCC->AHBENR |= RCC_AHBENR_GPIOA | RCC_AHBENR_GPIOB; 
  GPIOA->CRL  = 0;
  GPIOB->CRL &= 0xFFFFFF00;

  ADC1->CFGR = ADC_CFGR_ADEN |  ADC_CFGR_TEN | ADC_CFGR_VEN |  ADC_CFGR_SAMCTL_7_5;   //14.4MHz, temper & ref enable
  ADC1->CR = ADC_CR_SCAN | ADC_CR_T1_TRIG  | ADC_CR_ADIE;	  	// interruption

  ADC1->CHSR = ADC_CHSR_CH0 | ADC_CHSR_CH1   // phase b
             | ADC_CHSR_CH4 | ADC_CHSR_CH5   //phase c
             | ADC_CHSR_CH7                  // I common
	           | ADC_CHSR_CH9                  // voltage             
             | ADC_CHSR_CHT;                 // temperature


  NVIC_SetPriority(ADC1_IRQn, 0);
  NVIC_EnableIRQ(ADC1_IRQn);
}


uint8_t buzzerFreq          = 0;
uint8_t buzzerPattern       = 0;
uint8_t buzzerCount         = 0;
volatile uint32_t buzzerTimer = 0;
static uint8_t  buzzerPrev  = 0;
static uint8_t  buzzerIdx   = 0;





int16_t        batVoltage       = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE;
static int32_t batVoltageFixdt  = (400 * BAT_CELLS * BAT_CALIB_ADC) / BAT_CALIB_REAL_VOLTAGE << 16;  // Fixed-point filter output initialized at 400 V*100/cell = 4 V/cell converted to fixed-point

#define CURR_DC_MAX (I_DC_MAX * A2BIT_CONV)


uint8_t        enable       = 0;        // initially motor is disabled for SAFETY
static uint8_t enableFin    = 0;




int16_t max_cur_phaB=0;
int16_t  max_cur_phaC=0;
int16_t  max_cur_DC=0;
int16_t cur_phaB, cur_phaC, cur_DC;
// =================================
// DMA interrupt frequency =~ 16 kHz
// =================================
//void DMA1_Channel1_IRQHandler(void) 
//{
//  DMA1->IFCR = DMA_IFCR_CTCIF1;
	

void  ADC1_COMP_IRQHandler (void) 
{
  //int16_t cur_phaB, cur_phaC, cur_DC;

  ADC1->SR |= ADC_SR_ADIF;
  GPIOB->BSRR = GPIO_BRR_BR12;  // test

	adc_buffer.rrB = (ADC1->ADDR0 - ADC1->ADDR1) & 0xFFF;
	adc_buffer.rrC = (ADC1->ADDR4 - ADC1->ADDR5) & 0xFFF;
	adc_buffer.dcr = ADC1->ADDR7 & 0xFFF;     
  adc_buffer.batt1 = ADC1->ADDR9 & 0xFFF;  
  adc_buffer.temp =  ADC1->ADDR14 & 0xFFF;  

  // offset calibrate
  static uint8_t DummyCnt=100;
  if (DummyCnt >0) { DummyCnt--; return; }
 
  static int32_t phaseBV=0;
  static int32_t phaseCV=0;
  static int32_t commonIV=0;
  static int16_t offsetrrB = 0;
  static int16_t offsetrrC = 0;
  static int16_t offsetdcr = 0;

  static int16_t AvrCnt=1024;  
  if(AvrCnt>0)
  {
    phaseBV  +=  adc_buffer.rrB;
    phaseCV  +=  adc_buffer.rrC;    
    commonIV += adc_buffer.dcr;
    AvrCnt--;
    if(AvrCnt!=0) return;
    else
    {
      offsetrrB = (int16_t)(phaseBV/1024);
      offsetrrC = (int16_t)(phaseCV/1024);
      offsetdcr = (int16_t)(commonIV/1024);
    }
  }
  // offset calibrate end


  // Filter battery voltage at a slower sampling rate
  if (buzzerTimer % 1000 == 0) 
  {  
    filtLowPass32(adc_buffer.batt1, BAT_FILT_COEF, &batVoltageFixdt);
    batVoltage = (int16_t)(batVoltageFixdt >> 16);  // convert fixed-point to integer
  }
       
  // Get motor currents
  cur_phaB = (int16_t)(adc_buffer.rrB - offsetrrB);
  cur_phaC = (int16_t)(adc_buffer.rrC - offsetrrC);
  cur_DC   = (int16_t)(adc_buffer.dcr - offsetdcr);
  //if (cur_phaB>max_cur_phaB) max_cur_phaB = cur_phaB;
  //if (cur_phaC>max_cur_phaC) max_cur_phaC = cur_phaC;
  //if (cur_DC>max_cur_DC) max_cur_DC = cur_DC;    


  // Disable PWM when current limit is reached (current chopping)
  // This is the Level 2 of current protection. The Level 1 should kick in first given by I_MOT_MAX
  if((ABS(cur_DC)  > CURR_DC_MAX) || (enable == 0)) TIM1->BDTR &= ~TIM_BDTR_MOE;
  else                                              TIM1->BDTR |= TIM_BDTR_MOE;


  // Create square wave for buzzer
  buzzerTimer++;
  if (buzzerFreq != 0 && (buzzerTimer / 5000) % (buzzerPattern + 1) == 0) 		
  {
    if (buzzerPrev == 0)
    {
      buzzerPrev = 1;  // pause 2 periods
      if (++buzzerIdx > (buzzerCount + 2)) buzzerIdx = 1;
    }
    if (buzzerTimer % buzzerFreq == 0 && (buzzerIdx <= buzzerCount || buzzerCount == 0))
    {
      (BUZZER_PORT->ODR & (1<<BUZZER_PIN)) ? (BUZZER_PORT->BRR = 1<<BUZZER_PIN) : (BUZZER_PORT->BSRR = 1<<BUZZER_PIN);			
    }
  }
  else
  {
    if (buzzerPrev)
    {
      BUZZER_PORT->BRR = 1<<BUZZER_PIN;
      buzzerPrev = 0;
    }
	}
    
		
	// Adjust pwm_margin depending on the selected Control Type
  //if (rtP.z_ctrlTypSel == FOC_CTRL) pwm_margin = 110;
  if (rtP.z_ctrlTypSel == FOC_CTRL) pwm_margin = 300;
  else                              pwm_margin = 0;

  // ############################### MOTOR CONTROL ###############################
  int ur, vr, wr;
  static boolean_T OverrunFlag = false;

  // Check for overrun 
  if (OverrunFlag) return;
  OverrunFlag = true;
  // Make sure to stop motor in case of an error 
  enableFin = enable && !rtY.z_errCode;

  // ========================= MOTOR ===========================  
  // Get hall sensors values
  uint8_t hall_u = !(GPIOC->IDR & (1<<15));
  uint8_t hall_v = !(GPIOC->IDR & (1<<14));
  uint8_t hall_w = !(GPIOC->IDR & (1<<13));

  // Set motor inputs here 
  rtU.b_motEna      = enableFin;
  rtU.z_ctrlModReq  = ctrlModReq;
  rtU.r_inpTgt      = pwmr;
  rtU.b_hallA       = hall_u;
  rtU.b_hallB       = hall_v;
  rtU.b_hallC       = hall_w;
  rtU.i_phaAB       = cur_phaB;
  rtU.i_phaBC       = cur_phaC;
  rtU.i_DCLink      = cur_DC;

  // Step the controller 
  BLDC_controller_step(rtM);

  // Get motor outputs here 
  ur  = rtY.DC_phaA;
  vr  = rtY.DC_phaB;
  wr  = rtY.DC_phaC;

  // Apply commands 
  TIM1->CCR1  = (uint16_t)CLAMP(ur + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  TIM1->CCR2  = (uint16_t)CLAMP(vr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  TIM1->CCR3  = (uint16_t)CLAMP(wr + PWM_RES / 2, pwm_margin, PWM_RES-pwm_margin);
  // =================================================================

  // Indicate task complete 	
  OverrunFlag = false;
  GPIOB->BRR=GPIO_BRR_BR12;  // test
	
}

