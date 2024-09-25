#include "mm32_device.h"
#include <stdlib.h> // for abs()
#include "setup.h"
#include "config.h"
#include "defines.h"
#include "util.h"
#include "BLDC_controller.h"
#include <stdio.h>


//------------------------------------------------------------------------
// Global variables set here in main.c
//------------------------------------------------------------------------
extern volatile uint32_t buzzerTimer;
volatile uint32_t main_loop_counter;
int16_t batVoltageCalib;         // global variable for calibrated battery voltage
int16_t board_temp_deg_c;        // global variable for calibrated temperature in degrees Celsius


extern ExtY rtY;                  /* External outputs */
//---------------
extern InputStruct input;            // input structure

extern int16_t speedAvg;                // Average measured speed
extern int16_t speedAvgAbs;             // Average measured speed in absolute
extern uint8_t timeoutFlgSerial;        // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

extern volatile int pwmr;               // global variable for pwm right. -1000 to 1000
extern uint8_t enable;                  // global variable for motor enable
extern int16_t batVoltage;              // global variable for battery voltage



extern void SystemInit (void); 
extern volatile adc_buf_t adc_buffer;

//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
typedef struct
{
  uint16_t  start;

  int16_t   cmd;
  int16_t   cmd1;
  int16_t   speed_meas;
  int16_t   speedR_meas;

  int16_t   batVoltage;
  int16_t   boardTemp;
  uint16_t cmdLed;
  uint16_t  checksum;
} SerialFeedback;

static SerialFeedback Feedback;

static int16_t  speedRateFixdt;       // local fixed-point variable for speed rate limiter
static int32_t  speedFixdt;           // local fixed-point variable for speed low-pass filter

static uint32_t    inactivity_timeout_counter;
static uint16_t rate = RATE; // Adjustable rate to support multiple drive modes on startup

uint8_t volatile * TXponter;
int8_t volatile TXcouter;

void UART_tx(void)
{
  TXcouter = sizeof(Feedback);
  TXponter = (uint8_t *)&Feedback; 
  UART1->ISR &= ~UART_ISR_TX_INTF;
  while (!(UART1->CSR & UART_CSR_TXEPT)) {};
  UART1->TDR = *TXponter;
  TXcouter--; 
  TXponter++;
  UART1->IER |= UART_IER_TXIEN;
}

SerialCommand command_raw;
extern uint16_t timeoutCntSerial_R;
extern uint8_t  timeoutFlgSerial_R;
extern SerialCommand commandR; 

void UART1_IRQHandler(void)
{
  // receiver
  if(UART1->ISR & UART_ISR_RX_INTF)  // rx is not empty
  {
    static uint8_t state=0;
    static uint8_t * buffPointer;
    UART1->ICR |= UART_ICR_RXICLR; 
    uint8_t data = UART1->RDR;
    if (state==0)
    {
      if (data == (SERIAL_START_FRAME & 0xFF)) state++;
    }
    else if (state==1)
    {
      if (data == (SERIAL_START_FRAME >> 8))
      {
        state++;
        buffPointer = (uint8_t *)&command_raw + 2;
        command_raw.start = SERIAL_START_FRAME;
      }
      else state=0;
    }
    else if(state<8)
    {
      *buffPointer = data;
      buffPointer++;
      if (state==7)
      {
        state = 0;  
        uint16_t checksum = (uint16_t)(command_raw.start ^ command_raw.steer ^ command_raw.speed);
        if (command_raw.checksum == checksum) 
        {
          commandR = command_raw;
          timeoutFlgSerial_R = 0;         // Clear timeout flag
          timeoutCntSerial_R = 0;         // Reset timeout counter           
        }
      }
      else state++;
    }   
  }
  // transmitter
  else if (UART1->ISR & UART_ISR_TX_INTF) // tx buff null
  {
    UART1->ICR |= UART_ICR_TXICLR;
    TXcouter--;
    if (!(TXcouter>0)) UART1->IER &= ~UART_IER_TXIEN;    // disable interrupt
    UART1->TDR = *TXponter;
    TXponter++;
  }

}

void LedGame(void)
{
	delay_ms(100);	  
  LED1_ON;  		
	delay_ms(100);		
  LED2_ON;  		
	delay_ms(100);					
  LED3_ON;  		
	delay_ms(100);		
  LED4_ON;  		
	delay_ms(100);	
  LED1_OFF;  
  LED2_OFF;  
  LED3_OFF;  	
  LED4_OFF;  		
}


uint8_t uart_buff[64];

extern int16_t max_cur_phaB, max_cur_phaC, max_cur_DC;
extern int16_t cur_phaB, cur_phaC, cur_DC;
int main(void)
{
	uint32_t    buzzerTimer_prev = 0;
  int16_t speed=0;
	
	SystemInit(); 
  RCC->AHBENR &= ~RCC_AHBENR_DMA1EN;   // DMA1CLK_DISABLE();	
  delay_Init();
  GPIO_Init();
  //for (uint8_t i=0; i<5; i++) LedGame();  
	//delay_ms(100);
  TIM1_Init();
  ADC1_Init();   
  BLDC_Init();        // BLDC Controller Init	
  UART_Init();

  // Start ADC conversion  
	ADC1->CR |= ADC_CR_TRGEN;
	
  OFF_PORT->BSRR = 1<<OFF_PIN;   // Activate Latch  	

	Input_Lim_Init();   // Input Limitations Init
  Input_Init();       // Input Init

  poweronMelody();	  


  int32_t board_temp_adcFixdt = adc_buffer.temp << 16;  // Fixed-point filter output initialized with current ADC converted to fixed-point
  int16_t board_temp_adcFilt  = adc_buffer.temp;
	LED1_ON;  
  while(BUTTON_PORT->IDR & (1<<BUTTON_PIN)) delay_ms(10);  // Loop until button is released
  LED2_ON;  

  UART1->IER |= UART_IER_RX;
  while(1)
	{  
		
    // 1 ms = 16 ticks buzzerTimer
    if (!(buzzerTimer - buzzerTimer_prev > 16*DELAY_IN_MAIN_LOOP) ) continue; 
    
    
      
    readCommand();                        // Read Command: input1[0].cmd, input[0].cmd
    calcAvgSpeed();                       // Calculate average measured speed: speedAvg, speedAvgAbs

    // ####### MOTOR ENABLING: Only if the initial input is very small (for SAFETY) #######
    if (enable == 0 && !rtY.z_errCode && ABS(input.cmd) < 50)
    {
      beepShort(6);                     // make 2 beeps indicating the motor enable
      beepShort(4); 
      delay_ms(100);
      speedFixdt = 0;      // reset filters
      enable = 1;          // enable motors
    }


    // ####### LOW-PASS FILTER #######
    rateLimiter16(input.cmd, rate, &speedRateFixdt);
    filtLowPass32(speedRateFixdt >> 4, FILTER, &speedFixdt);
    speed = (int16_t)(speedFixdt >> 16);  // convert fixed-point to integer
    pwmr = speed;

    // ####### CALC BOARD TEMPERATURE #######
    filtLowPass32(adc_buffer.temp, TEMP_FILT_COEF, &board_temp_adcFixdt);
    board_temp_adcFilt  = (int16_t)(board_temp_adcFixdt >> 16);  // convert fixed-point to integer
    board_temp_deg_c    = (TEMP_CAL_HIGH_DEG_C - TEMP_CAL_LOW_DEG_C) * (board_temp_adcFilt - TEMP_CAL_LOW_ADC) / (TEMP_CAL_HIGH_ADC - TEMP_CAL_LOW_ADC) + TEMP_CAL_LOW_DEG_C;

    // ####### CALC CALIBRATED BATTERY VOLTAGE #######
    batVoltageCalib = batVoltage * BAT_CALIB_REAL_VOLTAGE / BAT_CALIB_ADC;
		

    // ####### FEEDBACK SERIAL OUT #######
    if (main_loop_counter % 2 == 0)     // Send data periodically every 10 ms
    {
	  	if(!(UART1->CSR & UART_CSR_TXFULL)) // tx is empty
      {
        Feedback.start	      = SERIAL_START_FRAME;
        Feedback.cmd          = input.cmd;
        Feedback.cmd1         = cur_phaB;  //data
        Feedback.speed_meas	  = rtY.n_mot;
        Feedback.speedR_meas	= cur_phaC;  // data
        Feedback.batVoltage	  = batVoltageCalib;
        Feedback.boardTemp	  = board_temp_deg_c;    
        Feedback.cmdLed       = cur_DC;  // data
        Feedback.checksum     = Feedback.start ^ Feedback.cmd ^ Feedback.cmd1 
                                               ^ Feedback.speed_meas  ^ Feedback.speedR_meas
                                               ^ Feedback.batVoltage ^ Feedback.boardTemp
                                               ^ Feedback.cmdLed;
        UART_tx();
      }
    }



    // ####### POWEROFF BY POWER-BUTTON #######
    if(BUTTON_PORT->IDR & (1<<BUTTON_PIN)) 	
		{
      while(BUTTON_PORT->IDR & (1<<BUTTON_PIN))	{};		
      poweroff(); 
		}			

    // ####### BEEP AND EMERGENCY POWEROFF #######
    if (TEMP_POWEROFF_ENABLE && board_temp_deg_c >= TEMP_POWEROFF && speedAvgAbs < 20)  // poweroff before mainboard burns OR low bat 3
      poweroff();
    else if ( BAT_DEAD_ENABLE && batVoltage < BAT_DEAD && speedAvgAbs < 20)
      poweroff();
    else if (rtY.z_errCode)    // 1 beep (low pitch): Motor error, disable motors
    {
      enable = 0;
      beepCount(1, 24, 1);
    }
    else if (timeoutFlgSerial)        // 3 beeps (low pitch): Serial timeout
      beepCount(3, 24, 1);
    else if (TEMP_WARNING_ENABLE && board_temp_deg_c >= TEMP_WARNING)     // 5 beeps (low pitch): Mainboard temperature warning
      beepCount(5, 24, 1);
    else if (BAT_LVL1_ENABLE && batVoltage < BAT_LVL1)      // 1 beep fast (medium pitch): Low bat 1
      beepCount(0, 10, 6);
    else if (BAT_LVL2_ENABLE && batVoltage < BAT_LVL2)     // 1 beep slow (medium pitch): Low bat 2 
      beepCount(0, 10, 30);
    else if (BEEPS_BACKWARD && (((speed < -50) && speedAvg < 0) )) // 1 beep fast (high pitch): Backward spinning motors
      beepCount(0, 5, 1); 
    else beepCount(0, 0, 0);  // do not beep
      

    // ####### INACTIVITY TIMEOUT #######
    inactivity_timeout_counter++;
    if ( abs(speed) > 50) inactivity_timeout_counter = 0;

    if (inactivity_timeout_counter > (INACTIVITY_TIMEOUT * 60 * 1000) / (DELAY_IN_MAIN_LOOP + 1))  // rest of main loop needs maybe 1ms
      poweroff();
      

    // Update states
    buzzerTimer_prev = buzzerTimer;
    main_loop_counter++;			   
  }
}
	
