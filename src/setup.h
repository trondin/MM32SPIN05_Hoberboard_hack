#ifndef SETUP_H
#define SETUP_H
#include "mm32_device.h"




void delay_Init(void);
void delay_ms(uint32_t count);

void UART_Init(void);
//void UART_tx(uint8_t symb);
void uart2_putchar(uint8_t symb);
void uart2_puts(uint8_t *string); 
void UART2_Init(void);

void GPIO_Init(void);
void TIM1_Init(void);
void ADC1_Init(void);

#endif
