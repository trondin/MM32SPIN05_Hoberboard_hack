/*
* This file is part of the hoverboard-firmware-hack project.
*
* Copyright (C) 2017-2018 Rene Hopf <renehopf@mac.com>
* Copyright (C) 2017-2018 Nico Stute <crinq@crinq.de>
* Copyright (C) 2017-2018 Niklas Fauth <niklas.fauth@kit.fail>
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

// Define to prevent recursive inclusion
#ifndef DEFINES_H
#define DEFINES_H

#include "mm32_device.h"
#include "config.h"

// =========================================================
//  POWER GPIO
// =========================================================
#define BUTTON_PORT GPIOB
#define BUTTON_PIN  5
#define OFF_PORT    GPIOB
#define OFF_PIN     2
// =========================================================
//  LED GPIO
// =========================================================
// PD3
#define LED1_OFF      GPIOD->BRR= 1<<3        
#define LED1_ON       GPIOD->BSRR = 1<<3   
#define LED1_TOGGLE  (GPIOD->ODR & (1<<3)) ? (GPIOD->BRR= 1<<3) : (GPIOD->BSRR = 1<<3)
// PA12
#define LED2_OFF      GPIOA->BRR = 1<<12     
#define LED2_ON       GPIOA->BSRR = 1<<12 
#define LED2_TOGGLE  (GPIOA->ODR & (1<<12)) ? (GPIOA->BRR= 1<<12) : (GPIOA->BSRR =1<<12)
 // PD2
#define LED3_OFF     GPIOD->BRR = 1<<2       
#define LED3_ON      GPIOD->BSRR = 1<<2
#define LED3_TOGGLE (GPIOD->ODR & (1<<2)) ? (GPIOD->BRR=1<<2) : (GPIOD->BSRR = 1<<2)
// PA15
#define LED4_OFF      GPIOA->BRR = 1<<15     
#define LED4_ON       GPIOA->BSRR = 1<<15
#define LED4_TOGGLE  (GPIOA->ODR & (1<<15)) ? (GPIOA->BRR=1<<15) : (GPIOA->BSRR = 1<<15)
// PB12
#define LED5_OFF      GPIOB->BRR = 1<<12         
#define LED5_ON       GPIOB->BSRR = 1<<12      
#define LED5_TOGGLE  (GPIOB->ODR & (1<<12)) ? (GPIOB->BRR=1<<12) : (GPIOB->BSRR = 1<<12)

// workaround
//#define TEST1_ON  GPIOD->CRL |= 0x8 << GPIO_CRL_CNF_MODE_0_Pos
//#define TEST1_OFF GPIOD->CRL &= ~(0x8 << GPIO_CRL_CNF_MODE_0_Pos)	
//#define TEST2_ON  GPIOD->CRL |= 0x8 << GPIO_CRL_CNF_MODE_1_Pos
//#define TEST2_OFF GPIOD->CRL &= ~(0x8 << GPIO_CRL_CNF_MODE_1_Pos)	


// =========================================================
//  BUZZER
// =========================================================
#define BUZZER_PORT GPIOB
#define BUZZER_PIN 9

/*
#define DC_CUR_PIN 1
#define U_CUR_PIN 4
#define RIGHT_V_CUR_PIN 5

#define DC_CUR_PORT GPIOC
#define U_CUR_PORT GPIOC
#define V_CUR_PORT GPIOC


#define DCLINK_PIN GPIO_PIN_2
#define DCLINK_PORT GPIOC


#define LED_PIN 2
#define LED_PORT GPIOB

#define BUZZER_PIN 4
#define BUZZER_PORT GPIOA


#define OFF_PIN 5
#define OFF_PORT GPIOA


#define BUTTON_PIN 1
#define BUTTON_PORT GPIOA

#define CHARGER_PIN 12
#define CHARGER_PORT GPIOA

*/

#define DELAY_TIM_FREQUENCY_US 1000000

#define MILLI_R (R * 1000)
#define MILLI_PSI (PSI * 1000)
#define MILLI_V (V * 1000)

#define NO 0
#define YES 1
#define ABS(a) (((a) < 0) ? -(a) : (a))
#define LIMIT(x, lowhigh) (((x) > (lowhigh)) ? (lowhigh) : (((x) < (-lowhigh)) ? (-lowhigh) : (x)))
#define SAT(x, lowhigh) (((x) > (lowhigh)) ? (1.0f) : (((x) < (-lowhigh)) ? (-1.0f) : (0.0f)))
#define SAT2(x, low, high) (((x) > (high)) ? (1.0f) : (((x) < (low)) ? (-1.0f) : (0.0f)))
#define STEP(from, to, step) (((from) < (to)) ? (MIN((from) + (step), (to))) : (MAX((from) - (step), (to))))
#define DEG(a) ((a)*M_PI / 180.0f)
#define RAD(a) ((a)*180.0f / M_PI)
#define SIGN(a) (((a) < 0) ? (-1) : (((a) > 0) ? (1) : (0)))
#define CLAMP(x, low, high) (((x) > (high)) ? (high) : (((x) < (low)) ? (low) : (x)))
#define IN_RANGE(x, low, high) (((x) >= (low)) && ((x) <= (high)))
#define SCALE(value, high, max) MIN(MAX(((max) - (value)) / ((max) - (high)), 0.0f), 1.0f)
#ifndef MIN
  #define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif
#ifndef MAX
  #define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif	
#define MIN3(a, b, c) MIN(a, MIN(b, c))
#define MAX3(a, b, c) MAX(a, MAX(b, c))
#define ARRAY_LEN(x) (uint32_t)(sizeof(x) / sizeof(*(x)))
#define MAP(x, in_min, in_max, out_min, out_max) (((((x) - (in_min)) * ((out_max) - (out_min))) / ((in_max) - (in_min))) + (out_min))


typedef struct {
  uint16_t dcr; 
  uint16_t rrB; 
  uint16_t rrC;
  uint16_t batt1;  
  uint16_t temp;
} adc_buf_t;


#endif // DEFINES_H

