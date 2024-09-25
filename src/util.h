#ifndef UTIL_H
#define UTIL_H

#include <stdint.h>


// Rx Structures USART
typedef struct{
  uint16_t  start;
  int16_t   steer;
  int16_t   speed;
  uint16_t  checksum;
} SerialCommand;


// Input Structure
typedef struct {
  int16_t   raw;    // raw input
  int16_t   cmd;    // command
  uint8_t   typ;    // type
  uint8_t   typDef; // type Defined
  int16_t   min;    // minimum
  int16_t   mid;    // middle
  int16_t   max;    // maximum
  int16_t   dband;  // deadband
} InputStruct;

// Initialization Functions
void BLDC_Init(void);
void Input_Lim_Init(void);
void Input_Init(void);

// Input Functions
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max);
void handleTimeout(void);
void readCommand(void);

// General Functions
void poweronMelody(void);
void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern);
void beepLong(uint8_t freq);
void beepShort(uint8_t freq);
void beepShortMany(uint8_t cnt, int8_t dir);

void calcAvgSpeed(void);

// Input Functions
int  checkInputType(int16_t min, int16_t mid, int16_t max);

void poweroff(void);

// Filtering Functions
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y);
void rateLimiter16(int16_t u, int16_t rate, int16_t *y);

#endif