#include <stdlib.h> // for abs()
#include "setup.h"
#include "defines.h"
#include "config.h"
#include "util.h"
#include "BLDC_controller.h"


// ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

/* =========================== Variable Definitions =========================== */

//------------------------------------------------------------------------
// Global variables set externally
//------------------------------------------------------------------------
extern volatile adc_buf_t adc_buffer;


extern int16_t batVoltage;
extern uint8_t backwardDrive;


extern uint8_t enable;                  // global variable for motor enable


extern volatile uint32_t main_loop_counter;



//------------------------------------------------------------------------
// Global variables set here in util.c
//------------------------------------------------------------------------
// Matlab defines - from auto-code generation
//---------------
RT_MODEL rtM_Right_;                    /* Real-time model */
RT_MODEL *const rtM = &rtM_Right_;


extern P rtP;                     /* Block parameters (auto storage) */
DW       rtDW;                    /* Observable states */
ExtU     rtU;                     /* External inputs */
ExtY     rtY;                     /* External outputs */
//---------------

InputStruct input =  {0, 0, 0, PRI_INPUT} ;


int16_t  speedAvg;                      // average measured speed
int16_t  speedAvgAbs;                   // average measured speed in absolute
uint8_t  timeoutFlgSerial = 0;          // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)

uint8_t  ctrlModReqRaw = CTRL_MOD_REQ;
uint8_t  ctrlModReq    = CTRL_MOD_REQ;  // Final control mode request 



//------------------------------------------------------------------------
// Local variables
//------------------------------------------------------------------------
static int16_t InputMax;             // [-] Input target maximum limitation
static int16_t InputMin;             // [-] Input target minimum limitation


//static uint8_t  cur_spd_valid  = 0;


uint16_t timeoutCntSerial_R = SERIAL_TIMEOUT;  // Timeout counter for Rx Serial command
uint8_t  timeoutFlgSerial_R = 0;               // Timeout Flag for Rx Serial command: 0 = OK, 1 = Problem detected (line disconnected or wrong Rx data)
SerialCommand commandR; 

// =========================== BLDC =========================== 
/* =========================== Initialization Functions =========================== */

void BLDC_Init(void) {
  /* Set BLDC controller parameters */ 
  rtP.b_angleMeasEna       = 0;            // Motor angle input: 0 = estimated angle, 1 = measured angle (e.g. if encoder is available)
  rtP.z_selPhaCurMeasABC   = 1;            // Left motor measured current phases {Green, Blue} = {iA, iB} -> do NOT change
  rtP.z_ctrlTypSel         = CTRL_TYP_SEL;
  rtP.b_diagEna            = DIAG_ENA;
  rtP.i_max                = (I_MOT_MAX * A2BIT_CONV) << 4;        // fixdt(1,16,4)
  rtP.n_max                = N_MOT_MAX << 4;                       // fixdt(1,16,4)
  rtP.b_fieldWeakEna       = FIELD_WEAK_ENA; 
  rtP.id_fieldWeakMax      = (FIELD_WEAK_MAX * A2BIT_CONV) << 4;   // fixdt(1,16,4)
  rtP.a_phaAdvMax          = PHASE_ADV_MAX << 4;                   // fixdt(1,16,4)
  rtP.r_fieldWeakHi        = FIELD_WEAK_HI << 4;                   // fixdt(1,16,4)
  rtP.r_fieldWeakLo        = FIELD_WEAK_LO << 4;                   // fixdt(1,16,4)


  /* Pack RIGHT motor data into RTM */
  rtM->defaultParam       = &rtP;
  rtM->dwork              = &rtDW;
  rtM->inputs             = &rtU;
  rtM->outputs            = &rtY;

  // Initialize BLDC controller
  BLDC_controller_initialize(rtM);
}


void Input_Lim_Init(void)      // Input Limitations - ! Do NOT touch !
{
  if (rtP.b_fieldWeakEna)
  {    
    InputMax = MAX( 1000, FIELD_WEAK_HI);
    InputMin = MIN(-1000,-FIELD_WEAK_HI);
  }
  else 
  {
    InputMax =  1000;
    InputMin = -1000;
  }
}

void Input_Init(void) 
{
  if (input.typDef == 3) input.typ = checkInputType(input.min, input.mid, input.max);
  else                    input.typ = input.typDef;
    
}
 

// Check Input Type
// This function identifies the input type: 0: Disabled, 1: Normal Pot, 2: Middle Resting Pot

int checkInputType(int16_t min, int16_t mid, int16_t max)
{
  int type = 0;  
  int16_t threshold = 200;

  if ((min / threshold) == (max / threshold) || (mid / threshold) == (max / threshold) || min > max || mid > max) type = 0; 
  else
  {
    if ((min / threshold) == (mid / threshold)) type = 1;
    else                                        type = 2;  
  }
  return type;
}

// =========================== Input Functions ===========================

 
// Calculate Input Command
// This function realizes dead-band around 0 and scales the input between [out_min, out_max]
void calcInputCmd(InputStruct *in, int16_t out_min, int16_t out_max) {
  switch (in->typ){
    case 1: // Input is a normal pot
      in->cmd = CLAMP(MAP(in->raw, in->min, in->max, 0, out_max), 0, out_max);
      break;
    case 2: // Input is a mid resting pot
      if( in->raw > in->mid - in->dband && in->raw < in->mid + in->dband ) {
        in->cmd = 0;
      } else if(in->raw > in->mid) {
        in->cmd = CLAMP(MAP(in->raw, in->mid + in->dband, in->max, 0, out_max), 0, out_max);
      } else {
        in->cmd = CLAMP(MAP(in->raw, in->mid - in->dband, in->min, 0, out_min), out_min, 0);
      }
      break;
    default: // Input is ignored
      in->cmd = 0;
      break;
  }
}

// Function to handle the ADC, UART and General timeout (Nunchuk, PPM, PWM)
void handleTimeout(void) 
{
  if (timeoutCntSerial_R++ >= SERIAL_TIMEOUT)    // Timeout qualification
  {  
    timeoutFlgSerial_R = 1;                         // Timeout detected
    timeoutCntSerial_R = SERIAL_TIMEOUT;            // Limit timout counter value
  } 
  timeoutFlgSerial = timeoutFlgSerial_R;          // Report Timeout only on the Primary Input

  // In case of timeout bring the system to a Safe State
  if (timeoutFlgSerial) 
  {
    ctrlModReq  = OPEN_MODE;                                          // Request OPEN_MODE. This will bring the motor power to 0 in a controlled way
    //input1.cmd  = 0;
    input.cmd  = 0;
  } else ctrlModReq  = ctrlModReqRaw;                                      // Follow the Mode request    
}


// Function to calculate the command to the motors. This function also manages:
// - timeout detection
// - MIN/MAX limitations and deadband
void readCommand(void) 
{
  input.raw = commandR.speed;
  calcInputCmd(&input, InputMin, InputMax);
  handleTimeout();
}


// =========================== buzzer =========================== 

extern uint8_t buzzerCount;             // global variable for the buzzer counts. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerFreq;              // global variable for the buzzer pitch. can be 1, 2, 3, 4, 5, 6, 7...
extern uint8_t buzzerPattern;           // global variable for the buzzer pattern. can be 1, 2, 3, 4, 5, 6, 7...

void poweronMelody(void) 
{
  buzzerCount = 0;  // prevent interraction with beep counter
  for (int i = 8; i > 2; i--) 
  {
    buzzerFreq = (uint8_t)i;
    delay_ms(100);
  }
  buzzerFreq = 0;
}

void beepCount(uint8_t cnt, uint8_t freq, uint8_t pattern) 
{
  buzzerCount   = cnt;
  buzzerFreq    = freq;
  buzzerPattern = pattern;
}

void beepLong(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    delay_ms(500);
    buzzerFreq = 0;
}

void beepShort(uint8_t freq) {
    buzzerCount = 0;  // prevent interraction with beep counter
    buzzerFreq = freq;
    delay_ms(100);
    buzzerFreq = 0;
}

void beepShortMany(uint8_t cnt, int8_t dir) 
{
  if (dir >= 0)    // increasing tone
    for(uint8_t i = 2*cnt; i >= 2; i=i-2) beepShort(i + 3);     
  else           // decreasing tone
    for(uint8_t i = 2; i <= 2*cnt; i=i+2) beepShort(i + 3);    
}


void calcAvgSpeed(void) 
{
  // Calculate measured average speed. The minus sign (-) is because motors spin in opposite directions
  speedAvg = 0;
  speedAvg -= rtY.n_mot;

  // Handle the case when SPEED_COEFFICIENT sign is negative (which is when most significant bit is 1)
  if (SPEED_COEFFICIENT & (1 << 16)) speedAvg    = -speedAvg;
  speedAvgAbs   = abs(speedAvg);
}



// =========================== power =========================== 
void poweroff(void) 
{
  //enable = 0;
  buzzerCount = 0;  // prevent interraction with beep counter
  buzzerPattern = 0;
  for (int i = 0; i < 8; i++) 
  {
    buzzerFreq = (uint8_t)i;
    delay_ms(100);
  }
  OFF_PORT->BRR = 1<<OFF_PIN;   // turn off power  
  while(1) {}
}




/* =========================== Filtering Functions =========================== */

  /* Low pass filter fixed-point 32 bits: fixdt(1,32,16)
  * Max:  32767.99998474121
  * Min: -32768
  * Res:  1.52587890625e-05
  * 
  * Inputs:       u     = int16 or int32
  * Outputs:      y     = fixdt(1,32,16)
  * Parameters:   coef  = fixdt(0,16,16) = [0,65535U]
  * 
  * Example: 
  * If coef = 0.8 (in floating point), then coef = 0.8 * 2^16 = 52429 (in fixed-point)
  * filtLowPass16(u, 52429, &y);
  * yint = (int16_t)(y >> 16); // the integer output is the fixed-point ouput shifted by 16 bits
  */
void filtLowPass32(int32_t u, uint16_t coef, int32_t *y)
{
  int64_t tmp;  
  tmp = ((int64_t)((u << 4) - (*y >> 12)) * coef) >> 4;
  tmp = CLAMP(tmp, -2147483648LL, 2147483647LL);  // Overflow protection: 2147483647LL = 2^31 - 1
  *y = (int32_t)tmp + (*y);
}

  /* rateLimiter16(int16_t u, int16_t rate, int16_t *y);
  * Inputs:       u     = int16
  * Outputs:      y     = fixdt(1,16,4)
  * Parameters:   rate  = fixdt(1,16,4) = [0, 32767] Do NOT make rate negative (>32767)
  */
void rateLimiter16(int16_t u, int16_t rate, int16_t *y)
{
  int16_t q0;
  int16_t q1;

  q0 = (u << 4)  - *y;

  if (q0 > rate) q0 = rate;
  else 
	{
    q1 = -rate;
    if (q0 < q1) q0 = q1;
  }

  *y = q0 + *y;
}






