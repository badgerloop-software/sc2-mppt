#ifndef __MPPT_H__
#define __MPPT_H__


#include "const.h"
#include "IOManagement.h"
#include "STM32TimerInterrupt_Generic.h"

extern volatile float targetVoltage[NUM_ARRAYS];
extern volatile float targetVoltage_C[NUM_ARRAYS];

// Sets up running of MPPT algorithm at specified rate
void initMPPT();

#endif // __MPPT_H__