#ifndef __MPPT_H__
#define __MPPT_H__


#include "const.h"
#include "IOManagement.h"
#include "STM32TimerInterrupt_Generic.h"

extern volatile float targetVoltage[NUM_ARRAYS];
extern volatile float targetVoltage_C[NUM_ARRAYS];

// Selectable outer-loop charge algorithm
enum class MpptAlgo : uint8_t { PerturbObserve, SafeCharge };
extern volatile MpptAlgo activeAlgo;

// Switch the active algorithm at runtime (resets PID state for a clean handoff)
void setMpptAlgo(MpptAlgo algo);

// Sets up running of MPPT algorithm at specified rate
void initMPPT();

#endif // __MPPT_H__