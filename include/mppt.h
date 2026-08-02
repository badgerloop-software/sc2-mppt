#ifndef __MPPT_H__
#define __MPPT_H__

#include "board_config.h"
#include "io_management.h"

// ------------- TYPES -------------

enum class MpptAlgo : uint8_t { PerturbObserve, SafeCharge };

// ------------- GLOBALS -------------

extern volatile float targetVoltage[NUM_ARRAYS];    // MPPT mode
extern volatile float targetVoltage_C[NUM_ARRAYS];  // constant current mode
extern volatile MpptAlgo activeAlgo;

// ------------- FUNCTIONS -------------

void setMpptAlgo(MpptAlgo algo);  // switches algo and resets PID
void initMppt();                  // start MPPT timer at MPPT_UPDATE_PERIOD

#endif  // __MPPT_H__
