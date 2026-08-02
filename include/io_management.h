#ifndef __IO_MANAGEMENT_H__
#define __IO_MANAGEMENT_H__

#include <Arduino.h>
#include <HardwareTimer.h>
// TimerInterrupt_Generic 1.13 expects MICROSEC_FORMAT as a bare name
#ifndef MICROSEC_FORMAT
#define MICROSEC_FORMAT TimerFormat_t::MICROSEC_FORMAT
#endif
#include "STM32TimerInterrupt_Generic.h"
#include "adc.h"
#include "board_config.h"
#include "ina281.h"
#include "pid.h"
#include "thermistor.h"

// ------------- TYPES -------------

// Solar array + battery readings
typedef struct ArrayData {
    float dutyCycle = 0;
    float voltage = 0;
    float current = 0;
    float curPower = 0;
    float temp = 0;
} ArrayData;

// CONST_CURR turns PWM off in the IO loop
enum class ChargeMode : bool { CONST_CURR, MPPT };

// ------------- GLOBALS -------------

extern volatile ArrayData arrayData[NUM_ARRAYS];
extern volatile float battVolt;
extern volatile bool boostEnabled;  // boost enable input
extern volatile ChargeMode chargeMode;

extern volatile float packSOC;                 // %
extern volatile float packChargeCurrentLimit;  // A
extern volatile float packCurrent;             // A, pack net
extern volatile float outputCurrent;           // A, into battery

// ------------- FUNCTIONS -------------

void initData();
void resetPID();
void resetArrayPID(uint8_t array);
void setVoltOut(float voltage);                  // clamped to V_TARGET_MAX
void setArrayVoltOut(float voltage, uint8_t array);
void clearOVFaultReset(uint8_t value);           // auto-clears after OV_FAULT_RST_PERIOD
void setCapDischarge(uint8_t value);

#endif  // __IO_MANAGEMENT_H__
