#ifndef __IO_MANAGEMENT_H__
#define __IO_MANAGEMENT_H__

#include "mbed.h"
#include "const.h"
#include "FastPWM.h"
#include "INA281.h"
#include "mutexless_analog.h"
#include "PID.h"
#include "thermistor.h"


// Solar array and battery data
typedef struct ArrayData {
    double dutyCycle = 0;
    float voltage = 0;
    float current = 0;
    float curPower = 0;
    float temp = 0;
} ArrayData;

extern volatile ArrayData arrayData[NUM_ARRAYS];
extern volatile float battVolt;

// If boost converter enabled
extern volatile bool boostEnabled;

// Charging algorithm mode
enum class ChargeMode : bool {CONST_CURR, MPPT};
extern volatile ChargeMode chargeMode;

// pack charge percentage
extern volatile float packSOC;

// pack charge current limit
extern volatile float packChargeCurrentLimit;

// net pack current (solar panel IN - motor OUT)
extern volatile float packCurrent;

// output current to battery
extern volatile float outputCurrent;

// Sets up automatic updating of IO at specified period
// New input data will automatically be written to arrayData
void initData(std::chrono::microseconds updatePeriod);

// Resets the duty cycle PID loops
void resetPID();

// Resets duty cycle PID for specified array
void resetArrayPID(int array);

// Sets voltage output for all arrays
// Value will be capped if outside V_MIN or V_MAX specified in const.h
void setVoltOut(float voltage);

// Sets voltage output for specified array
void setArrayVoltOut(float voltage, int array);

// Sets clearing of OV fault
void clearOVFaultReset(uint8_t value);

// Controls discharging of output capacitors thorugh resistor path
void setCapDischarge(uint8_t value);

#endif // __IO_MANAGEMENT_H__