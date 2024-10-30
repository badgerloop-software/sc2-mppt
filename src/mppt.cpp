#include "mppt.h"

void mpptUpdate();
Ticker mpptUpdater(mpptUpdate, MPPT_UPDATE_PERIOD, 0, MILLIS);
volatile float targetVoltage[NUM_ARRAYS] = {INIT_VOLT, INIT_VOLT, INIT_VOLT};
volatile float targetVoltage_C[NUM_ARRAYS] = {INIT_VOLT, INIT_VOLT, INIT_VOLT};

void mpptUpdate() {
    // Tracks last power
    static float oldPower[NUM_ARRAYS] = {0.0, 0.0, 0.0};
    static float stepSize[NUM_ARRAYS] = {INIT_VOLT_STEP, INIT_VOLT_STEP, INIT_VOLT_STEP};
    static int zeroDutyCounter[NUM_ARRAYS] = {0, 0, 0};
    static int maxDutyCounter[NUM_ARRAYS] = {0, 0, 0};
    static float stepSize_C[NUM_ARRAYS] = {INIT_VOLT_STEP, INIT_VOLT_STEP, INIT_VOLT_STEP};

    // Constant current mode. Try to match BMS provided charge current limit via peturb and observe
    if (chargeMode == ChargeMode::CONST_CURR) {
        bool decreasePower = (packCurrent >= packChargeCurrentLimit);
        for (int i = 0; i < NUM_ARRAYS; i++) {
            if ( (decreasePower && oldPower[i] < arrayData[i].curPower) // want to decrease power but we've stepped in the wrong direction
                || (oldPower[i] > arrayData[i].curPower)) { // want to increase power but power has decreased
                stepSize_C[i] *= -1;                
            }
            // step NOW voltage in desired direction
            targetVoltage_C[i] = arrayData[0].voltage + stepSize_C[i];
            if (targetVoltage_C[i] <= 0) targetVoltage_C[i] = 0.01;
            setArrayVoltOut(targetVoltage_C[i], i);

            oldPower[i] = arrayData[i].curPower;
        }
        return;
    }

    // control each array separately
    for (int i = 0; i < NUM_ARRAYS; i++) {
        // MPPT P&O Mode
        // Update the desired target voltage to reality
        targetVoltage[i] = arrayData[i].voltage;

        // consecutive cycles of 0 DUTY means we try bigger step in lower voltage
        // don't want to get stuck at 0 DUTY since it is usually not MAX POWER
        if (arrayData[i].dutyCycle < (PWM_DUTY_MIN + 0.01)) {
            zeroDutyCounter[i]++;
            if (zeroDutyCounter[i] > MAX_STUCK_CYCLES) {
                zeroDutyCounter[i] = 0;
                targetVoltage[i] = arrayData[i].voltage - MOVE_VOLTAGE;
                resetArrayPID(i); // reset the accumulated integral error
            }
        } else {
            zeroDutyCounter[i] = 0;
        }

        // consecutive cycles of 0.8 DUTY
        if (arrayData[i].dutyCycle > (PWM_DUTY_MAX - 0.01)) {
            maxDutyCounter[i]++;
            if (maxDutyCounter[i] > MAX_STUCK_CYCLES) {
                maxDutyCounter[i] = 0;
                targetVoltage[i] = arrayData[i].voltage + MOVE_VOLTAGE;
                resetArrayPID(i);
            }
        } else {
            maxDutyCounter[i] = 0;
        }

        // If last step increased power, step in same direction. Else step in opposite direction
        if (arrayData[i].curPower < oldPower[i]) {
            stepSize[i] *= -1;
        }
        

        /* UNTESTED FEATURE: variable step size
        // If last step increased power, do bigger step in same direction. Else smaller step opposite direction
        if (curPower < oldPower) {
            stepSize *= -0.8;
        } else {
            stepSize *= 1.2;
        }

        // Make sure step size not too large, do not allow 0
        if (stepSize > MAX_VOLT_STEP) stepSize = MAX_VOLT_STEP;
        else if (stepSize < -MAX_VOLT_STEP) stepSize = -MAX_VOLT_STEP;
        else if (stepSize > 0 & stepSize < MIN_VOLT_STEP) stepSize = MIN_VOLT_STEP;
        else if (stepSize < 0 & stepSize > -MIN_VOLT_STEP) stepSize = -MIN_VOLT_STEP;
        else if (stepSize == 0) stepSize = MIN_VOLT_STEP;
        */
        
        // Update voltage target for arrays. Do not allow negative
        targetVoltage[i] += stepSize[i];
        if (targetVoltage[i] <= 0) targetVoltage[i] = 0.01;
        setArrayVoltOut(targetVoltage[i], i);

        // Update power for next cycle
        oldPower[i] = arrayData[i].curPower;
    }
}