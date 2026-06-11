#include "mppt.h"
#include <math.h>

void mpptUpdate();
// Ticker mpptUpdater(mpptUpdate, MPPT_UPDATE_PERIOD, 0, MILLIS);
STM32Timer mpptUpdater(TIM6);
// Zero-filled here (valid for any NUM_ARRAYS); seeded to INIT_VOLT in initMPPT().
volatile float targetVoltage[NUM_ARRAYS] = {0};
volatile float targetVoltage_C[NUM_ARRAYS] = {0};

volatile MpptAlgo activeAlgo =
    (DEFAULT_MPPT_ALGO == MPPT_ALGO_SAFE_CHARGE) ? MpptAlgo::SafeCharge
                                                 : MpptAlgo::PerturbObserve;

void setMpptAlgo(MpptAlgo algo) {
    activeAlgo = algo;
    resetPID(); // clear integrators so the new algorithm starts from a clean state
}

// ---------------------------------------------------------------------------
// Algorithm 1: Perturb & Observe (original behavior)
// ---------------------------------------------------------------------------
static void mpptUpdatePO() {
    // Tracks last power. Zero-filled arrays are valid for any NUM_ARRAYS; the
    // step-size arrays need a nonzero seed, done once below.
    static float oldPower[NUM_ARRAYS] = {0};
    static float stepSize[NUM_ARRAYS];
    static int zeroDutyCounter[NUM_ARRAYS] = {0};
    static int maxDutyCounter[NUM_ARRAYS] = {0};
    static float stepSize_C[NUM_ARRAYS];
    static bool stepsSeeded = false;
    if (!stepsSeeded) {
        for (int i = 0; i < NUM_ARRAYS; i++) {
            stepSize[i] = INIT_VOLT_STEP;
            stepSize_C[i] = INIT_VOLT_STEP;
        }
        stepsSeeded = true;
    }

    // Constant current mode. Try to match BMS provided charge current limit via peturb and observe
    if (chargeMode == ChargeMode::CONST_CURR) {
        bool decreasePower = (packCurrent >= packChargeCurrentLimit);
        for (int i = 0; i < NUM_ARRAYS; i++) {
            if ( (decreasePower && oldPower[i] < arrayData[i].curPower) // want to decrease power but we've stepped in the wrong direction
                || (oldPower[i] > arrayData[i].curPower)) { // want to increase power but power has decreased
                stepSize_C[i] *= -1;                
            }
            // step NOW voltage in desired direction
            targetVoltage_C[i] = arrayData[i].voltage + stepSize_C[i];
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

// ---------------------------------------------------------------------------
// Algorithm 2: Incremental Conductance MPPT with CC-CV battery limiting
//
// MPPT tracks the panel maximum power point until a battery-side limit binds:
//   - CV phase: battVolt >= V_BATT_CV  -> hold pack voltage, taper current
//   - CC phase: charge current >= limit -> shed power
//   - Hard stop: boost disabled or battVolt >= V_BATT_MAX -> converters off
// Limits are enforced by pushing the array voltage setpoint toward Voc (right of
// the MPP), which reduces delivered power. The current ceiling is the tightest of
// the BMS charge limit and the fuse-derived ceiling (charge current is normally
// well under this, so the voltage ceiling is the main end-of-charge protector).
// ---------------------------------------------------------------------------
static void mpptUpdateSafeCharge() {
    static float prevV[NUM_ARRAYS] = {0};
    static float prevI[NUM_ARRAYS] = {0};

    // Hard stop: cut the converters (updateData() forces PWM=0 in CONST_CURR).
    if (!boostEnabled || battVolt >= V_BATT_MAX) {
        chargeMode = ChargeMode::CONST_CURR;
        return;
    }
    // Keep converters live for CC / CV / MPPT regulation.
    chargeMode = ChargeMode::MPPT;

    // Effective charge-current ceiling: tightest of BMS limit and fuse margin.
    float iLimit = packChargeCurrentLimit;
    if (I_CHG_MAX_FUSE < iLimit) iLimit = I_CHG_MAX_FUSE;

    bool cvPhase = battVolt >= V_BATT_CV;     // near full -> taper
    bool ccPhase = outputCurrent >= iLimit;   // current ceiling reached

    for (int i = 0; i < NUM_ARRAYS; i++) {
        float V = arrayData[i].voltage;
        float I = arrayData[i].current;
        float dV = V - prevV[i];
        float dI = I - prevI[i];
        float step;

        if (cvPhase || ccPhase) {
            // Shed power: move array voltage toward Voc (reduces boost output current).
            step = CC_CV_BACKOFF_STEP;
        } else if (fabsf(dV) < 1e-3f) {
            // No voltage change: decide from current change alone.
            if (fabsf(dI) < 1e-3f)      step = 0.0f;
            else if (dI > 0.0f)         step = INCCOND_STEP;
            else                        step = -INCCOND_STEP;
        } else {
            // Incremental conductance: at MPP, dI/dV == -I/V.
            float cond  = I / V;
            float dCond = dI / dV;
            if (dCond > -cond + INCCOND_DEADBAND)      step = INCCOND_STEP;   // left of MPP
            else if (dCond < -cond - INCCOND_DEADBAND) step = -INCCOND_STEP;  // right of MPP
            else                                       step = 0.0f;          // at MPP
        }

        targetVoltage[i] = V + step;
        if (targetVoltage[i] <= 0.0f) targetVoltage[i] = 0.01f;
        setArrayVoltOut(targetVoltage[i], i); // clamped to V_TARGET_MAX inside

        prevV[i] = V;
        prevI[i] = I;
    }
}

// Dispatches to the selected algorithm each outer-loop tick.
void mpptUpdate() {
    switch (activeAlgo) {
        case MpptAlgo::SafeCharge:    mpptUpdateSafeCharge(); break;
        case MpptAlgo::PerturbObserve:
        default:                      mpptUpdatePO();         break;
    }
}

void initMPPT() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        targetVoltage[i] = INIT_VOLT;
        targetVoltage_C[i] = INIT_VOLT;
    }
    if (mpptUpdater.attachInterruptInterval(MPPT_UPDATE_PERIOD, mpptUpdate)) {
        printf("starting MPPT timer\n");
    } else {
        printf("ERROR: couldn't start MPPT timer\n");
    }
}