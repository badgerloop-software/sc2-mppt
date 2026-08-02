#include "mppt.h"

#include <math.h>

#include "debug.h"

// Who writes what:
//   mpptUpdate (timer) -> targetVoltage / targetVoltage_C, chargeMode (SafeCharge)
//   io_management      -> reads targets via setArrayVoltOut; may seed targetVoltage on boost edge
//   CAN                -> pack limits used by the algo

// ------------- GLOBALS -------------
volatile float targetVoltage[NUM_ARRAYS] = {0};
volatile float targetVoltage_C[NUM_ARRAYS] = {0};

volatile MpptAlgo activeAlgo =
    (DEFAULT_MPPT_ALGO == MPPT_ALGO_SAFE_CHARGE) ? MpptAlgo::SafeCharge : MpptAlgo::PerturbObserve;

// ------------- LOCAL -------------
static void mpptUpdate();
static STM32Timer mpptUpdater(TIM6);

// ------------- LOCAL FUNCTIONS -------------

// Perturb and observe
static void mpptUpdatePo() {
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

    // Constant current: P&O toward BMS charge limit
    if (chargeMode == ChargeMode::CONST_CURR) {
        bool decreasePower = (packCurrent >= packChargeCurrentLimit);
        for (int i = 0; i < NUM_ARRAYS; i++) {
            // stepped wrong way, reverse
            if ((decreasePower && oldPower[i] < arrayData[i].curPower) ||
                (oldPower[i] > arrayData[i].curPower)) {
                stepSize_C[i] *= -1;
            }
            targetVoltage_C[i] = arrayData[i].voltage + stepSize_C[i];
            if (targetVoltage_C[i] <= 0) {
                targetVoltage_C[i] = 0.01f;
            }
            setArrayVoltOut(targetVoltage_C[i], i);
            oldPower[i] = arrayData[i].curPower;
        }
        return;
    }

    // control each array separately
    for (int i = 0; i < NUM_ARRAYS; i++) {
        // start from measured voltage
        targetVoltage[i] = arrayData[i].voltage;

        // stuck at 0 duty, try a bigger step down (usually not MPP)
        if (arrayData[i].dutyCycle < (PWM_DUTY_MIN + 0.01f)) {
            zeroDutyCounter[i]++;
            if (zeroDutyCounter[i] > MAX_STUCK_CYCLES) {
                zeroDutyCounter[i] = 0;
                targetVoltage[i] = arrayData[i].voltage - MOVE_VOLTAGE;
                resetArrayPID(i);
            }
        } else {
            zeroDutyCounter[i] = 0;
        }

        // stuck at max duty, step voltage up
        if (arrayData[i].dutyCycle > (PWM_DUTY_MAX - 0.01f)) {
            maxDutyCounter[i]++;
            if (maxDutyCounter[i] > MAX_STUCK_CYCLES) {
                maxDutyCounter[i] = 0;
                targetVoltage[i] = arrayData[i].voltage + MOVE_VOLTAGE;
                resetArrayPID(i);
            }
        } else {
            maxDutyCounter[i] = 0;
        }

        // power went down, reverse step
        if (arrayData[i].curPower < oldPower[i]) {
            stepSize[i] *= -1;
        }

        targetVoltage[i] += stepSize[i];
        if (targetVoltage[i] <= 0) {
            targetVoltage[i] = 0.01f;
        }
        setArrayVoltOut(targetVoltage[i], i);
        oldPower[i] = arrayData[i].curPower;
    }
}

// Inc conductance + CC/CV limits on the pack
static void mpptUpdateSafeCharge() {
    static float prevV[NUM_ARRAYS] = {0};
    static float prevI[NUM_ARRAYS] = {0};

    if (!boostEnabled || battVolt >= V_BATT_MAX) {
        chargeMode = ChargeMode::CONST_CURR;
        return;
    }
    chargeMode = ChargeMode::MPPT;

    // don't exceed fuse-limited charge current
    float iLimit = packChargeCurrentLimit;
    if (I_CHG_MAX_FUSE < iLimit) {
        iLimit = I_CHG_MAX_FUSE;
    }

    bool cvPhase = battVolt >= V_BATT_CV;
    bool ccPhase = outputCurrent >= iLimit;

    for (int i = 0; i < NUM_ARRAYS; i++) {
        float V = arrayData[i].voltage;
        float I = arrayData[i].current;
        float dV = V - prevV[i];
        float dI = I - prevI[i];
        float step;

        if (cvPhase || ccPhase) {
            // hit CC or CV, back off
            step = CC_CV_BACKOFF_STEP;
        } else if (fabsf(dV) < 1e-3f) {
            // V barely changed, use dI
            if (fabsf(dI) < 1e-3f) {
                step = 0.0f;
            } else if (dI > 0.0f) {
                step = INCCOND_STEP;
            } else {
                step = -INCCOND_STEP;
            }
        } else {
            // dI/dV vs -I/V, with deadband
            float cond = I / V;
            float dCond = dI / dV;
            if (dCond > -cond + INCCOND_DEADBAND) {
                step = INCCOND_STEP;
            } else if (dCond < -cond - INCCOND_DEADBAND) {
                step = -INCCOND_STEP;
            } else {
                step = 0.0f;
            }
        }

        targetVoltage[i] += step;
        if (targetVoltage[i] <= 0.0f) {
            targetVoltage[i] = 0.01f;
        }
        setArrayVoltOut(targetVoltage[i], i);

        prevV[i] = V;
        prevI[i] = I;
    }
}

static void mpptUpdate() {
    switch (activeAlgo) {
        case MpptAlgo::SafeCharge:
            mpptUpdateSafeCharge();
            break;
        case MpptAlgo::PerturbObserve:
        default:
            mpptUpdatePo();
            break;
    }
}

// ------------- PUBLIC FUNCTIONS -------------

void setMpptAlgo(MpptAlgo algo) {
    activeAlgo = algo;
    resetPID();
}

void initMppt() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        targetVoltage[i] = INIT_VOLT;
        targetVoltage_C[i] = INIT_VOLT;
    }

    if (!mpptUpdater.attachInterruptInterval(MPPT_UPDATE_PERIOD, mpptUpdate)) {
        debugError("ERROR: MPPT timer");
    }
}
