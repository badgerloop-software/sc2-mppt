#include "mppt_core.h"
#include <math.h>

float poNextStep(float curPower, float oldPower, float stepSize) {
    // If the last step lost power, reverse direction; otherwise keep going.
    return (curPower < oldPower) ? -stepSize : stepSize;
}

bool safeChargeHardStop(bool boostEnabled, float battVolt) {
    return !boostEnabled || battVolt >= V_BATT_MAX;
}

float effectiveChargeCurrentLimit(float bmsLimit) {
    // Tightest of: BMS charge limit, MPPT operating cap (leaves regen headroom on
    // the shared fuse), and the absolute fuse-derived ceiling.
    float limit = bmsLimit;
    if (I_CHG_MAX_MPPT < limit) limit = I_CHG_MAX_MPPT;
    if (I_CHG_MAX_FUSE < limit) limit = I_CHG_MAX_FUSE;
    return limit;
}

bool safeChargeCvActive(float battVolt) {
    return battVolt >= V_BATT_CV;
}

bool safeChargeCcActive(float chargeCurrent, float currentLimit) {
    return chargeCurrent >= currentLimit;
}

float incCondStep(float V, float I, float prevV, float prevI) {
    float dV = V - prevV;
    float dI = I - prevI;

    if (fabsf(dV) < 1e-3f) {
        // No voltage change: decide from current change alone.
        if (fabsf(dI) < 1e-3f) return 0.0f;
        return (dI > 0.0f) ? INCCOND_STEP : -INCCOND_STEP;
    }

    float cond  = I / V;   // instantaneous conductance
    float dCond = dI / dV; // incremental conductance
    if (dCond > -cond + INCCOND_DEADBAND)      return INCCOND_STEP;   // left of MPP
    else if (dCond < -cond - INCCOND_DEADBAND) return -INCCOND_STEP;  // right of MPP
    return 0.0f;                                                      // at MPP
}

float safeChargeArrayStep(float V, float I, float prevV, float prevI, bool limiting) {
    if (limiting) {
        // Push array voltage toward Voc to reduce delivered power.
        return CC_CV_BACKOFF_STEP;
    }
    return incCondStep(V, I, prevV, prevI);
}
