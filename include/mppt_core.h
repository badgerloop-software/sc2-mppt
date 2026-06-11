#ifndef __MPPT_CORE_H__
#define __MPPT_CORE_H__

// Pure, hardware-independent MPPT / charge decision logic.
//
// Nothing in here touches globals, timers, ADCs, PWM, or CAN, and it only
// includes const.h (which is pure #defines). That lets the whole decision layer
// be compiled and unit-tested natively on a desktop (see test/test_mppt_core),
// while the real firmware (mppt.cpp) calls these same functions so the tested
// code is the code that runs on the car.

#include "const.h"

// ---- Perturb & Observe ----

// Returns the next P&O step size: keep direction if power rose, reverse if it fell.
float poNextStep(float curPower, float oldPower, float stepSize);

// ---- Safe-charge (Incremental Conductance + CC-CV) ----

// True when the converters must be shut off entirely (boost disabled or pack at
// the absolute voltage ceiling).
bool safeChargeHardStop(bool boostEnabled, float battVolt);

// Effective pack charge-current ceiling: the tighter of the BMS limit and the
// fuse-derived limit.
float effectiveChargeCurrentLimit(float bmsLimit);

// Charge-phase detectors.
bool safeChargeCvActive(float battVolt);                       // near full -> taper
bool safeChargeCcActive(float chargeCurrent, float currentLimit); // current ceiling hit

// Incremental-conductance MPPT step (volts added to the array voltage setpoint).
// At the MPP, dI/dV == -I/V.
float incCondStep(float V, float I, float prevV, float prevI);

// Full per-array SafeCharge step: shed power if a battery limit is binding,
// otherwise track the MPP.
float safeChargeArrayStep(float V, float I, float prevV, float prevI, bool limiting);

#endif // __MPPT_CORE_H__
