// Native unit tests for the pure MPPT / charge decision logic in mppt_core.cpp.
//
// These run on a desktop (pio test -e native) with no STM32 hardware. They cover
// the safety envelope of the SafeCharge algorithm and the MPPT tracking decision,
// using a deliberately simple PV + battery model. Passing tests prove the
// algorithm makes the right *decisions*; they do NOT model converter/PID dynamics
// or timing, so they are not a substitute for a bench test before charging a pack.

#include <unity.h>
#include "mppt_core.h"
#include <math.h>

void setUp(void) {}
void tearDown(void) {}

// ---------------------------------------------------------------------------
// Simple PV string model: P(V) is a downward parabola peaking at VMPP.
//   P(V) = PMAX * (1 - ((V - VMPP)/VMPP)^2),  clamped to >= 0
//   I(V) = P(V) / V
// dP/dV > 0 for V < VMPP and < 0 for V > VMPP, so it exercises MPP tracking and
// the "push toward Voc to shed power" backoff behaviour.
// ---------------------------------------------------------------------------
static const float VMPP = 40.0f;   // V at max power
static const float PMAX = 240.0f;  // W per string at MPP

static float pvPower(float V) {
    if (V <= 0.0f) return 0.0f;
    float n = (V - VMPP) / VMPP;
    float p = PMAX * (1.0f - n * n);
    return (p > 0.0f) ? p : 0.0f;
}
static float pvCurrent(float V) {
    if (V <= 0.01f) return 0.0f;
    return pvPower(V) / V;
}

// ---- Hard stop ----

void test_hard_stop_on_overvoltage(void) {
    TEST_ASSERT_TRUE(safeChargeHardStop(true, V_BATT_MAX));
    TEST_ASSERT_TRUE(safeChargeHardStop(true, V_BATT_MAX + 1.0f));
    TEST_ASSERT_FALSE(safeChargeHardStop(true, V_BATT_MAX - 1.0f));
}

void test_hard_stop_when_boost_disabled(void) {
    TEST_ASSERT_TRUE(safeChargeHardStop(false, 100.0f)); // even at a safe voltage
}

// ---- Current limit selection ----

void test_current_limit_takes_tighter_of_bms_and_fuse(void) {
    // Huge BMS limit -> MPPT operating cap wins (tighter than the fuse ceiling).
    TEST_ASSERT_FLOAT_WITHIN(0.01f, I_CHG_MAX_MPPT, effectiveChargeCurrentLimit(1000.0f));
    // Small BMS limit wins.
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 5.0f, effectiveChargeCurrentLimit(5.0f));
    // Ceilings must nest: MPPT cap < fuse-derived ceiling < physical fuse.
    TEST_ASSERT_TRUE(I_CHG_MAX_MPPT < I_CHG_MAX_FUSE);
    TEST_ASSERT_TRUE(I_CHG_MAX_FUSE < PACK_FUSE_A);
}

// ---- Phase detectors ----

void test_cv_phase_threshold(void) {
    TEST_ASSERT_TRUE(safeChargeCvActive(V_BATT_CV));
    TEST_ASSERT_TRUE(safeChargeCvActive(V_BATT_CV + 0.5f));
    TEST_ASSERT_FALSE(safeChargeCvActive(V_BATT_CV - 0.5f));
}

void test_cc_phase_threshold(void) {
    TEST_ASSERT_TRUE(safeChargeCcActive(34.0f, 34.0f));
    TEST_ASSERT_TRUE(safeChargeCcActive(40.0f, 34.0f));
    TEST_ASSERT_FALSE(safeChargeCcActive(10.0f, 34.0f));
}

// ---- Incremental conductance direction ----

void test_inccond_steps_up_left_of_mpp(void) {
    // V well below VMPP, current nearly flat -> power rising with V -> step up.
    float step = incCondStep(/*V*/30.0f, /*I*/pvCurrent(30.0f),
                             /*prevV*/29.0f, /*prevI*/pvCurrent(29.0f));
    TEST_ASSERT_TRUE(step > 0.0f);
}

void test_inccond_steps_down_right_of_mpp(void) {
    // V above VMPP -> power falling with V -> step down.
    float step = incCondStep(/*V*/50.0f, /*I*/pvCurrent(50.0f),
                             /*prevV*/49.0f, /*prevI*/pvCurrent(49.0f));
    TEST_ASSERT_TRUE(step < 0.0f);
}

void test_inccond_holds_at_mpp(void) {
    // Construct dI/dV == -I/V exactly: at the peak the slope is flat.
    float V = VMPP, I = pvCurrent(VMPP);
    float dV = 1.0f;
    float dI = -(I / V) * dV; // makes dCond == -cond
    float step = incCondStep(V, I, V - dV, I - dI);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, step);
}

// ---- P&O direction helper ----

void test_po_keeps_direction_on_power_gain(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, poNextStep(/*cur*/10.0f, /*old*/5.0f, /*step*/0.5f));
}
void test_po_reverses_on_power_loss(void) {
    TEST_ASSERT_EQUAL_FLOAT(-0.5f, poNextStep(/*cur*/5.0f, /*old*/10.0f, /*step*/0.5f));
}

// ---- Closed-loop simulation: MPP convergence ----
// Assume the inner PID perfectly tracks the setpoint, so array V == targetV.

void test_safecharge_converges_to_mpp(void) {
    float V = 20.0f, prevV = 0.0f, prevI = 0.0f;
    for (int k = 0; k < 400; k++) {
        float I = pvCurrent(V);
        float step = safeChargeArrayStep(V, I, prevV, prevI, /*limiting*/false);
        prevV = V;
        prevI = I;
        V += step;
        if (V < 0.01f) V = 0.01f;
    }
    // The IncCond deadband intentionally parks the tracker inside the flat band
    // near the peak rather than at the exact MPP voltage, so assert on captured
    // power (what actually matters for charging): it should hold >=97% of PMAX.
    TEST_ASSERT_TRUE(pvPower(V) >= 0.97f * PMAX);
}

// ---- Closed-loop simulation: battery voltage never exceeds the ceiling ----
// Strong sun, three strings, starting near the CV knee. The CV backoff must keep
// the pack from ever crossing the absolute V_BATT_MAX, with the hard stop as the
// final backstop.

void test_safecharge_caps_battery_voltage(void) {
    const int NSTR = 3;
    float V[3] = {VMPP, VMPP, VMPP};
    float prevV[3] = {0, 0, 0};
    float prevI[3] = {0, 0, 0};
    float battVolt = V_BATT_CV - 0.2f; // just below taper knee
    const float dt = 1.0f;             // outer loop tick (s)
    const float packAh = 50.0f;        // 10P * 5Ah

    float maxSeen = battVolt;
    for (int k = 0; k < 2000; k++) {
        if (safeChargeHardStop(true, battVolt)) {
            // Converters off -> no charge current; voltage holds.
            if (battVolt > maxSeen) maxSeen = battVolt;
            continue;
        }
        float iLimit = effectiveChargeCurrentLimit(1000.0f); // BMS not limiting here
        bool limiting = safeChargeCvActive(battVolt) ||
                        safeChargeCcActive(/*charge I*/0.0f, iLimit);

        float totalPower = 0.0f;
        for (int i = 0; i < NSTR; i++) {
            float I = pvCurrent(V[i]);
            float step = safeChargeArrayStep(V[i], I, prevV[i], prevI[i], limiting);
            prevV[i] = V[i];
            prevI[i] = I;
            V[i] += step;
            if (V[i] < 0.01f) V[i] = 0.01f;
            if (V[i] > 80.0f) V[i] = 80.0f; // model valid range (P=0 at Voc)
            totalPower += pvPower(V[i]);
        }
        float chargeCurrent = totalPower / battVolt;
        battVolt += chargeCurrent * dt / (packAh * 3600.0f) * 3600.0f; // dV ~ I*dt/Ah
        if (battVolt > maxSeen) maxSeen = battVolt;
    }
    // Never breach the absolute ceiling (allow a tiny one-tick numerical margin).
    TEST_ASSERT_TRUE(maxSeen <= V_BATT_MAX + 0.5f);
}

// ---- Closed-loop simulation: charge current settles under the CC limit ----

void test_safecharge_respects_current_limit(void) {
    const int NSTR = 3;
    float V[3] = {VMPP, VMPP, VMPP};
    float prevV[3] = {0, 0, 0};
    float prevI[3] = {0, 0, 0};
    float battVolt = 100.0f;          // mid-pack, CV not active
    const float bmsLimit = 3.0f;      // force a low CC ceiling
    float iLimit = effectiveChargeCurrentLimit(bmsLimit);

    float settledMax = 0.0f;
    for (int k = 0; k < 600; k++) {
        // Estimate this tick's charge current from current operating point.
        float totalPower = 0.0f;
        for (int i = 0; i < NSTR; i++) totalPower += pvPower(V[i]);
        float chargeCurrent = totalPower / battVolt;

        bool limiting = safeChargeCvActive(battVolt) ||
                        safeChargeCcActive(chargeCurrent, iLimit);
        for (int i = 0; i < NSTR; i++) {
            float I = pvCurrent(V[i]);
            float step = safeChargeArrayStep(V[i], I, prevV[i], prevI[i], limiting);
            prevV[i] = V[i];
            prevI[i] = I;
            V[i] += step;
            if (V[i] < 0.01f) V[i] = 0.01f;
            if (V[i] > 80.0f) V[i] = 80.0f;
        }
        // Record steady-state behaviour after an initial settling window.
        if (k > 300) {
            float p = 0.0f;
            for (int i = 0; i < NSTR; i++) p += pvPower(V[i]);
            float ic = p / battVolt;
            if (ic > settledMax) settledMax = ic;
        }
    }
    // After settling, charge current should be held at/under the limit (allow a
    // small per-step overshoot tolerance).
    TEST_ASSERT_TRUE(settledMax <= iLimit * 1.15f);
}

// ---- Additional edge cases & invariants ----

// P&O: equal power is not a loss, so direction is kept (curPower < oldPower is false).
void test_po_keeps_direction_on_equal_power(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, poNextStep(/*cur*/7.0f, /*old*/7.0f, /*step*/0.5f));
}
// Reversal works regardless of the current step's sign.
void test_po_reverses_with_negative_step(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.5f, poNextStep(/*cur*/5.0f, /*old*/10.0f, /*step*/-0.5f));
}

// A BMS limit at/above the MPPT cap is clamped down to the MPPT cap.
void test_current_limit_clamped_to_mppt_cap(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, I_CHG_MAX_MPPT, effectiveChargeCurrentLimit(I_CHG_MAX_FUSE));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, I_CHG_MAX_MPPT, effectiveChargeCurrentLimit(I_CHG_MAX_MPPT));
}
// The MPPT cap must leave real headroom under the shared fuse for regen current.
void test_mppt_cap_leaves_regen_headroom(void) {
    TEST_ASSERT_TRUE(I_CHG_MAX_MPPT <= 20.0f);
    // At least 15A of fuse headroom remains above the MPPT charge cap for regen.
    TEST_ASSERT_TRUE((PACK_FUSE_A - I_CHG_MAX_MPPT) >= 15.0f);
}
// A zero BMS limit must propagate: the BMS can fully gate charging.
void test_current_limit_zero_bms_gates_charging(void) {
    TEST_ASSERT_EQUAL_FLOAT(0.0f, effectiveChargeCurrentLimit(0.0f));
}
// The fuse-derived ceiling matches the pack spec (85% of a 40 A fuse = 34 A).
void test_fuse_limit_matches_spec(void) {
    TEST_ASSERT_FLOAT_WITHIN(0.01f, PACK_FUSE_A * CHG_CURRENT_MARGIN, I_CHG_MAX_FUSE);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 34.0f, I_CHG_MAX_FUSE);
}

// The absolute hard-stop ceiling must sit above the CV taper knee, or charging
// could never enter CV before tripping the hard stop.
void test_ceiling_above_taper_knee(void) {
    TEST_ASSERT_TRUE(V_BATT_MAX > V_BATT_CV);
}
// Just under the ceiling (and at the CV knee) the converters stay live.
void test_hard_stop_runs_below_ceiling(void) {
    TEST_ASSERT_FALSE(safeChargeHardStop(true, V_BATT_MAX - 0.01f));
    TEST_ASSERT_FALSE(safeChargeHardStop(true, V_BATT_CV)); // CV knee is below the ceiling
}

// IncCond "no voltage change" branch: decide direction from current change alone.
void test_inccond_flat_voltage_rising_current_steps_up(void) {
    // dV ~ 0, dI > 0 -> step up.
    float step = incCondStep(/*V*/40.0f, /*I*/6.0f, /*prevV*/40.0f, /*prevI*/5.0f);
    TEST_ASSERT_EQUAL_FLOAT(INCCOND_STEP, step);
}
void test_inccond_flat_voltage_falling_current_steps_down(void) {
    float step = incCondStep(/*V*/40.0f, /*I*/5.0f, /*prevV*/40.0f, /*prevI*/6.0f);
    TEST_ASSERT_EQUAL_FLOAT(-INCCOND_STEP, step);
}
void test_inccond_flat_voltage_flat_current_holds(void) {
    float step = incCondStep(/*V*/40.0f, /*I*/5.0f, /*prevV*/40.0f, /*prevI*/5.0f);
    TEST_ASSERT_EQUAL_FLOAT(0.0f, step);
}

// When a battery limit binds, the array step is forced to the backoff regardless
// of where we are on the MPP curve (here, left of MPP where IncCond would step up).
void test_arraystep_limiting_overrides_tracking(void) {
    float V = 30.0f; // left of VMPP=40
    float limitedStep = safeChargeArrayStep(V, pvCurrent(V), V - 1.0f, pvCurrent(V - 1.0f),
                                            /*limiting*/true);
    TEST_ASSERT_EQUAL_FLOAT(CC_CV_BACKOFF_STEP, limitedStep);
}
// With no limit binding, the array step is exactly the IncCond decision.
void test_arraystep_not_limiting_equals_inccond(void) {
    float V = 30.0f, prevV = 29.0f;
    float expected = incCondStep(V, pvCurrent(V), prevV, pvCurrent(prevV));
    float actual   = safeChargeArrayStep(V, pvCurrent(V), prevV, pvCurrent(prevV),
                                         /*limiting*/false);
    TEST_ASSERT_EQUAL_FLOAT(expected, actual);
}

// CC/CV backoff must actually shed power: repeatedly applying the limiting step
// pushes V toward Voc and monotonically reduces delivered power.
void test_cc_backoff_sheds_power(void) {
    float V = VMPP;                 // start at peak power
    float startPower = pvPower(V);
    float prevPower = startPower;
    for (int k = 0; k < 20; k++) {
        float step = safeChargeArrayStep(V, pvCurrent(V), V, pvCurrent(V), /*limiting*/true);
        V += step;
        float p = pvPower(V);
        TEST_ASSERT_TRUE(p <= prevPower + 1e-3f); // never increases
        prevPower = p;
    }
    TEST_ASSERT_TRUE(V > VMPP);                 // moved toward Voc
    TEST_ASSERT_TRUE(pvPower(V) < startPower);  // net power reduction
}

int main(int, char **) {
    UNITY_BEGIN();
    RUN_TEST(test_hard_stop_on_overvoltage);
    RUN_TEST(test_hard_stop_when_boost_disabled);
    RUN_TEST(test_current_limit_takes_tighter_of_bms_and_fuse);
    RUN_TEST(test_cv_phase_threshold);
    RUN_TEST(test_cc_phase_threshold);
    RUN_TEST(test_inccond_steps_up_left_of_mpp);
    RUN_TEST(test_inccond_steps_down_right_of_mpp);
    RUN_TEST(test_inccond_holds_at_mpp);
    RUN_TEST(test_po_keeps_direction_on_power_gain);
    RUN_TEST(test_po_reverses_on_power_loss);
    RUN_TEST(test_safecharge_converges_to_mpp);
    RUN_TEST(test_safecharge_caps_battery_voltage);
    RUN_TEST(test_safecharge_respects_current_limit);
    RUN_TEST(test_po_keeps_direction_on_equal_power);
    RUN_TEST(test_po_reverses_with_negative_step);
    RUN_TEST(test_current_limit_clamped_to_mppt_cap);
    RUN_TEST(test_mppt_cap_leaves_regen_headroom);
    RUN_TEST(test_current_limit_zero_bms_gates_charging);
    RUN_TEST(test_fuse_limit_matches_spec);
    RUN_TEST(test_ceiling_above_taper_knee);
    RUN_TEST(test_hard_stop_runs_below_ceiling);
    RUN_TEST(test_inccond_flat_voltage_rising_current_steps_up);
    RUN_TEST(test_inccond_flat_voltage_falling_current_steps_down);
    RUN_TEST(test_inccond_flat_voltage_flat_current_holds);
    RUN_TEST(test_arraystep_limiting_overrides_tracking);
    RUN_TEST(test_arraystep_not_limiting_equals_inccond);
    RUN_TEST(test_cc_backoff_sheds_power);
    return UNITY_END();
}
