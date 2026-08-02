#include "debug.h"

#if SC2_DEBUG

#include <Arduino.h>

#include "io_management.h"
#include "mppt.h"

// ------------- LOCAL -------------

static int debugCounter;

// ------------- LOCAL FUNCTIONS -------------

static void debugPrint() {
#if SC2_DEBUG == 1
    printf("\033[2J\033[1;1H");
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf(
            "Array %i\nVoltage %5.2fV\t\tCurrent %5.2fA\nPower %5.2fW\t\tDuty Cycle %.4f\nTemp "
            "%.4f\n",
            i + 1, arrayData[i].voltage, arrayData[i].current, arrayData[i].curPower,
            arrayData[i].dutyCycle, arrayData[i].temp);
    }
    printf("Boost enable: %i\nBattery Voltage: %5.2f\n", boostEnabled, battVolt);
    printf("Mode: %s\n", (bool)chargeMode ? "MPPT" : "Current");
    printf("Current Limit: %f\n", packChargeCurrentLimit);
    printf("Target Voltage: %f\n", targetVoltage[0]);
    float totalInputPower = 0;
    for (int i = 0; i < NUM_ARRAYS; i++) {
        totalInputPower += arrayData[i].curPower;
    }
    printf("Output Current: %f\n", totalInputPower / battVolt);
#elif SC2_DEBUG == 2
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf("%5.2f,%5.2f,%5.2f,", arrayData[i].voltage, arrayData[i].current, arrayData[i].temp);
    }
    printf("%5.2f,%5.2f\n", battVolt, targetVoltage[0]);
#elif SC2_DEBUG == 3
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf(
            "Arr %d -> V: %5.2f || targetV_C: %5.2f || I: %5.2f || Out_I: %5.2f || P: %5.2f || "
            "PWM: %5.2f || targetV: %5.2f || BoostEn: %i || battV: %5.2f || Mode: %s || errorV: "
            "%5.2f\n",
            i + 1, arrayData[i].voltage, targetVoltage_C[i], arrayData[i].current, outputCurrent,
            arrayData[i].curPower, arrayData[i].dutyCycle, targetVoltage[i], boostEnabled, battVolt,
            (bool)chargeMode ? "MPPT" : "Current", targetVoltage[i] - arrayData[i].voltage);
    }
    printf(
        "------------------------------------------------------------------------------------------"
        "----------------\n");
#endif
}

// ------------- PUBLIC FUNCTIONS -------------

void debugInit() {
    Serial.begin(115200);
#if SC2_DEBUG == 2
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf("voltage%d,current%d,temp%d,", i, i, i);
    }
    printf("battVolt,targVolt\n");
#endif
}

void debugUpdate() {
#if SC2_DEBUG == 1
    if (debugCounter >= (200 / DATA_SEND_PERIOD)) {
        debugPrint();
        debugCounter = 0;
    }
    debugCounter++;
#else
    // modes 2 and 3 print every loop
    debugPrint();
#endif
}

void debugError(const char* msg) {
    printf("%s\n", msg);
}

#endif  // SC2_DEBUG
