#include <Arduino.h>

#include "canMppt.h"
#include "const.h"
#include "IOManagement.h"
#include "mppt.h"
#include "adc.h"

int counter;
bool past_boostenabled;
CANMPPT canBus(CAN1, DEF);

#if DEBUG_PRINT == 1
void debugPrint() {
    printf("\033[2J\033[1;1H");
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf("Array %i\nVoltage %5.2fV\t\tCurrent %5.2fA\nPower %5.2fW\t\tDuty Cycle %.4f\nTemp %.4f\n", 
                i+1, arrayData[i].voltage, arrayData[i].current, arrayData[i].curPower, 
                arrayData[i].dutyCycle, arrayData[i].temp);
    }
    printf("Boost enable: %i\nBattery Voltage: %5.2f\n", boostEnabled, battVolt);
    printf("Mode: %s\n", (bool)chargeMode ? "MPPT" : "Current");
    printf("Current Limit: %f\n", packChargeCurrentLimit);
    printf("Target Voltage: %f\n", targetVoltage[0]);
    // Compute current output current for feedback
    float totalInputPower = 0;
    for (int i = 0; i < NUM_ARRAYS; i++) {
        totalInputPower += arrayData[i].curPower;
    }
    float outputCurr = totalInputPower / battVolt;
    printf("Output Current: %f\n", outputCurr);
}   
#elif DEBUG_PRINT == 2
void debugPrint() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf("%5.2f,%5.2f,%5.2f,", arrayData[i].voltage, arrayData[i].current, arrayData[i].temp);
    }
    printf("%5.2f,%5.2f\n", battVolt, targetVoltage[0]);
}
#elif DEBUG_PRINT == 3
// array 0 printout only
void debugPrint() {
    // printf("V: %5.2f || targetV_C: %5.2f || I: %5.2f || Out_I: %5.2f || P: %5.2f || PWM: %5.2f || targetV: %5.2f || BoostEn: %i || battV: %5.2f || Mode: %s || errorV: %5.2f\n",
    //         arrayData[0].voltage, targetVoltage_C[0], arrayData[0].current, outputCurrent, arrayData[0].curPower, arrayData[0].dutyCycle,
    //         targetVoltage[0], boostEnabled, battVolt, (bool)chargeMode ? "MPPT" : "Current", targetVoltage[0] - arrayData[0].voltage);

    for (int i = 0; i < NUM_ARRAYS; i++) {
        printf("Arr %d -> V: %5.2f || targetV_C: %5.2f || I: %5.2f || Out_I: %5.2f || P: %5.2f || PWM: %5.2f || targetV: %5.2f || BoostEn: %i || battV: %5.2f || Mode: %s || errorV: %5.2f\n",
                i+1, arrayData[i].voltage, targetVoltage_C[i], arrayData[i].current, outputCurrent, arrayData[i].curPower, arrayData[i].dutyCycle,
                targetVoltage[i], boostEnabled, battVolt, (bool)chargeMode ? "MPPT" : "Current", targetVoltage[i] - arrayData[i].voltage);
    }
    printf("----------------------------------------------------------------------------------------------------------\n");
}
#endif


void setup() {

  // FIRMWARE FIX FOR THE FLOATING PINS:
  // Before doing ANY delays, instantly configure the PWM pins as outputs 
  // and force them LOW. This mimics the old firmware's rapid boot speed,
  // preventing the gate drivers from seeing a floating signal and saturating the inductor.

  pinMode(PWM_OUT_1, OUTPUT); digitalWrite(PWM_OUT_1, LOW);
  pinMode(PWM_OUT_2, OUTPUT); digitalWrite(PWM_OUT_2, LOW);
  pinMode(PWM_OUT_3, OUTPUT); digitalWrite(PWM_OUT_3, LOW);

  pinMode(DISCHARGE_CAPS_PIN, OUTPUT); digitalWrite(DISCHARGE_CAPS_PIN, HIGH);
  pinMode(OV_FAULT_RST_PIN, OUTPUT); digitalWrite(OV_FAULT_RST_PIN, LOW);
  // 1. HARDWARE STABILIZATION DELAY
  // Wait 2 seconds before doing ANYTHING. This gives the system to fully stabilize, and prevent brownouts 
  // delay(2000);
  #if DEBUG_PRINT
    int counter = 0;
  #endif
  #if DEBUG_PRINT == 2
    for (int i = 0; i < NUM_ARRAYS; i++) printf("voltage%d,current%d,temp%d,", i, i, i);
    printf("battVolt,targVolt\n");
  #endif

  Serial.begin(115200);
  initADC(ADC1);
  initData();
  initMPPT();   
  bool past_boostenabled = false;
}

void loop() {
  #if DEBUG_PRINT == 3
    debugPrint();
  #elif DEBUG_PRINT == 1 || DEBUG_PRINT == 4
    // Display digital and analog values every second (for testing) 
    if (counter >= (200 / DATA_SEND_PERIOD)) {
      debugPrint();
      counter = 0;
    }
    counter++;
  #endif

  // 2. CONTINUOUS PID SYNCHRONIZATION
    // When boost is off, constantly lock the target voltage to reality. 
    // This guarantees that the exact millisecond the switch is flipped ON, 
    // the PID error is exactly 0.00. This prevents the initial massive 72% PWM surge 
    // that causes the inductor to crackle, saturate, and EMI-crash the MCU.

  // if (!boostEnabled) {
  //       for (int i = 0; i < NUM_ARRAYS; i++) {
  //           targetVoltage[i] = arrayData[i].voltage;
  //       }
  //       resetPID();
  //   }
  /*
    #endif
    if (!past_boostenabled && boostEnabled) {
      setVoltOut(INIT_VOLT);
      resetPID();
    }
    past_boostenabled = boostEnabled;
  */
  


    canBus.sendMPPTData();
    canBus.runQueue(DATA_SEND_PERIOD);
}