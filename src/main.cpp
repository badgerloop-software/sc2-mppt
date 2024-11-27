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
    printf("Target Voltage: %f\n", targetVoltage);
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
    printf("%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f,%5.2f\n",
            arrayData[0].voltage, arrayData[0].current,arrayData[0].temp,
            arrayData[1].voltage, arrayData[1].current,arrayData[1].temp,
            arrayData[2].voltage, arrayData[2].current,arrayData[2].temp,
            battVolt, targetVoltage);
}
#elif DEBUG_PRINT == 3
// array 0 printout only
void debugPrint() {
    printf("V: %5.2f || targetV_C: %5.2f || I: %5.2f || Out_I: %5.2f || P: %5.2f || PWM: %5.2f || targetV: %5.2f || BoostEn: %i || battV: %5.2f || Mode: %s || errorV: %5.2f\n",
            arrayData[0].voltage, targetVoltage_C[0], arrayData[0].current, outputCurrent, arrayData[0].curPower, arrayData[0].dutyCycle,
            targetVoltage[0], boostEnabled, battVolt, (bool)chargeMode ? "MPPT" : "Current", targetVoltage[0] - arrayData[0].voltage);
}
#endif


void setup() {
  #if DEBUG_PRINT
    int counter = 0;
  #endif
  #if DEBUG_PRINT == 2
    printf("voltage0,current0,temp0,voltage1,current1,temp1,voltage2,current2,temp2,battVolt,targVolt\n");
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
    if (!past_boostenabled && boostEnabled) {
      resetPID();
    }
    past_boostenabled = boostEnabled;


    canBus.sendMPPTData();
    canBus.runQueue(DATA_SEND_PERIOD);
}