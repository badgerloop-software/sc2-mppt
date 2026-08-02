#include <Arduino.h>

#include "adc.h"
#include "board_config.h"
#include "can_mppt.h"
#include "debug.h"
#include "io_management.h"
#include "mppt.h"

// ------------- LOCAL -------------

static CanMppt canBus(CAN1, DEF);

// ------------- PUBLIC FUNCTIONS -------------

void setup() {
    // hold PWM low so gate drivers aren't floating at boot
    pinMode(PWM_OUT_1, OUTPUT);
    digitalWrite(PWM_OUT_1, LOW);
    pinMode(PWM_OUT_2, OUTPUT);
    digitalWrite(PWM_OUT_2, LOW);
    pinMode(PWM_OUT_3, OUTPUT);
    digitalWrite(PWM_OUT_3, LOW);

    pinMode(DISCHARGE_CAPS_PIN, OUTPUT);
    digitalWrite(DISCHARGE_CAPS_PIN, HIGH);
    pinMode(OV_FAULT_RST_PIN, OUTPUT);
    digitalWrite(OV_FAULT_RST_PIN, LOW);

    debugInit();

    initADC(ADC1);
    initData();
    initMppt();
}

void loop() {
    debugUpdate();

    canBus.sendMpptData();
    canBus.runQueue(DATA_SEND_PERIOD);
}
