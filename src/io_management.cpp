#include "io_management.h"

#include "debug.h"
#include "mppt.h"

// ------------- GLOBALS -------------
volatile ArrayData arrayData[NUM_ARRAYS];
volatile bool boostEnabled;
volatile float battVolt;
volatile ChargeMode chargeMode = ChargeMode::CONST_CURR;
volatile float packSOC = 50;  // default until BMS message arrives
volatile float packChargeCurrentLimit = 10;
volatile float packCurrent = 0;
volatile float outputCurrent = 0;

// ------------- LOCAL -------------
struct ArrayPins {
    uint32_t voltChannel;
    INA281Driver currPin;
    PID pidController;
    PinName pwmPin;
    uint32_t channel;         // PWM channel
    HardwareTimer* pwmTimer;  // PWM timer
};

// sized by NUM_ARRAYS (2 or 3 string board)
static ArrayPins arrayPins[NUM_ARRAYS] = {
    {VOLT_CHANNEL_1, INA281Driver(CURR_PIN_1, INA_SHUNT_R),
     PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), PWM_OUT_1, 0, nullptr},
#if NUM_ARRAYS >= 2
    {VOLT_CHANNEL_2, INA281Driver(CURR_PIN_2, INA_SHUNT_R),
     PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), PWM_OUT_2, 0, nullptr},
#endif
#if NUM_ARRAYS >= 3
    {VOLT_CHANNEL_3, INA281Driver(CURR_PIN_3, INA_SHUNT_R),
     PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), PWM_OUT_3, 0, nullptr},
#endif
};

// temp mux into one thermistor ADC
static Thermistor thermPin(NCP21XM472J03RA_Constants, THERM_PIN, THERM_RESISTANCE);

static STM32TimerInterrupt ovFaultResetDelayer(TIM7);
static STM32TimerInterrupt dataUpdater(TIM2);  // polls IO at fixed rate

// ------------- LOCAL FUNCTIONS -------------

static void completeOvFaultReset() {
    digitalWrite(OV_FAULT_RST_PIN, LOW);
    ovFaultResetDelayer.stopTimer();
}

// update arrayData + PWM from PID
static void updateData() {
    float totalPower = 0;
    static bool lastBoostEnabled = false;

    for (int i = 0; i < NUM_ARRAYS; i++) {
        // mux select = bits 0/1 of array index; set first so it can settle
        digitalWrite(THERM_MUX_SEL_0, i & 0x1);
        digitalWrite(THERM_MUX_SEL_1, i & 0x2);
        arrayData[i].voltage = readADC(arrayPins[i].voltChannel) * V_SCALE;
        arrayData[i].current = arrayPins[i].currPin.readCurrent();
        arrayData[i].temp = thermPin.get_temperature();
        arrayData[i].curPower = arrayData[i].voltage * arrayData[i].current;
        totalPower += arrayData[i].curPower;
    }

    boostEnabled = digitalRead(BOOST_ENABLED_PIN);

    // boost just turned on: seed targets near real V to avoid PWM surge
    if (boostEnabled && !lastBoostEnabled) {
        for (int i = 0; i < NUM_ARRAYS; i++) {
            targetVoltage[i] = arrayData[i].voltage - 0.5f;
            setArrayVoltOut(targetVoltage[i], i);
            resetArrayPID(i);
        }
    }
    lastBoostEnabled = boostEnabled;

    for (int i = 0; i < NUM_ARRAYS; i++) {
        // turn off boost converters
        if (!boostEnabled || arrayData[i].voltage > V_MAX || battVolt >= V_BATT_MAX ||
            chargeMode == ChargeMode::CONST_CURR) {
            arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ, 0);
        } else {
            arrayPins[i].pidController.setProcessValue(arrayData[i].voltage);
            // *100 because setPWM duty is 0-100
            arrayData[i].dutyCycle = arrayPins[i].pidController.compute() * 100.0f;
            arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ,
                                          arrayData[i].dutyCycle);
        }
    }

    battVolt = readADC(BATTERY_VOLT_CHANNEL) * BATT_V_SCALE;
    if (battVolt > 2.0f) {
        outputCurrent = totalPower / battVolt;
    } else {
        outputCurrent = 0.0f;
    }

    // P&O: mode from SOC. SafeCharge sets chargeMode itself.
    if (activeAlgo == MpptAlgo::PerturbObserve) {
        if (packSOC < 98) {
            chargeMode = ChargeMode::MPPT;
        } else {
            chargeMode = ChargeMode::CONST_CURR;
        }
    }
}

// ------------- PUBLIC FUNCTIONS -------------

void initData() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayPins[i].pidController.setInputLimits(PID_IN_MIN, PID_IN_MAX);
        arrayPins[i].pidController.setOutputLimits(PWM_DUTY_MIN, PWM_DUTY_MAX);
        arrayPins[i].pidController.setMode(AUTO_MODE);
        arrayPins[i].pidController.setSetPoint(INIT_VOLT);

        TIM_TypeDef* Instance = (TIM_TypeDef*)pinmap_peripheral(arrayPins[i].pwmPin, PinMap_PWM);
        arrayPins[i].pwmTimer = new HardwareTimer(Instance);
        arrayPins[i].channel = STM_PIN_CHANNEL(pinmap_function(arrayPins[i].pwmPin, PinMap_PWM));
        arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ, 0);
    }

    pinMode(BOOST_ENABLED_PIN, INPUT);
    pinMode(THERM_MUX_SEL_0, OUTPUT);
    pinMode(THERM_MUX_SEL_1, OUTPUT);
    pinMode(OV_FAULT_RST_PIN, OUTPUT);
    pinMode(DISCHARGE_CAPS_PIN, OUTPUT);
    digitalWrite(OV_FAULT_RST_PIN, LOW);
    digitalWrite(DISCHARGE_CAPS_PIN, HIGH);

    if (!dataUpdater.attachInterruptInterval(IO_UPDATE_PERIOD, updateData)) {
        debugError("ERROR: dataUpdater timer");
    }
}

void resetPID() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayPins[i].pidController.reset();
    }
}

void resetArrayPID(uint8_t array) {
    arrayPins[array].pidController.reset();
}

void setVoltOut(float voltage) {
    if (voltage > V_TARGET_MAX) {
        voltage = V_TARGET_MAX;
    }
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayPins[i].pidController.setSetPoint(voltage);
    }
}

void setArrayVoltOut(float voltage, uint8_t array) {
    if (voltage > V_TARGET_MAX) {
        voltage = V_TARGET_MAX;
    }
    arrayPins[array].pidController.setSetPoint(voltage);
}

void clearOVFaultReset(uint8_t value) {
    digitalWrite(OV_FAULT_RST_PIN, value);
    ovFaultResetDelayer.attachInterruptInterval(OV_FAULT_RST_PERIOD, completeOvFaultReset);
}

void setCapDischarge(uint8_t value) {
    digitalWrite(DISCHARGE_CAPS_PIN, value);
}
