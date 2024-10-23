#include "IOManagement.h"
#include <array>

// TODO: PID, TimeOut, Thermistor, pwm

// Solar array voltage, current, and PWM pins (controlled by PID) and storage variable
volatile ArrayData arrayData[NUM_ARRAYS];

struct ArrayPins {
    uint8_t voltPin;
    INA281Driver currPin;      // need to rewrite
    PID pidController;
    uint8_t pwmPin;             // (missing period_us)
};

ArrayPins arrayPins[NUM_ARRAYS] = {
    {(uint8_t)VOLT_PIN_1, INA281Driver(CURR_PIN_1, INA_SHUNT_R), PID(&arrayData[0].voltage, &arrayData[0].outputPWM, &arrayData[0].setPoint, P_TERM, I_TERM, D_TERM, 1), (uint8_t)PWM_OUT_1},
    {(uint8_t)VOLT_PIN_2, INA281Driver(CURR_PIN_2, INA_SHUNT_R), PID(&arrayData[1].voltage, &arrayData[1].outputPWM, &arrayData[1].setPoint, P_TERM, I_TERM, D_TERM, 1), (uint8_t)PWM_OUT_2},
    {(uint8_t)VOLT_PIN_3, INA281Driver(CURR_PIN_3, INA_SHUNT_R), PID(&arrayData[2].voltage, &arrayData[2].outputPWM, &arrayData[2].setPoint, P_TERM, I_TERM, D_TERM, 1), (uint8_t)PWM_OUT_3}
};

// Enables PWM-Voltage converters
volatile bool boostEnabled;

// Battery voltage pin and storage
volatile float battVolt;

// Temperature reading pins (single ADC, select thermistor via multiplexer)
Thermistor thermPin(NCP21XM472J03RA_Constants, PA_0, 10000);


// Misc controlled outputs. Default to nominal state
void completeOVFaultReset();
TimeoutCallback ovFaultResetDelayer((unsigned long)OV_FAULT_RST_PERIOD, &completeOVFaultReset);

// Pack charge current limit
volatile float packSOC = 100;
volatile float packChargeCurrentLimit = 10;
volatile float packCurrent = 0; 

// Current storage
volatile float outputCurrent = 0;

// Charging algorithm mode
volatile ChargeMode chargeMode = ChargeMode::CONST_CURR;

// Ticker to poll input readings at fixed rate
void updateData();
Ticker dataUpdater(updateData, IO_UPDATE_PERIOD, 0, MICROS);

// Updates arrayData with new input values and PWM outputs based on PID loop
void updateData() {
    float totalPower = 0;

    for (int i = 0; i < NUM_ARRAYS; i++) {
        // Update temperature mux selection at start for time to update, then read at end
        // Inputs corresponds to bits 0 and 1 of array number
        digitalWrite(THERM_MUX_SEL_0, i & 0x1);
        digitalWrite(THERM_MUX_SEL_1, i & 0x2);
        arrayData[i].voltage = analogRead(arrayPins[i].voltPin) * V_SCALE;
        arrayData[i].current = arrayPins[i].currPin.readCurrent();
        arrayData[i].curPower = arrayData[i].voltage * arrayData[i].current;
        arrayData[i].dutyCycle = analogRead(arrayPins[i].pwmPin);
        arrayData[i].temp = thermPin.get_temperature();

        totalPower += arrayData[i].curPower;
    }

    for (int i = 0; i < NUM_ARRAYS; i++) {
        if (arrayData[i].voltage > V_MAX || chargeMode == ChargeMode::CONST_CURR) {
            analogWrite(arrayPins[i].pwmPin, 0);
        
        } else {
            arrayPins[i].pidController.Compute();
            analogWrite(arrayPins[i].pwmPin, arrayData[i].outputPWM);
        }
    }

    boostEnabled = digitalRead(BOOST_ENABLED_PIN);
    battVolt = analogRead(BATTERY_VOLT_PIN) * BATT_V_SCALE;

    outputCurrent = totalPower / battVolt; // only used in debug printouts now.

    // Failed to program CONST_CURR_THRESH in time, change mode based on SOC instead
    if (packSOC < 98) chargeMode = ChargeMode::MPPT;
    else chargeMode = ChargeMode::CONST_CURR;
    /*
    if (packCurrent > CONST_CURR_THRESH) chargeMode = ChargeMode::CONST_CURR;
    else if (packCurrent < MPPT_THRESH) chargeMode = ChargeMode::MPPT;
    */
}

void initPID(int array) {
   arrayPins[array].pidController.SetOutputLimits(PWM_DUTY_MIN, PWM_DUTY_MAX);
   arrayPins[array].pidController.SetMode(1);                                              // Auto Mode
   arrayData[array].setPoint = INIT_VOLT;
   arrayPins[array].pidController.SetSampleTime(IO_UPDATE_PERIOD);
}

void initData(int updatePeriod) {
    // PID and PWM setup
    for (int i = 0; i < NUM_ARRAYS; i++) {
        initPID(i);

        // PWM Hardware Timer
        PinName pinNameToUse = digitalPinToPinName(arrayPins[i].pwmPin);
        TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(pinNameToUse, PinMap_PWM);
        if (Instance != nullptr)
        {
            HardwareTimer *MyTim = new HardwareTimer(Instance);
            MyTim->setPWM(  channel,                        // Arduino channel [1..4], unsure what to use
                            pinNameToUse,                       // pin used
                            1/(PWM_PERIOD_US * pow(10, -6)),    // frequency
                            0,                                  // duty cycle
                            PeriodCallback[index]           // timer period callback (timer rollover upon update event) (unsure)
                                                            // timer compare callback (extra parameter in constructor, not used in example)
                        );
        }
    }
    
    // initializing pins
    pinMode(BOOST_ENABLED_PIN, INPUT);
    pinMode(THERM_MUX_SEL_0, OUTPUT);
    pinMode(THERM_MUX_SEL_1, OUTPUT);
    pinMode(OV_FAULT_RST_PIN, OUTPUT);
    pinMode(DISCHARGE_CAPS_PIN, OUTPUT);
    digitalWrite(OV_FAULT_RST_PIN, LOW);
    digitalWrite(DISCHARGE_CAPS_PIN, HIGH);
}

void resetPID() {
    for (int i = 0; i < NUM_ARRAYS; i++) {
        // arrayPins[i].pidController.reset();
        arrayPins[i].pidController = PID(&arrayData[i].voltage, &arrayData[i].outputPWM, &arrayData[i].setPoint, P_TERM, I_TERM, D_TERM, 1);
        initPID(i);
    }
}

void resetArrayPID(int array) {
    // arrayPins[array].pidController.reset();
    arrayPins[array].pidController = PID(&arrayData[array].voltage, &arrayData[array].outputPWM, &arrayData[array].setPoint, P_TERM, I_TERM, D_TERM, 1);
    initPID(array);
}

void setVoltOut(double voltage) {
    if (voltage > V_TARGET_MAX) voltage = V_TARGET_MAX;
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayData[i].setPoint = voltage;
    }
}

void setArrayVoltOut(double voltage, int array) {
    if (voltage > V_TARGET_MAX) voltage = V_TARGET_MAX;
    arrayData[array].setPoint = voltage;
}

/**
    On CAN command, holds the OV fault reset low for fixed period of time to simulate
    manual reset via button press
*/
void completeOVFaultReset() {
    digitalWrite(OV_FAULT_RST_PIN, LOW);
}

void clearOVFaultReset(uint8_t value) {
    digitalWrite(OV_FAULT_RST_PIN, value);
    ovFaultResetDelayer.start();
}

void setCapDischarge(uint8_t value) {
    digitalWrite(DISCHARGE_CAPS_PIN, value);
}