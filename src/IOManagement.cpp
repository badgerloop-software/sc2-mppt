#include "IOManagement.h"
#include <array>

// Solar array voltage, current, and PWM pins (controlled by PID) and storage variable
volatile ArrayData arrayData[NUM_ARRAYS];

struct ArrayPins {
    uint32_t voltChannel;
    INA281Driver currPin;      
    PID pidController;
    PinName pwmPin;             
    uint32_t channel;               // channel to use for PWM
    HardwareTimer *pwmTimer;        // timer to set PWM output 
};

ArrayPins arrayPins[NUM_ARRAYS] = {
    {
        VOLT_CHANNEL_1, 
        INA281Driver(CURR_PIN_1, INA_SHUNT_R), 
        PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), 
        PWM_OUT_1
    },
    {
        VOLT_CHANNEL_2, 
        INA281Driver(CURR_PIN_2, INA_SHUNT_R), 
        PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), 
        PWM_OUT_2
    },
    {
        VOLT_CHANNEL_3, 
        INA281Driver(CURR_PIN_3, INA_SHUNT_R), 
        PID(P_TERM, I_TERM, D_TERM, PID_UPDATE_PERIOD), 
        PWM_OUT_3
    }
};

volatile bool boostEnabled; // Enables PWM-Voltage converters
volatile float battVolt; // Battery voltage pin and storage
volatile ChargeMode chargeMode = ChargeMode::CONST_CURR; // Charging algorithm mode

// Pack charge current limit
volatile float packSOC = 100;
volatile float packChargeCurrentLimit = 10;
volatile float packCurrent = 0; 
volatile float outputCurrent = 0; 

// Temperature reading pins (single ADC, select thermistor via multiplexer)
Thermistor thermPin(NCP21XM472J03RA_Constants, THERM_PIN, THERM_RESISTANCE);

// Misc controlled outputs. Default to nominal state
void completeOVFaultReset();
STM32TimerInterrupt ovFaultResetDelayer(TIM7);  

// Ticker to poll input readings at fixed rate
STM32TimerInterrupt dataUpdater(TIM2);

// Updates arrayData with new input values and PWM outputs based on PID loop
void updateData() {
    float totalPower = 0;

    for (int i = 0; i < NUM_ARRAYS; i++) {
        // Update temperature mux selection at start for time to update, then read at end
        // Inputs corresponds to bits 0 and 1 of array number
        digitalWrite(THERM_MUX_SEL_0, i & 0x1);
        digitalWrite(THERM_MUX_SEL_1, i & 0x2);
        arrayData[i].voltage = readADC(arrayPins[i].voltChannel) * V_SCALE;
        arrayData[i].current = arrayPins[i].currPin.readCurrent();
        
        arrayData[i].temp = thermPin.get_temperature();

        arrayData[i].curPower = arrayData[i].voltage * arrayData[i].current;
        totalPower += arrayData[i].curPower;
    }

    for (int i = 0; i < NUM_ARRAYS; i++) {
        if (arrayData[i].voltage > V_MAX || chargeMode == ChargeMode::CONST_CURR) {
            // turn off boost converters 
            arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ, 0);
        } else {
            arrayPins[i].pidController.setProcessValue(arrayData[i].voltage); // real world value, input
            arrayData[i].dutyCycle = arrayPins[i].pidController.compute() * 100; // multiply by 100 because the new PWM is percentage based, goes from 0 to 100
            arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ, arrayData[i].dutyCycle);
        }
    }

    boostEnabled = digitalRead(BOOST_ENABLED_PIN);
    battVolt = readADC(BATTERY_VOLT_CHANNEL) * BATT_V_SCALE;

    outputCurrent = totalPower / battVolt; // only used in debug printouts now.

    // Failed to program CONST_CURR_THRESH in time, change mode based on SOC instead
    if (packSOC < 98) chargeMode = ChargeMode::MPPT;
    else chargeMode = ChargeMode::CONST_CURR;
    /*
    if (packCurrent > CONST_CURR_THRESH) chargeMode = ChargeMode::CONST_CURR;
    else if (packCurrent < MPPT_THRESH) chargeMode = ChargeMode::MPPT;
    */
}

void initData() {
    // PID and PWM setup
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayPins[i].pidController.setInputLimits(PID_IN_MIN, PID_IN_MAX);
        arrayPins[i].pidController.setOutputLimits(PWM_DUTY_MIN, PWM_DUTY_MAX);
        arrayPins[i].pidController.setMode(AUTO_MODE);
        arrayPins[i].pidController.setSetPoint(INIT_VOLT);

        // Initialize PWM Hardware Timer ONCE 
        TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(arrayPins[i].pwmPin, PinMap_PWM);
        arrayPins[i].pwmTimer = new HardwareTimer(Instance);
        arrayPins[i].channel = STM_PIN_CHANNEL(pinmap_function(arrayPins[i].pwmPin, PinMap_PWM));
        arrayPins[i].pwmTimer->setPWM(arrayPins[i].channel, arrayPins[i].pwmPin, PWM_FREQ, 0);
    }
    
    // initialize digital pins
    pinMode(BOOST_ENABLED_PIN, INPUT);
    pinMode(THERM_MUX_SEL_0, OUTPUT);
    pinMode(THERM_MUX_SEL_1, OUTPUT);
    pinMode(OV_FAULT_RST_PIN, OUTPUT);
    pinMode(DISCHARGE_CAPS_PIN, OUTPUT);
    digitalWrite(OV_FAULT_RST_PIN, LOW);
    digitalWrite(DISCHARGE_CAPS_PIN, HIGH);

    // start the Timer to periodically read IO
    if(dataUpdater.attachInterruptInterval(IO_UPDATE_PERIOD, updateData)) { 
        printf("starting dataUpdater timer\n");
    } else {
        printf("problem starting dataUpdater timer\n");
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
    if (voltage > V_TARGET_MAX) voltage = V_TARGET_MAX;
    for (int i = 0; i < NUM_ARRAYS; i++) {
        arrayPins[i].pidController.setSetPoint(voltage);
    }
}

void setArrayVoltOut(float voltage, uint8_t array) {
    if (voltage > V_TARGET_MAX) voltage = V_TARGET_MAX;
    arrayPins[array].pidController.setSetPoint(voltage);
}

/**
    On CAN command, holds the OV fault reset low for fixed period of time to simulate
    manual reset via button press
*/
void completeOVFaultReset() {
    digitalWrite(OV_FAULT_RST_PIN, LOW);
    ovFaultResetDelayer.stopTimer();
}

void clearOVFaultReset(uint8_t value) {
    digitalWrite(OV_FAULT_RST_PIN, value);
    ovFaultResetDelayer.attachInterruptInterval(OV_FAULT_RST_PERIOD, completeOVFaultReset);
}

void setCapDischarge(uint8_t value) {
    digitalWrite(DISCHARGE_CAPS_PIN, value);
}