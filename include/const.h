#ifndef CONST_H
#define CONST_H
// this header should include all relevant constant values so they can all be easily changed

// constants used for testing
#define IO_UPDATE_PERIOD 100000    // us
#define PID_UPDATE_PERIOD (float)IO_UPDATE_PERIOD / 1000000 // divide by 1000000 because PID interval is in seconds

// Frequency of MPPT algo updates
#define MPPT_UPDATE_PERIOD 10 * IO_UPDATE_PERIOD



// ------------- TESTING/OTHER CONSTANTS -------------
// Whether to log data and steps in file. Should only be enabled
// when microcontroller outputting via serial 
// 1 for human readable mode
// 2 for logging/csv mode
// 3 for showing array 0 values only
#define DEBUG_PRINT 1



// ------------- IO INPUT CONSTANTS -------------
#define NUM_ARRAYS 3

// Analog input modifiers (derived from circuitry)
constexpr float V_SCALE = (103.3/3.3)*3.33;
constexpr float BATT_V_SCALE = 3.325 * 101;

//Input pins to read voltage for each solar array
#define VOLT_CHANNEL_1 ADC_CHANNEL_11 // PA_6
#define VOLT_CHANNEL_2 ADC_CHANNEL_9 // PA_4   
#define VOLT_CHANNEL_3 ADC_CHANNEL_6 // PA_1

//Input pins to read current for each solar array
#define CURR_PIN_1 ADC_CHANNEL_12 // PA_7       
#define CURR_PIN_2 ADC_CHANNEL_10 // PA_5
#define CURR_PIN_3 ADC_CHANNEL_8 // PA_3

// Thermistor Pin
#define THERM_PIN ADC_CHANNEL_5 // PA_0     A0 
#define THERM_RESISTANCE 10000 // 10k
#define INA_SHUNT_R 0.01

// Input pin to get battery voltage
#define BATTERY_VOLT_CHANNEL ADC_CHANNEL_15 // PB_0     D3

// Output pins for selecting thermistor via multiplexer
#define THERM_MUX_SEL_1 D12 // PB_4  for some reason this affected PA_7
#define THERM_MUX_SEL_0 D11 // PB_5

// Input pin for enabling boost
#define BOOST_ENABLED_PIN D4 // PB_7

// Output pins for voltage control of arrays via PWM
#define PWM_OUT_1 PA_9      // D1
#define PWM_OUT_2 PA_10     // D0
#define PWM_OUT_3 PA_8      // D9

// CAN triggered outputs
#define OV_FAULT_RST_PIN D5 // PB_6
#define DISCHARGE_CAPS_PIN D6 // PB_1

// Output voltage limit 
#define V_MAX 110

// --------------- PID/PWM CONSTANTS -----------------
// Loop parameters
#define P_TERM -0.7 //-3.65 // -2.9
#define I_TERM -0.2 // -0.06 // -0.1
#define D_TERM 0 //-0.012 // 0

// Input range
#define PID_IN_MIN 0
#define PID_IN_MAX 115

// PID output limits (controls PWM duty cycles)
#define PWM_DUTY_MIN 0
#define PWM_DUTY_MAX 0.8
#define PWM_PERIOD_US 13

// Frequency of PWM signal
#define PWM_FREQ 76923

// ------------- MPPT ALGO CONSTANTS -------------
// Initial voltage
#define INIT_VOLT 9

// initial step size for MPPT updates
#define INIT_VOLT_STEP 0.5

// Maximum voltage the algo loop can target (want safe offset under danger V_MAX)
#define V_TARGET_MAX 105

//Maximum current the algo loop can target would be the pack charge current limit
#define I_TARGET_MAX 20


// Current threshold to switch to constant current
#define CONST_CURR_THRESH packChargeCurrentLimit

// Current threshold to switch to MPPT
#define MPPT_THRESH packChargeCurrentLimit-0.7

#define CONST_CURR_SAFETY_MULT 0.9

// used for variable step size
#define MAX_VOLT_STEP 4
#define MIN_VOLT_STEP 0.5

// try moving targetVoltage when PWM is stuck at either limit 
#define MOVE_VOLTAGE 5
#define MAX_STUCK_CYCLES 5



// How fast to transmit data over CAN in ms (and debug prints if on) 
#define DATA_SEND_PERIOD 50

// Duration undervoltage fault reset asserted on command 
#define OV_FAULT_RST_PERIOD 250000     // us



#endif 