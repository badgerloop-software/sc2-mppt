#ifndef __BOARD_CONFIG_H__
#define __BOARD_CONFIG_H__

// ------------- TIMING -------------
#define IO_UPDATE_PERIOD 100000                                   // us
#define PID_UPDATE_PERIOD ((float)IO_UPDATE_PERIOD / 1000000.0f)  // PID interval is in seconds
#define MPPT_UPDATE_PERIOD (10 * IO_UPDATE_PERIOD)
#define DATA_SEND_PERIOD 50         // ms, CAN TX (+ debug if on)
#define OV_FAULT_RST_PERIOD 250000  // us

// ------------- DEBUG -------------
// only enable when serial is connected
// 0 off (not in flash) | 1 human | 2 csv | 3 per-array
#define SC2_DEBUG 0

// ------------- ARRAY COUNT -------------
// 2 or 3 depending on which MPPT board this is (relay board split)
#define NUM_ARRAYS 3

// ------------- IO PINS / SCALING -------------
// from divider circuitry
constexpr float V_SCALE = (103.3f / 3.3f) * 3.33f;
constexpr float BATT_V_SCALE = 3.325f * 101.0f;

// Array voltage ADC channels
#define VOLT_CHANNEL_1 ADC_CHANNEL_11  // PA_6
#define VOLT_CHANNEL_2 ADC_CHANNEL_9   // PA_4
#define VOLT_CHANNEL_3 ADC_CHANNEL_6   // PA_1

// Array current ADC channels (INA281)
#define CURR_PIN_1 ADC_CHANNEL_12  // PA_7
#define CURR_PIN_2 ADC_CHANNEL_10  // PA_5
#define CURR_PIN_3 ADC_CHANNEL_8   // PA_3

#define THERM_PIN ADC_CHANNEL_5  // PA_0
#define THERM_RESISTANCE 10000   // 10k
#define INA_SHUNT_R 0.01f

#define BATTERY_VOLT_CHANNEL ADC_CHANNEL_15  // PB_0

// Thermistor mux select
#define THERM_MUX_SEL_1 D12  // PB_4
#define THERM_MUX_SEL_0 D11  // PB_5

#define BOOST_ENABLED_PIN D4  // PB_7

// PWM outs for array voltage control
#define PWM_OUT_1 PA_9   // D1
#define PWM_OUT_2 PA_10  // D0
#define PWM_OUT_3 PA_8   // D9

// CAN-triggered digital outs
#define OV_FAULT_RST_PIN D5    // PB_6
#define DISCHARGE_CAPS_PIN D6  // PB_1

#define V_MAX 110  // PWM forced off above this

// ------------- PID / PWM -------------
#define P_TERM -0.7f
#define I_TERM 0.2f
#define D_TERM 0.0f

#define PID_IN_MIN 0
#define PID_IN_MAX 115

#define PWM_DUTY_MIN 0.0f
#define PWM_DUTY_MAX 0.72f
#define PWM_FREQ 100000  // Hz

// ------------- MPPT ALGO -------------
#define INIT_VOLT 9.0f
#define INIT_VOLT_STEP -0.5f
#define V_TARGET_MAX 105.0f         // keep a bit under V_MAX
#define CONST_CURR_SAFETY_MULT 0.9f  // derate BMS limit

// if PWM stuck at a limit, move targetVoltage
#define MOVE_VOLTAGE 5.0f
#define MAX_STUCK_CYCLES 5

#define MPPT_ALGO_PO 0
#define MPPT_ALGO_SAFE_CHARGE 1
#define DEFAULT_MPPT_ALGO MPPT_ALGO_SAFE_CHARGE

// ------------- PACK (29S10P Samsung INR21700-50S) -------------
#define PACK_SERIES_CELLS 29
#define PACK_PARALLEL_CELLS 10
#define CELL_V_CHG_MAX 4.20f
#define CELL_V_CV_TARGET 4.18f
#define PACK_FUSE_A 40.0f
#define CHG_CURRENT_MARGIN 0.85f

#define V_BATT_MAX 118.8f  // stop boosting
#define V_BATT_CV 118.2f   // start CV backoff
#define I_CHG_MAX_FUSE (PACK_FUSE_A * CHG_CURRENT_MARGIN)

#define INCCOND_STEP 0.3f
#define INCCOND_DEADBAND 0.05f
#define CC_CV_BACKOFF_STEP 0.5f

#endif  // __BOARD_CONFIG_H__
