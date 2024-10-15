#include "thermistor.h"
#include <math.h>

/*
    Initialize all parameters and pin
*/
Thermistor::Thermistor(const Thermistor_Constants constants, int therm_pin, float R, float vcc) : thermVoltPin(therm_pin) {
    this->mult_const = constants.mult_const;
    this->add_const = constants.add_const;
    this->vcc = vcc;
    this->R = R;
}

/*
    Retrive new temperature reading
*/
float Thermistor::get_temperature() {
    int valueRead = analogRead(this->thermVoltPin);
    float therm_voltage = (float)valueRead * 3.3/1024.0;
    float R_Therm = R / (vcc / therm_voltage - 1);

    return mult_const * std::log(R_Therm) + add_const;
}