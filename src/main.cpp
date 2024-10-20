#include <Arduino.h>

#define PWM_PIN PA_9
#define PWM_FREQ 76923
HardwareTimer *MyTim;
uint32_t channel;

void setup() {
  // Automatically retrieve TIM instance and channel associated to pin
  // This is used to be compatible with all STM32 series automatically.
  TIM_TypeDef *Instance = (TIM_TypeDef *)pinmap_peripheral(PWM_PIN, PinMap_PWM);
  MyTim = new HardwareTimer(Instance);

  uint32_t dutyCycle = 20; // in percentage, so 20 = 20%
  channel = STM_PIN_CHANNEL(pinmap_function(PWM_PIN, PinMap_PWM));

  MyTim->setPWM(channel, PWM_PIN, PWM_FREQ, dutyCycle); // set duty cycle
}

void loop() {
  // test setting different dutyCycles
  delay(5000);
  MyTim->setPWM(channel, PWM_PIN, PWM_FREQ, 20);
  delay(5000);
  MyTim->setPWM(channel, PWM_PIN, PWM_FREQ, 50);
}