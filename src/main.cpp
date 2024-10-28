#include <Arduino.h>
#include "const.h"
#include "IOManagement.h"
#include "mppt.h"
#include "canMppt.h"

void debugPrint() {
  printf("hi\n");
}

void setup() {
  initData();
}

void loop() {
  // update IOManagement Ticker
  // update printout Ticker
}