#include "library.h"

void setup() {
  Serial.begin(115200);
  setupMotor();
  // setupEncoder();
  initEncoder(true);  // Initialize encoders with interrupts enabled

}

void loop() {
  writePwm();
  command();
  debugEncoder();
}
