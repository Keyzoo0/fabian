#include "library.h"

void setup() {
  Serial.begin(115200);
  setupMotor();
  
  // Set initial PID setpoints (X=0 untuk lurus, D=0 untuk diam)
  setPIDSetpoints(0, 0);
  
  Serial.println("Robot differential drive ready!");
  Serial.println("Send format: X:value,Y:value,D:value");
}


void loop() {
  // Parse input PID dari serial
  parsePIDInput();
  
  // Compute PID dan update target PWM
  computePID();
  
  // Update PWM secara gradual (dari command.ino)
  updatePWM();
  
  // Write PWM ke motor
  writePwm();
  
  delay(1);
}