
void readENC_A() {
  if (digitalRead(ENC1B) > 0) {
    countEnc_A++;
    pulseL++;
  } else {
    countEnc_A--;
    pulseL--;
  }
}

void readENC_B() {
  if (digitalRead(ENC2B) > 0) {
    countEnc_B++;
    pulseR++;
  } else {
    countEnc_B--;
    pulseR--;
  }
}

// Function to calculate RPM for Motor 1
void calculateRPM1() {
  // Calculate RPM
  rpmMotor1 = pulseL * 60 / 104;  // Assuming 104 pulses per revolution for motor 1
  pulseL = 0;  // Reset pulse count
  lastMillisMotor1 = millis();  // Reset timer for next RPM calculation
}

// Function to calculate RPM for Motor 2
void calculateRPM2() {
  // Calculate RPM
  rpmMotor2 = pulseR * 60 / 104;  // Assuming 104 pulses per revolution for motor 2
  pulseR = 0;  // Reset pulse count
  lastMillisMotor2 = millis();  // Reset timer for next RPM calculation
}

void initEncoder(bool state) {
  pinMode(LED, OUTPUT);
  pinMode(ENC1A, INPUT);
  pinMode(ENC1B, INPUT);
  pinMode(ENC2A, INPUT);
  pinMode(ENC2B, INPUT);

  if (state) {
    // Attach interrupts for both motors
    attachInterrupt(digitalPinToInterrupt(ENC1A), readENC_A, RISING);  // Motor 1
    attachInterrupt(digitalPinToInterrupt(ENC2A), readENC_B, RISING);  // Motor 2
  } else {
    // Detach interrupts when not needed
    detachInterrupt(digitalPinToInterrupt(ENC1A));
    detachInterrupt(digitalPinToInterrupt(ENC2A));
  }
}


void debugEncoder() {
  // Calculate RPM for motor 1 every second
  if (millis() - lastMillisMotor1 >= interval) {
    calculateRPM1();
  }

  // Calculate RPM for motor 2 every second
  if (millis() - lastMillisMotor2 >= interval) {
    calculateRPM2();
  }

  // Print RPM values for both motors
  Serial.printf("RPM Motor 1: %ld\t", rpmMotor1);
  Serial.printf("RPM Motor 2: %ld\n", rpmMotor2);

}
