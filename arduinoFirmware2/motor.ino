void writePwm() {
  if (pwm1 > 0) {
    // Jika PWM positif, motor 1A aktif dan motor 1B mati
    ledcWrite(MTR1A_PWM_CHANNEL, pwm1);
    ledcWrite(MTR1B_PWM_CHANNEL, 0);
  } else if (pwm1 < 0) {
    // Jika PWM negatif, motor 1B aktif dan motor 1A mati
    ledcWrite(MTR1A_PWM_CHANNEL, 0);
    ledcWrite(MTR1B_PWM_CHANNEL, -pwm1);
  } else {
    // Jika PWM 0, matikan kedua motor 1A dan 1B
    ledcWrite(MTR1A_PWM_CHANNEL, 0);
    ledcWrite(MTR1B_PWM_CHANNEL, 0);
  }

  if (pwm2 > 0) {
    // Jika PWM positif, motor 2A aktif dan motor 2B mati
    ledcWrite(MTR2A_PWM_CHANNEL, pwm2);
    ledcWrite(MTR2B_PWM_CHANNEL, 0);
  } else if (pwm2 < 0) {
    // Jika PWM negatif, motor 2B aktif dan motor 2A mati
    ledcWrite(MTR2A_PWM_CHANNEL, 0);
    ledcWrite(MTR2B_PWM_CHANNEL, -pwm2);
  } else {
    // Jika PWM 0, matikan kedua motor 2A dan 2B
    ledcWrite(MTR2A_PWM_CHANNEL, 0);
    ledcWrite(MTR2B_PWM_CHANNEL, 0);
  }
}

void setupMotor(){
   pinMode(MTR1A, OUTPUT);
  pinMode(MTR1B, OUTPUT);
  pinMode(MTR2A, OUTPUT);
  pinMode(MTR2B, OUTPUT);

  // Konfigurasi LEDC PWM untuk motor 1 dan motor 2
  ledcSetup(MTR1A_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MTR1B_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MTR2A_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);
  ledcSetup(MTR2B_PWM_CHANNEL, PWM_FREQ, PWM_RESOLUTION);

  // Tentukan pin untuk PWM
  ledcAttachPin(MTR1A, MTR1A_PWM_CHANNEL);
  ledcAttachPin(MTR1B, MTR1B_PWM_CHANNEL);
  ledcAttachPin(MTR2A, MTR2A_PWM_CHANNEL);
  ledcAttachPin(MTR2B, MTR2B_PWM_CHANNEL);
}