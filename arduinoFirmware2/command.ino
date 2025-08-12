// Define a constant for exponential increase
const float expFactor = 0.05;  // Adjust this value for exponential speed (lower for slower, higher for faster)

int targetPwm1 = 0;
int targetPwm2 = 0;



void command() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Read serial input until newline
    input.trim();  // Remove leading/trailing spaces or invisible characters
    Serial.println(input);

    if (input.startsWith("m1")) {
      targetPwm1 = input.substring(2).toInt();  // Get target PWM for m1
      Serial.print("Target PWM1: ");
      Serial.println(targetPwm1);
    }
    else if (input.startsWith("m2")) {
      targetPwm2 = input.substring(2).toInt();  // Get target PWM for m2
      Serial.print("Target PWM2: ");
      Serial.println(targetPwm2);
    }
    else if(input.startsWith("r")) {
      ESP.restart();  // Restart ESP if command is "r"
    }
  }
}

void updatePWM() {
  // Gradually increase pwm1 to reach target using exponential function
  if (pwm1 != targetPwm1) {
    pwm1 += (targetPwm1 - pwm1) * expFactor;
    pwm1 = constrain(pwm1, 0, 255);  // Ensure pwm1 stays within valid range
  }

  // Gradually increase pwm2 to reach target using exponential function
  if (pwm2 != targetPwm2) {
    pwm2 += (targetPwm2 - pwm2) * expFactor;
    pwm2 = constrain(pwm2, 0, 255);  // Ensure pwm2 stays within valid range
  }
}
