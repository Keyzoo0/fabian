// PID Controller untuk robot differential drive
// Format input: "X:+83,Y:-15,D:175.7\n"
// Format setpoint: "SP_X:0,SP_D:100\n"

// PID constants untuk X (rotasi)
float kp_x = 2.0;
float ki_x = 0.1;
float kd_x = 0.5;

// PID constants untuk D (translasi)
float kp_d = 1.5;
float ki_d = 0.05;
float kd_d = 0.3;

// PID variables untuk X
float error_x = 0, prev_error_x = 0, integral_x = 0, derivative_x = 0;
float setpoint_x = 0; // Target rotasi (biasanya 0 untuk lurus)

// PID variables untuk D
float error_d = 0, prev_error_d = 0, integral_d = 0, derivative_d = 0;
float setpoint_d = 0; // Target translasi (0 untuk diam, positif untuk maju)

// Input values
float current_x = 0;
float current_d = 0;

// PID outputs
int pid_output_x = 0; // Output untuk rotasi (-125 to +125)
int pid_output_d = 0; // Output untuk translasi (-125 to +125)

// Timing
unsigned long lastTime = 0;
unsigned long lastPrintTime = 0;
float dt = 0;

// Status tracking
bool object_detected = false;
bool pid_active = false;

void parseSerialInput() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    
    // Check if it's a setpoint command
    if (input.startsWith("SP_X:")) {
      parseSetpointInput(input);
    }
    // Check if it's sensor data
    else if (input.indexOf("X:") != -1 && input.indexOf("D:") != -1) {
      parsePIDInput(input);
    }
    // Handle other commands
    else {
      handleOtherCommands(input);
    }
  }
}

void parseSetpointInput(String input) {
  // Parse format "SP_X:0,SP_D:100"
  int sp_x_pos = input.indexOf("SP_X:");
  int sp_d_pos = input.indexOf(",SP_D:");
  
  if (sp_x_pos != -1 && sp_d_pos != -1) {
    // Extract SP_X value
    String sp_x_str = input.substring(sp_x_pos + 5, sp_d_pos);
    float new_setpoint_x = sp_x_str.toFloat();
    
    // Extract SP_D value
    String sp_d_str = input.substring(sp_d_pos + 6);
    float new_setpoint_d = sp_d_str.toFloat();
    
    setPIDSetpoints(new_setpoint_x, new_setpoint_d);
  }
}

void parsePIDInput(String input) {
  // Parse format "X:+83,Y:-15,D:175.7"
  int x_pos = input.indexOf("X:");
  int y_pos = input.indexOf(",Y:");
  int d_pos = input.indexOf(",D:");
  
  if (x_pos != -1 && d_pos != -1) {
    // Extract X value
    String x_str = input.substring(x_pos + 2, y_pos);
    current_x = x_str.toFloat();
    
    // Extract D value
    String d_str = input.substring(d_pos + 3);
    current_d = d_str.toFloat();
    
    // Check if object is detected (distance > 0)
    object_detected = (current_d > 0);
    pid_active = object_detected;
    
    Serial.print("Sensor Data - X: ");
    Serial.print(current_x);
    Serial.print(", D: ");
    Serial.print(current_d);
    Serial.print(", Obj: ");
    Serial.println(object_detected ? "YES" : "NO");
  }
}

void handleOtherCommands(String input) {
  if (input.startsWith("m1")) {
    targetPwm1 = input.substring(2).toInt();
    pid_active = false; // Disable PID when manual control
    Serial.print("Manual PWM1: ");
    Serial.println(targetPwm1);
  }
  else if (input.startsWith("m2")) {
    targetPwm2 = input.substring(2).toInt();
    pid_active = false; // Disable PID when manual control
    Serial.print("Manual PWM2: ");
    Serial.println(targetPwm2);
  }
  else if (input.startsWith("PID_ON")) {
    pid_active = true;
    Serial.println("PID Control: ON");
  }
  else if (input.startsWith("PID_OFF")) {
    pid_active = false;
    targetPwm1 = 0;
    targetPwm2 = 0;
    Serial.println("PID Control: OFF");
  }
  else if (input.startsWith("r")) {
    ESP.restart();
  }
}

void computePID() {
  unsigned long now = millis();
  dt = (now - lastTime) / 1000.0; // Convert to seconds
  
  if (dt >= 0.01 && pid_active) { // Run PID at 100Hz max, only if active
    // Calculate errors
    error_x = setpoint_x - current_x;
    error_d = setpoint_d - current_d;
    
    // Only run distance PID if object is detected
    if (object_detected) {
      // Calculate integral (dengan anti-windup)
      integral_x += error_x * dt;
      integral_d += error_d * dt;
      
      // Anti-windup: limit integral
      integral_x = constrain(integral_x, -50, 50);
      integral_d = constrain(integral_d, -50, 50);
      
      // Calculate derivative
      if (dt > 0) {
        derivative_x = (error_x - prev_error_x) / dt;
        derivative_d = (error_d - prev_error_d) / dt;
      }
      
      // Calculate PID outputs
      float pid_x = (kp_x * error_x) + (ki_x * integral_x) + (kd_x * derivative_x);
      float pid_d = (kp_d * error_d) + (ki_d * integral_d) + (kd_d * derivative_d);
      
      // Constrain outputs to -125 to +125
      pid_output_x = constrain((int)pid_x, -125, 125);
      pid_output_d = constrain((int)pid_d, -125, 125);
      
      // Calculate motor PWM values
      // pid_output_d: translasi (maju/mundur untuk kedua motor)
      // pid_output_x: rotasi (diferensial antara motor kiri dan kanan)
      
      targetPwm1 = pid_output_d + pid_output_x; // Motor kiri: translasi + rotasi
      targetPwm2 = pid_output_d - pid_output_x; // Motor kanan: translasi - rotasi
      
      // Constrain final PWM values
      targetPwm1 = constrain(targetPwm1, -255, 255);
      targetPwm2 = constrain(targetPwm2, -255, 255);
    } else {
      // No object detected, stop robot
      pid_output_x = 0;
      pid_output_d = 0;
      targetPwm1 = 0;
      targetPwm2 = 0;
    }
    
    // Store previous errors
    prev_error_x = error_x;
    prev_error_d = error_d;
    lastTime = now;
  } else if (!pid_active) {
    // PID is disabled, maintain current targets or use manual control
    // targetPwm1 and targetPwm2 are set by manual commands
  }
}

void printPIDStatus() {
  unsigned long now = millis();
  
  // Print status every 500ms
  if (now - lastPrintTime >= 500) {
    Serial.println("=== PID STATUS ===");
    Serial.print("Mode: ");
    Serial.println(pid_active ? "PID_ACTIVE" : "MANUAL");
    Serial.print("Object: ");
    Serial.println(object_detected ? "DETECTED" : "NOT_FOUND");
    Serial.print("Setpoints - X: ");
    Serial.print(setpoint_x);
    Serial.print(", D: ");
    Serial.println(setpoint_d);
    Serial.print("Current - X: ");
    Serial.print(current_x);
    Serial.print(", D: ");
    Serial.println(current_d);
    Serial.print("Errors - X: ");
    Serial.print(error_x);
    Serial.print(", D: ");
    Serial.println(error_d);
    Serial.print("PID Outputs - X: ");
    Serial.print(pid_output_x);
    Serial.print(", D: ");
    Serial.println(pid_output_d);
    Serial.print("Motor PWM - M1: ");
    Serial.print(targetPwm1);
    Serial.print(", M2: ");
    Serial.println(targetPwm2);
    Serial.println("==================");
    
    lastPrintTime = now;
  }
}

void setPIDSetpoints(float sp_x, float sp_d) {
  setpoint_x = sp_x;
  setpoint_d = sp_d;
  
  // Reset integral untuk menghindari windup saat setpoint berubah
  integral_x = 0;
  integral_d = 0;
  
  Serial.print("New setpoints - X: ");
  Serial.print(setpoint_x);
  Serial.print(", D: ");
  Serial.println(setpoint_d);
  
  // Auto-enable PID when setpoints are set
  if (sp_d != 0 || sp_x != 0) {
    pid_active = true;
    Serial.println("PID Auto-enabled");
  }
}

void resetPID() {
  error_x = 0; prev_error_x = 0; integral_x = 0; derivative_x = 0;
  error_d = 0; prev_error_d = 0; integral_d = 0; derivative_d = 0;
  pid_output_x = 0;
  pid_output_d = 0;
  targetPwm1 = 0;
  targetPwm2 = 0;
  setpoint_x = 0;
  setpoint_d = 0;
  pid_active = false;
  object_detected = false;
  Serial.println("PID Reset Complete");
}

void setPIDGains(float new_kp_x, float new_ki_x, float new_kd_x, 
                float new_kp_d, float new_ki_d, float new_kd_d) {
  kp_x = new_kp_x;
  ki_x = new_ki_x;
  kd_x = new_kd_x;
  kp_d = new_kp_d;
  ki_d = new_ki_d;
  kd_d = new_kd_d;
  
  Serial.println("PID Gains Updated:");
  Serial.print("X: Kp="); Serial.print(kp_x); 
  Serial.print(", Ki="); Serial.print(ki_x); 
  Serial.print(", Kd="); Serial.println(kd_x);
  Serial.print("D: Kp="); Serial.print(kp_d); 
  Serial.print(", Ki="); Serial.print(ki_d); 
  Serial.print(", Kd="); Serial.println(kd_d);
}