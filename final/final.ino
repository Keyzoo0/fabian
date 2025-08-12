#define pinM1A 17
#define pinM1B 16
#define pinM2A 26
#define pinM2B 27

#define M1A 1
#define M1B 2
#define M2A 3
#define M2B 4

#define M1 1 
#define M2 2

#define FREQUENCY 20000
#define RESOLUTION 10

#define d2r(x) x*(PI/180)
#define LengthAlpha 0.15

// PID variables untuk X
float kpX = 2.0, kiX = 0.1, kdX = 0.5;
float errorX = 0, lastErrorX = 0, integralX = 0;
float setpointX = 0; // Range: -10 to 10
float pidOutputX = 0;

// PID variables untuk Distance (D)
float kpD = 1.5, kiD = 0.05, kdD = 0.3;
float errorD = 0, lastErrorD = 0, integralD = 0;
float setpointD = 200; // Range: 19-20
float pidOutputD = 0;

// Current values dari serial
float currentX = 0;
float currentD = 0;

int lx, ly, teta, rx, ry, atanVal, m1, m2, m3, m4;
int lambda = 5;
const int DEADZONE_RIGHT = 15;

unsigned long lastTime = 0;
float dt = 0;

void setupMotor() {
  ledcAttachPin(pinM1A, M1A);
  ledcAttachPin(pinM1B, M1B);
  ledcAttachPin(pinM2A, M2A);
  ledcAttachPin(pinM2B, M2B);
  
  ledcSetup(M1A, FREQUENCY, RESOLUTION);
  ledcSetup(M1B, FREQUENCY, RESOLUTION);
  ledcSetup(M2A, FREQUENCY, RESOLUTION);
  ledcSetup(M2B, FREQUENCY, RESOLUTION);
  
  ledcWrite(M1A, 0);
  ledcWrite(M1B, 0);
  ledcWrite(M2A, 0);
  ledcWrite(M2B, 0);
}

void readSerial() {
  if (Serial.available()) {
    String data = Serial.readStringUntil('\n');
    data.trim();
    
    // Parse data format: X:+258,Y:-65,D:173.0
    int xIndex = data.indexOf("X:");
    int yIndex = data.indexOf("Y:");
    int dIndex = data.indexOf("D:");
    
    if (xIndex != -1 && dIndex != -1) {
      // Extract X value
      int xStart = xIndex + 2;
      int xEnd = data.indexOf(",", xStart);
      if (xEnd == -1) xEnd = data.length();
      currentX = data.substring(xStart, xEnd).toFloat();
      
      // Extract D value
      int dStart = dIndex + 2;
      currentD = data.substring(dStart).toFloat();
      
      Serial.print("Received - X: ");
      Serial.print(currentX);
      Serial.print(", D: ");
      Serial.println(currentD);
      delay(1);
    }
  }
}

float calculatePID_X(float setpoint, float input, float &error, float &lastError, float &integral, float kp, float ki, float kd) {
  error = setpoint - input;
  
  // Integral term dengan anti-windup
  integral += error * dt;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  
  // Derivative term
  float derivative = (error - lastError) / dt;
  
  // PID output
  float output = kp * error + ki * integral + kd * derivative;
  
  // Limit output untuk teta (-100 to 100)
  if (output > 100) output = 100;
  if (output < -100) output = -100;
  
  lastError = error;
  return output;
}

float calculatePID_D(float setpoint, float input, float &error, float &lastError, float &integral, float kp, float ki, float kd) {
  error = setpoint - input;
  
  // Integral term dengan anti-windup
  integral += error * dt;
  if (integral > 100) integral = 100;
  if (integral < -100) integral = -100;
  
  // Derivative term
  float derivative = (error - lastError) / dt;
  
  // PID output
  float output = kp * error + ki * integral + kd * derivative;
  
  // Limit output untuk ly (-100 to 100)
  if (output > 100) output = 100;
  if (output < -100) output = -100;
  
  lastError = error;
  return output;
}

void updatePID() {
  unsigned long currentTime = millis();
  dt = (currentTime - lastTime) / 1000.0; // Convert to seconds
  if (dt <= 0) dt = 0.01; // Minimum dt
  
  // Calculate PID for X (output to teta)
  pidOutputX = calculatePID_X(setpointX, currentX, errorX, lastErrorX, integralX, kpX, kiX, kdX);
  teta = (int)pidOutputX;
  
  // Calculate PID for D (output to ly)
  pidOutputD = calculatePID_D(setpointD, currentD, errorD, lastErrorD, integralD, kpD, kiD, kdD);
  ly = (int)pidOutputD;
  
  lastTime = currentTime;
  
  // Debug output
  Serial.print("PID - X_error: ");
  Serial.print(errorX);
  Serial.print(", D_error: ");
  Serial.print(errorD);
  Serial.print(", teta: ");
  Serial.print(teta);
  Serial.print(", ly: ");
  Serial.println(ly);
}

void forwardKinematic() {
  // ly sudah diset oleh PID Distance
  // teta sudah diset oleh PID X
  
  kinematik(ly, 0, teta);
  set_pwm(M1, m1);
  set_pwm(M2, m2);
  
  // Serial.printf("Motor - ly: %3d | lx: %3d | teta: %3d | m1: %3d | m2: %3d | lambda: %2d\n", 
                // ly, -lx, teta, m1, m2, lambda);
}

void kinematik(int x, int y, int th) {
  m1 = lambda * (cos(d2r(0)) * x + sin(d2r(0)) * y + (th));
  m2 = lambda * (cos(d2r(180)) * x + sin(d2r(180)) * y + (th));
  
  // Limit motor values
  if (m1 > 1022) m1 = 1022;
  else if (m1 < -1022) m1 = -1022;
  
  if (m2 > 1022) m2 = 1022;
  else if (m2 < -1022) m2 = -1022;
}

void set_pwm(byte MTR, int val_pwm) {
  switch (MTR) {
    case M1:
      if (val_pwm > 0) {
        ledcWrite(M1A, val_pwm);
        ledcWrite(M1B, 0);
      } else if (val_pwm < 0) {
        ledcWrite(M1B, abs(val_pwm));
        ledcWrite(M1A, 0);
      } else {
        ledcWrite(M1A, 0);
        ledcWrite(M1B, 0);
      }
      break;
      
    case M2:
      if (val_pwm > 0) {
        ledcWrite(M2A, val_pwm);
        ledcWrite(M2B, 0);
      } else if (val_pwm < 0) {
        ledcWrite(M2B, abs(val_pwm));
        ledcWrite(M2A, 0);
      } else {
        ledcWrite(M2A, 0);
        ledcWrite(M2B, 0);
      }
      break;
  }
}

void setSetpoints() {
  // Fungsi untuk mengatur setpoint via serial command
  // Format: "SETX:value" atau "SETD:value"
  if (Serial.available()) {
    String command = Serial.readStringUntil('\n');
    command.trim();
    
    if (command.startsWith("SETX:")) {
      float newSetpointX = command.substring(5).toFloat();
      if (newSetpointX >= -10 && newSetpointX <= 10) {
        setpointX = newSetpointX;
        // Serial.print("New X setpoint: ");
        // Serial.println(setpointX);
      }
    }
    else if (command.startsWith("SETD:")) {
      float newSetpointD = command.substring(5).toFloat();
      if (newSetpointD >= 19 && newSetpointD <= 20) {
        setpointD = newSetpointD;
        // Serial.print("New D setpoint: ");
        // Serial.println(setpointD);
      }
    }
  }
}

void setup() {
  // Serial.begin(115200);
  setupMotor();
  lastTime = millis();
  
  // Serial.println("Arduino PID Controller Ready");
  // Serial.println("Send data format: X:+258,Y:-64,D:172.9");
  // Serial.println("Set setpoints: SETX:value (-10 to 10) or SETD:value (19 to 20)");
}

void loop() {
  readSerial();
  //setSetpoints();  // Uncomment jika ingin mengubah setpoint via serial
  updatePID();
  forwardKinematic();
  
  delay(10); // Small delay untuk stabilitas
}