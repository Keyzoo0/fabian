/*
 * Arduino PID Control System with Half Resolution Constraint
 * Menerima data serial dari Python dan mengkonversi ke nilai y,z dengan PID
 * 
 * MODIFIKASI: PID output dibatasi setengah resolusi (500 untuk 10-bit)
 * sehingga y + z tidak melebihi batas maksimum PWM (1024)
 * 
 * Data format dari Python: {error_x,error_y,distance,area,mode,tombol_ditekan,setpoint_jarak,kp_jarak,ki_jarak,kd_jarak,kp_arahhadap,ki_arahhadap,kd_arahhadap}
 * 
 * Mode Manual:
 * w -> y = 600, z = 0   (maju, range tinggi)
 * s -> y = -600, z = 0  (mundur, range tinggi)
 * a -> y = 0, z = -100  (kiri, range kecil)
 * d -> y = 0, z = 100   (kanan, range kecil)
 * none -> y = 0, z = 0  (stop)
 * 
 * Mode Auto with Different Resolution:
 * PID Arah Hadap: setpoint = -10 to 10 px (tolerance zone), input = error_x, output = z (-100 to 100)
 * PID Jarak: setpoint ± 3 cm (tolerance zone), input = distance, output = y (-600 to 600)
 */

#define pinM1A 16
#define pinM1B 17
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

// Konstanta untuk setengah resolusi
#define MAX_PWM_VALUE (1 << RESOLUTION)  // 2^10 = 1024 untuk 10-bit
#define HALF_RESOLUTION (MAX_PWM_VALUE / 2)  // 512 untuk safety, tapi kita pakai 500
#define MAX_PID_OUTPUT_Y 600  // Maksimal output PID untuk Y (maju/mundur) - dinaikkan ke 600
#define MAX_PID_OUTPUT_Z 100  // Maksimal output PID untuk Z (kiri/kanan) - dikurangi ke 100

#define d2r(x) x*(PI / 180)
#define LengthAlpha 0.15  //diameter roda

// Variabel untuk data yang diterima dari Python
int error_x = 0;
int error_y = 0;
float distance = 0.0;
int area = 0;
String mode = "";
String tombol_ditekan = "";
float setpoint_jarak = 0.0;
float kp_jarak = 0.0;
float ki_jarak = 0.0;
float kd_jarak = 0.0;
float kp_arahhadap = 0.0;
float ki_arahhadap = 0.0;
float kd_arahhadap = 0.0;

// Variabel untuk kontrol motor
int y = 0, z = 0; // Nilai y,z untuk kinematik (dibatasi ±250)
int lx, ly, teta, rx, ry, atanVal, m1, m2;
int lambda = 1; // Dikurangi lagi karena Y range nya besar (600)

// === PID VARIABLES ===
// PID Arah Hadap (error_x -> z) with half resolution
float pid_arah_setpoint = 0.0;  // Target center
float pid_arah_tolerance = 20.0;  // ±20 pixels tolerance zone
float pid_arah_input = 0.0;
float pid_arah_output = 0.0;
float pid_arah_error = 0.0;
float pid_arah_last_error = 0.0;
float pid_arah_integral = 0.0;
float pid_arah_derivative = 0.0;
unsigned long pid_arah_last_time = 0;

// PID Jarak (distance -> y) with half resolution
float pid_jarak_setpoint = 50.0;  // Default 50cm
float pid_jarak_tolerance = 10.0;  // ±10cm tolerance zone
float pid_jarak_input = 0.0;
float pid_jarak_output = 0.0;
float pid_jarak_error = 0.0;
float pid_jarak_last_error = 0.0;
float pid_jarak_integral = 0.0;
float pid_jarak_derivative = 0.0;
unsigned long pid_jarak_last_time = 0;

// PID sample time (ms)
const unsigned long SAMPLE_TIME = 50;  // 20Hz

void setup() {
  Serial.begin(115200);
  setupMotor();
  
  // Initialize PID times
  pid_arah_last_time = millis();
  pid_jarak_last_time = millis();
  
  Serial.println("=================================");
  Serial.println("Arduino PID Control System");
  Serial.println("Asymmetric Range Mode (Y:600, Z:100)");
  Serial.println("=================================");
  Serial.println("PWM Resolution: " + String(RESOLUTION) + " bit (" + String(MAX_PWM_VALUE) + " max)");
  Serial.println("Y Max Output: ±" + String(MAX_PID_OUTPUT_Y) + " | Z Max Output: ±" + String(MAX_PID_OUTPUT_Z));
  Serial.println("Lambda factor: " + String(lambda));
  Serial.println("Ready to receive data from Python...");
  Serial.println("Manual controls:");
  Serial.println("W -> y=600, z=0   (Forward - High Speed)");
  Serial.println("S -> y=-600, z=0  (Backward - High Speed)");
  Serial.println("A -> y=0, z=-100  (Left - Gentle Turn)");
  Serial.println("D -> y=0, z=100   (Right - Gentle Turn)");
  Serial.println("none -> y=0, z=0  (Stop)");
  Serial.println();
  Serial.println("Auto controls:");
  Serial.println("PID Arah: error_x ±20px tolerance -> z (-100 to 100)");
  Serial.println("PID Jarak: setpoint ±10cm tolerance -> y (-600 to 600)");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData.length() > 0) {
      Serial.println("📨 Data diterima: " + receivedData);
      
      if (parseData(receivedData)) {
        processControl();
      } else {
        Serial.println("❌ Error parsing data");
        stopMotors();
      }
    }
  }
}

bool parseData(String data) {
  // Hapus kurung kurawal
  if (data.startsWith("{") && data.endsWith("}")) {
    data = data.substring(1, data.length() - 1);
  } else {
    Serial.println("❌ Format tidak valid - missing braces");
    return false;
  }
  
  // Split berdasarkan koma
  int fieldCount = 0;
  int startIndex = 0;
  
  for (int i = 0; i <= data.length(); i++) {
    if (i == data.length() || data.charAt(i) == ',') {
      String field = data.substring(startIndex, i);
      field.trim();
      
      // Parse setiap field
      switch (fieldCount) {
        case 0: error_x = field.toInt(); break;
        case 1: error_y = field.toInt(); break;
        case 2: distance = field.toFloat(); break;
        case 3: area = field.toInt(); break;
        case 4: mode = field; break;
        case 5: tombol_ditekan = field; break;
        case 6: setpoint_jarak = field.toFloat(); break;
        case 7: kp_jarak = field.toFloat(); break;
        case 8: ki_jarak = field.toFloat(); break;
        case 9: kd_jarak = field.toFloat(); break;
        case 10: kp_arahhadap = field.toFloat(); break;
        case 11: ki_arahhadap = field.toFloat(); break;
        case 12: kd_arahhadap = field.toFloat(); break;
      }
      
      fieldCount++;
      startIndex = i + 1;
    }
  }
  
  if (fieldCount != 13) {
    Serial.println("❌ Invalid field count: " + String(fieldCount) + " (expected 13)");
    return false;
  }
  
  return true;
}

void processControl() {
  Serial.println("✅ === PROCESSING CONTROL ===");
  Serial.println("🎮 Mode: " + mode);
  
  if (mode == "manual") {
    processManualMode();
  } else if (mode == "auto") {
    processAutoMode();
  } else {
    Serial.println("❌ Unknown mode: " + mode);
    stopMotors();
    return;
  }
  
  // Eksekusi gerakan berdasarkan y,z
  executeMovement();
  Serial.println("============================");
}

void processManualMode() {
  Serial.println("🎮 MANUAL MODE ACTIVE");
  Serial.println("⌨️  Tombol: " + tombol_ditekan);
  
  // Reset PID when switching to manual
  resetPID();
  
  // Reset y,z
  y = 0;
  z = 0;
  
  // Konversi tombol ke nilai y,z (Y: ±600, Z: ±100)
  if (tombol_ditekan == "w") {
    y = MAX_PID_OUTPUT_Y;  // Maju (600)
    z = 0;
    Serial.println("⬆️  FORWARD - y=" + String(y) + ", z=0 (High Speed)");
  }
  else if (tombol_ditekan == "s") {
    y = -MAX_PID_OUTPUT_Y; // Mundur (-600)
    z = 0;
    Serial.println("⬇️  BACKWARD - y=" + String(y) + ", z=0 (High Speed)");
  }
  else if (tombol_ditekan == "a") {
    y = 0;
    z = -MAX_PID_OUTPUT_Z; // Kiri (-100)
    Serial.println("⬅️  LEFT - y=0, z=" + String(z) + " (Gentle Turn)");
  }
  else if (tombol_ditekan == "d") {
    y = 0;
    z = MAX_PID_OUTPUT_Z;  // Kanan (100)
    Serial.println("➡️  RIGHT - y=0, z=" + String(z) + " (Gentle Turn)");
  }
  else if (tombol_ditekan == "none") {
    y = 0;
    z = 0;    // Stop
    Serial.println("⏸️  STOP - y=0, z=0");
  }
  else {
    Serial.println("❓ Unknown key: " + tombol_ditekan + " - STOP");
    y = 0;
    z = 0;
  }
}

void processAutoMode() {
  Serial.println("🤖 AUTO MODE ACTIVE");
  
  if (area > 0) {
    Serial.println("🎯 Object detected - Area: " + String(area));
    Serial.println("📊 Current distance: " + String(distance, 1) + " cm");
    Serial.println("📊 Error X: " + String(error_x) + " px");
    Serial.println("📊 Setpoint jarak: " + String(setpoint_jarak, 1) + " cm");
    
    // Update PID setpoints dan inputs
    pid_arah_setpoint = 0.0;  // Center target
    pid_arah_input = (float)error_x;
    
    pid_jarak_setpoint = setpoint_jarak;  // Dari web interface
    pid_jarak_input = distance;
    
    // Check tolerance zones dan hitung PID
    bool arah_in_tolerance = checkArahTolerance();
    bool jarak_in_tolerance = checkJarakTolerance();
    
    // Hitung PID hanya jika di luar tolerance zone
    if (!arah_in_tolerance) {
      computePID_Arah();
    } else {
      pid_arah_output = 0.0;  // Stop arah correction jika dalam tolerance
      resetPID_Arah_Integral();  // Reset integral untuk mencegah windup
      Serial.println("✅ Arah dalam tolerance zone (±20 px)");
    }
    
    if (!jarak_in_tolerance) {
      computePID_Jarak();
    } else {
      pid_jarak_output = 0.0;  // Stop jarak correction jika dalam tolerance
      resetPID_Jarak_Integral();  // Reset integral untuk mencegah windup
      Serial.println("✅ Jarak dalam tolerance zone (±10 cm)");
    }
    
    // Set output ke y dan z dengan pembatasan berbeda
    y = constrainY((int)(pid_jarak_output * -1));   // Output PID jarak -> y (±250)
    z = constrainZ((int)(pid_arah_output * -1));    // Output PID arah -> z (±200)
    
    Serial.println("🤖 PID Arah - Error: " + String(pid_arah_error, 1) + 
                   " | In Tolerance: " + String(arah_in_tolerance ? "YES" : "NO") +
                   " | Output Z: " + String(z));
    Serial.println("🤖 PID Jarak - Error: " + String(pid_jarak_error, 1) + 
                   " | In Tolerance: " + String(jarak_in_tolerance ? "YES" : "NO") +
                   " | Output Y: " + String(y));
                   
    // Tampilkan total kombinasi untuk monitoring
    int total_magnitude = abs(y) + abs(z);
    int max_possible = MAX_PID_OUTPUT_Y + MAX_PID_OUTPUT_Z;  // 600 + 100 = 700
    Serial.println("📊 Total magnitude: " + String(total_magnitude) + " / " + String(max_possible) + " (max safe: " + String(MAX_PWM_VALUE) + ")");
  }
  else {
    Serial.println("❌ No object detected - STOP");
    y = 0;
    z = 0;
    resetPID();  // Reset PID saat tidak ada object
  }
}

// Fungsi untuk membatasi nilai Y (±600)
int constrainY(int value) {
  if (value > MAX_PID_OUTPUT_Y) return MAX_PID_OUTPUT_Y;
  else if (value < -MAX_PID_OUTPUT_Y) return -MAX_PID_OUTPUT_Y;
  else return value;
}

// Fungsi untuk membatasi nilai Z (±100)
int constrainZ(int value) {
  if (value > MAX_PID_OUTPUT_Z) return MAX_PID_OUTPUT_Z;
  else if (value < -MAX_PID_OUTPUT_Z) return -MAX_PID_OUTPUT_Z;
  else return value;
}

void computePID_Arah() {
  unsigned long now = millis();
  unsigned long timeChange = now - pid_arah_last_time;
  
  if (timeChange >= SAMPLE_TIME) {
    // Hitung error dengan tolerance zone
    pid_arah_error = pid_arah_setpoint - pid_arah_input;  // 0 - error_x
    
    // Integral term
    pid_arah_integral += pid_arah_error * timeChange;
    
    // Anti-windup: batasi integral
    float max_integral = 1000.0;
    if (pid_arah_integral > max_integral) pid_arah_integral = max_integral;
    else if (pid_arah_integral < -max_integral) pid_arah_integral = -max_integral;
    
    // Derivative term
    pid_arah_derivative = (pid_arah_error - pid_arah_last_error) / timeChange;
    
    // PID output
    pid_arah_output = (kp_arahhadap * pid_arah_error) + 
                      (ki_arahhadap * pid_arah_integral) + 
                      (kd_arahhadap * pid_arah_derivative);
    
    // Batasi output ke Z maksimal ±100
    if (pid_arah_output > MAX_PID_OUTPUT_Z) pid_arah_output = MAX_PID_OUTPUT_Z;
    else if (pid_arah_output < -MAX_PID_OUTPUT_Z) pid_arah_output = -MAX_PID_OUTPUT_Z;
    
    // Simpan untuk iterasi berikutnya
    pid_arah_last_error = pid_arah_error;
    pid_arah_last_time = now;
    
    Serial.println("🎯 PID Arah - P:" + String(kp_arahhadap * pid_arah_error, 1) +
                   " I:" + String(ki_arahhadap * pid_arah_integral, 1) +
                   " D:" + String(kd_arahhadap * pid_arah_derivative, 1) +
                   " Out:" + String(pid_arah_output, 1));
  }
}

void computePID_Jarak() {
  unsigned long now = millis();
  unsigned long timeChange = now - pid_jarak_last_time;
  
  if (timeChange >= SAMPLE_TIME) {
    // Hitung error dengan tolerance zone
    pid_jarak_error = pid_jarak_setpoint - pid_jarak_input;  // setpoint - distance
    
    // Integral term
    pid_jarak_integral += pid_jarak_error * timeChange;
    
    // Anti-windup: batasi integral
    float max_integral = 500.0;
    if (pid_jarak_integral > max_integral) pid_jarak_integral = max_integral;
    else if (pid_jarak_integral < -max_integral) pid_jarak_integral = -max_integral;
    
    // Derivative term
    pid_jarak_derivative = (pid_jarak_error - pid_jarak_last_error) / timeChange;
    
    // PID output
    pid_jarak_output = (kp_jarak * pid_jarak_error) + 
                       (ki_jarak * pid_jarak_integral) + 
                       (kd_jarak * pid_jarak_derivative);
    
    // Batasi output ke Y maksimal ±600
    if (pid_jarak_output > MAX_PID_OUTPUT_Y) pid_jarak_output = MAX_PID_OUTPUT_Y;
    else if (pid_jarak_output < -MAX_PID_OUTPUT_Y) pid_jarak_output = -MAX_PID_OUTPUT_Y;
    
    // Simpan untuk iterasi berikutnya
    pid_jarak_last_error = pid_jarak_error;
    pid_jarak_last_time = now;
    
    Serial.println("📏 PID Jarak - P:" + String(kp_jarak * pid_jarak_error, 1) +
                   " I:" + String(ki_jarak * pid_jarak_integral, 1) +
                   " D:" + String(kd_jarak * pid_jarak_derivative, 1) +
                   " Out:" + String(pid_jarak_output, 1));
  }
}

// === TOLERANCE CHECK FUNCTIONS ===
bool checkArahTolerance() {
  // Cek apakah error_x dalam range -20 sampai +20 pixels
  float abs_error = abs(error_x);
  return (abs_error <= pid_arah_tolerance);
}

bool checkJarakTolerance() {
  // Cek apakah distance dalam range setpoint ± 10cm
  float distance_error = abs(pid_jarak_setpoint - distance);
  return (distance_error <= pid_jarak_tolerance);
}

// === PARTIAL PID RESET FUNCTIONS ===
void resetPID_Arah_Integral() {
  pid_arah_integral = 0.0;
  Serial.println("🔄 PID Arah Integral Reset");
}

void resetPID_Jarak_Integral() {
  pid_jarak_integral = 0.0;
  Serial.println("🔄 PID Jarak Integral Reset");
}

void resetPID() {
  // Reset PID Arah Hadap
  pid_arah_integral = 0.0;
  pid_arah_derivative = 0.0;
  pid_arah_last_error = 0.0;
  pid_arah_output = 0.0;
  pid_arah_last_time = millis();
  
  // Reset PID Jarak
  pid_jarak_integral = 0.0;
  pid_jarak_derivative = 0.0;
  pid_jarak_last_error = 0.0;
  pid_jarak_output = 0.0;
  pid_jarak_last_time = millis();
  
  Serial.println("🔄 PID Reset");
}

void executeMovement() {
  Serial.println("🔧 Executing movement - y=" + String(y) + ", z=" + String(z));
 
  kinematik(y, 0 , -z);
  
  // Set PWM motor
  set_pwm(M1, m1);
  set_pwm(M2, m2);
  
  Serial.println("⚙️  Motor M1: " + String(m1) + " | Motor M2: " + String(m2));
  
  // Verifikasi bahwa nilai PWM tidak melebihi batas
  if (abs(m1) > MAX_PWM_VALUE || abs(m2) > MAX_PWM_VALUE) {
    Serial.println("⚠️  WARNING: PWM value exceeds maximum!");
    Serial.println("⚠️  M1: " + String(m1) + " | M2: " + String(m2) + " | Max: " + String(MAX_PWM_VALUE));
  }
}

void stopMotors() {
  y = 0;
  z = 0;
  kinematik(0, 0, 0);
  set_pwm(M1, 0);
  set_pwm(M2, 0);
  resetPID();
  Serial.println("🛑 MOTORS STOPPED");
}

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
  
  Serial.println("✅ Motor setup complete");
}

void kinematik(int x, int y, int th) {
  m1 = lambda * (cos(d2r(180)) * x + sin(d2r(180)) * y + th);
  m2 = lambda * (cos(d2r(0)) * x + sin(d2r(0)) * y + th);

  // Batasi nilai PWM ke maksimum resolusi
  if (m1 > (MAX_PWM_VALUE-1)) m1 = (MAX_PWM_VALUE-1);  // 1023 untuk 10-bit
  else if (m1 < -(MAX_PWM_VALUE-1)) m1 = -(MAX_PWM_VALUE-1);

  if (m2 > (MAX_PWM_VALUE-1)) m2 = (MAX_PWM_VALUE-1);  // 1023 untuk 10-bit
  else if (m2 < -(MAX_PWM_VALUE-1)) m2 = -(MAX_PWM_VALUE-1);
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