/*
 * Simple Arduino Serial Receiver
 * Menerima data serial dari Python dan menampilkan di Serial Monitor
 * 
 * Data format: {error_x,error_y,distance,area,mode,tombol_ditekan,setpoint_jarak,kp_jarak,ki_jarak,kd_jarak,kp_arahhadap,ki_arahhadap,kd_arahhadap}
 */

#define LED_PIN 2  // LED untuk indikator data diterima

// Variabel untuk menyimpan data yang diterima
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

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  
  // Blinking startup
  for(int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(200);
    digitalWrite(LED_PIN, LOW);
    delay(200);
  }
  
  Serial.println("=================================");
  Serial.println("Arduino Serial Receiver Ready");
  Serial.println("=================================");
  Serial.println("Waiting for data from Python...");
  Serial.println();
}

void loop() {
  if (Serial.available()) {
    String receivedData = Serial.readStringUntil('\n');
    receivedData.trim();
    
    if (receivedData.length() > 0) {
      // Blinking LED saat data diterima
      digitalWrite(LED_PIN, HIGH);
      
      Serial.println("📨 Data diterima: " + receivedData);
      
      if (parseData(receivedData)) {
        printData();
      } else {
        Serial.println("❌ Error parsing data");
      }
      
      digitalWrite(LED_PIN, LOW);
      Serial.println("----------------------------");
    }
  }
  delay(3000);
}

bool parseData(String data) {
  // Hapus kurung kurawal
  if (data.startsWith("{") && data.endsWith("}")) {
    data = data.substring(1, data.length() - 1);
  } else {
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
  
  return fieldCount == 13;
}

void printData() {
  Serial.println("✅ DATA BERHASIL DI-PARSE:");
  Serial.println("🎯 Error X: " + String(error_x) + " px");
  Serial.println("🎯 Error Y: " + String(error_y) + " px");
  Serial.println("📏 Distance: " + String(distance, 1) + " cm");
  Serial.println("📐 Area: " + String(area) + " px²");
  Serial.println("🎮 Mode: " + mode);
  Serial.println("⌨️  Tombol: " + tombol_ditekan);
  Serial.println("🎯 Setpoint: " + String(setpoint_jarak, 1) + " cm");
  Serial.println("⚙️  PID Jarak: Kp=" + String(kp_jarak, 2) + 
                 " Ki=" + String(ki_jarak, 3) + 
                 " Kd=" + String(kd_jarak, 3));
  Serial.println("⚙️  PID Arah: Kp=" + String(kp_arahhadap, 2) + 
                 " Ki=" + String(ki_arahhadap, 3) + 
                 " Kd=" + String(kd_arahhadap, 3));
  
  // Tampilkan pesan berdasarkan mode
  if (mode == "manual") {
    Serial.println("🎮 MODE MANUAL AKTIF");
    if (tombol_ditekan != "none" && tombol_ditekan.length() > 0) {
      Serial.println("⌨️  TOMBOL DITEKAN: " + tombol_ditekan);
    } else {
      Serial.println("⌨️  TIDAK ADA TOMBOL DITEKAN");
    }
  } else if (mode == "auto") {
    Serial.println("🤖 MODE AUTO PID AKTIF");
    if (area > 0) {
      Serial.println("🎯 OBJEK TERDETEKSI - PID AKTIF");
    } else {
      Serial.println("❌ TIDAK ADA OBJEK TERDETEKSI");
    }
  }
}