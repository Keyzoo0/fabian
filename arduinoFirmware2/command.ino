void command(){
   if (Serial.available()) {
    String input = Serial.readStringUntil('\n');  // Baca input serial hingga baris baru
    input.trim();                                 // Hapus spasi atau karakter tak terlihat
    Serial.println(input);
    if (input.startsWith("m1")) {
      pwm1 = input.substring(2).toInt();  // Ambil nilai PWM setelah "m1"
      Serial.println(pwm1);
    }
    else if (input.startsWith("m2")) {
      pwm2 = input.substring(2).toInt();  // Ambil nilai PWM setelah "m2"
      Serial.println(pwm2);

    }
    else if(input.startsWith("r")){
      ESP.restart();
    }
  }
}