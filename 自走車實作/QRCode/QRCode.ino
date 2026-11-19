String qrData = "";

void setup() {
  Serial.begin(9600);
  Serial.println("QR Scanner Test Start");
}

void loop() {

  while (Serial.available()) {

    char c = Serial.read();

    // 如果遇到換行，代表掃描完成
    if (c == '\n' || c == '\r') {

      qrData.trim();

      if (qrData.length() > 0) {

        Serial.print("Scanned: ");
        Serial.println(qrData);

        if (qrData == "STATION_A") {
          Serial.println(">>> ARRIVED AT A");
        }
        else if (qrData == "STATION_B") {
          Serial.println(">>> ARRIVED AT B");
        }
        else if (qrData == "STATION_C") {
          Serial.println(">>> ARRIVED AT C");
        }
        else {
          Serial.println(">>> UNKNOWN QR");
        }

      }

      qrData = "";
    }
    else {
      qrData += c;
    }
  }
}
