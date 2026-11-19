#include <SoftwareSerial.h>
#include <Servo.h>   //載入函式庫

Servo servoLeft;   // 建立SERVO物件(左馬達)
Servo servoRight;   // 建立SERVO物件(右馬達)
SoftwareSerial mySerial(0, 1); // RX, TX

void setup() {
  mySerial.begin(9600);
  Serial.begin(9600);
}

void loop() {
  if (mySerial.available()) {
    String inputString = mySerial.readStringUntil('\n'); // 讀取串口值，直到遇到換行符
    inputString.trim(); // 移除字串前後的空格
    if (inputString == "Left") {
      Serial.println("Turn Left");
    } else if (inputString == "Right") {
      Serial.println("Turn Right");
    } else if (inputString == "Forward") {
      Serial.println("Go Forward");
    } else if (inputString == "Backward") {
      Serial.println("Go Backward");
    }
  }
}
