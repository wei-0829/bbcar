#include <Servo.h>

Servo m1, m2;

#define THRESHOLD 500

const int STOP_US = 1500;
const int BASE_L = 1700;
const int BASE_R = 1300;

const int sensor[5] = {A0, A1, A2, A3, A4};

void setup() {
  Serial.begin(9600);
  m1.attach(12);
  m2.attach(13);
}

void loop() {

  int s[5];
  for (int i = 0; i < 5; i++) {
    s[i] = analogRead(sensor[i]);
  }

  // 印出數值
  for (int i = 0; i < 5; i++) {
    Serial.print(s[i]);
    Serial.print("\t");
  }
  Serial.println();

  bool L2 = s[0] < THRESHOLD;
  bool L1 = s[1] < THRESHOLD;
  bool C  = s[2] < THRESHOLD;
  bool R1 = s[3] < THRESHOLD;
  bool R2 = s[4] < THRESHOLD;

  if (C) {
    forward();
  }
  else if (L1 || L2) {
    turnLeftSmall();
  }
  else if (R1 || R2) {
    turnRightSmall();
  }
  else {
    stopCar();
  }

  delay(100);   // 先放慢方便觀察
}

// ====== 動作函式 ======

void forward() {
  m1.writeMicroseconds(BASE_L);
  m2.writeMicroseconds(BASE_R);
}

void turnLeftSmall() {
  m1.writeMicroseconds(1550);
  m2.writeMicroseconds(BASE_R);
}

void turnRightSmall() {
  m1.writeMicroseconds(BASE_L);
  m2.writeMicroseconds(1450);
}

void stopCar() {
  m1.writeMicroseconds(STOP_US);
  m2.writeMicroseconds(STOP_US);
}