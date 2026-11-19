#include <Servo.h>

Servo m1, m2;

// 腳位
const int servoPin1 = 12;
const int servoPin2 = 13;

// 已校正好的基準
const int STOP = 1500;

// 速度參數（可微調）
const int FORWARD_L = 1700; // 左輪前進
const int FORWARD_R = 1300; // 右輪前進

const int TURN_SPEED = 1700; // 原地轉向速度

// 時間參數（重點在這裡調）
const int FORWARD_TIME = 1000; // 直走多久（ms）
const int TURN_TIME    = 600;  // 轉 90 度大概多久（ms）

void setup() {
  m1.attach(servoPin1);
  m2.attach(servoPin2);

  // 保險：開機先停
  stopCar();
  delay(5000);
}

void loop() {
  for (int i = 0; i < 4; i++) {
    forward();
    delay(FORWARD_TIME);

    stopCar();
    delay(500);

    turnRight90();
    delay(TURN_TIME);

    stopCar();
    delay(500);
  }

  // 正方形走完就停住
  stopCar();
  while (1); // 永久停止
}

/* ===== 動作函式 ===== */

void forward() {
  m1.writeMicroseconds(FORWARD_L);
  m2.writeMicroseconds(FORWARD_R);
}

void turnRight90() {
  // 原地右轉：兩輪同方向
  m1.writeMicroseconds(TURN_SPEED);
  m2.writeMicroseconds(TURN_SPEED);
}

void stopCar() {
  m1.writeMicroseconds(STOP);
  m2.writeMicroseconds(STOP);
}