#include <Servo.h>

Servo m1, m2;

void setup() {
  m1.attach(12);
  m2.attach(13);

  // 兩輪固定 1500
  m1.writeMicroseconds(1500);
  m2.writeMicroseconds(1500);
}

void loop() {
  // 不做任何事，保持 1500
}