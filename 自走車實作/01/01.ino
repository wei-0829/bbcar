#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(10, 11);

Servo m1, m2;

#define THRESHOLD 400

const int STOP_US = 1500;
const int BASE_L = 1350;
const int BASE_R = 1650;

const int sensor[5] = {A0, A1, A2, A3, A4};

// ====== 狀態 ======
enum State { IDLE, LINE_FOLLOW };
State state = IDLE;

// ====== 轉彎模式 ======
enum TurnMode { NONE, TURN_LEFT, TURN_RIGHT };
TurnMode turnMode = NONE;

// ====== 轉彎流程 ======
enum TurnState { TURN_IDLE, TURN_PREPARE, TURNING, TURN_FINISH };
TurnState turnState = TURN_IDLE;

unsigned long turnTimer = 0;

String inputLine = "";

void setup() {

  Serial.begin(9600);
  BT.begin(9600);

  m1.attach(12);
  m2.attach(13);

  stopCar();

  BT.println("READY");
  BT.println("Commands: START / STOP");
}

void loop() {

  readBluetooth();

  if (state == LINE_FOLLOW) {
    lineFollow();
  }
}

// ====== 藍牙 ======

void readBluetooth() {

  while (BT.available()) {

    char c = BT.read();

    if (c == '\n') {
      inputLine.trim();
      handleCommand(inputLine);
      inputLine = "";
    }
    else {
      inputLine += c;
    }
  }
}

void handleCommand(String cmd) {

  if (cmd == "START") {
    state = LINE_FOLLOW;
    BT.println("OK START");
    return;
  }

  if (cmd == "STOP") {
    state = IDLE;
    stopCar();
    BT.println("OK STOP");
    return;
  }

  BT.println("UNKNOWN CMD");
}

// ====== 主循線 ======

void lineFollow() {

  int s[5];
  for (int i = 0; i < 5; i++) {
    s[i] = analogRead(sensor[i]);
  }

  bool L2 = s[0] < THRESHOLD;
  bool L1 = s[1] < THRESHOLD;
  bool C  = s[2] < THRESHOLD;
  bool R1 = s[3] < THRESHOLD;
  bool R2 = s[4] < THRESHOLD;

  // ====== 如果在轉彎流程 ======

  if (turnState == TURN_PREPARE) {

    stopCar();

    if (millis() - turnTimer > 1000) {
      turnState = TURNING;
    }

    return;
  }

  if (turnState == TURNING) {

    if (turnMode == TURN_LEFT) {

      if (C && !L1 && !R1) {
        turnState = TURN_FINISH;
        turnTimer = millis();
        stopCar();
      }
      else {
        pivotLeft();
      }
    }

    if (turnMode == TURN_RIGHT) {

      if (C && !L1 && !R1) {
        turnState = TURN_FINISH;
        turnTimer = millis();
        stopCar();
      }
      else {
        pivotRight();
      }
    }

    return;
  }

  if (turnState == TURN_FINISH) {

    stopCar();

    if (millis() - turnTimer > 1000) {

      turnState = TURN_IDLE;
      turnMode = NONE;

      forward();
    }

    return;
  }

  // ====== 正常循線 ======

  if (C && !L1 && !R1) {
    forward();
  }

  else if (L2 && L1 && C) {

    turnMode = TURN_LEFT;
    turnState = TURN_PREPARE;
    turnTimer = millis();
  }

  else if (R2 && R1 && C) {

    turnMode = TURN_RIGHT;
    turnState = TURN_PREPARE;
    turnTimer = millis();
  }

  else if (L1) {
    pivotLeft();
  }

  else if (R1) {
    pivotRight();
  }

  else {
    forward();
  }
}

// ====== 動作控制 ======

void forward() {
  m1.writeMicroseconds(BASE_L);
  m2.writeMicroseconds(BASE_R);
}

void pivotLeft() {
  m1.writeMicroseconds(BASE_L);
  m2.writeMicroseconds(STOP_US);
}

void pivotRight() {
  m1.writeMicroseconds(STOP_US);
  m2.writeMicroseconds(BASE_R);
}

void stopCar() {
  m1.writeMicroseconds(STOP_US);
  m2.writeMicroseconds(STOP_US);
}
