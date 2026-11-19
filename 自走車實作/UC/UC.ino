#include <Servo.h>
#include <SoftwareSerial.h>

SoftwareSerial BT(10, 11); // RX, TX

Servo m1, m2;

const int STOP_US = 1500;
const int FWD_L = 1700;   // 你已校正好的
const int FWD_R = 1300;

enum State { IDLE, NAVIGATING, EMERGENCY_STOP };
State state = IDLE;

String inputLine = "";

void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  m1.attach(12);
  m2.attach(13);

  stopCar();

  BT.println("=== BOOK CAR READY ===");
  BT.println("Commands:");
  BT.println("GO A/B/C/D/E");
  BT.println("STOP");
  BT.println("STATUS");
}

void loop() {
  readBluetooth();

  if (state == NAVIGATING) {
    forwardCar();
  }
}

void readBluetooth() {
  while (BT.available()) {
    char c = BT.read();

    if (c == '\n') {
      inputLine.trim();
      handleCommand(inputLine);
      inputLine = "";
    } else {
      inputLine += c;
    }
  }
}

void handleCommand(String cmd) {
  if (cmd == "STOP") {
    state = EMERGENCY_STOP;
    stopCar();
    BT.println("OK STOP");
    return;
  }

  if (cmd == "STATUS") {
    BT.print("STATE: ");
    if (state == IDLE) BT.println("IDLE");
    else if (state == NAVIGATING) BT.println("NAVIGATING");
    else BT.println("STOPPED");
    return;
  }

  if (cmd.startsWith("GO")) {
    state = NAVIGATING;
    BT.print("OK");
    BT.println(cmd);
    return;
  }

  BT.println("UNKNOWN COMMAND");
}

void forwardCar() {
  m1.writeMicroseconds(FWD_L);
  m2.writeMicroseconds(FWD_R);
}

void stopCar() {
  m1.writeMicroseconds(STOP_US);
  m2.writeMicroseconds(STOP_US);
}