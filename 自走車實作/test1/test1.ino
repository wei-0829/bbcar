#include <Servo.h>
#include <SoftwareSerial.h>

// =========================
// 藍牙通訊
// =========================
SoftwareSerial BT(10, 11);   // HC-06 RX, TX

// =========================
// 馬達與循線感測器
// =========================
Servo motorR, motorL;   // 右輪、左輪


#define THRESHOLD 400

// 左右輪分開校正停止值
const int STOP_R = 1500;   // 右輪停止值
const int STOP_L = 1500;   // 左輪停止值

// 前進速度（保留你現在已經可正常運作的值）
const int FORWARD_R = 1350;   // 右輪前進
const int FORWARD_L = 1650;   // 左輪前進

// 後退速度（只有迴轉時用）
const int BACKWARD_R = 1650;  // 右輪後退
const int BACKWARD_L = 1350;  // 左輪後退

const int sensorPins[5] = {A0, A1, A2, A3, A4};   // L2, L1, C, R1, R2

// =========================
// 轉彎 / 停頓參數
// =========================
const unsigned long PREPARE_STOP_MS   = 2000;   // 到路口先停 2 秒
const unsigned long FINISH_STOP_MS    = 2000;   // 左/右/迴轉後停一下
const unsigned long TURN_LEFT_MS      = 1100;   // 左轉時間
const unsigned long TURN_RIGHT_MS     = 1100;   // 右轉時間
const unsigned long TURN_BACK_MS      = 1100;   // 180度迴轉時間（只轉一次）
const unsigned long STRAIGHT_CROSS_MS = 400;    // 直走穿越路口時間

// =========================
// 地圖 / 站點
// =========================
struct Point {
  int x;
  int y;
};

enum Facing {
  FACE_UP = 0,
  FACE_RIGHT = 1,
  FACE_DOWN = 2,
  FACE_LEFT = 3
};

const int CAPACITY = 5;
const int NUM_STATIONS = 7;

const char stationNames[NUM_STATIONS] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};

Point stationPos[NUM_STATIONS] = {
  {4, 4},  // A
  {2, 0},  // B
  {1, 1},  // C
  {4, 2},  // D
  {0, 3},  // E
  {3, 2},  // F
  {2, 4}   // G
};

Point depot = {0, 0};

// =========================
// 貨單資料
// =========================
const int MAX_JOBS = 20;
const int MAX_ITEMS_PER_JOB = 5;
const int MAX_ROUTE_POINTS = 10;
const int MAX_PATH_POINTS = 20;

struct Job {
  char items[MAX_ITEMS_PER_JOB];
  int itemCount;

  char uniqueStations[MAX_ITEMS_PER_JOB];
  int uniqueCount;

  char route[MAX_ROUTE_POINTS];
  int routeCount;
};

Job jobs[MAX_JOBS];
int jobCount = 0;

// =========================
// 手機輸入狀態
// =========================
enum InputMode {
  WAIT_CMD,
  INPUT_ORDERS
};

InputMode inputMode = WAIT_CMD;
String inputLine = "";
String orderBuffer = "";

// =========================
// 任務狀態
// =========================
enum MissionState {
  M_IDLE,
  M_READY,
  M_TRAVEL,
  M_UNLOAD,
  M_REORIENT,
  M_DEPOT_WAIT,
  M_DONE
};

MissionState missionState = M_IDLE;

// =========================
// 移動狀態機
// =========================
enum MoveState {
  MOVE_FOLLOW,     // 一般循線
  MOVE_CROSS,      // 直走穿越路口
  MOVE_PREPARE,    // 路口先停
  MOVE_TURNING,    // 固定秒數轉彎
  MOVE_FINISH      // 轉彎後停一下
};

MoveState moveState = MOVE_FOLLOW;

enum NavAction {
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

enum TurnDir {
  TD_LEFT,
  TD_RIGHT
};

NavAction pendingAction = ACT_STRAIGHT;
TurnDir turnDir = TD_LEFT;
bool intersectionLatched = false;
unsigned long moveTimer = 0;
unsigned long currentTurnDuration = 0;

// =========================
// 目前配送進度
// =========================
int currentJobIndex = 0;
int currentRouteIndex = 0;
char currentTarget = 'S';
bool currentTargetIsDepot = false;

Point currentPos = {0, 0};
Facing currentFacing = FACE_RIGHT;  // 起始朝右

Point currentPath[MAX_PATH_POINTS];
int pathLen = 0;
int pathIndex = 0;

// =========================
// 訊息輸出
// =========================
void logMsg(const String &msg) {
  Serial.println(msg);
  BT.listen();
  BT.println(msg);
}

// =========================
// 站點 / 距離工具
// =========================
int stationIndex(char ch) {
  for (int i = 0; i < NUM_STATIONS; i++) {
    if (stationNames[i] == ch) return i;
  }
  return -1;
}

Point getPointByStation(char ch) {
  if (ch == 'S') return depot;
  int idx = stationIndex(ch);
  if (idx >= 0) return stationPos[idx];
  return depot;
}

int manhattan(Point a, Point b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
}

String pointToString(Point p) {
  return "(" + String(p.x) + "," + String(p.y) + ")";
}

// =========================
// 馬達控制
// =========================
void stopCar() {
  motorR.writeMicroseconds(STOP_R);
  motorL.writeMicroseconds(STOP_L);
}

void forward() {
  motorR.writeMicroseconds(FORWARD_R);
  motorL.writeMicroseconds(FORWARD_L);
}

// 左轉：右輪動，左輪停
void pivotLeft() {
  motorR.writeMicroseconds(FORWARD_R);
  motorL.writeMicroseconds(STOP_L);
}

// 右轉：左輪動，右輪停
void pivotRight() {
  motorR.writeMicroseconds(STOP_R);
  motorL.writeMicroseconds(FORWARD_L);
}

// 原地左轉：右輪後退、左輪前進
void spinLeft() {
  motorR.writeMicroseconds(BACKWARD_R);
  motorL.writeMicroseconds(FORWARD_L);
}

void executePivot() {
  if (turnDir == TD_LEFT) {
    pivotLeft();
  } else {
    pivotRight();
  }
}

void rotateFacingLeft() {
  currentFacing = (Facing)((currentFacing + 3) % 4);
}

void rotateFacingRight() {
  currentFacing = (Facing)((currentFacing + 1) % 4);
}

// 完成一次動作後，更新朝向
void updateFacingAfterAction() {
  if (pendingAction == ACT_BACK) {
    currentFacing = (Facing)((currentFacing + 2) % 4);  // 180度
  } else if (pendingAction == ACT_LEFT) {
    rotateFacingLeft();
  } else if (pendingAction == ACT_RIGHT) {
    rotateFacingRight();
  }
}

// =========================
// 貨單工具
// =========================
int totalRemaining(int counts[]) {
  int s = 0;
  for (int i = 0; i < NUM_STATIONS; i++) s += counts[i];
  return s;
}

bool stationExistsInJob(Job &job, char station) {
  for (int i = 0; i < job.uniqueCount; i++) {
    if (job.uniqueStations[i] == station) return true;
  }
  return false;
}

void addItemToJob(Job &job, char station) {
  if (job.itemCount < MAX_ITEMS_PER_JOB) {
    job.items[job.itemCount++] = station;
  }
}

void buildUniqueStations(Job &job) {
  job.uniqueCount = 0;
  for (int i = 0; i < job.itemCount; i++) {
    char s = job.items[i];
    if (!stationExistsInJob(job, s)) {
      job.uniqueStations[job.uniqueCount++] = s;
    }
  }
}

void clearJobs() {
  jobCount = 0;
  for (int i = 0; i < MAX_JOBS; i++) {
    jobs[i].itemCount = 0;
    jobs[i].uniqueCount = 0;
    jobs[i].routeCount = 0;
  }
}

// =========================
// 純貪婪分貨單
// =========================
int pickNearestStationFromPoint(int counts[], Point from) {
  int bestIdx = -1;
  int bestDist = 9999;

  for (int i = 0; i < NUM_STATIONS; i++) {
    if (counts[i] <= 0) continue;
    int dist = manhattan(from, stationPos[i]);
    if (dist < bestDist) {
      bestDist = dist;
      bestIdx = i;
    }
  }
  return bestIdx;
}

void buildGreedyJobs(String orders) {
  clearJobs();

  int counts[NUM_STATIONS] = {0};

  for (int i = 0; i < orders.length(); i++) {
    char ch = orders.charAt(i);
    int idx = stationIndex(ch);
    if (idx >= 0) counts[idx]++;
  }

  while (totalRemaining(counts) > 0 && jobCount < MAX_JOBS) {
    Job &job = jobs[jobCount];
    job.itemCount = 0;

    int capacityLeft = CAPACITY;
    Point currentPoint = depot;

    while (capacityLeft > 0 && totalRemaining(counts) > 0) {
      int nextIdx = pickNearestStationFromPoint(counts, currentPoint);
      if (nextIdx < 0) break;

      int take = min(counts[nextIdx], capacityLeft);
      for (int k = 0; k < take; k++) {
        addItemToJob(job, stationNames[nextIdx]);
      }

      counts[nextIdx] -= take;
      capacityLeft -= take;
      currentPoint = stationPos[nextIdx];
    }

    buildUniqueStations(job);
    jobCount++;
  }
}

// =========================
// 最近鄰排路徑
// =========================
void buildRouteForJob(Job &job) {
  job.routeCount = 0;

  bool visited[MAX_ITEMS_PER_JOB] = {false};
  Point current = depot;

  for (int step = 0; step < job.uniqueCount; step++) {
    int bestIdx = -1;
    int bestDist = 9999;

    for (int i = 0; i < job.uniqueCount; i++) {
      if (visited[i]) continue;
      int sIdx = stationIndex(job.uniqueStations[i]);
      int dist = manhattan(current, stationPos[sIdx]);

      if (dist < bestDist) {
        bestDist = dist;
        bestIdx = i;
      }
    }

    if (bestIdx >= 0) {
      visited[bestIdx] = true;
      job.route[job.routeCount++] = job.uniqueStations[bestIdx];
      current = stationPos[stationIndex(job.uniqueStations[bestIdx])];
    }
  }

  job.route[job.routeCount++] = 'S';
}

void buildAllRoutes() {
  for (int i = 0; i < jobCount; i++) {
    buildRouteForJob(jobs[i]);
  }
}

void printOneJob(int idx) {
  BT.listen();

  BT.print("JOB ");
  BT.print(idx + 1);
  BT.print(" items: ");
  for (int i = 0; i < jobs[idx].itemCount; i++) {
    BT.print(jobs[idx].items[i]);
    if (i < jobs[idx].itemCount - 1) BT.print(",");
  }
  BT.println();

  BT.print("JOB ");
  BT.print(idx + 1);
  BT.print(" route: S -> ");
  for (int i = 0; i < jobs[idx].routeCount; i++) {
    BT.print(jobs[idx].route[i]);
    if (i < jobs[idx].routeCount - 1) BT.print(" -> ");
  }
  BT.println();
}

void printAllJobs() {
  logMsg("=== PLAN RESULT ===");
  BT.listen();
  BT.print("Total jobs = ");
  BT.println(jobCount);

  for (int i = 0; i < jobCount; i++) {
    printOneJob(i);
  }
  logMsg("===================");
}

// =========================
// 建立逐格路徑
// =========================
void buildPath(Point from, Point to) {
  pathLen = 0;
  pathIndex = 0;

  Point p = from;

  while (p.x != to.x && pathLen < MAX_PATH_POINTS) {
    p.x += (to.x > p.x) ? 1 : -1;
    currentPath[pathLen++] = p;
  }

  while (p.y != to.y && pathLen < MAX_PATH_POINTS) {
    p.y += (to.y > p.y) ? 1 : -1;
    currentPath[pathLen++] = p;
  }
}

Facing desiredFacingTo(Point from, Point to) {
  if (to.x > from.x) return FACE_RIGHT;
  if (to.x < from.x) return FACE_LEFT;
  if (to.y > from.y) return FACE_UP;
  return FACE_DOWN;
}

NavAction relativeAction(Facing now, Facing want) {
  if (now == want) return ACT_STRAIGHT;
  if (((now + 1) % 4) == want) return ACT_RIGHT;
  if (((now + 3) % 4) == want) return ACT_LEFT;
  return ACT_BACK;
}

// =========================
// 感測器 / 路口 / 循線邏輯
// =========================
void readLineSensors(int s[5], bool &L2, bool &L1, bool &C, bool &R1, bool &R2) {
  for (int i = 0; i < 5; i++) {
    s[i] = analogRead(sensorPins[i]);
  }

  L2 = s[0] < THRESHOLD;
  L1 = s[1] < THRESHOLD;
  C  = s[2] < THRESHOLD;
  R1 = s[3] < THRESHOLD;
  R2 = s[4] < THRESHOLD;
}

int blackCount(bool L2, bool L1, bool C, bool R1, bool R2) {
  return (L2 ? 1 : 0) + (L1 ? 1 : 0) + (C ? 1 : 0) + (R1 ? 1 : 0) + (R2 ? 1 : 0);
}

// 兩個以上黑就算路口
bool isIntersection(bool L2, bool L1, bool C, bool R1, bool R2) {
  return blackCount(L2, L1, C, R1, R2) >= 2;
}

// 直走時只做簡單修正
void lineFollowStraight(bool L2, bool L1, bool C, bool R1, bool R2) {
  if (L1 || L2) {
    pivotLeft();
  } else if (R1 || R2) {
    pivotRight();
  } else {
    forward();
  }
}

// =========================
// 移動狀態機控制
// =========================
void resetMoveState() {
  moveState = MOVE_FOLLOW;
  pendingAction = ACT_STRAIGHT;
  intersectionLatched = false;
  moveTimer = 0;
  currentTurnDuration = 0;
}

void beginTurnAction(NavAction action) {
  pendingAction = action;
  moveTimer = millis();

  // 直走：先停 2 秒，再直走穿越路口
  if (action == ACT_STRAIGHT) {
    currentTurnDuration = STRAIGHT_CROSS_MS;
    moveState = MOVE_PREPARE;
    intersectionLatched = true;
    return;
  }

  moveState = MOVE_PREPARE;

  if (action == ACT_LEFT) {
    turnDir = TD_LEFT;
    currentTurnDuration = TURN_LEFT_MS;
  } else if (action == ACT_RIGHT) {
    turnDir = TD_RIGHT;
    currentTurnDuration = TURN_RIGHT_MS;
  } else {
    // 迴轉：只做一次 180 度
    turnDir = TD_LEFT;
    currentTurnDuration = TURN_BACK_MS;
  }
}

void handleReachedIntersection() {
  if (pathIndex < pathLen) {
    currentPos = currentPath[pathIndex];
    pathIndex++;
    logMsg("Intersection " + pointToString(currentPos));
  }

  if (pathIndex >= pathLen) {
    stopCar();

    if (!currentTargetIsDepot) {
      logMsg("Arrived at " + String(currentTarget));
      missionState = M_UNLOAD;
      moveState = MOVE_FOLLOW;
      moveTimer = millis();
    } else {
      logMsg("Back to S");
      missionState = M_REORIENT;
      beginTurnAction(relativeAction(currentFacing, FACE_RIGHT));
    }
    return;
  }

  Facing want = desiredFacingTo(currentPos, currentPath[pathIndex]);
  NavAction action = relativeAction(currentFacing, want);
  beginTurnAction(action);
}

// =========================
// 核心移動邏輯
// =========================
void processTravel() {
  int s[5];
  bool L2, L1, C, R1, R2;
  readLineSensors(s, L2, L1, C, R1, R2);

  switch (moveState) {
    case MOVE_FOLLOW:
      if (isIntersection(L2, L1, C, R1, R2)) {
        if (!intersectionLatched) {
          intersectionLatched = true;
          handleReachedIntersection();
        }
        return;
      } else {
        // 離開路口後才允許偵測下一個路口
        intersectionLatched = false;
      }

      lineFollowStraight(L2, L1, C, R1, R2);
      return;

    case MOVE_CROSS:
      // 直走穿越路口：停完後再往前推一小段
      forward();
      if (millis() - moveTimer >= currentTurnDuration) {
        moveState = MOVE_FOLLOW;
      }
      return;

    case MOVE_PREPARE:
      stopCar();
      if (millis() - moveTimer >= PREPARE_STOP_MS) {
        if (pendingAction == ACT_STRAIGHT) {
          moveState = MOVE_CROSS;
          moveTimer = millis();
        } else {
          moveState = MOVE_TURNING;
          moveTimer = millis();
        }
      }
      return;

    case MOVE_TURNING:
      // 只有迴轉時，改成兩輪一起動
      if (pendingAction == ACT_BACK) {
        spinLeft();
      } else {
        executePivot();
      }

      if (millis() - moveTimer >= currentTurnDuration) {
        stopCar();
        updateFacingAfterAction();
        moveState = MOVE_FINISH;
        moveTimer = millis();
      }
      return;

    case MOVE_FINISH:
      stopCar();
      if (millis() - moveTimer >= FINISH_STOP_MS) {
        moveState = MOVE_FOLLOW;
        intersectionLatched = true;
        // 避免剛轉完還壓在路口上又被重複判定一次
      }
      return;
  }
}

// 回到起點後，重新朝右
void processReorient() {
  switch (moveState) {
    case MOVE_FOLLOW:
      if (pendingAction == ACT_STRAIGHT) {
        missionState = M_DEPOT_WAIT;
        moveTimer = millis();
        logMsg("Facing RIGHT at S, wait 5 sec");
        stopCar();
      }
      return;

    case MOVE_CROSS:
      forward();
      if (millis() - moveTimer >= currentTurnDuration) {
        moveState = MOVE_FOLLOW;
      }
      return;

    case MOVE_PREPARE:
      stopCar();
      if (millis() - moveTimer >= PREPARE_STOP_MS) {
        if (pendingAction == ACT_STRAIGHT) {
          moveState = MOVE_CROSS;
          moveTimer = millis();
        } else {
          moveState = MOVE_TURNING;
          moveTimer = millis();
        }
      }
      return;

    case MOVE_TURNING:
      if (pendingAction == ACT_BACK) {
        spinLeft();
      } else {
        executePivot();
      }

      if (millis() - moveTimer >= currentTurnDuration) {
        stopCar();
        updateFacingAfterAction();
        moveState = MOVE_FINISH;
        moveTimer = millis();
      }
      return;

    case MOVE_FINISH:
      stopCar();
      if (millis() - moveTimer >= FINISH_STOP_MS) {
        missionState = M_DEPOT_WAIT;
        moveTimer = millis();
        logMsg("Facing RIGHT at S, wait 5 sec");
      }
      return;
  }
}

// =========================
// 任務流程
// =========================
void prepareTravelTo(char target) {
  currentTarget = target;
  currentTargetIsDepot = (target == 'S');

  Point targetPos = getPointByStation(target);
  buildPath(currentPos, targetPos);

  resetMoveState();

  if (pathLen == 0) {
    if (currentTargetIsDepot) {
      missionState = M_REORIENT;
      beginTurnAction(relativeAction(currentFacing, FACE_RIGHT));
    } else {
      logMsg("Arrived at " + String(currentTarget));
      missionState = M_UNLOAD;
      moveTimer = millis();
    }
    return;
  }

  Facing want = desiredFacingTo(currentPos, currentPath[0]);
  beginTurnAction(relativeAction(currentFacing, want));
  missionState = M_TRAVEL;
}

void startJob(int idx) {
  if (idx >= jobCount) {
    missionState = M_DONE;
    logMsg("ALL JOBS DONE");
    stopCar();
    return;
  }

  currentJobIndex = idx;
  currentRouteIndex = 0;

  logMsg("==== START JOB " + String(idx + 1) + " ====");
  printOneJob(idx);

  prepareTravelTo(jobs[idx].route[currentRouteIndex]);
}

void resetMissionRuntimeState() {
  currentPos = depot;
  currentFacing = FACE_RIGHT;   // 你說會保證車頭朝右
  resetMoveState();
}

void startMission() {
  if (jobCount == 0) {
    logMsg("NO PLAN, PLEASE INPUT ORDERS FIRST");
    return;
  }

  resetMissionRuntimeState();
  missionState = M_READY;
  startJob(0);
}

void gotoNextTargetAfterUnload() {
  currentRouteIndex++;

  if (currentRouteIndex < jobs[currentJobIndex].routeCount) {
    prepareTravelTo(jobs[currentJobIndex].route[currentRouteIndex]);
  } else {
    missionState = M_DONE;
    logMsg("JOB ROUTE END");
  }
}

void afterDepotPauseDone() {
  int nextJob = currentJobIndex + 1;
  if (nextJob < jobCount) {
    startJob(nextJob);
  } else {
    missionState = M_DONE;
    logMsg("ALL DELIVERY FINISHED");
    stopCar();
  }
}

// =========================
// 藍牙命令
// =========================
void handleCommand(String cmd) {
  cmd.trim();

  if (cmd == "START") {
    inputMode = INPUT_ORDERS;
    orderBuffer = "";
    jobCount = 0;
    missionState = M_IDLE;
    stopCar();
    logMsg("INPUT MODE ON");
    logMsg("Press A~G buttons, then STOP");
    return;
  }

  if (cmd == "STOP") {
    if (inputMode == INPUT_ORDERS) {
      inputMode = WAIT_CMD;

      logMsg("ORDER = " + orderBuffer);

      buildGreedyJobs(orderBuffer);
      buildAllRoutes();
      printAllJobs();

      missionState = M_READY;
      logMsg("PLAN READY, SEND GO");
    } else {
      missionState = M_IDLE;
      stopCar();
      logMsg("MISSION STOP");
    }
    return;
  }

  if (cmd == "GO") {
    if (missionState == M_READY) {
      logMsg("GO pressed, direct start");
      startMission();
    } else {
      logMsg("NOT READY");
    }
    return;
  }

  if (inputMode == INPUT_ORDERS) {
    if (cmd.length() == 1) {
      char ch = cmd.charAt(0);
      int idx = stationIndex(ch);
      if (idx >= 0) {
        orderBuffer += ch;
        logMsg("BUFFER = " + orderBuffer);
        return;
      }
    }
  }

  logMsg("UNKNOWN CMD");
}

void readBluetooth() {
  BT.listen();

  while (BT.available()) {
    char c = BT.read();

    if (c == '\n') {
      inputLine.trim();
      if (inputLine.length() > 0) handleCommand(inputLine);
      inputLine = "";
    } else if (c != '\r') {
      inputLine += c;
    }
  }
}

// =========================
// setup / loop
// =========================
void setup() {
  Serial.begin(9600);
  BT.begin(9600);

  motorR.attach(12);   // 原本 m1，實際是右輪
  motorL.attach(13);   // 原本 m2，實際是左輪

  stopCar();

  logMsg("READY");
  logMsg("Use buttons:");
  logMsg("START");
  logMsg("A~G");
  logMsg("STOP");
  logMsg("GO");
}

void loop() {
  readBluetooth();

  switch (missionState) {
    case M_TRAVEL:
      processTravel();
      break;

    case M_UNLOAD:
      stopCar();
      if (millis() - moveTimer >= 5000) {
        logMsg("Unload complete at " + String(currentTarget));
        gotoNextTargetAfterUnload();
      }
      break;

    case M_REORIENT:
      processReorient();
      break;

    case M_DEPOT_WAIT:
      stopCar();
      if (millis() - moveTimer >= 5000) {
        afterDepotPauseDone();
      }
      break;

    case M_IDLE:
    case M_READY:
    case M_DONE:
    default:
      stopCar();
      break;
  }
}