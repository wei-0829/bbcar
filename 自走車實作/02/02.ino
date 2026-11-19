#include <Servo.h>
#include <SoftwareSerial.h>

// =========================
// 藍牙通訊
// =========================
SoftwareSerial BT(10, 11);   // HC-06 藍牙模組 RX, TX

// =========================
// 馬達與循線感測器
// =========================
Servo m1, m2;

#define THRESHOLD 400   // 黑白判斷閾值

const int STOP_US = 1500;    // 停止脈寬
const int BASE_L  = 1350;    // 左輪前進脈寬
const int BASE_R  = 1650;    // 右輪前進脈寬

const int sensorPins[5] = {A0, A1, A2, A3, A4};  // L2, L1, C, R1, R2

// =========================
// 轉彎參數
// =========================
const unsigned long PREPARE_STOP_MS = 1000;   // 路口前停 1 秒
const unsigned long FINISH_STOP_MS  = 1000;   // 動作後停 1 秒
const unsigned long TURN_BLIND_MS   = 500;    // 轉彎先盲轉多久再開始看 C

// =========================
// 地圖座標定義
// =========================
struct Point {
  int x;
  int y;
};

// 車頭方向
enum Facing {
  FACE_UP = 0,
  FACE_RIGHT = 1,
  FACE_DOWN = 2,
  FACE_LEFT = 3
};

const int CAPACITY = 5;      // 每張貨單最大容量
const int NUM_STATIONS = 7;  // A~G

// 站點名稱
const char stationNames[NUM_STATIONS] = {'A', 'B', 'C', 'D', 'E', 'F', 'G'};

// 站點座標
Point stationPos[NUM_STATIONS] = {
  {4, 4},  // A
  {2, 0},  // B
  {1, 1},  // C
  {4, 2},  // D
  {0, 3},  // E
  {3, 2},  // F
  {2, 4}   // G
};

// 起點 S
Point depot = {0, 0};

// =========================
// 貨單資料結構
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
// 任務執行狀態
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
// 動作流程狀態
// =========================
enum MotionStage {
  STAGE_MOVE,
  STAGE_PREPARE,
  STAGE_TURN,
  STAGE_FINISH
};

MotionStage motionStage = STAGE_MOVE;

// 路口後要執行的動作
enum NavAction {
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

// 實際轉彎方向
enum TurnDir {
  TD_LEFT,
  TD_RIGHT
};

NavAction pendingAction = ACT_STRAIGHT;
TurnDir turnDir = TD_LEFT;
int turnsRemaining = 0;
bool intersectionLatched = false;

unsigned long stateTimer = 0;

// =========================
// 目前配送進度
// =========================
int currentJobIndex = 0;
int currentRouteIndex = 0;
char currentTarget = 'S';
bool currentTargetIsDepot = false;

Point currentPos = {0, 0};
Facing currentFacing = FACE_RIGHT;   // 起始朝右

Point currentPath[MAX_PATH_POINTS];
int pathLen = 0;
int pathIndex = 0;

// 剛從起點 S 出發時，忽略起點特殊黑線圖樣
bool leavingDepot = false;
bool depotReleased = false;   // 是否真的已經離開起點圖樣

// =========================
// 訊息輸出函式
// =========================
void logMsg(const String &msg) {
  Serial.println(msg);
  BT.listen();
  BT.println(msg);
}

// =========================
// 站點 / 距離工具函式
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
// 馬達控制函式
// =========================
void stopCar() {
  m1.writeMicroseconds(STOP_US);
  m2.writeMicroseconds(STOP_US);
}

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

void rotateFacingLeft() {
  currentFacing = (Facing)((currentFacing + 3) % 4);
}

void rotateFacingRight() {
  currentFacing = (Facing)((currentFacing + 1) % 4);
}

// =========================
// 貨單工具函式
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

void beginAction(NavAction action) {
  pendingAction = action;
  motionStage = STAGE_PREPARE;
  stateTimer = millis();

  if (action == ACT_STRAIGHT) {
    turnsRemaining = 0;
  } else if (action == ACT_LEFT) {
    turnDir = TD_LEFT;
    turnsRemaining = 1;
  } else if (action == ACT_RIGHT) {
    turnDir = TD_RIGHT;
    turnsRemaining = 1;
  } else {
    turnDir = TD_LEFT;
    turnsRemaining = 2;
  }
}

void executePivot() {
  if (turnDir == TD_LEFT) pivotLeft();
  else pivotRight();
}

// C 黑就直走；C 白也先走，直到 L1/R1 黑才修正
void lineFollowSimple(bool L1, bool C, bool R1) {
  if (C) {
    forward();
  } else if (L1) {
    pivotLeft();
  } else if (R1) {
    pivotRight();
  } else {
    forward();
  }
}

// L2 或 R2 黑，或五顆中黑色 >= 2 顆，視為路口
int blackCount(bool L2, bool L1, bool C, bool R1, bool R2) {
  return (L2 ? 1 : 0) + (L1 ? 1 : 0) + (C ? 1 : 0) + (R1 ? 1 : 0) + (R2 ? 1 : 0);
}

bool isIntersection(bool L2, bool L1, bool C, bool R1, bool R2) {
  int blacks = blackCount(L2, L1, C, R1, R2);
  return L2 || R2 || (blacks >= 2);
}

// 起點 S 特殊黑線圖樣：L2、L1、C 黑，R1、R2 白
bool isDepotMarker(bool L2, bool L1, bool C, bool R1, bool R2) {
  return (L2 && L1 && C && !R1 && !R2);
}

// GO 時檢查是否已經正確放在起點
bool checkDepotReady() {
  int s[5];
  bool L2, L1, C, R1, R2;

  for (int i = 0; i < 5; i++) {
    s[i] = analogRead(sensorPins[i]);
  }

  L2 = s[0] < THRESHOLD;
  L1 = s[1] < THRESHOLD;
  C  = s[2] < THRESHOLD;
  R1 = s[3] < THRESHOLD;
  R2 = s[4] < THRESHOLD;

  return (L2 && L1 && C && !R1 && !R2);
}

// 按下 GO 的瞬間，才正式重設導航狀態
void resetMissionRuntimeState() {
  currentPos = depot;
  currentFacing = FACE_RIGHT;
  intersectionLatched = false;
  motionStage = STAGE_MOVE;
  pendingAction = ACT_STRAIGHT;
  turnsRemaining = 0;
  leavingDepot = true;
  depotReleased = false;
}

// =========================
// 任務流程
// =========================
void prepareTravelTo(char target) {
  currentTarget = target;
  currentTargetIsDepot = (target == 'S');

  Point targetPos = getPointByStation(target);
  buildPath(currentPos, targetPos);

  if (currentPos.x == depot.x && currentPos.y == depot.y && target != 'S') {
    leavingDepot = true;
    depotReleased = false;
  } else {
    leavingDepot = false;
    depotReleased = false;
  }

  if (pathLen == 0) {
    if (currentTargetIsDepot) {
      missionState = M_REORIENT;
      NavAction a = relativeAction(currentFacing, FACE_RIGHT);
      beginAction(a);
    } else {
      logMsg("Arrived at " + String(currentTarget));
      missionState = M_UNLOAD;
      stateTimer = millis();
    }
    return;
  }

  Facing want = desiredFacingTo(currentPos, currentPath[0]);
  NavAction a = relativeAction(currentFacing, want);
  beginAction(a);
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

void startMission() {
  if (jobCount == 0) {
    logMsg("NO PLAN, PLEASE INPUT ORDERS FIRST");
    return;
  }

  resetMissionRuntimeState();

  missionState = M_READY;
  startJob(0);
}

void onTargetArrived() {
  stopCar();

  if (!currentTargetIsDepot) {
    logMsg("Arrived at " + String(currentTarget));
    missionState = M_UNLOAD;
    stateTimer = millis();
  } else {
    logMsg("Back to S");
    missionState = M_REORIENT;
    NavAction a = relativeAction(currentFacing, FACE_RIGHT);
    beginAction(a);
  }
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
// 感測器
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

// =========================
// 移動 / 路口 / 轉彎
// =========================
void handleReachedIntersection() {
  if (pathIndex < pathLen) {
    currentPos = currentPath[pathIndex];
    pathIndex++;
    logMsg("Intersection " + pointToString(currentPos));
  }

  if (pathIndex >= pathLen) {
    onTargetArrived();
    return;
  }

  Facing want = desiredFacingTo(currentPos, currentPath[pathIndex]);
  NavAction a = relativeAction(currentFacing, want);
  beginAction(a);
}

void processTravel() {
  int s[5];
  bool L2, L1, C, R1, R2;
  readLineSensors(s, L2, L1, C, R1, R2);

  // =========================
  // 一般循線前進
  // =========================
  if (motionStage == STAGE_MOVE) {

    // 剛從 S 出發時，兩段式解除起點保護
    if (leavingDepot) {
      // 第一步：先確認真的離開起點圖樣
      if (!depotReleased) {
        if (!isDepotMarker(L2, L1, C, R1, R2)) {
          depotReleased = true;
        }

        lineFollowSimple(L1, C, R1);
        return;
      }

      // 第二步：離開起點後，還要再看到一次正常單線狀態
      // 才允許進入一般路口判斷
      if (C && !L2 && !R2) {
        leavingDepot = false;
        intersectionLatched = false;
      }

      lineFollowSimple(L1, C, R1);
      return;
    }

    // 一般路口判斷
    if (isIntersection(L2, L1, C, R1, R2)) {
      if (!intersectionLatched) {
        intersectionLatched = true;
        handleReachedIntersection();
      }
      return;
    } else {
      intersectionLatched = false;
    }

    lineFollowSimple(L1, C, R1);
    return;
  }

  // =========================
  // 到路口先停 1 秒
  // =========================
  if (motionStage == STAGE_PREPARE) {
    stopCar();

    if (millis() - stateTimer >= PREPARE_STOP_MS) {
      if (turnsRemaining == 0) {
        motionStage = STAGE_FINISH;
        stateTimer = millis();
      } else {
        motionStage = STAGE_TURN;
        stateTimer = millis();   // 進入 STAGE_TURN 時重設時間，盲轉計時從這裡開始
      }
    }
    return;
  }

  // =========================
  // 真正轉彎
  // 先盲轉，再看 C，C 黑就停
  // =========================
  if (motionStage == STAGE_TURN) {
    executePivot();

    // 先盲轉一段時間，不看 C
    if (millis() - stateTimer < TURN_BLIND_MS) {
      return;
    }

    // 盲轉結束後，只要 C 黑就停
    if (C) {
      stopCar();

      if (turnDir == TD_LEFT) rotateFacingLeft();
      else rotateFacingRight();

      turnsRemaining--;

      if (turnsRemaining > 0) {
        motionStage = STAGE_PREPARE;
        stateTimer = millis();
      } else {
        motionStage = STAGE_FINISH;
        stateTimer = millis();
      }
    }
    return;
  }

  // =========================
  // 動作做完後再停 1 秒
  // =========================
  if (motionStage == STAGE_FINISH) {
    stopCar();

    if (millis() - stateTimer >= FINISH_STOP_MS) {
      motionStage = STAGE_MOVE;
    }
    return;
  }
}

void processReorient() {
  int s[5];
  bool L2, L1, C, R1, R2;
  readLineSensors(s, L2, L1, C, R1, R2);

  if (pendingAction == ACT_STRAIGHT) {
    missionState = M_DEPOT_WAIT;
    stateTimer = millis();
    logMsg("Facing RIGHT at S, wait 10 sec");
    stopCar();
    return;
  }

  if (motionStage == STAGE_PREPARE) {
    stopCar();
    if (millis() - stateTimer >= PREPARE_STOP_MS) {
      motionStage = STAGE_TURN;
      stateTimer = millis();   // 盲轉計時起點
    }
    return;
  }

  if (motionStage == STAGE_TURN) {
    executePivot();

    // 先盲轉一段時間，不看 C
    if (millis() - stateTimer < TURN_BLIND_MS) {
      return;
    }

    // 盲轉後，只要 C 黑就停
    if (C) {
      stopCar();

      if (turnDir == TD_LEFT) rotateFacingLeft();
      else rotateFacingRight();

      turnsRemaining--;

      if (turnsRemaining > 0) {
        motionStage = STAGE_PREPARE;
        stateTimer = millis();
      } else {
        motionStage = STAGE_FINISH;
        stateTimer = millis();
      }
    }
    return;
  }

  if (motionStage == STAGE_FINISH) {
    stopCar();
    if (millis() - stateTimer >= FINISH_STOP_MS) {
      missionState = M_DEPOT_WAIT;
      stateTimer = millis();
      logMsg("Facing RIGHT at S, wait 10 sec");
    }
    return;
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
      if (checkDepotReady()) {
        logMsg("GO pressed, start from depot marker");
        startMission();
      } else {
        logMsg("Depot marker not ready");
      }
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

  m1.attach(12);
  m2.attach(13);

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
      if (millis() - stateTimer >= 10000) {
        logMsg("Unload complete at " + String(currentTarget));
        gotoNextTargetAfterUnload();
      }
      break;

    case M_REORIENT:
      processReorient();
      break;

    case M_DEPOT_WAIT:
      stopCar();
      if (millis() - stateTimer >= 10000) {
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