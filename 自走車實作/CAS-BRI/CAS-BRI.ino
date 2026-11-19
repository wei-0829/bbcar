#include <Arduino.h>
#include <Servo.h>
#include <SoftwareSerial.h>

#ifdef __AVR__
  #include <avr/pgmspace.h>
#endif

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
const int16_t STOP_R = 1500;   // 右輪停止值
const int16_t STOP_L = 1500;   // 左輪停止值

// 前進速度（保留你現在已經可正常運作的值）
const int16_t FORWARD_R = 1350;   // 右輪前進
const int16_t FORWARD_L = 1650;   // 左輪前進

// 後退速度（只有迴轉時用）
const int16_t BACKWARD_R = 1650;  // 右輪後退
const int16_t BACKWARD_L = 1350;  // 左輪後退

const uint8_t sensorPins[5] = {A0, A1, A2, A3, A4};   // L2, L1, C, R1, R2

// =========================
// 轉彎 / 停頓參數
// =========================
const uint16_t PREPARE_STOP_MS   = 2000;   // 到路口先停 2 秒
const uint16_t FINISH_STOP_MS    = 2000;   // 左/右/迴轉後停一下
const uint16_t TURN_LEFT_MS      = 1100;   // 左轉時間
const uint16_t TURN_RIGHT_MS     = 1100;   // 右轉時間
const uint16_t TURN_BACK_MS      = 1250;   // 180度迴轉時間（只轉一次）
const uint16_t STRAIGHT_CROSS_MS = 400;    // 直走穿越路口時間
const uint16_t UNLOAD_WAIT_MS    = 5000;
const uint16_t DEPOT_WAIT_MS     = 5000;

// =========================
// 地圖 / 站點
// =========================
struct Point {
  int8_t x;
  int8_t y;
};

enum Facing : uint8_t {
  FACE_UP = 0,
  FACE_RIGHT = 1,
  FACE_DOWN = 2,
  FACE_LEFT = 3
};

const uint8_t CAPACITY = 5;
const uint8_t NUM_STATIONS = 8;

// 地圖座標設定：
//   (0, 0) 為地圖正中心
//   x 範圍：-2 ~ 2
//   y 範圍：-2 ~ 2
//   裝貨點 / 起點 S 固定在最下方中央：(0, -2)
const int8_t MAP_X_MIN = -2;
const int8_t MAP_X_MAX =  2;
const int8_t MAP_Y_MIN = -2;
const int8_t MAP_Y_MAX =  2;

const Point depot = {0, -2};

// A ~ H 貨架位置。放在 Flash，減少 SRAM 使用量。
// 注意：不要把貨架放在 S = (0, -2)，避免貨架與裝貨點重疊。
const int8_t stationX[NUM_STATIONS] PROGMEM = {-2, -1,  0, -2, -1,  1,  2,  2};
const int8_t stationY[NUM_STATIONS] PROGMEM = { 2,  1,  0,  0, -1,  2,  1, -1};

// =========================
// 貨單資料
// =========================
const uint8_t MAX_JOBS = 20;
const uint8_t MAX_ITEMS_PER_JOB = CAPACITY;
const uint8_t MAX_ROUTE_POINTS = CAPACITY + 1;   // 最多 5 個貨架 + 回 S
const uint8_t MAX_PATH_POINTS = 10;              // 4 x 4 地圖，最遠曼哈頓距離 8，10 已足夠
const uint8_t MAX_ORDER_CHARS = MAX_JOBS * CAPACITY;
const uint8_t MAX_INPUT_LINE = 16;

struct Job {
  // stations[] 只存「不同貨架」，不再同時存 items[] 與 uniqueStations[]，減少重複資料。
  char stations[MAX_ITEMS_PER_JOB];
  uint8_t stationLoads[MAX_ITEMS_PER_JOB];
  uint8_t stopCount;
  uint8_t itemCount;

  char route[MAX_ROUTE_POINTS];
  uint8_t routeCount;
};

Job jobs[MAX_JOBS];
uint8_t jobCount = 0;

// =========================
// 手機輸入狀態
// =========================
enum InputMode : uint8_t {
  WAIT_CMD,
  INPUT_ORDERS
};

InputMode inputMode = WAIT_CMD;
char inputLine[MAX_INPUT_LINE];
uint8_t inputLen = 0;
char orderBuffer[MAX_ORDER_CHARS + 1];
uint8_t orderLen = 0;

// =========================
// 任務狀態
// =========================
enum MissionState : uint8_t {
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
enum MoveState : uint8_t {
  MOVE_FOLLOW,     // 一般循線
  MOVE_CROSS,      // 直走穿越路口
  MOVE_PREPARE,    // 路口先停
  MOVE_TURNING,    // 固定秒數轉彎
  MOVE_FINISH      // 轉彎後停一下
};

MoveState moveState = MOVE_FOLLOW;

enum NavAction : uint8_t {
  ACT_STRAIGHT,
  ACT_LEFT,
  ACT_RIGHT,
  ACT_BACK
};

enum TurnDir : uint8_t {
  TD_LEFT,
  TD_RIGHT
};

NavAction pendingAction = ACT_STRAIGHT;
TurnDir turnDir = TD_LEFT;
bool intersectionLatched = false;
unsigned long moveTimer = 0;
uint16_t currentTurnDuration = 0;

// =========================
// 目前配送進度
// =========================
uint8_t currentJobIndex = 0;
uint8_t currentRouteIndex = 0;
char currentTarget = 'S';
bool currentTargetIsDepot = false;

Point currentPos = {0, -2};
Facing currentFacing = FACE_RIGHT;  // 起點在 S=(0,-2)，車頭預設朝右

Point currentPath[MAX_PATH_POINTS];
uint8_t pathLen = 0;
uint8_t pathIndex = 0;

// =========================
// 訊息輸出：不使用 String，避免 SRAM heap 碎片
// =========================
void logMsg(const __FlashStringHelper *msg) {
  Serial.println(msg);
  BT.listen();
  BT.println(msg);
}

void logMsg(const char *msg) {
  Serial.println(msg);
  BT.listen();
  BT.println(msg);
}

void printPointTo(Stream &out, Point p) {
  out.print('(');
  out.print(p.x);
  out.print(',');
  out.print(p.y);
  out.print(')');
}

void logIntersection(Point p) {
  Serial.print(F("Intersection "));
  printPointTo(Serial, p);
  Serial.println();

  BT.listen();
  BT.print(F("Intersection "));
  printPointTo(BT, p);
  BT.println();
}

void logStationLine(const __FlashStringHelper *prefix, char station) {
  Serial.print(prefix);
  Serial.println(station);

  BT.listen();
  BT.print(prefix);
  BT.println(station);
}

void logStartJob(uint8_t idx) {
  Serial.print(F("==== START JOB "));
  Serial.print(idx + 1);
  Serial.println(F(" ===="));

  BT.listen();
  BT.print(F("==== START JOB "));
  BT.print(idx + 1);
  BT.println(F(" ===="));
}

void logOrderBuffer() {
  Serial.print(F("ORDER = "));
  Serial.println(orderBuffer);

  BT.listen();
  BT.print(F("ORDER = "));
  BT.println(orderBuffer);
}

void logBufferNow() {
  Serial.print(F("BUFFER = "));
  Serial.println(orderBuffer);

  BT.listen();
  BT.print(F("BUFFER = "));
  BT.println(orderBuffer);
}

// =========================
// 站點 / 距離工具
// =========================
char stationName(uint8_t idx) {
  return char('A' + idx);
}

int8_t stationIndex(char ch) {
  if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
  if (ch >= 'A' && ch < char('A' + NUM_STATIONS)) return ch - 'A';
  return -1;
}

Point getStationPoint(uint8_t idx) {
  Point p;
  p.x = (int8_t)pgm_read_byte(&stationX[idx]);
  p.y = (int8_t)pgm_read_byte(&stationY[idx]);
  return p;
}

Point getPointByStation(char ch) {
  if (ch == 'S') return depot;
  int8_t idx = stationIndex(ch);
  if (idx >= 0) return getStationPoint((uint8_t)idx);
  return depot;
}

int16_t manhattan(Point a, Point b) {
  return abs(a.x - b.x) + abs(a.y - b.y);
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
void initJob(Job &job) {
  job.stopCount = 0;
  job.itemCount = 0;
  job.routeCount = 0;
}

void clearJobs() {
  jobCount = 0;
  for (uint8_t i = 0; i < MAX_JOBS; i++) {
    initJob(jobs[i]);
  }
}

void addStationToJob(Job &job, uint8_t stationIdx, uint8_t amount) {
  if (amount == 0) return;
  if (job.stopCount >= MAX_ITEMS_PER_JOB) return;
  if (job.itemCount + amount > CAPACITY) return;

  job.stations[job.stopCount] = stationName(stationIdx);
  job.stationLoads[job.stopCount] = amount;
  job.stopCount++;
  job.itemCount += amount;
}

// =========================
// CAS-BRI：容量感知節省值分批路徑改善法
// =========================
void buildFullStationJobsFirst(uint8_t counts[]) {
  // CAS-BRI Step 1：如果單一貨架貨物數 >= CAPACITY，先直接成為滿載貨單。
  for (uint8_t i = 0; i < NUM_STATIONS && jobCount < MAX_JOBS; i++) {
    while (counts[i] >= CAPACITY && jobCount < MAX_JOBS) {
      Job &job = jobs[jobCount];
      initJob(job);

      addStationToJob(job, i, CAPACITY);
      counts[i] -= CAPACITY;
      jobCount++;
    }
  }
}

void buildRemainingCASBRIJobs(uint8_t counts[]) {
  // CAS-BRI Step 2：對剩餘不足一車的站點做 Clarke-Wright Saving 合併。
  // saving(i,j) = d(S,i) + d(S,j) - d(i,j)
  const uint8_t MAX_SAVINGS = NUM_STATIONS * (NUM_STATIONS - 1) / 2;

  struct SavingItem {
    int8_t value;
    uint8_t a;
    uint8_t b;
  };

  SavingItem savings[MAX_SAVINGS];
  uint8_t savingCount = 0;

  bool routeActive[NUM_STATIONS];
  uint8_t routeLoad[NUM_STATIONS];
  uint8_t routeLen[NUM_STATIONS];
  uint8_t routeStations[NUM_STATIONS][NUM_STATIONS];
  int8_t routeOfStation[NUM_STATIONS];

  for (uint8_t i = 0; i < NUM_STATIONS; i++) {
    routeActive[i] = false;
    routeLoad[i] = 0;
    routeLen[i] = 0;
    routeOfStation[i] = -1;
  }

  // 先把每個仍有貨物的站點各自視為一條路線。
  for (uint8_t i = 0; i < NUM_STATIONS; i++) {
    if (counts[i] > 0) {
      routeActive[i] = true;
      routeLoad[i] = counts[i];
      routeLen[i] = 1;
      routeStations[i][0] = i;
      routeOfStation[i] = i;
    }
  }

  // 建立所有站點配對的 saving 值。
  for (uint8_t i = 0; i < NUM_STATIONS; i++) {
    if (counts[i] == 0) continue;
    for (uint8_t j = i + 1; j < NUM_STATIONS; j++) {
      if (counts[j] == 0) continue;

      Point pi = getStationPoint(i);
      Point pj = getStationPoint(j);
      int16_t value = manhattan(depot, pi) + manhattan(depot, pj) - manhattan(pi, pj);
      if (value > 127) value = 127;
      if (value < -128) value = -128;

      savings[savingCount].value = (int8_t)value;
      savings[savingCount].a = i;
      savings[savingCount].b = j;
      savingCount++;
    }
  }

  // 由大到小排序 saving。
  for (uint8_t i = 0; i + 1 < savingCount; i++) {
    for (uint8_t j = i + 1; j < savingCount; j++) {
      if (savings[j].value > savings[i].value) {
        SavingItem tmp = savings[i];
        savings[i] = savings[j];
        savings[j] = tmp;
      }
    }
  }

  // 依 saving 由大到小嘗試合併，不可超過載貨上限 CAPACITY。
  for (uint8_t k = 0; k < savingCount; k++) {
    uint8_t a = savings[k].a;
    uint8_t b = savings[k].b;
    int8_t ra = routeOfStation[a];
    int8_t rb = routeOfStation[b];

    if (ra < 0 || rb < 0) continue;
    if (ra == rb) continue;
    if (!routeActive[ra] || !routeActive[rb]) continue;
    if (routeLoad[ra] + routeLoad[rb] > CAPACITY) continue;

    // 把 rb 路線接到 ra 後面，站點順序之後會再用路徑改善重新排列。
    for (uint8_t t = 0; t < routeLen[rb]; t++) {
      uint8_t stationIdx = routeStations[rb][t];
      routeStations[ra][routeLen[ra]++] = stationIdx;
      routeOfStation[stationIdx] = ra;
    }

    routeLoad[ra] += routeLoad[rb];
    routeActive[rb] = false;
    routeLen[rb] = 0;
    routeLoad[rb] = 0;
  }

  // 把合併後的每條路線轉成貨單。
  for (uint8_t r = 0; r < NUM_STATIONS && jobCount < MAX_JOBS; r++) {
    if (!routeActive[r]) continue;
    if (routeLen[r] == 0) continue;

    Job &job = jobs[jobCount];
    initJob(job);

    for (uint8_t t = 0; t < routeLen[r]; t++) {
      uint8_t stationIdx = routeStations[r][t];
      addStationToJob(job, stationIdx, counts[stationIdx]);
      counts[stationIdx] = 0;
    }

    if (job.itemCount > 0) {
      jobCount++;
    }
  }
}

void buildCASBRIJobs() {
  clearJobs();

  uint8_t counts[NUM_STATIONS] = {0};

  // 1. 統計每個站點的貨物數量。
  for (uint8_t i = 0; i < orderLen; i++) {
    int8_t idx = stationIndex(orderBuffer[i]);
    if (idx >= 0 && counts[idx] < 255) counts[idx]++;
  }

  // 2. CAS-BRI Step 1：同一貨架若可滿載，先獨立成單。
  buildFullStationJobsFirst(counts);

  // 3. CAS-BRI Step 2~3：剩餘貨物用節省值合併，並受 CAPACITY 約束。
  buildRemainingCASBRIJobs(counts);
}

// =========================
// CAS-BRI Step 4：貨單內路徑改善
// 因為 CAPACITY = 5，每趟最多 5 個貨物；去重後貨架數也最多 5。
// 所以直接窮舉全排列，找出 S 出發、走完所有貨架、回 S 的最短順序。
// =========================
int16_t routeDistanceWithDepotReturn(char route[], uint8_t count) {
  int16_t distance = 0;
  Point current = depot;

  for (uint8_t i = 0; i < count; i++) {
    Point next = getPointByStation(route[i]);
    distance += manhattan(current, next);
    current = next;
  }

  distance += manhattan(current, depot);
  return distance;
}

void bruteForceRouteDFS(Job &job, char perm[], bool used[], uint8_t depth, uint8_t n,
                        int16_t &bestDistance, char bestRoute[]) {
  if (depth == n) {
    int16_t d = routeDistanceWithDepotReturn(perm, n);
    if (d < bestDistance) {
      bestDistance = d;
      for (uint8_t i = 0; i < n; i++) {
        bestRoute[i] = perm[i];
      }
    }
    return;
  }

  for (uint8_t i = 0; i < n; i++) {
    if (used[i]) continue;
    used[i] = true;
    perm[depth] = job.stations[i];
    bruteForceRouteDFS(job, perm, used, depth + 1, n, bestDistance, bestRoute);
    used[i] = false;
  }
}

void buildRouteForJob(Job &job) {
  job.routeCount = 0;

  if (job.stopCount == 0) {
    job.route[job.routeCount++] = 'S';
    return;
  }

  char perm[MAX_ITEMS_PER_JOB];
  char bestRoute[MAX_ITEMS_PER_JOB];
  bool used[MAX_ITEMS_PER_JOB] = {false};
  int16_t bestDistance = 32767;

  bruteForceRouteDFS(job, perm, used, 0, job.stopCount, bestDistance, bestRoute);

  for (uint8_t i = 0; i < job.stopCount; i++) {
    job.route[job.routeCount++] = bestRoute[i];
  }

  // 每張貨單最後都回到裝貨點 S。
  job.route[job.routeCount++] = 'S';
}

void buildAllRoutes() {
  for (uint8_t i = 0; i < jobCount; i++) {
    buildRouteForJob(jobs[i]);
  }
}

// =========================
// 貨單距離計算
// =========================
// 計算單一貨單路線距離。
// 路線格式是：從 S 出發，依序走 job.route[]，最後 job.route[] 裡已經包含回到 S。
// 距離單位是地圖上的格子數，也就是曼哈頓距離。
int16_t calculateJobDistance(Job &job) {
  int16_t distance = 0;
  Point current = depot;

  for (uint8_t i = 0; i < job.routeCount; i++) {
    Point next = getPointByStation(job.route[i]);
    distance += manhattan(current, next);
    current = next;
  }

  return distance;
}

int16_t calculateTotalDistance() {
  int16_t total = 0;

  for (uint8_t i = 0; i < jobCount; i++) {
    total += calculateJobDistance(jobs[i]);
  }

  return total;
}

void printOneJob(uint8_t idx) {
  BT.listen();

  BT.print(F("JOB "));
  BT.print(idx + 1);
  BT.print(F(" items: "));
  bool first = true;
  for (uint8_t s = 0; s < jobs[idx].stopCount; s++) {
    for (uint8_t k = 0; k < jobs[idx].stationLoads[s]; k++) {
      if (!first) BT.print(',');
      BT.print(jobs[idx].stations[s]);
      first = false;
    }
  }
  BT.println();

  BT.print(F("JOB "));
  BT.print(idx + 1);
  BT.print(F(" route: S -> "));
  for (uint8_t i = 0; i < jobs[idx].routeCount; i++) {
    BT.print(jobs[idx].route[i]);
    if (i < jobs[idx].routeCount - 1) BT.print(F(" -> "));
  }
  BT.println();

  BT.print(F("JOB "));
  BT.print(idx + 1);
  BT.print(F(" distance = "));
  BT.println(calculateJobDistance(jobs[idx]));
}

void printAllJobs() {
  logMsg(F("=== PLAN RESULT ==="));
  BT.listen();
  BT.print(F("Total jobs = "));
  BT.println(jobCount);

  for (uint8_t i = 0; i < jobCount; i++) {
    printOneJob(i);
  }

  BT.print(F("Total distance = "));
  BT.println(calculateTotalDistance());

  logMsg(F("==================="));
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
void readLineSensors(uint16_t s[5], bool &L2, bool &L1, bool &C, bool &R1, bool &R2) {
  for (uint8_t i = 0; i < 5; i++) {
    s[i] = analogRead(sensorPins[i]);
  }

  L2 = s[0] < THRESHOLD;
  L1 = s[1] < THRESHOLD;
  C  = s[2] < THRESHOLD;
  R1 = s[3] < THRESHOLD;
  R2 = s[4] < THRESHOLD;
}

uint8_t blackCount(bool L2, bool L1, bool C, bool R1, bool R2) {
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
    logIntersection(currentPos);
  }

  if (pathIndex >= pathLen) {
    stopCar();

    if (!currentTargetIsDepot) {
      logStationLine(F("Arrived at "), currentTarget);
      missionState = M_UNLOAD;
      moveState = MOVE_FOLLOW;
      moveTimer = millis();
    } else {
      logMsg(F("Back to S"));
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
  uint16_t s[5];
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
        logMsg(F("Facing RIGHT at S, wait 5 sec"));
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
        logMsg(F("Facing RIGHT at S, wait 5 sec"));
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
      logStationLine(F("Arrived at "), currentTarget);
      missionState = M_UNLOAD;
      moveTimer = millis();
    }
    return;
  }

  Facing want = desiredFacingTo(currentPos, currentPath[0]);
  beginTurnAction(relativeAction(currentFacing, want));
  missionState = M_TRAVEL;
}

void startJob(uint8_t idx) {
  if (idx >= jobCount) {
    missionState = M_DONE;
    logMsg(F("ALL JOBS DONE"));
    stopCar();
    return;
  }

  currentJobIndex = idx;
  currentRouteIndex = 0;

  logStartJob(idx);
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
    logMsg(F("NO PLAN, PLEASE INPUT ORDERS FIRST"));
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
    logMsg(F("JOB ROUTE END"));
  }
}

void afterDepotPauseDone() {
  uint8_t nextJob = currentJobIndex + 1;
  if (nextJob < jobCount) {
    startJob(nextJob);
  } else {
    missionState = M_DONE;
    logMsg(F("ALL DELIVERY FINISHED"));
    stopCar();
  }
}

// =========================
// 藍牙命令
// =========================
void clearOrderBuffer() {
  orderLen = 0;
  orderBuffer[0] = '\0';
}

void appendOrderChar(char ch) {
  if (orderLen >= MAX_ORDER_CHARS) {
    logMsg(F("ORDER BUFFER FULL"));
    return;
  }
  orderBuffer[orderLen++] = ch;
  orderBuffer[orderLen] = '\0';
  logBufferNow();
}

void trimInput(char *s) {
  uint8_t start = 0;
  while (s[start] == ' ' || s[start] == '\t' || s[start] == '\n' || s[start] == '\r') start++;

  uint8_t len = strlen(s + start);
  while (len > 0) {
    char c = s[start + len - 1];
    if (c == ' ' || c == '\t' || c == '\n' || c == '\r') len--;
    else break;
  }

  if (start > 0) {
    for (uint8_t i = 0; i < len; i++) s[i] = s[start + i];
  }
  s[len] = '\0';
}

void handleCommand(char *cmd) {
  trimInput(cmd);

  if (strcmp(cmd, "START") == 0) {
    inputMode = INPUT_ORDERS;
    clearOrderBuffer();
    jobCount = 0;
    missionState = M_IDLE;
    stopCar();
    logMsg(F("INPUT MODE ON"));
    logMsg(F("Press A~G buttons, then STOP"));
    return;
  }

  if (strcmp(cmd, "STOP") == 0) {
    if (inputMode == INPUT_ORDERS) {
      inputMode = WAIT_CMD;

      logOrderBuffer();

      buildCASBRIJobs();
      buildAllRoutes();
      printAllJobs();

      missionState = M_READY;
      logMsg(F("PLAN READY, SEND GO"));
    } else {
      missionState = M_IDLE;
      stopCar();
      logMsg(F("MISSION STOP"));
    }
    return;
  }

  if (strcmp(cmd, "GO") == 0) {
    if (missionState == M_READY) {
      logMsg(F("GO pressed, direct start"));
      startMission();
    } else {
      logMsg(F("NOT READY"));
    }
    return;
  }

  if (inputMode == INPUT_ORDERS && strlen(cmd) == 1) {
    char ch = cmd[0];
    int8_t idx = stationIndex(ch);
    if (idx >= 0) {
      appendOrderChar(stationName((uint8_t)idx));
      return;
    }
  }

  logMsg(F("UNKNOWN CMD"));
}

void readBluetooth() {
  BT.listen();

  while (BT.available()) {
    char c = BT.read();

    if (c == '\n') {
      inputLine[inputLen] = '\0';
      if (inputLen > 0) handleCommand(inputLine);
      inputLen = 0;
      inputLine[0] = '\0';
    } else if (c != '\r') {
      if (inputLen < MAX_INPUT_LINE - 1) {
        inputLine[inputLen++] = c;
      } else {
        // 太長就丟掉，避免越界。
        inputLen = 0;
        inputLine[0] = '\0';
        logMsg(F("INPUT TOO LONG"));
      }
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

  logMsg(F("READY - CAS-BRI OPTIMIZED"));
  logMsg(F("Map: center=(0,0), S=(0,-2), x/y=-2~2"));
  logMsg(F("Use buttons:"));
  logMsg(F("START"));
  logMsg(F("A~G"));
  logMsg(F("STOP"));
  logMsg(F("GO"));
}

void loop() {
  readBluetooth();

  switch (missionState) {
    case M_TRAVEL:
      processTravel();
      break;

    case M_UNLOAD:
      stopCar();
      if (millis() - moveTimer >= UNLOAD_WAIT_MS) {
        logStationLine(F("Unload complete at "), currentTarget);
        gotoNextTargetAfterUnload();
      }
      break;

    case M_REORIENT:
      processReorient();
      break;

    case M_DEPOT_WAIT:
      stopCar();
      if (millis() - moveTimer >= DEPOT_WAIT_MS) {
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
