/*
 * arm.ino — 팔(XL430×2) + 바스켓 힌지(XL430×2) 제어
 * ──────────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성 (전부 XL430, 12V):
 *   팔:    ARM_ID_A=9,  ARM_ID_B=10  (한 쌍 — B는 코드에서 Reverse 설정)
 *   바스켓: CONT_ID_A=5, CONT_ID_B=6  (한 쌍 — B는 코드에서 Reverse 설정)
 *
 * dxl 인스턴스는 main.ino 에서 선언됨
 */

extern Dynamixel2Arduino dxl;

// ── 모터 ID ──────────────────────────────────────────────
#define ARM_ID_A      9
#define ARM_ID_B      10
#define CONT_ID_A     5
#define CONT_ID_B     6

// ── 팔 위치 (도, 실측) ───────────────────────────────────
#define ARM_DOWN_DEG   132.25f   // 집기 위치
#define ARM_UP_DEG     245.0f    // 투하 위치

// ── 바스켓 위치 (도, 실측) ───────────────────────────────
#define CONT_A_CLOSED  179.30f   // ID5 닫힘
#define CONT_A_OPEN     94.75f   // ID5 열림
#define CONT_B_CLOSED  266.92f   // ID6 닫힘
#define CONT_B_OPEN    353.14f   // ID6 열림

// ── 토크 제한 (%) ────────────────────────────────────────
#define ARM_TORQUE_PCT       80
#define CONTAINER_TORQUE_PCT 60

// ── 모터 한 개 초기화 헬퍼 ───────────────────────────────
static void _initOne(uint8_t id, int pwmLimit, bool reverse) {
  if (!dxl.ping(id)) {
    Serial.print("[팔] 초기화 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.writeControlTableItem(ControlTableItem::DRIVE_MODE, id, reverse ? 1 : 0);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, pwmLimit);
  dxl.torqueOn(id);
}

// ── 초기화 ───────────────────────────────────────────────
void armSetup() {
  int armLimit  = 885 * ARM_TORQUE_PCT  / 100;
  int contLimit = 885 * CONTAINER_TORQUE_PCT / 100;
  _initOne(ARM_ID_A,  armLimit,  false);
  _initOne(ARM_ID_B,  armLimit,  true);   // 반대쪽 모터 Reverse
  _initOne(CONT_ID_A, contLimit, false);
  _initOne(CONT_ID_B, contLimit, false);
  containerClose();
  armDown();
  Serial.println("[팔] 초기화 완료 (팔 내림 + 바스켓 닫힘)");
}

// ── 팔 내리기 (집기 위치) ────────────────────────────────
void armDown() {
  dxl.setGoalPosition(ARM_ID_A, ARM_DOWN_DEG, UNIT_DEGREE);
  dxl.setGoalPosition(ARM_ID_B, ARM_DOWN_DEG, UNIT_DEGREE);
  delay(3000);
}

// ── 팔 올리기 (바스켓 투하 위치) ────────────────────────
void armUp() {
  dxl.setGoalPosition(ARM_ID_A, ARM_UP_DEG, UNIT_DEGREE);
  dxl.setGoalPosition(ARM_ID_B, ARM_UP_DEG, UNIT_DEGREE);
  delay(3000);
}

// ── 바스켓 닫기 ──────────────────────────────────────────
void containerClose() {
  dxl.setGoalPosition(CONT_ID_A, CONT_A_CLOSED, UNIT_DEGREE);
  dxl.setGoalPosition(CONT_ID_B, CONT_B_CLOSED, UNIT_DEGREE);
  delay(3000);
}

// ── 바스켓 열기 (합판 아래로 젖힘) ─────────────────────
void containerOpen() {
  dxl.setGoalPosition(CONT_ID_A, CONT_A_OPEN, UNIT_DEGREE);
  dxl.setGoalPosition(CONT_ID_B, CONT_B_OPEN, UNIT_DEGREE);
  delay(3000);
}
