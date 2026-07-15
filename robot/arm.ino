/*
 * arm.ino — 팔(XL430×2) + 컨테이너 힌지(XL430×2) 제어
 * ───────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성 (전부 XL430, 12V):
 *   ID 2: 팔 왼쪽  — 팔 관절 좌측
 *   ID 3: 팔 오른쪽 — 팔 관절 우측, 마주보고 장착 → 위치 반전
 *   ID 4: 컨테이너 힌지 왼쪽
 *   ID 5: 컨테이너 힌지 오른쪽 — 마주보고 장착 → 위치 반전
 *
 * dxl 인스턴스는 main.ino 에서 선언됨
 */

extern Dynamixel2Arduino dxl;

// ── 모터 ID (Dynamixel Wizard로 배정 필요) ───────────────
#define ARM_L_ID        2
#define ARM_R_ID        3
#define CONTAINER_L_ID  4
#define CONTAINER_R_ID  5

// ── 팔 위치 raw 0-4095 (실물 테스트 후 조정) ────────────
#define ARM_DOWN_RAW    0       // 집기 위치 (그리퍼 아래)
#define ARM_UP_RAW      1706    // 투하 위치 (~150°, 컨테이너 위)

// ── 컨테이너 위치 raw 0-4095 (실물 테스트 후 조정) ──────
#define CONTAINER_CLOSED_RAW   0      // 합판 수직 (닫힘)
#define CONTAINER_OPEN_RAW     1024   // 합판 ~90° (아래로 젖힘)

// ── 토크 제한 (%) ────────────────────────────────────────
#define ARM_TORQUE_PCT       80
#define CONTAINER_TORQUE_PCT 60

// 반대쪽 모터 위치 (마주보고 장착 시 방향 반전)
static int mirrorRaw(int raw) { return 4095 - raw; }

// ── 초기화 ───────────────────────────────────────────────
void armSetup() {
  struct { uint8_t id; int pwmLimit; } motors[4] = {
    { ARM_L_ID,       885 * ARM_TORQUE_PCT / 100 },
    { ARM_R_ID,       885 * ARM_TORQUE_PCT / 100 },
    { CONTAINER_L_ID, 885 * CONTAINER_TORQUE_PCT / 100 },
    { CONTAINER_R_ID, 885 * CONTAINER_TORQUE_PCT / 100 },
  };
  for (auto& m : motors) {
    if (!dxl.ping(m.id)) {
      Serial.print("[팔] 초기화 실패 ID="); Serial.println(m.id);
      continue;
    }
    dxl.torqueOff(m.id);
    dxl.setOperatingMode(m.id, OP_POSITION);
    dxl.writeControlTableItem(GOAL_PWM, m.id, m.pwmLimit);
    dxl.torqueOn(m.id);
  }
  containerClose();
  armDown();
  Serial.println("[팔] 초기화 완료 (내림 + 컨테이너 닫힘)");
}

// ── 팔 내리기 (집기 위치) ────────────────────────────────
void armDown() {
  dxl.setGoalPosition(ARM_L_ID, ARM_DOWN_RAW,              UNIT_RAW);
  dxl.setGoalPosition(ARM_R_ID, mirrorRaw(ARM_DOWN_RAW),   UNIT_RAW);
  delay(1200);
}

// ── 팔 올리기 (컨테이너 투하 위치) ─────────────────────
void armUp() {
  dxl.setGoalPosition(ARM_L_ID, ARM_UP_RAW,                UNIT_RAW);
  dxl.setGoalPosition(ARM_R_ID, mirrorRaw(ARM_UP_RAW),     UNIT_RAW);
  delay(1200);
}

// ── 컨테이너 닫기 ────────────────────────────────────────
void containerClose() {
  dxl.setGoalPosition(CONTAINER_L_ID, CONTAINER_CLOSED_RAW,            UNIT_RAW);
  dxl.setGoalPosition(CONTAINER_R_ID, mirrorRaw(CONTAINER_CLOSED_RAW), UNIT_RAW);
  delay(800);
}

// ── 컨테이너 열기 (합판 아래로 젖힘) ───────────────────
void containerOpen() {
  dxl.setGoalPosition(CONTAINER_L_ID, CONTAINER_OPEN_RAW,            UNIT_RAW);
  dxl.setGoalPosition(CONTAINER_R_ID, mirrorRaw(CONTAINER_OPEN_RAW), UNIT_RAW);
  delay(1500);
}
