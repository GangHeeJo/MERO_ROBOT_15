/*
 * arm.ino — 팔(XL430×2, ID 2) + 바스켓 힌지(XL430×2, ID 3) 제어
 * ──────────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성 (전부 XL430, 12V):
 *   ID 2: 팔 관절 모터 2개 (같은 ID — 한쪽은 Dynamixel Wizard에서 Drive Mode Reverse 설정)
 *   ID 3: 바스켓 힌지 모터 2개 (같은 ID — 한쪽은 Drive Mode Reverse 설정)
 *
 * Dynamixel Wizard 사전 설정:
 *   - 마주보고 장착된 쪽 모터(각 쌍의 한 개): Drive Mode bit0=1 (Reverse Mode)
 *   - 이후 같은 ID 설정하면 동일 명령으로 반대 방향 회전 → 같은 관절 동작
 *
 * dxl 인스턴스는 main.ino 에서 선언됨
 */

extern Dynamixel2Arduino dxl;

// ── 모터 ID ──────────────────────────────────────────────
#define ARM_ID       2
#define CONTAINER_ID 3

// ── 팔 위치 raw 0-4095 (실물 테스트 후 조정) ────────────
#define ARM_DOWN_RAW    0       // 집기 위치 (그리퍼 아래)  ⚠️ 임의값 — 실측 필요
#define ARM_UP_RAW      1706    // 투하 위치 (~150°)       ⚠️ 임의값 — 실측 필요

// ── 바스켓 위치 raw 0-4095 (실물 테스트 후 조정) ────────
#define CONTAINER_CLOSED_RAW   0      // 합판 수직 (닫힘)       ⚠️ 임의값 — 실측 필요
#define CONTAINER_OPEN_RAW     1024   // 합판 ~90° (열림)       ⚠️ 임의값 — 실측 필요

// ── 토크 제한 (%) ────────────────────────────────────────
#define ARM_TORQUE_PCT       80
#define CONTAINER_TORQUE_PCT 60

// ── 초기화 ───────────────────────────────────────────────
void armSetup() {
  uint8_t ids[2] = { ARM_ID, CONTAINER_ID };
  int     limits[2] = {
    885 * ARM_TORQUE_PCT / 100,
    885 * CONTAINER_TORQUE_PCT / 100,
  };
  for (int i = 0; i < 2; i++) {
    if (!dxl.ping(ids[i])) {
      Serial.print("[팔] 초기화 실패 ID="); Serial.println(ids[i]);
      continue;
    }
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_POSITION);
    dxl.writeControlTableItem(PWM_LIMIT, ids[i], limits[i]);
    dxl.torqueOn(ids[i]);
  }
  containerClose();
  armDown();
  Serial.println("[팔] 초기화 완료 (팔 내림 + 바스켓 닫힘)");
}

// ── 팔 내리기 (집기 위치) ────────────────────────────────
void armDown() {
  dxl.setGoalPosition(ARM_ID, ARM_DOWN_RAW, UNIT_RAW);
  delay(1200);
}

// ── 팔 올리기 (바스켓 투하 위치) ────────────────────────
void armUp() {
  dxl.setGoalPosition(ARM_ID, ARM_UP_RAW, UNIT_RAW);
  delay(1200);
}

// ── 바스켓 닫기 ──────────────────────────────────────────
void containerClose() {
  dxl.setGoalPosition(CONTAINER_ID, CONTAINER_CLOSED_RAW, UNIT_RAW);
  delay(800);
}

// ── 바스켓 열기 (합판 아래로 젖힘) ─────────────────────
void containerOpen() {
  dxl.setGoalPosition(CONTAINER_ID, CONTAINER_OPEN_RAW, UNIT_RAW);
  delay(1500);
}
