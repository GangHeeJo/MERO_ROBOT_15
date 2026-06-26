/*
 * arm.ino — 팔 관절 제어 (XL430 × 6)
 * ──────────────────────────────────────
 * TODO: 팔 관절 구성 확정 후 구현
 *
 * 확정 필요 사항:
 *   1. 각 관절 ID 및 역할
 *      예시) ID 3: 베이스 회전, ID 4: 어깨, ID 5: 팔꿈치, ID 6: 손목 상하, ID 7: 손목 회전, ID 8: 집게 베이스
 *   2. 각 관절 동작 범위 (각도)
 *   3. armPickUp / armTransport / armDrop / armHome 각 자세의 관절 각도 값
 *
 * dxl 인스턴스는 main.ino 에서 정의됨 (extern 참조)
 *
 * 구현 순서:
 *   1. Dynamixel Wizard로 각 관절 ID 확인
 *   2. 수동으로 각 자세 잡아보며 각도 기록
 *   3. 아래 TODO 부분에 실제 각도 값 입력
 */

// dxl 인스턴스는 main.ino 에서 선언됨
extern Dynamixel2Arduino dxl;

// ── 팔 관절 ID ───────────────────────────────────────────
// TODO: Dynamixel Wizard로 확인 후 실제 ID로 변경
#define ARM_ID_BASE     3
#define ARM_ID_SHOULDER 4
#define ARM_ID_ELBOW    5
#define ARM_ID_WRIST    6
#define ARM_ID_WRIST_R  7
#define ARM_ID_GRIPPER_BASE 8

// 팔 관절 ID 목록 (초기화 루프용)
static const uint8_t ARM_IDS[] = {
  ARM_ID_BASE, ARM_ID_SHOULDER, ARM_ID_ELBOW,
  ARM_ID_WRIST, ARM_ID_WRIST_R, ARM_ID_GRIPPER_BASE
};
static const int ARM_ID_COUNT = sizeof(ARM_IDS) / sizeof(ARM_IDS[0]);

static void setArmPos(uint8_t id, float deg);

// ── 초기화 ───────────────────────────────────────────────
void armSetup() {
  for (int i = 0; i < ARM_ID_COUNT; i++) {
    uint8_t id = ARM_IDS[i];
    if (!dxl.ping(id)) {
      Serial.print("[팔] ID ");
      Serial.print(id);
      Serial.println(" 응답 없음 — 연결 확인 필요");
      continue;
    }
    dxl.torqueOff(id);
    dxl.setOperatingMode(id, OP_POSITION);
    dxl.torqueOn(id);
  }
  Serial.println("[팔] 초기화 완료 (arm.ino — 각도 TODO)");
}

// ── 집기 자세 ─────────────────────────────────────────────
// 물체가 ARRIVE_THRESHOLD_MM(30mm) 이내에 있을 때 호출됨
// mx, my: 물체까지 오프셋(mm). 접근 후 거의 0에 가까우므로 고정 자세 사용 가능
void armPickUp(float mx, float my) {
  // TODO: 집기 자세 각도 입력
  // 예시) 팔을 바닥 방향으로 뻗어 물체 위치에 그리퍼를 위치
  // setArmPos(ARM_ID_SHOULDER, 90.0f);
  // setArmPos(ARM_ID_ELBOW,    120.0f);
  // delay(1000);
  Serial.println("[팔] armPickUp 미구현");
}

// ── 이동 자세 ─────────────────────────────────────────────
// 물체를 집은 후 로봇이 보관함까지 주행하는 동안 유지할 자세
// 물체가 바닥에 끌리거나 균형을 잃지 않도록 팔을 접은 상태
void armTransport() {
  // TODO: 이동 자세 각도 입력
  // 예시) 팔을 몸 쪽으로 접어 무게중심 안정화
  // setArmPos(ARM_ID_SHOULDER, 45.0f);
  // setArmPos(ARM_ID_ELBOW,    60.0f);
  // delay(800);
  Serial.println("[팔] armTransport 미구현");
}

// ── 드롭 자세 ─────────────────────────────────────────────
// 로봇이 보관함 앞에 정지한 후 물체를 내려놓기 위한 자세
void armDrop() {
  // TODO: 드롭 자세 각도 입력
  // 예시) 보관함 방향으로 팔을 뻗어 그리퍼를 보관함 위에 위치
  // setArmPos(ARM_ID_BASE,     90.0f);   // 보관함 방향으로 회전
  // setArmPos(ARM_ID_SHOULDER, 70.0f);
  // setArmPos(ARM_ID_ELBOW,    100.0f);
  // delay(1000);
  Serial.println("[팔] armDrop 미구현");
}

// ── 홈 자세 ───────────────────────────────────────────────
// 팔의 기본 대기 자세 (로봇 주행 방해 최소화)
void armHome() {
  // TODO: 홈 자세 각도 입력
  // setArmPos(ARM_ID_BASE,     180.0f);
  // setArmPos(ARM_ID_SHOULDER, 180.0f);
  // setArmPos(ARM_ID_ELBOW,    180.0f);
  // delay(1000);
  Serial.println("[팔] armHome 미구현");
}

// ── 내부: 단일 관절 이동 ─────────────────────────────────
static void setArmPos(uint8_t id, float deg) {
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}
