/*
 * gripper.ino — 그리퍼 제어 (XC330 × 2)
 * ──────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성:
 *   ID 1: 그리퍼 좌측 손가락 (XC330)
 *   ID 2: 그리퍼 우측 손가락 (XC330, 좌측과 대칭 방향)
 *
 * 전원:
 *   XC330 동작 전압 12V → UGV 내장 12V 배터리 → OpenRB 연결
 *
 * 사전 설정 (Dynamixel Wizard):
 *   - Baudrate: 57600
 *   - ID: 1 (좌), 2 (우)
 *
 * dxl 인스턴스는 main.ino 에서 정의됨 (extern 참조)
 */

// dxl 인스턴스는 main.ino 에서 선언됨
extern Dynamixel2Arduino dxl;

// ── 설정값 — 실물 테스트 후 조정 ────────────────────────
#define GRIPPER_ID_LEFT   1
#define GRIPPER_ID_RIGHT  2

// TODO: 실물 테스트 후 실제 각도로 조정
#define FINGER_OPEN_DEG    150.0f
#define FINGER_CLOSE_DEG    90.0f

// 우측 손가락은 좌측과 대칭 → 각도 반전
#define FINGER_RIGHT_OPEN_DEG  (180.0f - FINGER_OPEN_DEG)
#define FINGER_RIGHT_CLOSE_DEG (180.0f - FINGER_CLOSE_DEG)

// 파손 방지 토크 제한 (%)
#define GRIPPER_TORQUE_LIMIT_PCT 60

static bool initDxl(uint8_t id);
static void setFingerPos(uint8_t id, float deg);

// ── 초기화 ───────────────────────────────────────────────
void gripperSetup() {
  bool ok1 = initDxl(GRIPPER_ID_LEFT);
  bool ok2 = initDxl(GRIPPER_ID_RIGHT);

  if (!ok1 || !ok2) {
    Serial.println("[그리퍼] ❌ 초기화 실패 — ID/전원/배선 확인");
    return;
  }
  gripperOpen();
  Serial.println("[그리퍼] ✅ 초기화 완료 (열림 상태)");
}

// ── 그리퍼 열기 ──────────────────────────────────────────
void gripperOpen() {
  setFingerPos(GRIPPER_ID_LEFT,  FINGER_OPEN_DEG);
  setFingerPos(GRIPPER_ID_RIGHT, FINGER_RIGHT_OPEN_DEG);
  delay(600);
  Serial.println("[그리퍼] 열림");
}

// ── 그리퍼 닫기 ──────────────────────────────────────────
void gripperClose() {
  setFingerPos(GRIPPER_ID_LEFT,  FINGER_CLOSE_DEG);
  setFingerPos(GRIPPER_ID_RIGHT, FINGER_RIGHT_CLOSE_DEG);
  delay(600);
  Serial.println("[그리퍼] 닫힘");
}

// ── 내부: 다이나믹셀 초기화 ──────────────────────────────
static bool initDxl(uint8_t id) {
  if (!dxl.ping(id)) {
    Serial.print("[그리퍼] ID ");
    Serial.print(id);
    Serial.println(" 응답 없음");
    return false;
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(GOAL_PWM, id, 885 * GRIPPER_TORQUE_LIMIT_PCT / 100);
  dxl.torqueOn(id);
  return true;
}

// ── 내부: 목표 각도로 이동 ────────────────────────────────
static void setFingerPos(uint8_t id, float deg) {
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}
