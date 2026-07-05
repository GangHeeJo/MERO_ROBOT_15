/*
 * gripper.ino — 그리퍼 제어 (XL430 × 1)
 * ──────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성:
 *   ID 1: 그리퍼 모터 (XL430) — 단일 모터, 기계적 연동으로 양 손가락 구동
 *
 * 전원:
 *   XL430 동작 전압 12V → UGV 내장 12V 배터리 → OpenRB 연결
 *
 * 사전 설정 (Dynamixel Wizard):
 *   - Baudrate: 1000000
 *   - ID: 1
 *
 * dxl 인스턴스는 main.ino 에서 정의됨 (extern 참조)
 */

// dxl 인스턴스는 main.ino 에서 선언됨
extern Dynamixel2Arduino dxl;

// ── 설정값 — 실물 테스트 후 조정 ────────────────────────
#define GRIPPER_ID  1

// TODO: 실물 테스트 후 실제 각도로 조정
#define FINGER_OPEN_DEG   238.0f   // 실측: raw 2700
#define FINGER_CLOSE_DEG   90.0f   // 닫힘 방향 한계 (load threshold로 감지)

// 파손 방지 토크 제한 (%)
#define GRIPPER_TORQUE_LIMIT_PCT 60

// Load 기반 집기 감지 (실측: 물체 잡을 때 ~30%, 빈 손 ~0%)
// PRESENT_LOAD 단위: 0.1% (200 = 20%)
#define GRIP_LOAD_THRESHOLD 200

// ── 초기화 ───────────────────────────────────────────────
void gripperSetup() {
  if (!dxl.ping(GRIPPER_ID)) {
    Serial.println("[그리퍼] ❌ 초기화 실패 — ID/전원/배선 확인");
    return;
  }
  dxl.torqueOff(GRIPPER_ID);
  dxl.setOperatingMode(GRIPPER_ID, OP_POSITION);
  dxl.writeControlTableItem(GOAL_PWM, GRIPPER_ID, 885 * GRIPPER_TORQUE_LIMIT_PCT / 100);
  dxl.torqueOn(GRIPPER_ID);
  gripperOpen();
  Serial.println("[그리퍼] ✅ 초기화 완료 (열림 상태)");
}

// ── 그리퍼 열기 ──────────────────────────────────────────
void gripperOpen() {
  dxl.setGoalPosition(GRIPPER_ID, FINGER_OPEN_DEG, UNIT_DEGREE);
  delay(600);
  Serial.println("[그리퍼] 열림");
}

// ── 그리퍼 닫기 — 성공 여부 반환 ────────────────────────
// true: load 임계값 초과 → 물체 잡음
// false: load 낮음 → 빈 손으로 닫힘 (미스)
bool gripperClose() {
  dxl.setGoalPosition(GRIPPER_ID, FINGER_CLOSE_DEG, UNIT_DEGREE);
  delay(600);

  int32_t load = dxl.readControlTableItem(
    ControlTableItem::PRESENT_LOAD,
    GRIPPER_ID
  );

  int32_t abs_load = load < 0 ? -load : load;
  bool gripped = abs_load >= GRIP_LOAD_THRESHOLD;

  Serial.print("[그리퍼] 닫힘 — 부하: ");
  Serial.print(load);
  Serial.println(gripped ? " (잡음)" : " (미스, 임계값 미달)");

  return gripped;
}
