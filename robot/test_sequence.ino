/*
 * test_sequence.ino — 전체 동작 통합 테스트 (1회 자동 실행)
 *
 * 순서:
 *   1. 그리퍼 열기
 *   2. 그리퍼 닫기 (load 기반 집기 감지)
 *   3. 팔 올리기
 *   4. 그리퍼 열기 (컨테이너 투하)
 *   5. 팔 내리기
 *   6. 컨테이너 열기
 *   7. 컨테이너 닫기
 *
 * ⚠️ 필요 라이브러리: Dynamixel2Arduino
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN -1
#define BAUDRATE    1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ── ID ───────────────────────────────────────────────────
#define GRIPPER_ID   1
#define ARM_ID_A     9
#define ARM_ID_B     10
#define CONT_ID_A    5
#define CONT_ID_B    6

// ── 그리퍼 위치 (도) ──────────────────────────────────────
#define FINGER_OPEN_DEG   265.0f
#define FINGER_CLOSE_DEG  110.0f
#define GRIP_LOAD_THRESHOLD 200   // PRESENT_LOAD 기준 (0.1% 단위, 200=20%)

// ── 팔 위치 (도) ─────────────────────────────────────────
#define ARM_DOWN_DEG  132.25f
#define ARM_UP_DEG    245.0f

// ── 컨테이너 위치 (도, 실측) ─────────────────────────────
#define CONT_A_OPEN    94.75f
#define CONT_A_CLOSED  179.30f
#define CONT_B_OPEN    4.22f
#define CONT_B_CLOSED  88.95f

// ── 속도 (Profile Velocity, 낮을수록 느림) ───────────────
#define SPEED_ARM   30
#define SPEED_CONT  30
#define SPEED_GRIP  50

// ── 모터 초기화 ──────────────────────────────────────────
void initMotor(uint8_t id, bool reverse, int speed) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.writeControlTableItem(DRIVE_MODE, id, reverse ? 1 : 0);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, speed);
  Serial.print("ID "); Serial.print(id); Serial.println(" 준비");
}

void moveTo(uint8_t id, float deg, const char* label) {
  Serial.print(label); Serial.print(" -> "); Serial.print(deg); Serial.println("°");
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}

// ── 그리퍼 닫기 (load 기반) ──────────────────────────────
bool gripClose() {
  moveTo(GRIPPER_ID, FINGER_CLOSE_DEG, "그리퍼 닫기");
  delay(800);
  int32_t load = dxl.readControlTableItem(PRESENT_LOAD, GRIPPER_ID);
  int32_t abs_load = load < 0 ? -load : load;
  bool gripped = abs_load >= GRIP_LOAD_THRESHOLD;
  Serial.print("  load="); Serial.print(abs_load);
  Serial.println(gripped ? " → 집음" : " → 미스");
  return gripped;
}

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  // ── 초기화 ─────────────────────────────────────────────
  Serial.println("=== 초기화 ===");
  initMotor(GRIPPER_ID, false, SPEED_GRIP);
  initMotor(ARM_ID_A,   false, SPEED_ARM);
  initMotor(ARM_ID_B,   true,  SPEED_ARM);
  initMotor(CONT_ID_A,  false, SPEED_CONT);
  initMotor(CONT_ID_B,  false, SPEED_CONT);
  delay(500);

  // ── [1] 그리퍼 열기 ────────────────────────────────────
  Serial.println("\n[1] 그리퍼 열기");
  moveTo(GRIPPER_ID, FINGER_OPEN_DEG, "그리퍼");
  delay(1000);

  // ── [2] 그리퍼 닫기 ────────────────────────────────────
  Serial.println("\n[2] 그리퍼 닫기 (집기)");
  bool gripped = gripClose();
  if (!gripped) Serial.println("  ※ 물체 없음 — 계속 진행");
  delay(500);

  // ── [3] 팔 올리기 ──────────────────────────────────────
  Serial.println("\n[3] 팔 올리기");
  moveTo(ARM_ID_A, ARM_UP_DEG, "팔A");
  moveTo(ARM_ID_B, ARM_UP_DEG, "팔B");
  delay(3000);

  // ── [4] 그리퍼 열기 (투하) ─────────────────────────────
  Serial.println("\n[4] 그리퍼 열기 (컨테이너 투하)");
  moveTo(GRIPPER_ID, FINGER_OPEN_DEG, "그리퍼");
  delay(1000);

  // ── [5] 팔 내리기 ──────────────────────────────────────
  Serial.println("\n[5] 팔 내리기");
  moveTo(ARM_ID_A, ARM_DOWN_DEG, "팔A");
  moveTo(ARM_ID_B, ARM_DOWN_DEG, "팔B");
  delay(3000);

  // ── [6] 컨테이너 열기 ──────────────────────────────────
  Serial.println("\n[6] 컨테이너 열기");
  moveTo(CONT_ID_A, CONT_A_OPEN,   "컨테이너A");
  moveTo(CONT_ID_B, CONT_B_OPEN,   "컨테이너B");
  delay(3000);

  // ── [7] 컨테이너 닫기 ──────────────────────────────────
  Serial.println("\n[7] 컨테이너 닫기");
  moveTo(CONT_ID_A, CONT_A_CLOSED, "컨테이너A");
  moveTo(CONT_ID_B, CONT_B_CLOSED, "컨테이너B");
  delay(3000);

  Serial.println("\n=== 완료 ===");
}

void loop() {}
