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
#define ARM_ID       2
#define CONT_ID      3

// ── 그리퍼 위치 (raw, 0~4095, 실측) ───────────────────────
#define FINGER_OPEN_RAW   2400
#define FINGER_CLOSE_RAW  1150
#define GRIP_LOAD_THRESHOLD 200   // PRESENT_LOAD 기준 (0.1% 단위, 200=20%)

// ── 팔 위치 (raw, 0~4095, 실측) ───────────────────────────
#define ARM_DOWN_RAW   1480
#define ARM_UP_RAW     2850

// ── 컨테이너 위치 (raw, 0~4095, 실측) ─────────────────────
#define CONT_OPEN_RAW    1000
#define CONT_CLOSED_RAW  2100

// ── 속도 (Profile Velocity, 낮을수록 느림) ───────────────
#define SPEED_ARM   40
#define SPEED_CONT  40
#define SPEED_GRIP  50

// ── 모터 초기화 ──────────────────────────────────────────
// DRIVE_MODE(reverse)는 Wizard에서 이미 설정됨 — 여기서 건드리지 않는다.
void initMotor(uint8_t id, int speed) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, speed);
  Serial.print("ID "); Serial.print(id); Serial.println(" 준비");
}

void moveTo(uint8_t id, int32_t raw, const char* label) {
  Serial.print(label); Serial.print(" -> "); Serial.println(raw);
  dxl.setGoalPosition(id, raw, UNIT_RAW);
}

// ── 그리퍼 닫기 (load 기반) ──────────────────────────────
bool gripClose() {
  moveTo(GRIPPER_ID, FINGER_CLOSE_RAW, "그리퍼 닫기");
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
  initMotor(GRIPPER_ID, SPEED_GRIP);
  initMotor(ARM_ID,     SPEED_ARM);
  initMotor(CONT_ID,    SPEED_CONT);
  delay(500);

  // ── [1] 그리퍼 열기 ────────────────────────────────────
  Serial.println("\n[1] 그리퍼 열기");
  moveTo(GRIPPER_ID, FINGER_OPEN_RAW, "그리퍼");
  delay(1000);

  // ── [2] 그리퍼 닫기 ────────────────────────────────────
  Serial.println("\n[2] 그리퍼 닫기 (집기)");
  bool gripped = gripClose();
  if (!gripped) Serial.println("  ※ 물체 없음 — 계속 진행");
  delay(500);

  // ── [3] 팔 올리기 ──────────────────────────────────────
  Serial.println("\n[3] 팔 올리기");
  moveTo(ARM_ID, ARM_UP_RAW, "팔");
  delay(3000);

  // ── [4] 그리퍼 열기 (투하) ─────────────────────────────
  Serial.println("\n[4] 그리퍼 열기 (컨테이너 투하)");
  moveTo(GRIPPER_ID, FINGER_OPEN_RAW, "그리퍼");
  delay(1000);

  // ── [5] 팔 내리기 ──────────────────────────────────────
  Serial.println("\n[5] 팔 내리기");
  moveTo(ARM_ID, ARM_DOWN_RAW, "팔");
  delay(3000);

  // ── [6] 컨테이너 열기 ──────────────────────────────────
  Serial.println("\n[6] 컨테이너 열기");
  moveTo(CONT_ID, CONT_OPEN_RAW, "컨테이너");
  delay(3000);

  // ── [7] 컨테이너 닫기 ──────────────────────────────────
  Serial.println("\n[7] 컨테이너 닫기");
  moveTo(CONT_ID, CONT_CLOSED_RAW, "컨테이너");
  delay(3000);

  Serial.println("\n=== 완료 ===");
}

void loop() {}
