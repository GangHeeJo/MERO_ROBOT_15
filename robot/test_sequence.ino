/*
 * test_sequence.ino — 현재 위치에서 조금씩만 움직이는 테스트
 *
 * 현재 위치를 읽어서 ±DELTA만큼만 이동 → 실측값 모를 때 안전하게 확인용
 *
 * Serial Monitor(115200)에서 움직임 확인 후
 * 실제 값은 ARM_UP_RAW 등에 반영
 *
 * ⚠️ 필요 라이브러리: Dynamixel2Arduino
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
#define BAUDRATE     1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define GRIPPER_ID    1
#define ARM_ID_A      9
#define ARM_ID_B      10
#define CONT_ID_A     5
#define CONT_ID_B     6

// ── 팔 위치 (도) ─────────────────────────────────────────
#define ARM_DOWN_DEG  132.25f   // 집기 위치 (실측)
#define ARM_UP_DEG    245.0f    // 투하 위치 (실측)

#define ARM_SPEED     30        // Profile Velocity (낮을수록 느림, 0=최대)

void initArm(uint8_t id, bool reverse) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.writeControlTableItem(DRIVE_MODE, id, reverse ? 1 : 0);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, ARM_SPEED);  // torqueOn 후에 써야 적용됨
  Serial.print("ID "); Serial.print(id);
  Serial.print(reverse ? " (Reverse)" : " (Normal)");
  Serial.print("  현재="); Serial.print(dxl.getPresentPosition(id, UNIT_DEGREE));
  Serial.println("°");
}

void moveTo(uint8_t id, float deg, const char* label) {
  Serial.print(label); Serial.print(" ID="); Serial.print(id);
  Serial.print(" -> "); Serial.print(deg); Serial.println("°");
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  Serial.println("=== 팔 초기화 ===");
  initArm(ARM_ID_A, false);
  initArm(ARM_ID_B, true);
  delay(500);

  Serial.println("\n[1] 팔 내리기 → " + String(ARM_DOWN_DEG) + "°");
  moveTo(ARM_ID_A, ARM_DOWN_DEG, "팔A");
  moveTo(ARM_ID_B, ARM_DOWN_DEG, "팔B");
  delay(3000);

  Serial.println("\n[2] 팔 올리기 → " + String(ARM_UP_DEG) + "°");
  moveTo(ARM_ID_A, ARM_UP_DEG, "팔A");
  moveTo(ARM_ID_B, ARM_UP_DEG, "팔B");
  delay(3000);

  Serial.println("\n[3] 팔 다시 내리기 → " + String(ARM_DOWN_DEG) + "°");
  moveTo(ARM_ID_A, ARM_DOWN_DEG, "팔A");
  moveTo(ARM_ID_B, ARM_DOWN_DEG, "팔B");
  delay(3000);

  Serial.println("=== 완료 ===");
}

void loop() {}
