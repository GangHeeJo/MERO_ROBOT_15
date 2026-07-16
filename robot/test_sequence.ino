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

#define GRIPPER_DELTA   30    // 그리퍼: ±30° (살짝)
#define ARM_DELTA       100   // 팔: ±100 raw (~8.8°)
#define CONTAINER_DELTA 100   // 바스켓: ±100 raw (~8.8°)

void initMotor(uint8_t id, bool reverse) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.writeControlTableItem(DRIVE_MODE, id, reverse ? 1 : 0);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  Serial.print("ID "); Serial.print(id);
  Serial.print(reverse ? " (Reverse)" : " (Normal)");
  Serial.print(" 현재 위치(raw)=");
  Serial.println(dxl.getPresentPosition(id, UNIT_RAW));
}

void moveDelta(uint8_t id, int delta, int waitMs, const char* label) {
  int cur = (int)dxl.getPresentPosition(id, UNIT_RAW);
  int target = cur + delta;
  if (target < 0) target = 0;
  Serial.print(label);
  Serial.print(" ID="); Serial.print(id);
  Serial.print(" "); Serial.print(cur); Serial.print(" -> "); Serial.println(target);
  dxl.setGoalPosition(id, target, UNIT_RAW);
  delay(waitMs);
}

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  Serial.println("=== 모터 초기화 ===");
  initMotor(GRIPPER_ID, false);
  initMotor(ARM_ID_A,   false);
  initMotor(ARM_ID_B,   true);   // 반대쪽 모터 Reverse
  initMotor(CONT_ID_A,  false);
  initMotor(CONT_ID_B,  true);   // 반대쪽 모터 Reverse
  delay(1000);

  Serial.println("\n=== 시퀀스 시작 (조금씩만 이동) ===");

  Serial.println("\n[1] 그리퍼 열기 방향");
  moveDelta(GRIPPER_ID, +GRIPPER_DELTA, 1000, "그리퍼 열기");

  Serial.println("[2] 그리퍼 닫기 방향");
  moveDelta(GRIPPER_ID, -GRIPPER_DELTA, 1000, "그리퍼 닫기");

  Serial.println("[3] 팔 올리기 방향");
  moveDelta(ARM_ID_A, +ARM_DELTA, 50, "팔A 올리기");
  moveDelta(ARM_ID_B, +ARM_DELTA, 1200, "팔B 올리기");

  Serial.println("[4] 팔 내리기 방향");
  moveDelta(ARM_ID_A, -ARM_DELTA, 50, "팔A 내리기");
  moveDelta(ARM_ID_B, -ARM_DELTA, 1200, "팔B 내리기");

  Serial.println("[5] 바스켓 열기 방향");
  moveDelta(CONT_ID_A, +CONTAINER_DELTA, 50, "바스켓A 열기");
  moveDelta(CONT_ID_B, +CONTAINER_DELTA, 1200, "바스켓B 열기");

  Serial.println("[6] 바스켓 닫기 방향");
  moveDelta(CONT_ID_A, -CONTAINER_DELTA, 50, "바스켓A 닫기");
  moveDelta(CONT_ID_B, -CONTAINER_DELTA, 1200, "바스켓B 닫기");

  Serial.println("\n=== 완료. Serial Monitor에서 각 위치 확인 ===");
}

void loop() {}
