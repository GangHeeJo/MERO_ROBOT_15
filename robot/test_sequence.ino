/*
 * test_sequence.ino — 업로드하면 자동으로 시퀀스 1회 실행
 *
 * 순서:
 *   1. 그리퍼 열기
 *   2. 그리퍼 닫기 (집기)
 *   3. 팔 올리기
 *   4. 그리퍼 열기 (바스켓 투하)
 *   5. 팔 내리기
 *   6. 바스켓 열기
 *   7. 바스켓 닫기
 *
 * ⚠️ ARM_UP_RAW, CONTAINER_OPEN_RAW 등 placeholder — 모터 움직임 보고 조정
 * ⚠️ 필요 라이브러리: Dynamixel2Arduino
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL   Serial1
#define DXL_DIR_PIN  -1
#define BAUDRATE     1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ── ID ───────────────────────────────────────────────────
#define GRIPPER_ID    1
#define ARM_ID        2
#define CONTAINER_ID  3

// ── 그리퍼 위치 (실측 완료) ──────────────────────────────
#define FINGER_OPEN_DEG   265.0f
#define FINGER_CLOSE_DEG  110.0f

// ── 팔 위치 raw (⚠️ 실측 필요, 움직임 보고 조정) ─────────
#define ARM_DOWN_RAW   0
#define ARM_UP_RAW     1706

// ── 바스켓 위치 raw (⚠️ 실측 필요) ──────────────────────
#define CONTAINER_CLOSED_RAW  0
#define CONTAINER_OPEN_RAW    1024

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  // ── 모터 초기화 ─────────────────────────────────────────
  uint8_t ids[3] = { GRIPPER_ID, ARM_ID, CONTAINER_ID };
  for (int i = 0; i < 3; i++) {
    if (!dxl.ping(ids[i])) {
      Serial.print("ping 실패 ID="); Serial.println(ids[i]);
      continue;
    }
    dxl.torqueOff(ids[i]);
    dxl.setOperatingMode(ids[i], OP_POSITION);
    dxl.torqueOn(ids[i]);
    Serial.print("ID "); Serial.print(ids[i]); Serial.println(" 준비 완료");
  }

  delay(1000);

  // ── 시퀀스 실행 ─────────────────────────────────────────
  Serial.println("\n=== 시퀀스 시작 ===");

  Serial.println("1. 그리퍼 열기");
  dxl.setGoalPosition(GRIPPER_ID, FINGER_OPEN_DEG, UNIT_DEGREE);
  delay(1200);

  Serial.println("2. 그리퍼 닫기 (집기)");
  dxl.setGoalPosition(GRIPPER_ID, FINGER_CLOSE_DEG, UNIT_DEGREE);
  delay(1200);

  Serial.println("3. 팔 올리기");
  dxl.setGoalPosition(ARM_ID, ARM_UP_RAW, UNIT_RAW);
  delay(1500);

  Serial.println("4. 그리퍼 열기 (바스켓 투하)");
  dxl.setGoalPosition(GRIPPER_ID, FINGER_OPEN_DEG, UNIT_DEGREE);
  delay(1000);

  Serial.println("5. 팔 내리기");
  dxl.setGoalPosition(ARM_ID, ARM_DOWN_RAW, UNIT_RAW);
  delay(1500);

  Serial.println("6. 바스켓 열기");
  dxl.setGoalPosition(CONTAINER_ID, CONTAINER_OPEN_RAW, UNIT_RAW);
  delay(1500);

  Serial.println("7. 바스켓 닫기");
  dxl.setGoalPosition(CONTAINER_ID, CONTAINER_CLOSED_RAW, UNIT_RAW);
  delay(1000);

  Serial.println("=== 시퀀스 완료 ===");
}

void loop() {
  // 반복 없음
}
