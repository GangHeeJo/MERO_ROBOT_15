/*
 * test_step.ino — Serial Monitor에서 Enter로 한 단계씩 진행
 *
 * Serial Monitor(115200) 열고 Enter 누를 때마다 다음 동작 실행
 *
 * ⚠️ 필요 라이브러리: Dynamixel2Arduino
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN -1
#define BAUDRATE    1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define GRIPPER_ID   1
#define ARM_ID       2
#define CONT_ID      3

#define FINGER_OPEN_RAW   2400
#define FINGER_CLOSE_RAW  1150
#define GRIP_LOAD_THRESHOLD 200

#define ARM_DOWN_RAW   1480
#define ARM_UP_RAW     2850

#define CONT_OPEN_RAW    1000
#define CONT_CLOSED_RAW  2100

#define SPEED_GRIP  50
#define SPEED_ARM   40
#define SPEED_CONT  40

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
  Serial.print("  "); Serial.print(label); Serial.print(" -> "); Serial.println(raw);
  dxl.setGoalPosition(id, raw, UNIT_RAW);
}

void waitEnter(const char* msg) {
  Serial.println("");
  Serial.print("▶ "); Serial.print(msg); Serial.println("  (Enter로 실행)");
  while (true) {
    if (Serial.available()) {
      Serial.read();
      break;
    }
  }
  delay(100);
}

int step = 0;

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  Serial.println("=== 초기화 ===");
  initMotor(GRIPPER_ID, SPEED_GRIP);
  initMotor(ARM_ID,     SPEED_ARM);
  initMotor(CONT_ID,    SPEED_CONT);

  Serial.println("\n=== 단계별 테스트 시작 ===");
}

void loop() {
  switch (step) {
    case 0:
      waitEnter("[1] 그리퍼 열기");
      moveTo(GRIPPER_ID, FINGER_OPEN_RAW, "그리퍼");
      step++;
      break;

    case 1:
      waitEnter("[2] 그리퍼 닫기 (집기)");
      moveTo(GRIPPER_ID, FINGER_CLOSE_RAW, "그리퍼");
      delay(800);
      {
        int32_t load = dxl.readControlTableItem(PRESENT_LOAD, GRIPPER_ID);
        int32_t abs_load = load < 0 ? -load : load;
        Serial.print("  load="); Serial.print(abs_load);
        Serial.println(abs_load >= GRIP_LOAD_THRESHOLD ? " → 집음" : " → 미스");
      }
      step++;
      break;

    case 2:
      waitEnter("[3] 팔 올리기");
      moveTo(ARM_ID, ARM_UP_RAW, "팔");
      delay(4000);
      step++;
      break;

    case 3:
      waitEnter("[4] 그리퍼 열기 (투하)");
      moveTo(GRIPPER_ID, FINGER_OPEN_RAW, "그리퍼");
      delay(1500);
      step++;
      break;

    case 4:
      waitEnter("[5] 팔 내리기");
      moveTo(ARM_ID, ARM_DOWN_RAW, "팔");
      delay(4000);
      step++;
      break;

    case 5:
      waitEnter("[6] 컨테이너 열기");
      moveTo(CONT_ID, CONT_OPEN_RAW, "컨테이너");
      delay(4000);
      step++;
      break;

    case 6:
      waitEnter("[7] 컨테이너 닫기");
      moveTo(CONT_ID, CONT_CLOSED_RAW, "컨테이너");
      delay(4000);
      step++;
      break;

    case 7:
      Serial.println("\n=== 완료. 리셋하면 다시 시작 ===");
      while (true) {}
      break;
  }
}
