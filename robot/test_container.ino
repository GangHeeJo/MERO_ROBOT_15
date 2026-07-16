/*
 * test_container.ino — 컨테이너 단독 테스트
 *
 * Serial Monitor(115200) 열고 확인:
 *   현재 위치 → 닫기 → 열기 → 닫기
 *
 * ⚠️ 필요 라이브러리: Dynamixel2Arduino
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN -1
#define BAUDRATE    1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

#define CONT_ID_A  5
#define CONT_ID_B  6

#define CONT_A_OPEN    94.75f
#define CONT_A_CLOSED  179.30f
#define CONT_B_OPEN    4.22f
#define CONT_B_CLOSED  88.95f

#define SPEED  30

void initCont(uint8_t id) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.writeControlTableItem(DRIVE_MODE, id, 0);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, SPEED);
  Serial.print("ID "); Serial.print(id);
  Serial.print("  현재="); Serial.print(dxl.getPresentPosition(id, UNIT_DEGREE));
  Serial.println("°");
}

void moveTo(uint8_t id, float deg, const char* label) {
  Serial.print(label); Serial.print(" ID"); Serial.print(id);
  Serial.print(" -> "); Serial.print(deg); Serial.println("°");
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  Serial.println("=== 컨테이너 초기화 ===");
  initCont(CONT_ID_A);
  initCont(CONT_ID_B);
  delay(1000);

  Serial.println("\n[1] 닫기");
  moveTo(CONT_ID_A, CONT_A_CLOSED, "A");
  moveTo(CONT_ID_B, CONT_B_CLOSED, "B");
  delay(4000);

  Serial.println("\n[2] 열기");
  moveTo(CONT_ID_A, CONT_A_OPEN, "A");
  moveTo(CONT_ID_B, CONT_B_OPEN, "B");
  delay(4000);

  Serial.println("\n[3] 다시 닫기");
  moveTo(CONT_ID_A, CONT_A_CLOSED, "A");
  moveTo(CONT_ID_B, CONT_B_CLOSED, "B");
  delay(4000);

  Serial.println("=== 완료 ===");
}

void loop() {}
