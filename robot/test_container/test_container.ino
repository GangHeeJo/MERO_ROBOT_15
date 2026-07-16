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

#define CONT_ID  3

#define CONT_OPEN_RAW    1000
#define CONT_CLOSED_RAW  2100

#define SPEED  40

// DRIVE_MODE(reverse)는 Wizard에서 이미 설정됨 — 여기서 건드리지 않는다.
void initCont(uint8_t id) {
  if (!dxl.ping(id)) {
    Serial.print("ping 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(PROFILE_VELOCITY, id, SPEED);
  Serial.print("ID "); Serial.print(id);
  Serial.print("  현재="); Serial.println(dxl.getPresentPosition(id, UNIT_RAW));
}

void moveTo(uint8_t id, int32_t raw, const char* label) {
  Serial.print(label); Serial.print(" ID"); Serial.print(id);
  Serial.print(" -> "); Serial.println(raw);
  dxl.setGoalPosition(id, raw, UNIT_RAW);
}

void setup() {
  Serial.begin(115200);
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  Serial.println("=== 컨테이너 초기화 ===");
  initCont(CONT_ID);
  delay(1000);

  Serial.println("\n[1] 닫기");
  moveTo(CONT_ID, CONT_CLOSED_RAW, "컨테이너");
  delay(4000);

  Serial.println("\n[2] 열기");
  moveTo(CONT_ID, CONT_OPEN_RAW, "컨테이너");
  delay(4000);

  Serial.println("\n[3] 다시 닫기");
  moveTo(CONT_ID, CONT_CLOSED_RAW, "컨테이너");
  delay(4000);

  Serial.println("=== 완료 ===");
}

void loop() {}
