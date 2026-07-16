/*
 * test_diag.ino — ID별 DRIVE_MODE / PROFILE_VELOCITY 등 컨트롤테이블 읽기 전용 진단
 * (움직이지 않음, 확인용)
 */

#include <Dynamixel2Arduino.h>
using namespace ControlTableItem;

#define DXL_SERIAL  Serial1
#define DXL_DIR_PIN -1
#define BAUDRATE    1000000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void dump(uint8_t id) {
  Serial.print("=== ID "); Serial.print(id); Serial.println(" ===");
  if (!dxl.ping(id)) {
    Serial.println("  ping 실패");
    return;
  }
  int32_t driveMode = dxl.readControlTableItem(DRIVE_MODE, id);
  int32_t opMode     = dxl.readControlTableItem(OPERATING_MODE, id);
  int32_t profVel    = dxl.readControlTableItem(PROFILE_VELOCITY, id);
  int32_t profAcc    = dxl.readControlTableItem(PROFILE_ACCELERATION, id);
  int32_t torqueEn   = dxl.readControlTableItem(TORQUE_ENABLE, id);
  int32_t pos        = dxl.getPresentPosition(id, UNIT_RAW);

  Serial.print("  DRIVE_MODE="); Serial.print(driveMode);
  Serial.print(" (bit0 reverse="); Serial.print(driveMode & 0x01);
  Serial.print(", bit2 time-based-profile="); Serial.print((driveMode >> 2) & 0x01);
  Serial.println(")");
  Serial.print("  OPERATING_MODE="); Serial.println(opMode);
  Serial.print("  PROFILE_VELOCITY="); Serial.println(profVel);
  Serial.print("  PROFILE_ACCELERATION="); Serial.println(profAcc);
  Serial.print("  TORQUE_ENABLE="); Serial.println(torqueEn);
  Serial.print("  PRESENT_POSITION(raw)="); Serial.println(pos);
}

void setup() {
  Serial.begin(115200);
  while (!Serial && millis() < 3000) {}
  dxl.begin(BAUDRATE);
  dxl.setPortProtocolVersion(2.0f);

  delay(3000);
  Serial.println("=== 컨트롤테이블 진단 시작 ===");
  dump(1);
  dump(2);
  dump(3);
  Serial.println("=== 완료 ===");
}

void loop() {}
