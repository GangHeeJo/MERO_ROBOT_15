/*
 * test_container.ino — 컨테이너(바스켓 힌지, ID3) 3초 열림/3초 닫힘 반복 테스트
 * ──────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 독립 스케치 — robot.ino(그리퍼+팔+컨테이너)와 별개 폴더, 이것만 단독 업로드됨.
 *
 * 동작: 열기 → 3초 대기 → 닫기 → 3초 대기 → (계속 반복)
 */

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1
#define DXL_BAUD_RATE        1000000
#define DXL_PROTOCOL_VERSION 2.0f
#define CONT_ID                3

#define CONT_CLOSED_RAW  2100
#define CONT_OPEN_RAW    1000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (!dxl.ping(CONT_ID)) {
    Serial.println("[컨테이너] ping 실패 — ID/전원/배선 확인");
    while (true) {}
  }

  dxl.torqueOff(CONT_ID);
  dxl.setOperatingMode(CONT_ID, OP_POSITION);
  dxl.writeControlTableItem(PWM_LIMIT, CONT_ID, 885 * 60 / 100);  // 토크 60%
  dxl.torqueOn(CONT_ID);
  dxl.writeControlTableItem(PROFILE_VELOCITY, CONT_ID, 40);

  Serial.println("[컨테이너] 3초 열림 / 3초 닫힘 반복 시작");
}

void loop() {
  Serial.println("[컨테이너] 열기");
  dxl.setGoalPosition(CONT_ID, CONT_OPEN_RAW, UNIT_RAW);
  delay(3000);

  Serial.println("[컨테이너] 닫기");
  dxl.setGoalPosition(CONT_ID, CONT_CLOSED_RAW, UNIT_RAW);
  delay(3000);
}
