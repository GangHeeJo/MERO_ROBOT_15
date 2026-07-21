/*
 * test_camera_servo.ino — 카메라 회전 모터(ID4) 180도 이동 후 원위치 복귀 테스트
 * ──────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 독립 스케치 — robot.ino(그리퍼+팔+컨테이너)와 별개 폴더, 이것만 단독 업로드됨.
 * 카메라·바퀴 없이 모터 하나만 동작 확인하는 용도.
 *
 * 동작: 현재 위치 읽기 → 그 위치에서 180도(raw +2048, 0~4095 순환) 이동 →
 *       2초 대기 → 원래 위치로 복귀
 */

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1
#define DXL_BAUD_RATE        1000000
#define DXL_PROTOCOL_VERSION 2.0f
#define CAM_ID                4

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  if (!dxl.ping(CAM_ID)) {
    Serial.println("[카메라모터] ping 실패 — ID/전원/배선 확인");
    return;
  }

  dxl.torqueOff(CAM_ID);
  dxl.setOperatingMode(CAM_ID, OP_POSITION);
  dxl.writeControlTableItem(PROFILE_VELOCITY, CAM_ID, 50);  // 속도 (낮을수록 느림)
  dxl.torqueOn(CAM_ID);

  int32_t start_pos = dxl.getPresentPosition(CAM_ID, UNIT_RAW);
  Serial.print("[카메라모터] 시작 위치(raw): ");
  Serial.println(start_pos);

  int32_t target_pos = (start_pos + 2048) % 4096;  // 180도 = raw 2048
  Serial.print("[카메라모터] 180도 이동 목표(raw): ");
  Serial.println(target_pos);
  dxl.setGoalPosition(CAM_ID, target_pos, UNIT_RAW);
  delay(2000);

  Serial.println("[카메라모터] 원위치로 복귀");
  dxl.setGoalPosition(CAM_ID, start_pos, UNIT_RAW);
  delay(2000);

  Serial.println("[카메라모터] 테스트 완료");
}

void loop() {
  // 1회성 테스트라 아무것도 안 함
}
