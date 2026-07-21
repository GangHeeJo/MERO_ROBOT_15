/*
 * test_camera_servo.ino — 카메라 회전 모터(ID4) 180도 이동 후 원위치 복귀 테스트
 * ──────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 독립 스케치 — robot.ino(그리퍼+팔+컨테이너)와 별개 폴더, 이것만 단독 업로드됨.
 * 카메라·바퀴 없이 모터 하나만 동작 확인하는 용도.
 *
 * 동작: 현재 위치 읽기 → 그 위치에서 180도(raw +2048, 0~4095 순환) 이동 →
 *       실제 도착 확인(폴링) → 원래 위치로 복귀 → 도착 확인
 * 고정 delay 대신 목표 근처(허용오차 이내)에 실제로 왔는지 계속 읽어서
 * 판단 — 속도 설정과 무관하게 확실히 기다림 (최대 5초 타임아웃)
 */

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1
#define DXL_BAUD_RATE        1000000
#define DXL_PROTOCOL_VERSION 2.0f
#define CAM_ID                4

#define POS_TOLERANCE   20    // 이 이내로 들어오면 "도착"으로 인정
#define MOVE_TIMEOUT_MS 5000

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 목표 위치까지 실제로 도착할 때까지 대기 (타임아웃 시 false)
bool waitUntilReached(uint8_t id, int32_t goal_raw) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < MOVE_TIMEOUT_MS) {
    int32_t cur = dxl.getPresentPosition(id, UNIT_RAW);
    int32_t err = cur - goal_raw;
    if (err < 0) err = -err;
    Serial.print("  현재 위치: "); Serial.println(cur);
    if (err <= POS_TOLERANCE) return true;
    delay(100);
  }
  return false;
}

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
  bool go_ok = waitUntilReached(CAM_ID, target_pos);
  Serial.println(go_ok ? "[카메라모터] 180도 도착" : "[카메라모터] 180도 도착 타임아웃 — 확인 필요");

  delay(1000);

  Serial.println("[카메라모터] 원위치로 복귀");
  dxl.setGoalPosition(CAM_ID, start_pos, UNIT_RAW);
  bool back_ok = waitUntilReached(CAM_ID, start_pos);
  Serial.println(back_ok ? "[카메라모터] 원위치 도착" : "[카메라모터] 원위치 도착 타임아웃 — 확인 필요");

  Serial.println("[카메라모터] 테스트 완료");
}

void loop() {
  // 1회성 테스트라 아무것도 안 함
}
