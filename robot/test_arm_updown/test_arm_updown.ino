/*
 * test_arm_updown.ino — 팔(ID2) 위/아래 이동 단독 테스트
 * ──────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 독립 스케치 — robot.ino(그리퍼+팔+컨테이너)와 별개 폴더, 이것만 단독 업로드됨.
 * 이미 실측 확정된 ARM_DOWN_RAW/ARM_UP_RAW 값으로 기본 Dynamixel 통신·전원이
 * 살아있는지부터 확인하는 용도 (카메라 서보 등 다른 문제와 분리해서 진단).
 *
 * 동작: 위로 이동 → 도착 확인 → 2초 대기 → 아래로 이동 → 도착 확인
 * 도착 확인은 고정 delay 대신 목표 근처(허용오차 이내)에 실제로 왔는지
 * 계속 읽어서 판단 — 속도 설정과 무관하게 확실히 기다림 (최대 5초 타임아웃)
 */

#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1
#define DXL_BAUD_RATE        1000000
#define DXL_PROTOCOL_VERSION 2.0f
#define ARM_ID                2

#define ARM_DOWN_RAW   1480
#define ARM_UP_RAW     2850
#define POS_TOLERANCE  20     // 이 이내로 들어오면 "도착"으로 인정
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

  if (!dxl.ping(ARM_ID)) {
    Serial.println("[팔] ping 실패 — ID/전원/배선 확인");
    return;
  }

  dxl.torqueOff(ARM_ID);
  dxl.setOperatingMode(ARM_ID, OP_POSITION);
  dxl.writeControlTableItem(PWM_LIMIT, ARM_ID, 885 * 80 / 100);  // 토크 80%
  dxl.torqueOn(ARM_ID);
  dxl.writeControlTableItem(PROFILE_VELOCITY, ARM_ID, 40);

  Serial.println("[팔] 위로 이동...");
  dxl.setGoalPosition(ARM_ID, ARM_UP_RAW, UNIT_RAW);
  bool up_ok = waitUntilReached(ARM_ID, ARM_UP_RAW);
  Serial.println(up_ok ? "[팔] 위 도착" : "[팔] 위 도착 타임아웃 — 확인 필요");

  delay(2000);

  Serial.println("[팔] 아래로 이동...");
  dxl.setGoalPosition(ARM_ID, ARM_DOWN_RAW, UNIT_RAW);
  bool down_ok = waitUntilReached(ARM_ID, ARM_DOWN_RAW);
  Serial.println(down_ok ? "[팔] 아래 도착" : "[팔] 아래 도착 타임아웃 — 확인 필요");

  Serial.println("[팔] 테스트 완료");
}

void loop() {
  // 1회성 테스트라 아무것도 안 함
}
