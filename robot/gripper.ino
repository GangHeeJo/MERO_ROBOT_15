/*
 * gripper.ino — 그리퍼 제어 (XL430 × 2, 자유도 2)
 * ──────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * Jetson → OpenRB: USB-C 시리얼 (/dev/ttyACM0, 115200)
 *
 * 구성:
 *   ID 1: 그리퍼 좌측 손가락
 *   ID 2: 그리퍼 우측 손가락 (좌측과 대칭 방향)
 *
 * 전원:
 *   XL430 동작 전압 12V → OpenRB 초록 단자에 12V 배터리 연결 필수
 *   (얇은 전선 사용 금지 — 과열/합선 위험)
 *
 * 사전 설정 (Dynamixel Wizard):
 *   - 모든 다이나믹셀 Baudrate 통일 (기본 57600)
 *   - 각 서보에 고유 ID 부여 (ID 1, 2)
 *
 * 필요 라이브러리:
 *   Dynamixel2Arduino (라이브러리 매니저에서 설치)
 */

#include <Dynamixel2Arduino.h>

// ──────────────────────────────────────
// OpenRB 전용 시리얼 설정
// ──────────────────────────────────────
// OpenRB는 DXL_SERIAL이 내장 Dynamixel 포트에 연결됨
#define DXL_SERIAL   Serial1
#define DEBUG_SERIAL Serial   // Jetson과 통신하는 USB 시리얼

const uint8_t DXL_DIR_PIN = -1;   // OpenRB는 방향핀 불필요 (내장 처리)

// ──────────────────────────────────────
// 설정값 — 실물 테스트 후 조정
// ──────────────────────────────────────
#define DXL_PROTOCOL_VERSION 2.0f
#define DXL_BAUD_RATE        57600

#define GRIPPER_ID_LEFT   1   // 좌측 손가락 ID
#define GRIPPER_ID_RIGHT  2   // 우측 손가락 ID

// TODO: 실물 테스트 후 실제 각도로 조정
#define FINGER_OPEN_DEG   150.0f   // 열린 상태 각도
#define FINGER_CLOSE_DEG   90.0f  // 닫힌 상태 각도 (물체 집기)

// 우측 손가락은 좌측과 대칭 → 각도 반전
// TODO: 실제 기구 구조에 따라 방향 조정
#define FINGER_RIGHT_OPEN_DEG  (180.0f - FINGER_OPEN_DEG)
#define FINGER_RIGHT_CLOSE_DEG (180.0f - FINGER_CLOSE_DEG)

// 집기 시 토크 제한 (%) — 물체 파손 방지
// 다각형 물체에 스펀지/수세미 부착 시 더 높여도 됨
#define GRIPPER_TORQUE_LIMIT_PCT 60

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 내부 함수 선언
static bool initDxl(uint8_t id);
static void setFingerPos(uint8_t id, float deg);

// ──────────────────────────────────────
// 초기화 (main.ino의 setup()에서 호출)
// ──────────────────────────────────────
void gripperSetup() {
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  bool ok1 = initDxl(GRIPPER_ID_LEFT);
  bool ok2 = initDxl(GRIPPER_ID_RIGHT);

  if (!ok1 || !ok2) {
    DEBUG_SERIAL.println("[그리퍼] ❌ 초기화 실패 — ID/전원/배선 확인");
    return;
  }

  // 시작 시 열린 상태로 초기화
  gripperOpen();
  DEBUG_SERIAL.println("[그리퍼] ✅ 초기화 완료 (열림 상태)");
}

// ──────────────────────────────────────
// 그리퍼 열기
// ──────────────────────────────────────
void gripperOpen() {
  setFingerPos(GRIPPER_ID_LEFT,  FINGER_OPEN_DEG);
  setFingerPos(GRIPPER_ID_RIGHT, FINGER_RIGHT_OPEN_DEG);
  delay(600);
  DEBUG_SERIAL.println("[그리퍼] 열림");
}

// ──────────────────────────────────────
// 그리퍼 닫기 (물체 집기)
// ──────────────────────────────────────
void gripperClose() {
  setFingerPos(GRIPPER_ID_LEFT,  FINGER_CLOSE_DEG);
  setFingerPos(GRIPPER_ID_RIGHT, FINGER_RIGHT_CLOSE_DEG);
  delay(600);
  DEBUG_SERIAL.println("[그리퍼] 닫힘");
}

// ──────────────────────────────────────
// 내부: 다이나믹셀 1개 초기화
// ──────────────────────────────────────
static bool initDxl(uint8_t id) {
  if (!dxl.ping(id)) {
    DEBUG_SERIAL.print("[그리퍼] ID ");
    DEBUG_SERIAL.print(id);
    DEBUG_SERIAL.println(" 응답 없음");
    return false;
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  // 토크 제한 (885 = 100% PWM 기준)
  dxl.writeControlTableItem(GOAL_PWM, id, 885 * GRIPPER_TORQUE_LIMIT_PCT / 100);
  dxl.torqueOn(id);
  return true;
}

// ──────────────────────────────────────
// 내부: 목표 각도로 이동
// ──────────────────────────────────────
static void setFingerPos(uint8_t id, float deg) {
  dxl.setGoalPosition(id, deg, UNIT_DEGREE);
}
