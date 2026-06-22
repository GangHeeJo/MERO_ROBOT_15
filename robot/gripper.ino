/*
 * gripper.ino — 다이나믹셀 그리퍼 제어
 * ──────────────────────────────────────
 * 다이나믹셀 서보로 집게 열기/닫기 제어
 *
 * 지원 모델: X 시리즈 (XL430, XM430 등) — Protocol 2.0
 *   AX/MX 시리즈 사용 시: DXL_PROTOCOL_VERSION = 1.0 으로 변경
 *
 * 필요 라이브러리:
 *   Dynamixel2Arduino (라이브러리 매니저에서 설치)
 *
 * 필요 하드웨어:
 *   - 레벨 시프터 (ESP32 3.3V ↔ 다이나믹셀 5V TTL 변환)
 *   - 방향 제어 핀 (반이중 UART 전환용)
 *
 * 배선:
 *   ESP32 TX2 → 레벨시프터 → 다이나믹셀 DATA
 *   ESP32 RX2 ← 레벨시프터 ← 다이나믹셀 DATA
 *   ESP32 DIR_PIN → 레벨시프터 방향 제어
 */

#include <Dynamixel2Arduino.h>

// ──────────────────────────────────────
// 설정값 — 실제 하드웨어에 맞게 수정
// ──────────────────────────────────────

// 다이나믹셀 통신 시리얼 포트 (ESP32 Serial2 사용)
#define DXL_SERIAL   Serial2

// 방향 제어 핀 — 반이중 UART TX/RX 전환용
// TODO: 실제 배선 핀 번호로 변경
#define DXL_DIR_PIN  2

// 다이나믹셀 ID (Dynamixel Wizard로 설정한 값)
// TODO: 실제 설정된 ID로 변경
#define DXL_ID       1

// 프로토콜 버전
// X 시리즈: 2.0 / AX·MX 시리즈: 1.0
#define DXL_PROTOCOL_VERSION 2.0f

// 통신 속도 (다이나믹셀 기본값: 57600)
#define DXL_BAUD_RATE 57600

// 그리퍼 위치 (단위: 도, 0.0 ~ 360.0)
// TODO: 실물 테스트 후 실제 각도로 조정
#define GRIPPER_OPEN_DEG  150.0f   // 집게 열린 상태
#define GRIPPER_CLOSE_DEG  90.0f   // 집게 닫힌 상태 (물체 집기)

// 집기 시 토크 제한 (%) — 물체 파손 방지
// 100 = 최대 토크, 50 = 50%
#define GRIPPER_TORQUE_LIMIT 60

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// 내부 함수 선언
static void setPosition(float degrees);

// ──────────────────────────────────────
// 초기화 (main.ino의 setup()에서 호출)
// ──────────────────────────────────────
void gripperSetup() {
  // 다이나믹셀 시리얼 시작
  DXL_SERIAL.begin(DXL_BAUD_RATE);
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);

  // 다이나믹셀 연결 확인
  if (!dxl.ping(DXL_ID)) {
    Serial.println("[그리퍼] ❌ 다이나믹셀 응답 없음 — 배선·ID·전원 확인");
    return;
  }
  Serial.println("[그리퍼] ✅ 다이나믹셀 연결 확인");

  // 위치 제어 모드로 설정
  dxl.torqueOff(DXL_ID);
  dxl.setOperatingMode(DXL_ID, OP_POSITION);

  // 토크 제한 설정 (물체 파손 방지)
  dxl.writeControlTableItem(GOAL_PWM, DXL_ID, GRIPPER_TORQUE_LIMIT * 8);  // 885 = 100%

  // 토크 켜고 열린 상태로 초기화
  dxl.torqueOn(DXL_ID);
  setPosition(GRIPPER_OPEN_DEG);
  delay(1000);

  Serial.println("[그리퍼] 초기화 완료 (열림 상태)");
}

// ──────────────────────────────────────
// 그리퍼 열기
// ──────────────────────────────────────
void gripperOpen() {
  setPosition(GRIPPER_OPEN_DEG);
  delay(600);
  Serial.println("[그리퍼] 열림");
}

// ──────────────────────────────────────
// 그리퍼 닫기 (물체 집기)
// ──────────────────────────────────────
void gripperClose() {
  setPosition(GRIPPER_CLOSE_DEG);
  delay(600);
  Serial.println("[그리퍼] 닫힘");
}

// ──────────────────────────────────────
// 내부: 목표 각도로 이동
// ──────────────────────────────────────
static void setPosition(float degrees) {
  dxl.setGoalPosition(DXL_ID, degrees, UNIT_DEGREE);
}
