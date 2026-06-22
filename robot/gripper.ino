/*
 * gripper.ino — 그리퍼(집게) 서보 제어
 * ──────────────────────────────────────
 * 서보모터로 집게 열기/닫기 제어
 */

#include <ESP32Servo.h>

// ──────────────────────────────────────
// 설정값 — 실제 하드웨어에 맞게 수정
// ──────────────────────────────────────

// TODO: 실제 서보 연결 핀으로 변경
#define GRIPPER_PIN 13

// TODO: 실제 서보 동작 범위 측정 후 각도 조정
#define GRIPPER_OPEN_DEG  90   // 집게 열린 상태 각도
#define GRIPPER_CLOSE_DEG  0   // 집게 닫힌 상태 각도

// 서보 동작 후 안정화 대기 시간 (ms)
#define GRIPPER_DELAY_MS 500

Servo gripperServo;

// ──────────────────────────────────────
// 초기화 (main.ino의 setup()에서 호출)
// ──────────────────────────────────────
void gripperSetup() {
  gripperServo.attach(GRIPPER_PIN);
  gripperServo.write(GRIPPER_OPEN_DEG);   // 시작 시 열린 상태
  delay(GRIPPER_DELAY_MS);
  Serial.println("[그리퍼] 초기화 완료 (열림 상태)");
}

// ──────────────────────────────────────
// 그리퍼 열기
// ──────────────────────────────────────
void gripperOpen() {
  gripperServo.write(GRIPPER_OPEN_DEG);
  delay(GRIPPER_DELAY_MS);
  Serial.println("[그리퍼] 열림");
}

// ──────────────────────────────────────
// 그리퍼 닫기 (물체 집기)
// ──────────────────────────────────────
void gripperClose() {
  gripperServo.write(GRIPPER_CLOSE_DEG);
  delay(GRIPPER_DELAY_MS);
  Serial.println("[그리퍼] 닫힘");
}
