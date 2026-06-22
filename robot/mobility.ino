/*
 * mobility.ino — 바퀴 이동 제어
 * ──────────────────────────────────
 * Waveshare General Driver JSON 명령으로 모터 제어
 * 명령 형식: {"T":1, "L": 좌속도, "R": 우속도}
 * 속도 범위: -0.5 ~ +0.5 (0.5 = 100% PWM, 음수 = 역방향)
 */

// ──────────────────────────────────────
// 설정값 — 실제 환경에 맞게 조정
// ──────────────────────────────────────

// 모터 드라이버에 연결된 시리얼 포트
// TODO: 실제 연결 포트로 변경 (Serial2 등)
#define MOTOR_SERIAL Serial

// 이동 속도
#define MOVE_SPEED  0.3    // 기본 이동 속도
#define TURN_SPEED  0.25   // 회전 속도
#define SLOW_SPEED  0.15   // 목표 근접 시 감속 속도

// 도착 판정 거리 (mm)
// mx, my 모두 이 값 이내면 목표 도달로 판단
#define ARRIVE_THRESHOLD_MM 20.0

// ──────────────────────────────────────
// 드롭존 좌표 (카메라 중심 기준 mm)
// TODO: 대회 환경 실측 후 수정
// ──────────────────────────────────────
struct DropZone { float x; float y; };

// cls 이름 순서: d6, d8, d12, d20, apple, banana, orange, pineapple
const char* DROP_CLS[]   = {"d6", "d8", "d12", "d20", "apple", "banana", "orange", "pineapple"};
DropZone    DROP_ZONES[] = {
  {200.0,   0.0},   // d6
  {200.0,  50.0},   // d8
  {200.0, 100.0},   // d12
  {200.0, 150.0},   // d20
  {-200.0,  0.0},   // apple
  {-200.0, 50.0},   // banana
  {-200.0, 100.0},  // orange
  {-200.0, 150.0},  // pineapple
};
const int DROP_COUNT = 8;

// cls 이름으로 드롭존 좌표 반환
void getDropZone(const char* cls, float& outX, float& outY) {
  for (int i = 0; i < DROP_COUNT; i++) {
    if (strcmp(cls, DROP_CLS[i]) == 0) {
      outX = DROP_ZONES[i].x;
      outY = DROP_ZONES[i].y;
      return;
    }
  }
  // 알 수 없는 클래스면 원점 반환
  Serial.print("[경고] 알 수 없는 cls: ");
  Serial.println(cls);
  outX = 0.0;
  outY = 0.0;
}

// ──────────────────────────────────────
// 모터 제어 함수
// ──────────────────────────────────────

void mobilitySetup() {
  // TODO: MOTOR_SERIAL이 Serial2면 핀·속도 설정 필요
  // Serial2.begin(115200, SERIAL_8N1, RX_PIN, TX_PIN);
  Serial.println("[모빌리티] 초기화 완료");
}

// 좌/우 바퀴 속도 직접 지정 (-0.5 ~ +0.5)
void setMotor(float left, float right) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}\n", left, right);
  MOTOR_SERIAL.print(cmd);
}

void stopMotor() {
  setMotor(0.0, 0.0);
}

void moveForward(float speed = MOVE_SPEED) {
  setMotor(speed, speed);
}

void moveBackward(float speed = MOVE_SPEED) {
  setMotor(-speed, -speed);
}

// 제자리 좌회전 (반시계 방향)
void turnLeft(float speed = TURN_SPEED) {
  setMotor(-speed, speed);
}

// 제자리 우회전 (시계 방향)
void turnRight(float speed = TURN_SPEED) {
  setMotor(speed, -speed);
}

// ──────────────────────────────────────
// 목표 좌표로 이동 (mx, my 기반)
// 반환: true = 도달, false = 이동 중
//
// 좌표계:
//   mx 양수 → 오른쪽, 음수 → 왼쪽
//   my 양수 → 앞쪽(전진), 음수 → 뒤쪽(후진)
// ──────────────────────────────────────
bool moveToward(float mx, float my) {
  float dist = sqrt(mx * mx + my * my);

  // 목표 도달
  if (dist < ARRIVE_THRESHOLD_MM) {
    stopMotor();
    return true;
  }

  // 근접 시 감속
  float speed = (dist < 100.0) ? SLOW_SPEED : MOVE_SPEED;

  // 좌우 편차가 크면 먼저 방향 정렬 후 전진
  if (abs(mx) > ARRIVE_THRESHOLD_MM) {
    if (mx > 0) turnRight(TURN_SPEED);
    else         turnLeft(TURN_SPEED);
  } else {
    if (my > 0) moveForward(speed);
    else         moveBackward(speed);
  }

  return false;
}
