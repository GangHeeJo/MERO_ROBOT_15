/*
 * MERO_AI_ROBOT — ESP32 로봇 제어 코드
 * ──────────────────────────────────────
 * 역할: Jetson(비전팀)으로부터 JSON 수신 → 모터·그리퍼 제어
 *
 * 통신:
 *   Jetson → ESP32 : USB Serial (115200 baud)
 *   수신 JSON 형식 :
 *     {"objects":[...], "target":{"id":1,"cls":"d8","cx":342,"cy":218,"mx":12.3,"my":-5.1,"conf":0.91}}
 *   target == null 이면 대기
 *
 * 모터 명령 형식 (Waveshare General Driver):
 *   {"T":1, "L": 좌속도, "R": 우속도}
 *   속도 범위: -0.5 ~ +0.5  (0.5 = 100% PWM, 음수 = 역방향)
 *
 * 필요 라이브러리:
 *   - ArduinoJson (라이브러리 매니저에서 설치)
 *
 * 작성: 로봇팀
 * 연동: 비전팀 arducam_test.py
 */

#include <Arduino.h>
#include <ArduinoJson.h>

// ──────────────────────────────────────
// 설정값 — 하드웨어에 맞게 수정
// ──────────────────────────────────────

// Jetson과 통신하는 시리얼 포트
// USB로 연결 시 Serial, GPIO UART 사용 시 Serial2
#define JETSON_SERIAL Serial

// 그리퍼 서보 핀 번호 (TODO: 실제 핀으로 변경)
#define GRIPPER_PIN 13

// 물체에 도달한 것으로 판단하는 거리 임계값 (mm)
// mx, my 둘 다 이 값 이내면 "도착"으로 판단
#define ARRIVE_THRESHOLD_MM 20.0

// 이동 속도 (0.0 ~ 0.5)
#define MOVE_SPEED     0.3   // 기본 이동 속도
#define TURN_SPEED     0.25  // 회전 속도
#define SLOW_SPEED     0.15  // 근접 시 감속

// 그리퍼 서보 각도
#define GRIPPER_OPEN   90    // 열림 각도
#define GRIPPER_CLOSE  0     // 닫힘 각도 (TODO: 실제 값으로 조정)

// 목표 지점 좌표 mm (카메라 중심 기준) — TODO: 대회 환경 측정 후 수정
// cls 값에 따라 물체를 내려놓을 위치
struct DropZone { float x; float y; };
DropZone DROP_ZONES[] = {
  {200.0, 0.0},   // [0] d6
  {200.0, 50.0},  // [1] d8
  {200.0, 100.0}, // [2] d12
  {200.0, 150.0}, // [3] d20
  {-200.0, 0.0},  // [4] apple
  {-200.0, 50.0}, // [5] banana
  {-200.0, 100.0},// [6] orange
  {-200.0, 150.0},// [7] pineapple
};

// ──────────────────────────────────────
// 상태 머신
// ──────────────────────────────────────
enum State {
  IDLE,      // 대기: target == null
  APPROACH,  // 물체로 이동 중
  PICK,      // 물체 집기
  CARRY,     // 목표 지점으로 이동
  DROP,      // 내려놓기
  RETURN     // 초기 위치로 복귀
};

State currentState = IDLE;

// 현재 타겟 정보
struct Target {
  int   id;
  char  cls[16];
  float mx;   // mm (이미지 중심 기준)
  float my;
  float conf;
  bool  valid;
} currentTarget;

// ──────────────────────────────────────
// 모터 제어 함수 (Waveshare General Driver 명령)
// ──────────────────────────────────────

// 좌/우 바퀴 속도 직접 지정
// left, right: -0.5 ~ +0.5
void setMotor(float left, float right) {
  char cmd[64];
  snprintf(cmd, sizeof(cmd), "{\"T\":1,\"L\":%.2f,\"R\":%.2f}\n", left, right);
  // TODO: 모터 드라이버에 연결된 시리얼로 전송
  // 예) Serial2.print(cmd);
  // 현재는 디버그용으로 Serial 출력
  Serial.print("[모터] ");
  Serial.print(cmd);
}

void stopMotor() {
  setMotor(0.0, 0.0);
}

// 직진
void moveForward(float speed = MOVE_SPEED) {
  setMotor(speed, speed);
}

// 후진
void moveBackward(float speed = MOVE_SPEED) {
  setMotor(-speed, -speed);
}

// 좌회전 (제자리)
void turnLeft(float speed = TURN_SPEED) {
  setMotor(-speed, speed);
}

// 우회전 (제자리)
void turnRight(float speed = TURN_SPEED) {
  setMotor(speed, -speed);
}

// ──────────────────────────────────────
// 그리퍼 제어
// ──────────────────────────────────────
void gripperOpen() {
  // TODO: 서보 라이브러리 사용 시 아래 코드로 교체
  // myServo.write(GRIPPER_OPEN);
  Serial.println("[그리퍼] 열림");
  delay(500);
}

void gripperClose() {
  // TODO: 서보 라이브러리 사용 시 아래 코드로 교체
  // myServo.write(GRIPPER_CLOSE);
  Serial.println("[그리퍼] 닫힘");
  delay(500);
}

// ──────────────────────────────────────
// 클래스 이름 → 드롭존 인덱스 변환
// ──────────────────────────────────────
int getDropZoneIndex(const char* cls) {
  if (strcmp(cls, "d6")         == 0) return 0;
  if (strcmp(cls, "d8")         == 0) return 1;
  if (strcmp(cls, "d12")        == 0) return 2;
  if (strcmp(cls, "d20")        == 0) return 3;
  if (strcmp(cls, "apple")      == 0) return 4;
  if (strcmp(cls, "banana")     == 0) return 5;
  if (strcmp(cls, "orange")     == 0) return 6;
  if (strcmp(cls, "pineapple")  == 0) return 7;
  return -1;  // 알 수 없는 클래스
}

// ──────────────────────────────────────
// mx, my 기반 이동 제어
// mx: 양수 = 오른쪽, 음수 = 왼쪽
// my: 양수 = 아래쪽(전진), 음수 = 위쪽(후진)
// ──────────────────────────────────────
bool moveToward(float mx, float my) {
  float dist = sqrt(mx * mx + my * my);

  // 목표 도달
  if (dist < ARRIVE_THRESHOLD_MM) {
    stopMotor();
    return true;
  }

  float speed = (dist < 100.0) ? SLOW_SPEED : MOVE_SPEED;

  // 좌우 편차가 크면 먼저 회전
  if (abs(mx) > ARRIVE_THRESHOLD_MM) {
    if (mx > 0) turnRight(TURN_SPEED);
    else         turnLeft(TURN_SPEED);
  } else {
    // 전후 이동
    if (my > 0) moveForward(speed);
    else         moveBackward(speed);
  }

  return false;
}

// ──────────────────────────────────────
// JSON 파싱 — Jetson으로부터 수신
// ──────────────────────────────────────
bool parseTarget(const String& json) {
  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, json);

  if (err) {
    Serial.print("[오류] JSON 파싱 실패: ");
    Serial.println(err.c_str());
    return false;
  }

  JsonVariant target = doc["target"];

  // target == null → 탐지 없음
  if (target.isNull()) {
    currentTarget.valid = false;
    return true;
  }

  currentTarget.id   = target["id"]   | -1;
  currentTarget.mx   = target["mx"]   | 0.0f;
  currentTarget.my   = target["my"]   | 0.0f;
  currentTarget.conf = target["conf"] | 0.0f;
  strlcpy(currentTarget.cls, target["cls"] | "unknown", sizeof(currentTarget.cls));
  currentTarget.valid = true;

  return true;
}

// ──────────────────────────────────────
// 상태 머신 업데이트
// ──────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      // target 수신되면 집으러 이동
      if (currentTarget.valid) {
        Serial.print("[상태] IDLE → APPROACH | 타겟: ");
        Serial.println(currentTarget.cls);
        gripperOpen();  // 미리 그리퍼 열기
        currentState = APPROACH;
      }
      break;

    case APPROACH:
      // target이 사라지면 다시 대기
      if (!currentTarget.valid) {
        stopMotor();
        currentState = IDLE;
        break;
      }
      // 물체 위치로 이동, 도달하면 집기
      if (moveToward(currentTarget.mx, currentTarget.my)) {
        Serial.println("[상태] APPROACH → PICK");
        currentState = PICK;
      }
      break;

    case PICK:
      stopMotor();
      gripperClose();
      Serial.print("[상태] PICK → CARRY | cls: ");
      Serial.println(currentTarget.cls);
      currentState = CARRY;
      break;

    case CARRY: {
      // cls에 따라 목표 지점으로 이동
      int idx = getDropZoneIndex(currentTarget.cls);
      if (idx < 0) {
        Serial.println("[오류] 알 수 없는 클래스 → IDLE");
        currentState = IDLE;
        break;
      }
      DropZone dz = DROP_ZONES[idx];
      if (moveToward(dz.x, dz.y)) {
        Serial.println("[상태] CARRY → DROP");
        currentState = DROP;
      }
      break;
    }

    case DROP:
      stopMotor();
      gripperOpen();
      Serial.println("[상태] DROP → RETURN");
      delay(300);
      currentState = RETURN;
      break;

    case RETURN:
      // 초기 위치(0,0) 복귀
      if (moveToward(0.0, 0.0)) {
        Serial.println("[상태] RETURN → IDLE");
        currentTarget.valid = false;
        currentState = IDLE;
      }
      break;
  }
}

// ──────────────────────────────────────
// 초기화
// ──────────────────────────────────────
void setup() {
  JETSON_SERIAL.begin(115200);
  Serial.println("MERO 로봇 제어 시작");
  Serial.println("Jetson 연결 대기 중...");

  gripperOpen();  // 시작 시 그리퍼 열기
  currentTarget.valid = false;
}

// ──────────────────────────────────────
// 메인 루프
// ──────────────────────────────────────
void loop() {
  // Jetson으로부터 JSON 한 줄 수신
  if (JETSON_SERIAL.available()) {
    String line = JETSON_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseTarget(line);
    }
  }

  // 상태 머신 실행
  updateStateMachine();
}
