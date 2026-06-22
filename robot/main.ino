/*
 * MERO_AI_ROBOT — ESP32 메인 제어
 * ──────────────────────────────────
 * 역할: Jetson JSON 수신 → 상태 머신 실행
 *
 * 파일 구성:
 *   main.ino     — 시리얼 수신, 상태 머신 (이 파일)
 *   mobility.ino — 바퀴 이동 제어
 *   gripper.ino  — 그리퍼 서보 제어
 *
 * 필요 라이브러리:
 *   ArduinoJson (라이브러리 매니저에서 설치)
 */

#include <ArduinoJson.h>

// ──────────────────────────────────────
// Jetson 시리얼 포트
// USB 연결: Serial / GPIO UART 연결: Serial2
// ──────────────────────────────────────
#define JETSON_SERIAL Serial

// ──────────────────────────────────────
// 공유 데이터 구조체
// mobility.ino, gripper.ino에서도 사용됨
// ──────────────────────────────────────

// 상태 머신 상태
enum State {
  IDLE,      // 대기: target 없음
  APPROACH,  // 물체로 이동 중
  PICK,      // 물체 집기
  CARRY,     // 목표 드롭존으로 이동 중
  DROP,      // 내려놓기
  RETURN     // 초기 위치 복귀
};

// 현재 타겟 정보 (비전팀이 보낸 JSON에서 파싱)
struct Target {
  int   id;
  char  cls[16];
  float mx;     // 카메라 중심 기준 가로 거리 (mm), 양수=오른쪽
  float my;     // 카메라 중심 기준 세로 거리 (mm), 양수=앞쪽
  float conf;
  bool  valid;  // false면 탐지 없음
};

State  currentState  = IDLE;
Target currentTarget = {0, "", 0, 0, 0, false};

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

  // target == null → 탐지된 물체 없음
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
// 상태 머신
// ──────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      if (currentTarget.valid) {
        Serial.print("[IDLE→APPROACH] 타겟: ");
        Serial.println(currentTarget.cls);
        gripperOpen();
        currentState = APPROACH;
      }
      break;

    case APPROACH:
      if (!currentTarget.valid) {
        stopMotor();
        currentState = IDLE;
        break;
      }
      // 물체 위치로 이동, 도달하면 PICK
      if (moveToward(currentTarget.mx, currentTarget.my)) {
        Serial.println("[APPROACH→PICK]");
        currentState = PICK;
      }
      break;

    case PICK:
      stopMotor();
      gripperClose();
      Serial.print("[PICK→CARRY] cls: ");
      Serial.println(currentTarget.cls);
      currentState = CARRY;
      break;

    case CARRY: {
      // cls에 따라 드롭존 좌표 가져와서 이동
      float dx, dy;
      getDropZone(currentTarget.cls, dx, dy);
      if (moveToward(dx, dy)) {
        Serial.println("[CARRY→DROP]");
        currentState = DROP;
      }
      break;
    }

    case DROP:
      stopMotor();
      gripperOpen();
      Serial.println("[DROP→RETURN]");
      delay(300);
      currentState = RETURN;
      break;

    case RETURN:
      // 원점(0, 0)으로 복귀
      if (moveToward(0.0, 0.0)) {
        Serial.println("[RETURN→IDLE]");
        currentTarget.valid = false;
        currentState = IDLE;
      }
      break;
  }
}

// ──────────────────────────────────────
// 초기화 & 메인 루프
// ──────────────────────────────────────
void setup() {
  JETSON_SERIAL.begin(115200);
  mobilitySetup();   // mobility.ino 초기화
  gripperSetup();    // gripper.ino 초기화
  Serial.println("MERO 로봇 제어 시작. Jetson 대기 중...");
}

void loop() {
  // Jetson으로부터 JSON 한 줄 수신
  if (JETSON_SERIAL.available()) {
    String line = JETSON_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseTarget(line);
    }
  }

  updateStateMachine();
}
