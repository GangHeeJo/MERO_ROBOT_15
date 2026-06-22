/*
 * MERO_AI_ROBOT — OpenRB 메인 제어
 * ──────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 역할: Jetson JSON 수신 → 팔·그리퍼 제어
 *
 * 연결:
 *   Jetson → OpenRB: USB-C (/dev/ttyACM0, 115200)
 *   OpenRB → Dynamixel: 내장 포트 (XL430 × 6, XL330 × 2)
 *
 * Jetson이 보내는 JSON:
 *   {"cmd":"pick", "cls":"d8", "mx":12.3, "my":-5.1}  ← 집기
 *   {"cmd":"drop", "cls":"d8"}                          ← 내려놓기
 *   {"cmd":"home"}                                       ← 홈
 *   {"cmd":"idle"}                                       ← 대기
 *
 * 파일 구성:
 *   main.ino     — 시리얼 수신, 상태 머신 (이 파일)
 *   arm.ino      — 팔 관절 제어 XL430 × 6 (구성 확정 후 작성)
 *   gripper.ino  — 그리퍼 손가락 XL330 × 2
 *
 * 바퀴 제어는 Jetson Python(arducam_test.py)이 ESP32에 직접 담당.
 * OpenRB는 팔·그리퍼만 제어함.
 *
 * 필요 라이브러리:
 *   ArduinoJson, Dynamixel2Arduino
 */

#include <ArduinoJson.h>

// ──────────────────────────────────────
// Jetson 시리얼 포트 (OpenRB USB-C)
// ──────────────────────────────────────
#define JETSON_SERIAL Serial

// ──────────────────────────────────────
// 공유 데이터 구조체
// gripper.ino, arm.ino에서도 사용됨
// ──────────────────────────────────────

// 상태 머신 상태 (팔·그리퍼만, 바퀴는 Python이 담당)
enum State {
  IDLE,    // 대기: pick 명령 대기
  PICK,    // 팔 내리기 + 그리퍼 닫기
  DROP,    // 드롭존으로 팔 이동 + 그리퍼 열기
  RETURN   // 팔 홈 복귀
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
bool parseCommand(const String& json) {
  StaticJsonDocument<256> doc;
  DeserializationError err = deserializeJson(doc, json);

  if (err) {
    Serial.print("[오류] JSON 파싱 실패: ");
    Serial.println(err.c_str());
    return false;
  }

  const char* cmd = doc["cmd"] | "idle";

  // idle / home → 타겟 무효화
  if (strcmp(cmd, "idle") == 0 || strcmp(cmd, "home") == 0) {
    currentTarget.valid = false;
    if (strcmp(cmd, "home") == 0) currentState = RETURN;
    return true;
  }

  // pick → 물체 집기 명령
  if (strcmp(cmd, "pick") == 0) {
    currentTarget.mx   = doc["mx"]  | 0.0f;
    currentTarget.my   = doc["my"]  | 0.0f;
    strlcpy(currentTarget.cls, doc["cls"] | "unknown", sizeof(currentTarget.cls));
    currentTarget.valid = true;
    return true;
  }

  return true;
}

// ──────────────────────────────────────
// 상태 머신 (팔·그리퍼 전용)
// 바퀴 제어는 Jetson Python이 담당.
// Jetson이 타겟 도달 판단 후 pick 명령을 보내면 여기서 팔 시퀀스 실행.
// ──────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      // pick 명령 수신 시 팔 시퀀스 시작
      if (currentTarget.valid) {
        Serial.print("[IDLE→PICK] 타겟: ");
        Serial.println(currentTarget.cls);
        currentState = PICK;
      }
      break;

    case PICK:
      // 팔을 물체 위치로 내리고 그리퍼 닫기
      armMoveToTarget(currentTarget.mx, currentTarget.my);
      gripperClose();
      Serial.print("[PICK→DROP] cls: ");
      Serial.println(currentTarget.cls);
      currentState = DROP;
      break;

    case DROP:
      // cls에 따른 드롭존으로 팔 이동 후 그리퍼 열기
      armMoveToDrop(currentTarget.cls);
      gripperOpen();
      Serial.println("[DROP→RETURN]");
      currentState = RETURN;
      break;

    case RETURN:
      // 팔 홈 포지션 복귀
      armHome();
      Serial.println("[RETURN→IDLE]");
      currentTarget.valid = false;
      currentState = IDLE;
      break;
  }
}

// ──────────────────────────────────────
// 초기화 & 메인 루프
// ──────────────────────────────────────
void setup() {
  JETSON_SERIAL.begin(115200);
  armSetup();        // arm.ino 초기화 (팔 관절 XL430 × 6)
  gripperSetup();    // gripper.ino 초기화 (집게 손가락 XL330 × 2)
  Serial.println("MERO OpenRB 시작. Jetson pick 명령 대기 중...");
}

void loop() {
  // Jetson으로부터 JSON 한 줄 수신
  if (JETSON_SERIAL.available()) {
    String line = JETSON_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseCommand(line);
    }
  }

  updateStateMachine();
}
