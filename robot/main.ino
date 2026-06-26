/*
 * MERO_AI_ROBOT — OpenRB 메인 제어
 * ──────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 역할: Jetson JSON 수신 → 팔·그리퍼 제어
 *
 * 연결:
 *   Jetson → OpenRB: USB-C (/dev/ttyACM1, 115200)
 *   OpenRB → Dynamixel: 내장 포트 (XL430 × 6, XC330 × 2)
 *
 * Jetson이 보내는 명령:
 *   {"cmd":"grip", "cls":"d8", "mx":12.3, "my":-5.1}  ← 물체 집기
 *   {"cmd":"drop"}                                       ← 보관함에 내려놓기
 *   {"cmd":"idle"}                                       ← 대기
 *
 * OpenRB가 보내는 응답:
 *   {"status":"gripped"}  ← 집기 완료 (Python → GO_TO_STORAGE로 전환)
 *   {"status":"done"}     ← 내려놓기+홈 복귀 완료 (Python → SEARCHING 복귀)
 *
 * 상태 머신:
 *   IDLE → (grip) → GRIPPING → HOLDING → (drop) → DROPPING → RETURNING → IDLE
 *
 * 파일 구성:
 *   main.ino    — 시리얼 수신, 상태 머신 (이 파일)
 *   arm.ino     — 팔 관절 XL430 × 6 (구성 확정 후 구현)
 *   gripper.ino — 그리퍼 손가락 XC330 × 2
 *
 * 필요 라이브러리: ArduinoJson, Dynamixel2Arduino
 */

#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>

// ── Dynamixel 공유 인스턴스 ──────────────────────────────
// arm.ino, gripper.ino 에서 extern 으로 참조
#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1     // OpenRB는 방향핀 내장 처리
#define DXL_BAUD_RATE        57600
#define DXL_PROTOCOL_VERSION 2.0f

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ── Jetson 시리얼 ────────────────────────────────────────
#define JETSON_SERIAL Serial

// ── 상태 머신 ────────────────────────────────────────────
enum State {
  IDLE,      // 대기 — grip 명령 대기
  GRIPPING,  // 물체 집기 시퀀스 (팔 하강 + 그리퍼 닫기 + 팔 이동)
  HOLDING,   // 물체 들고 대기 — drop 명령 대기
  DROPPING,  // 보관함 내려놓기 시퀀스 (팔 이동 + 그리퍼 열기)
  RETURNING  // 팔 홈 복귀
};

// ── 현재 타겟 정보 ────────────────────────────────────────
struct Target {
  char  cls[16];
  float mx;     // 집기 위치 가로 오프셋 (mm, 양수=오른쪽)
  float my;     // 집기 위치 세로 오프셋 (mm, 양수=앞쪽)
  bool  valid;
};

State  currentState  = IDLE;
Target currentTarget = {"", 0, 0, false};

// ── JSON 파싱 ─────────────────────────────────────────────
void parseCommand(const String& json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return;

  const char* cmd = doc["cmd"] | "idle";

  if (strcmp(cmd, "idle") == 0) {
    return;   // 상태 변경 없이 무시
  }

  if (strcmp(cmd, "grip") == 0 && currentState == IDLE) {
    strlcpy(currentTarget.cls, doc["cls"] | "unknown", sizeof(currentTarget.cls));
    currentTarget.mx    = doc["mx"] | 0.0f;
    currentTarget.my    = doc["my"] | 0.0f;
    currentTarget.valid = true;
    currentState        = GRIPPING;
    JETSON_SERIAL.print("[OpenRB] grip 수신 → GRIPPING: ");
    JETSON_SERIAL.println(currentTarget.cls);
    return;
  }

  if (strcmp(cmd, "drop") == 0 && currentState == HOLDING) {
    currentState = DROPPING;
    JETSON_SERIAL.println("[OpenRB] drop 수신 → DROPPING");
    return;
  }
}

// ── 상태 머신 ─────────────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      break;   // parseCommand()에서 GRIPPING으로 전환

    case GRIPPING:
      // 1. 팔을 집기 위치로 하강
      armPickUp(currentTarget.mx, currentTarget.my);
      // 2. 그리퍼 닫기
      gripperClose();
      // 3. 이동 자세로 팔 이동 (물체를 든 채 로봇이 주행할 수 있는 자세)
      armTransport();
      // 집기 완료 → Jetson에 신호 → Python GO_TO_STORAGE 전환
      JETSON_SERIAL.println("{\"status\":\"gripped\"}");
      currentState = HOLDING;
      break;

    case HOLDING:
      break;   // parseCommand()에서 DROPPING으로 전환

    case DROPPING:
      // 1. 보관함 드롭 자세로 팔 이동
      armDrop();
      // 2. 그리퍼 열기
      gripperOpen();
      currentState = RETURNING;
      break;

    case RETURNING:
      // 팔 홈 복귀
      armHome();
      currentTarget.valid = false;
      currentState        = IDLE;
      // 내려놓기+복귀 완료 → Jetson에 신호 → Python SEARCHING 복귀
      JETSON_SERIAL.println("{\"status\":\"done\"}");
      break;
  }
}

// ── 초기화 & 메인 루프 ───────────────────────────────────
void setup() {
  JETSON_SERIAL.begin(115200);
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
  armSetup();
  gripperSetup();
  JETSON_SERIAL.println("[OpenRB] 준비 완료. grip 명령 대기 중...");
}

void loop() {
  if (JETSON_SERIAL.available()) {
    String line = JETSON_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseCommand(line);
    }
  }
  updateStateMachine();
}
