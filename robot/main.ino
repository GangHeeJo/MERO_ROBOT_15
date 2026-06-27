/*
 * MERO_AI_ROBOT — OpenRB 메인 제어
 * ──────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 역할: Jetson JSON 수신 → 그리퍼 제어
 *
 * 연결:
 *   Jetson → OpenRB: USB-C (/dev/ttyACM1, 115200)
 *   OpenRB → Dynamixel: 내장 포트 (XC330 × 1 그리퍼)
 *
 * Jetson이 보내는 명령:
 *   {"cmd":"grip", "cls":"d8"}  ← 그리퍼 닫기
 *   {"cmd":"drop"}              ← 그리퍼 열기
 *   {"cmd":"idle"}              ← 대기
 *
 * OpenRB가 보내는 응답:
 *   {"status":"gripped"}     ← 집기 완료 (Python → GO_TO_STORAGE로 전환)
 *   {"status":"grip_failed"} ← 전류 미달, 집기 실패 (Python → SEARCHING 복귀)
 *   {"status":"done"}        ← 내려놓기 완료 (Python → SEARCHING 복귀)
 *
 * 상태 머신:
 *   IDLE → (grip) → GRIPPING → HOLDING → (drop) → DROPPING → IDLE
 *
 * 파일 구성:
 *   main.ino    — 시리얼 수신, 상태 머신 (이 파일)
 *   gripper.ino — 그리퍼 XC330 × 1 (랙-피니언)
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

State currentState = IDLE;
char  currentCls[16] = "";

// ── JSON 파싱 ─────────────────────────────────────────────
void parseCommand(const String& json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return;

  const char* cmd = doc["cmd"] | "idle";

  if (strcmp(cmd, "idle") == 0) {
    return;   // 상태 변경 없이 무시
  }

  if (strcmp(cmd, "grip") == 0 && currentState == IDLE) {
    strlcpy(currentCls, doc["cls"] | "unknown", sizeof(currentCls));
    currentState = GRIPPING;
    JETSON_SERIAL.print("[OpenRB] grip 수신 → GRIPPING: ");
    JETSON_SERIAL.println(currentCls);
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
      if (gripperClose()) {
        JETSON_SERIAL.println("{\"status\":\"gripped\"}");
        currentState = HOLDING;
      } else {
        gripperOpen();
        currentCls[0] = '\0';
        JETSON_SERIAL.println("{\"status\":\"grip_failed\"}");
        currentState = IDLE;
      }
      break;

    case HOLDING:
      break;   // parseCommand()에서 DROPPING으로 전환

    case DROPPING:
      gripperOpen();
      currentCls[0] = '\0';
      JETSON_SERIAL.println("{\"status\":\"done\"}");
      currentState = IDLE;
      break;
  }
}

// ── 초기화 & 메인 루프 ───────────────────────────────────
void setup() {
  JETSON_SERIAL.begin(115200);
  dxl.begin(DXL_BAUD_RATE);
  dxl.setPortProtocolVersion(DXL_PROTOCOL_VERSION);
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
