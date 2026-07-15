/*
 * MERO_AI_ROBOT — OpenRB 메인 제어
 * ──────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 역할: Jetson JSON 수신 → 그리퍼 + 팔 + 컨테이너 제어
 *
 * 연결:
 *   Jetson → OpenRB: USB-C (/dev/ttyACM1, 115200)
 *   OpenRB → Dynamixel: 내장 포트 (XL430 × 5)
 *     ID 1: 그리퍼  ID 2: 팔(양쪽, 한쪽 Reverse Mode)  ID 3: 바스켓 힌지(양쪽, 한쪽 Reverse Mode)
 *
 * Jetson이 보내는 명령:
 *   {"cmd":"grip", "cls":"d8"}  ← 집기 → 팔 올려 컨테이너 투하 → 팔 내림
 *   {"cmd":"dump"}              ← 컨테이너 열어서 쏟기 (도착 지점)
 *   {"cmd":"idle"}              ← 대기
 *
 * OpenRB가 보내는 응답:
 *   {"status":"gripped"}     ← 집기+투하+복귀 완료 (Python → SEARCHING 복귀)
 *   {"status":"grip_failed"} ← 전류 미달, 집기 실패 (Python → SEARCHING 복귀)
 *   {"status":"dumped"}      ← 컨테이너 열기 완료 (Python → SEARCHING 복귀)
 *
 * 상태 머신:
 *   IDLE → (grip) → GRIPPING → (성공) → LIFTING → IDLE
 *                             → (실패) → IDLE
 *        → (dump) → DUMPING → IDLE
 *
 * 파일 구성:
 *   main.ino    — 시리얼 수신, 상태 머신 (이 파일)
 *   gripper.ino — 그리퍼 XL430 × 1 (ID 1, 랙-피니언)
 *   arm.ino     — 팔 XL430 × 2 (ID 2) + 컨테이너 XL430 × 2 (ID 3)
 *
 * 필요 라이브러리: ArduinoJson, Dynamixel2Arduino
 */

#include <ArduinoJson.h>
#include <Dynamixel2Arduino.h>

using namespace ControlTableItem;

// ── Dynamixel 공유 인스턴스 ──────────────────────────────
#define DXL_SERIAL           Serial1
#define DXL_DIR_PIN          -1
#define DXL_BAUD_RATE        1000000
#define DXL_PROTOCOL_VERSION 2.0f

Dynamixel2Arduino dxl(DXL_SERIAL, DXL_DIR_PIN);

// ── Jetson 시리얼 ────────────────────────────────────────
#define JETSON_SERIAL Serial

// ── 상태 머신 ────────────────────────────────────────────
enum State {
  IDLE,      // 대기
  GRIPPING,  // 그리퍼 닫기
  LIFTING,   // 팔 올림 → 그리퍼 열기 → 팔 내림 (컨테이너 투하)
  DUMPING,   // 컨테이너 열기 (도착 지점)
};

State currentState = IDLE;
char  currentCls[16] = "";

// ── JSON 파싱 ─────────────────────────────────────────────
void parseCommand(const String& json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return;

  const char* cmd = doc["cmd"] | "idle";

  if (strcmp(cmd, "grip") == 0 && currentState == IDLE) {
    strlcpy(currentCls, doc["cls"] | "unknown", sizeof(currentCls));
    currentState = GRIPPING;
    JETSON_SERIAL.print("[OpenRB] grip → GRIPPING: ");
    JETSON_SERIAL.println(currentCls);
    return;
  }

  if (strcmp(cmd, "dump") == 0 && currentState == IDLE) {
    currentState = DUMPING;
    JETSON_SERIAL.println("[OpenRB] dump → DUMPING");
    return;
  }
}

// ── 상태 머신 ─────────────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      break;

    case GRIPPING:
      if (gripperClose()) {
        currentState = LIFTING;
        JETSON_SERIAL.println("[OpenRB] 집기 성공 → LIFTING");
      } else {
        gripperOpen();
        currentCls[0] = '\0';
        JETSON_SERIAL.println("{\"status\":\"grip_failed\"}");
        currentState = IDLE;
      }
      break;

    case LIFTING:
      armUp();
      gripperOpen();
      armDown();
      currentCls[0] = '\0';
      JETSON_SERIAL.println("{\"status\":\"gripped\"}");
      currentState = IDLE;
      break;

    case DUMPING:
      containerOpen();
      delay(500);
      containerClose();
      JETSON_SERIAL.println("{\"status\":\"dumped\"}");
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
  armSetup();
  JETSON_SERIAL.println("[OpenRB] 준비 완료. grip/dump 명령 대기 중...");
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
