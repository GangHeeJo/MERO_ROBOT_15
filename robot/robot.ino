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
 *   {"cmd":"grip", "cls":"d8"}  ← 집기 → 팔 올려 컨테이너 투하 → 팔 내림 → 그리퍼 다시 닫기
 *   {"cmd":"dump"}              ← 컨테이너 열어서 쏟기 (도착 지점)
 *   {"cmd":"gripper_open"}      ← 그리퍼 미리 열기 (물체로 접근/전진하기 직전)
 *   {"cmd":"gripper_close"}     ← 그리퍼 대기 상태로 닫기 (접근 중 타겟 놓쳐 취소할 때)
 *   {"cmd":"idle"}              ← 대기
 *   {"cmd":"start"}             ← 경기 시작 — 전원 켤 때 시작 크기 규정으로 올려둔 팔을 내림
 *   {"cmd":"reset_fault"}       ← safety.ino가 fatal fault로 멈춘 뒤 사람이 확인하고 재개할 때
 *
 * OpenRB가 보내는 응답:
 *   {"status":"gripped"}         ← 집기+투하+복귀+재닫힘 완료 (Python → SEARCHING 복귀)
 *   {"status":"grip_failed"}     ← 전류 미달, 집기 실패 (Python → SEARCHING 복귀)
 *   {"status":"dumped"}          ← 컨테이너 열기 완료 (Python → SEARCHING 복귀)
 *   {"status":"gripper_opened"}  ← gripper_open 명령 처리 완료
 *   {"status":"gripper_closed"}  ← gripper_close 명령 처리 완료
 *   {"status":"started"}         ← start 명령으로 팔 내리기 완료
 *   {"status":"motor_fault"}     ← 과열/전압/충격/엔코더 등 자동복구 불가 (reset_fault 필요)
 *   {"status":"motor_recovered"} ← overload 자동복구 성공, 현재 동작은 중단하고 IDLE 복귀
 *   {"status":"motion_aborted"}  ← 위 두 경우 외 safety 개입으로 동작 중단
 *   {"status":"fault_reset"}     ← reset_fault 처리 완료
 *
 * 상태 머신:
 *   IDLE(그리퍼 닫힘) → (grip) → GRIPPING → (성공) → LIFTING → IDLE(그리퍼 다시 닫힘)
 *                                          → (실패) → IDLE(그리퍼 다시 닫힘)
 *        → (dump) → DUMPING → IDLE
 *        → (gripper_open/gripper_close) → IDLE (그리퍼만 열고/닫고 상태 변화 없음)
 *   safety.ino가 overload/hardware error를 감지하면 어느 상태에서든 즉시 IDLE로 복귀한다.
 *   IDLE 상태의 그리퍼 기본값은 "닫힘" — 대기 중 엉뚱한 물체가 벌어진 집게 안으로
 *   들어와 잡히는 걸 방지하기 위함. 실제로 집으러 갈 때만 gripper_open으로 미리 연다.
 *
 * 파일 구성:
 *   main.ino    — 시리얼 수신, 상태 머신 (이 파일)
 *   gripper.ino — 그리퍼 XL430 × 1 (ID 1, 랙-피니언)
 *   arm.ino     — 팔 XL430 × 2 (ID 2) + 컨테이너 XL430 × 2 (ID 3)
 *   safety.ino  — Dynamixel overload/hardware error 감시 및 자동 복구
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

// safety.ino 함수들
bool safeDelay(uint32_t wait_ms);
bool safetyPoll();
bool safetyHasRecoveredFlag();
bool safetyConsumeRecoveredFlag();
bool safetyHasFatalFault();
uint8_t safetyGetLastFaultId();
uint8_t safetyGetLastHardwareError();
void safetyResetFaultFlags();

// ── 상태 머신 ────────────────────────────────────────────
enum State {
  IDLE,      // 대기
  GRIPPING,  // 그리퍼 닫기
  LIFTING,   // 팔 올림 → 그리퍼 열기 → 팔 내림 (컨테이너 투하)
  DUMPING,   // 컨테이너 열기 (도착 지점)
};

State currentState = IDLE;
char  currentCls[16] = "";

// safety 개입으로 동작이 중단됐을 때 원인별로 Jetson에 상태를 알린다.
void sendSafetyAbortStatus(const char* where) {
  currentCls[0] = '\0';
  currentState = IDLE;

  if (safetyHasFatalFault()) {
    JETSON_SERIAL.print("{\"status\":\"motor_fault\",\"where\":\"");
    JETSON_SERIAL.print(where);
    JETSON_SERIAL.print("\",\"id\":");
    JETSON_SERIAL.print(safetyGetLastFaultId());
    JETSON_SERIAL.print(",\"hw_error\":");
    JETSON_SERIAL.print(safetyGetLastHardwareError());
    JETSON_SERIAL.println("}");
    return;
  }

  if (safetyConsumeRecoveredFlag()) {
    JETSON_SERIAL.print("{\"status\":\"motor_recovered\",\"where\":\"");
    JETSON_SERIAL.print(where);
    JETSON_SERIAL.print("\",\"id\":");
    JETSON_SERIAL.print(safetyGetLastFaultId());
    JETSON_SERIAL.print(",\"hw_error\":");
    JETSON_SERIAL.print(safetyGetLastHardwareError());
    JETSON_SERIAL.println("}");
    return;
  }

  JETSON_SERIAL.print("{\"status\":\"motion_aborted\",\"where\":\"");
  JETSON_SERIAL.print(where);
  JETSON_SERIAL.println("\"}");
}

// ── JSON 파싱 ─────────────────────────────────────────────
void parseCommand(const String& json) {
  StaticJsonDocument<256> doc;
  if (deserializeJson(doc, json)) return;

  const char* cmd = doc["cmd"] | "idle";

  // 복구 불가능 상태를 사람이 확인한 뒤 다시 명령을 받을 때 사용
  if (strcmp(cmd, "reset_fault") == 0) {
    safetyResetFaultFlags();
    currentState = IDLE;
    currentCls[0] = '\0';
    JETSON_SERIAL.println("{\"status\":\"fault_reset\"}");
    return;
  }

  if (safetyHasFatalFault()) {
    JETSON_SERIAL.print("{\"status\":\"motor_fault\",\"detail\":\"reset_required\",\"id\":");
    JETSON_SERIAL.print(safetyGetLastFaultId());
    JETSON_SERIAL.print(",\"hw_error\":");
    JETSON_SERIAL.print(safetyGetLastHardwareError());
    JETSON_SERIAL.println("}");
    return;
  }

  if (strcmp(cmd, "grip") == 0 && currentState == IDLE) {
    strncpy(currentCls, doc["cls"] | "unknown", sizeof(currentCls) - 1);
    currentCls[sizeof(currentCls) - 1] = '\0';
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

  // 경기 시작 — 시작 크기 규정 때문에 올려둔 팔을 정상 동작 위치(내림)로 내린다.
  if (strcmp(cmd, "start") == 0 && currentState == IDLE) {
    if (!armDown()) {
      sendSafetyAbortStatus("arm_down_on_start");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"started\"}");
    return;
  }

  // 물체 쪽으로 전진하기 직전 — 그리퍼를 미리 열어둔다.
  if (strcmp(cmd, "gripper_open") == 0 && currentState == IDLE) {
    if (!gripperOpen()) {
      sendSafetyAbortStatus("gripper_open_request");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"gripper_opened\"}");
    return;
  }

  // 접근 중 타겟을 놓쳐 grip을 포기할 때 — 열어뒀던 그리퍼를 대기 상태로 되돌린다.
  if (strcmp(cmd, "gripper_close") == 0 && currentState == IDLE) {
    if (!gripperCloseIdle()) {
      sendSafetyAbortStatus("gripper_close_request");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"gripper_closed\"}");
    return;
  }
}

// ── 상태 머신 ─────────────────────────────────────────────
void updateStateMachine() {
  switch (currentState) {

    case IDLE:
      break;

    case GRIPPING:
      // load 임계값 미달(집기 실패)이어도 팔은 그대로 올려서 시퀀스를 계속 진행한다.
      gripperClose();

      if (safetyHasFatalFault() || safetyHasRecoveredFlag()) {
        sendSafetyAbortStatus("gripping");
        break;
      }

      // 집은 뒤 팔 올리기 전에 조금 더 대기 (완전히 쥘 시간을 준다)
      if (!safeDelay(500)) {
        sendSafetyAbortStatus("grip_settle_wait");
        break;
      }

      currentState = LIFTING;
      JETSON_SERIAL.println("[OpenRB] 집기 시도 완료 → LIFTING");
      break;

    case LIFTING:
      if (!armUp()) {
        sendSafetyAbortStatus("arm_up");
        break;
      }
      if (!gripperOpen()) {
        sendSafetyAbortStatus("gripper_open_after_lift");
        break;
      }
      // 그리퍼 다 열리고 물체가 실제로 떨어질 시간을 더 준다.
      if (!safeDelay(800)) {
        sendSafetyAbortStatus("drop_wait");
        break;
      }
      if (!armDown()) {
        sendSafetyAbortStatus("arm_down");
        break;
      }
      // 대기 상태 안전을 위해 그리퍼를 다시 닫아둔다 (엉뚱한 물체가 끼어드는 것 방지).
      if (!gripperCloseIdle()) {
        sendSafetyAbortStatus("gripper_close_after_drop");
        break;
      }

      currentCls[0] = '\0';
      JETSON_SERIAL.println("{\"status\":\"gripped\"}");
      currentState = IDLE;
      break;

    case DUMPING:
      if (!containerOpen()) {
        sendSafetyAbortStatus("container_open");
        break;
      }
      if (!safeDelay(500)) {
        sendSafetyAbortStatus("dump_wait");
        break;
      }
      if (!containerClose()) {
        sendSafetyAbortStatus("container_close");
        break;
      }

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
  // IDLE 중에도 임의의 shutdown/overload가 생길 수 있으므로 계속 감시한다.
  if (safetyPoll()) {
    sendSafetyAbortStatus("background_poll");
    return;
  }

  if (JETSON_SERIAL.available()) {
    String line = JETSON_SERIAL.readStringUntil('\n');
    line.trim();
    if (line.length() > 0) {
      parseCommand(line);
    }
  }
  updateStateMachine();
}
