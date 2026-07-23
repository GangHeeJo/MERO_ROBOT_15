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
 *   {"cmd":"grip", "cls":"d8"}  ← 집기 → 팔을 중간 위치(ARM_CHECK_RAW)까지만 올려 정지
 *                                  → at_check_pos 전송 → Jetson의 confirm_grip/reject_grip 대기
 *   {"cmd":"confirm_grip"}      ← (CHECKING 중만 유효) 카메라로 그립 확인됨 → 팔 마저 올려 투하 계속
 *   {"cmd":"reject_grip"}       ← (CHECKING 중만 유효) 그립 안 됨 → 그리퍼 열고 팔 내려 원위치
 *   {"cmd":"dump"}              ← 컨테이너 열어서 쏟기 (도착 지점)
 *   {"cmd":"gripper_open"}      ← 그리퍼 미리 열기 (물체로 접근/전진하기 직전)
 *   {"cmd":"gripper_close"}     ← 그리퍼 대기 상태로 닫기 (접근 중 타겟 놓쳐 취소할 때)
 *   {"cmd":"basket_open"}       ← 임시 디버깅용 — 바스켓을 열고 계속 열린 상태 유지 (자동으로 안 닫힘)
 *   {"cmd":"basket_close"}      ← 임시 디버깅용 — 바스켓을 닫힘 위치로 복귀
 *   {"cmd":"idle"}              ← 대기
 *   {"cmd":"start"}             ← 경기 시작 — 전원 켤 때 시작 크기 규정으로 올려둔 팔을 내림 + 카메라 정면 리셋
 *   {"cmd":"cam_backward"}      ← 보관함 이동 전 카메라를 뒤로 180도 회전 (후방을 봄)
 *   {"cmd":"cam_forward"}       ← 보관함 왕복 끝나고 카메라를 다시 정면으로 회전
 *   {"cmd":"reset_fault"}       ← safety.ino가 fatal fault로 멈춘 뒤 사람이 확인하고 재개할 때
 *
 * OpenRB가 보내는 응답:
 *   {"status":"at_check_pos","cls":"d8"} ← 팔이 중간 위치에서 정지, 카메라 확인 대기 중
 *   {"status":"gripped"}         ← 집기+투하+복귀+재닫힘 완료 (Python → SEARCHING 복귀)
 *   {"status":"grip_failed"}     ← 그립 확인 실패(reject_grip 또는 확인 타임아웃) → 그리퍼 열고 팔 내려 원위치 (Python → SEARCHING 복귀)
 *   {"status":"dumped"}          ← 컨테이너 열기 완료 (Python → SEARCHING 복귀)
 *   {"status":"gripper_opened"}  ← gripper_open 명령 처리 완료
 *   {"status":"gripper_closed"}  ← gripper_close 명령 처리 완료
 *   {"status":"basket_opened"}   ← basket_open 명령 처리 완료
 *   {"status":"basket_closed"}   ← basket_close 명령 처리 완료
 *   {"status":"started"}         ← start 명령으로 팔 내리기 완료
 *   {"status":"cam_backward_done"} ← 카메라 후방 회전 완료
 *   {"status":"cam_forward_done"}  ← 카메라 정면 회전 완료
 *   {"status":"motor_fault"}     ← 과열/전압/충격/엔코더 등 자동복구 불가 (reset_fault 필요)
 *   {"status":"motor_recovered"} ← overload 자동복구 성공, 현재 동작은 중단하고 IDLE 복귀
 *   {"status":"motion_aborted"}  ← 위 두 경우 외 safety 개입으로 동작 중단
 *   {"status":"fault_reset"}     ← reset_fault 처리 완료
 *
 * 상태 머신:
 *   IDLE(그리퍼 닫힘) → (grip) → GRIPPING → (집기 시도 완료) → CHECKING(팔 중간 정지, at_check_pos 전송)
 *        CHECKING → (confirm_grip) → LIFTING → IDLE(그리퍼 다시 닫힘, gripped 전송)
 *        CHECKING → (reject_grip 또는 확인 타임아웃) → IDLE(그리퍼 열었다 다시 닫힘, grip_failed 전송)
 *        → (dump) → DUMPING → IDLE
 *        → (gripper_open/gripper_close) → IDLE (그리퍼만 열고/닫고 상태 변화 없음)
 *        → (basket_open/basket_close) → IDLE (바스켓만 열고/닫고 상태 변화 없음, 임시 디버깅용)
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
  CHECKING,  // 팔 중간 정지 — 카메라로 그립 확인, Jetson의 confirm_grip/reject_grip 대기
  LIFTING,   // 팔 올림 → 그리퍼 열기 → 팔 내림 (컨테이너 투하)
  DUMPING,   // 컨테이너 열기 (도착 지점)
};

State currentState = IDLE;
char  currentCls[16] = "";
uint32_t checkStartMs = 0;

// CHECKING 중 Jetson 응답이 아예 안 올 때(시리얼 문제 등) 팔이 중간 위치에
// 무한정 멈춰있지 않도록 하는 안전망. Python 쪽 GRIP_CHECK 판단(수 초)보다
// 넉넉히 길게 잡아서, 정상 동작에서는 이 타임아웃보다 항상 먼저 confirm/reject가 온다.
#define CHECK_TIMEOUT_MS 8000

// CHECKING에서 그립 실패로 판단됐을 때(reject_grip 또는 타임아웃) 공통 정리 동작 —
// 그리퍼를 열면서 동시에 팔을 내리고(gripperOpenWithArmDown) → 그리퍼 대기 상태로 재닫힘.
// 실패 시 sendSafetyAbortStatus를 호출하고 false를 반환하며, 호출부는 그 이상 아무 것도
// 더 하지 않아야 한다.
bool doRejectGrip(const char* ctx) {
  String w;
  w = String(ctx) + "_open_down";
  if (!gripperOpenWithArmDown()) { sendSafetyAbortStatus(w.c_str()); return false; }
  w = String(ctx) + "_close";
  if (!gripperCloseIdle()) { sendSafetyAbortStatus(w.c_str()); return false; }
  return true;
}

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

  // 그립 중간 확인(카메라) 결과 — 확인됨 → 팔 마저 올리는 LIFTING으로 이어간다.
  if (strcmp(cmd, "confirm_grip") == 0 && currentState == CHECKING) {
    currentState = LIFTING;
    JETSON_SERIAL.println("[OpenRB] confirm_grip → LIFTING 계속");
    return;
  }

  // 그립 중간 확인(카메라) 결과 — 실패로 판단됨 → 그리퍼 열고 팔 내려 원위치.
  if (strcmp(cmd, "reject_grip") == 0 && currentState == CHECKING) {
    if (!doRejectGrip("reject_grip_cmd")) return;
    currentCls[0] = '\0';
    currentState = IDLE;
    JETSON_SERIAL.println("{\"status\":\"grip_failed\"}");
    return;
  }

  if (strcmp(cmd, "dump") == 0 && currentState == IDLE) {
    currentState = DUMPING;
    JETSON_SERIAL.println("[OpenRB] dump → DUMPING");
    return;
  }

  // 경기 시작 — 시작 크기 규정 때문에 올려둔 팔을 정상 동작 위치(내림)로 내린다.
  // 카메라도 정면으로 리셋 — 이전 테스트/경기에서 cam_backward로 후방을 보고 있던
  // 상태가 그대로 남아있을 수 있어, 경기 시작 시점에 확실히 정면으로 되돌려둔다.
  // 카메라 실패는 팔 내리기(경기 규정상 필수)를 막을 이유가 아니라서 abort하지 않고 로그만 남긴다.
  if (strcmp(cmd, "start") == 0 && currentState == IDLE) {
    if (!armDown()) {
      sendSafetyAbortStatus("arm_down_on_start");
      return;
    }
    if (!camForward()) {
      JETSON_SERIAL.println("[OpenRB] 경고: 카메라 정면 복귀 실패 (start는 계속 진행)");
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

  // 디버깅용 — 팔을 시작 크기 규정 위치(올림)로 수동 복귀
  if (strcmp(cmd, "arm_up") == 0 && currentState == IDLE) {
    if (!armUp()) {
      sendSafetyAbortStatus("arm_up_request");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"arm_up_done\"}");
    return;
  }

  // 임시 디버깅용 — 바스켓을 열고 그대로 유지 (dump와 달리 자동으로 안 닫힘, 수동 확인/비우기용)
  if (strcmp(cmd, "basket_open") == 0 && currentState == IDLE) {
    if (!containerOpen()) {
      sendSafetyAbortStatus("basket_open_request");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"basket_opened\"}");
    return;
  }

  // 임시 디버깅용 — 바스켓을 닫힘 위치로 복귀
  if (strcmp(cmd, "basket_close") == 0 && currentState == IDLE) {
    if (!containerClose()) {
      sendSafetyAbortStatus("basket_close_request");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"basket_closed\"}");
    return;
  }

  // 보관함으로 이동하기 직전 — 카메라를 뒤로 180도 돌려 후방을 보게 한다.
  // ID4(카메라)는 그리퍼/팔/컨테이너와 완전히 별개 모터라, 그쪽이 GRIPPING/LIFTING/
  // DUMPING 중이어도 상관없이 처리한다 (currentState==IDLE 조건 없음). 실패해도
  // sendSafetyAbortStatus()는 부르지 않는다 — 그건 currentState를 IDLE로 되돌려서
  // 한창 진행 중이던 그리퍼/팔 시퀀스를 끊어버릴 수 있기 때문.
  if (strcmp(cmd, "cam_backward") == 0) {
    if (!camBackward()) {
      JETSON_SERIAL.println("{\"status\":\"cam_backward_failed\"}");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"cam_backward_done\"}");
    return;
  }

  // 보관함 왕복 끝나고 복귀 — 카메라를 다시 정면으로 돌린다. (마찬가지로 IDLE 무관)
  if (strcmp(cmd, "cam_forward") == 0) {
    if (!camForward()) {
      JETSON_SERIAL.println("{\"status\":\"cam_forward_failed\"}");
      return;
    }
    JETSON_SERIAL.println("{\"status\":\"cam_forward_done\"}");
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
      // 2000ms -> 200ms로 단축 — 매우 공격적, 물체를 완전히 쥐기 전에
      // 팔이 올라가기 시작하면 놓치거나 삐뚤게 들릴 위험 있음, 실물 확인 필수.
      if (!safeDelay(200)) {
        sendSafetyAbortStatus("grip_settle_wait");
        break;
      }

      if (!armCheck()) {
        sendSafetyAbortStatus("arm_check");
        break;
      }
      checkStartMs = millis();
      currentState = CHECKING;
      JETSON_SERIAL.print("{\"status\":\"at_check_pos\",\"cls\":\"");
      JETSON_SERIAL.print(currentCls);
      JETSON_SERIAL.println("\"}");
      break;

    case CHECKING:
      // Jetson이 confirm_grip/reject_grip을 보내면 parseCommand()가 상태를 바꾼다.
      // 여기서는 응답이 아예 안 올 때를 대비한 타임아웃만 본다.
      if (millis() - checkStartMs >= CHECK_TIMEOUT_MS) {
        JETSON_SERIAL.println("[OpenRB] 그립 확인 타임아웃 → 그리퍼 열고 팔 내림");
        if (!doRejectGrip("check_timeout")) break;
        currentCls[0] = '\0';
        currentState = IDLE;
        JETSON_SERIAL.println("{\"status\":\"grip_failed\"}");
      }
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
      // 그리퍼 다 열리고 물체가 바스켓에 완전히 떨어질 시간을 준다.
      // 2000ms -> 500ms로 단축(사이클 속도 우선) — 이 delay가 짧으면 물체가
      // 아직 집게 사이에 걸쳐있는 상태에서 다시 닫혀버릴 위험이 있음.
      if (!safeDelay(500)) {
        sendSafetyAbortStatus("drop_wait");
        break;
      }
      // ⚠️ 올리고→열고→내리고→닫고 순서(2026-07-22 재변경) — 열린 그리퍼를 단 채로
      // 팔이 내려가는 구간이 생긴다. 예전에 이 순서에서 "팔 내려가는 동안 엉뚱한
      // 물체가 벌어진 집게에 끼는" 문제가 있어서 한 번 닫고→내리는 순서로 바꿨던
      // 적이 있음(progress.md 2026-07-22 참고) — 사용자 요청으로 다시 되돌림.
      if (!armDown()) {
        sendSafetyAbortStatus("arm_down");
        break;
      }
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
  camSetup();
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
