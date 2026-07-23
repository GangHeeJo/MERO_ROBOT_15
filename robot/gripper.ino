/*
 * gripper.ino — 그리퍼 제어 (XL430 × 1)
 * ──────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성:
 *   ID 1: 그리퍼 모터 (XL430) — 단일 모터, 기계적 연동으로 양 손가락 구동
 *
 * 전원:
 *   XL430 동작 전압 12V → UGV 내장 12V 배터리 → OpenRB 연결
 *
 * 사전 설정 (Dynamixel Wizard):
 *   - Baudrate: 1000000
 *   - ID: 1
 *
 * dxl 인스턴스는 main.ino 에서 정의됨 (extern 참조)
 * safeDelay/safeSetGoalPosition/safetyPoll은 safety.ino에서 제공
 */

// dxl 인스턴스는 main.ino 에서 선언됨
extern Dynamixel2Arduino dxl;

bool safeDelay(uint32_t wait_ms);
bool safeSetGoalPosition(uint8_t id, int32_t goal_raw, uint32_t wait_ms);
bool safetyPoll();
bool safetyHasFatalFault();

// ── 설정값 (raw, 0~4095, 실측) ───────────────────────────
#define GRIPPER_ID  1

#define FINGER_OPEN_RAW   2600   // 열림 (기존 2400에서 확대 — 기계적 한계 실물 확인 필요)
#define FINGER_CLOSE_RAW  750    // 닫힘 (닫히는 순간 전류 피크 예상)

// 파손 방지 토크 제한 (%)
#define GRIPPER_TORQUE_LIMIT_PCT 80

// 속도 (Profile Velocity, 낮을수록 느림)
#define GRIPPER_SPEED 200

// Load 기반 집기 감지 (실측: 물체 잡을 때 ~30%, 빈 손 ~0%)
// PRESENT_LOAD 단위: 0.1% (300 = 30%)
#define GRIP_LOAD_THRESHOLD 300

// 닫힘 중 load 감지 기반 안전 정지 설정
#define GRIPPER_CLOSE_TIMEOUT_MS       1000
#define GRIPPER_LOAD_CHECK_INTERVAL_MS 20
#define GRIPPER_LOAD_CONFIRM_COUNT     3
// 닫기 시작 직후(정지 관성/스티션으로 순간 전류 튀는 구간)는 load 체크를 건너뛴다 —
// 안 그러면 물체에 닿기도 전에 "잡았다"고 오판해서 조금만 닫혔다 바로 열리는 문제 발생
#define GRIPPER_LOAD_GRACE_MS          150
// 이 raw 값(닫히는 방향, OPEN→CLOSE 사이)에 도달하기 전까지는 load를 아예 판별하지
// 않고 무조건 계속 조인다 — 물체와 접촉하기엔 아직 너무 벌어진 구간이라 여기서
// load가 튀는 건 전부 오탐이기 때문. 실측 후 조정.
#define GRIPPER_LOAD_CHECK_RAW         1600
#define GRIPPER_EXTRA_CLOSE_RAW        46
#define GRIPPER_EXTRA_CLOSE_MS         150
#define GRIPPER_TARGET_TOLERANCE_RAW   23

// ── 초기화 ───────────────────────────────────────────────
void gripperSetup() {
  if (!dxl.ping(GRIPPER_ID)) {
    Serial.println("[그리퍼] ❌ 초기화 실패 — ID/전원/배선 확인");
    return;
  }
  dxl.torqueOff(GRIPPER_ID);
  dxl.setOperatingMode(GRIPPER_ID, OP_POSITION);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, GRIPPER_ID, 885 * GRIPPER_TORQUE_LIMIT_PCT / 100);
  dxl.torqueOn(GRIPPER_ID);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, GRIPPER_ID, GRIPPER_SPEED);
  gripperCloseIdle();
  Serial.println("[그리퍼] ✅ 초기화 완료 (닫힘 상태 — 대기 중 오탐 방지)");
}

// ── 그리퍼 열기 ──────────────────────────────────────────
bool gripperOpen() {
  bool ok = safeSetGoalPosition(GRIPPER_ID, FINGER_OPEN_RAW, 300);
  if (ok) {
    Serial.println("[그리퍼] 열림");
  }
  return ok;
}

// ── 그리퍼 닫기 (대기 상태용 — load 감지 없이 단순 위치 이동) ──
// gripperClose()와 달리 물체를 잡으려는 게 아니라 그냥 손가락을 오므려두는 용도.
// IDLE 진입/복귀 시 엉뚱한 물체가 벌어진 집게 안으로 들어와 잡히는 걸 막기 위해 사용.
bool gripperCloseIdle() {
  bool ok = safeSetGoalPosition(GRIPPER_ID, FINGER_CLOSE_RAW, 300);
  if (ok) {
    Serial.println("[그리퍼] 닫힘 (대기 상태)");
  }
  return ok;
}

// ── 그리퍼 열기 + 팔 내리기 동시 구동 (그립 실패 원위치용) ──────
// GRIP_CHECK에서 reject 판정이 나거나 확인 타임아웃일 때 쓴다. 그리퍼를 다 열고 나서
// 팔을 내리면 원위치까지 걸리는 시간이 두 배가 되므로, 두 모터에 목표 위치만 먼저
// 넣어두고 대기는 한 번만 해서 실제로 동시에 움직이게 한다.
bool gripperOpenWithArmDown() {
  if (safetyHasFatalFault()) return false;
  if (safetyPoll()) return false;

  dxl.setGoalPosition(GRIPPER_ID, FINGER_OPEN_RAW, UNIT_RAW);
  dxl.setGoalPosition(ARM_ID, ARM_DOWN_RAW, UNIT_RAW);

  bool ok = safeDelay(300);
  if (ok) {
    Serial.println("[그리퍼+팔] 그립 실패 원위치 — 그리퍼 열기 + 팔 내림 동시 완료");
  }
  return ok;
}

// ── 그리퍼 닫기 — 성공 여부 반환 ────────────────────────
// true: load 임계값 초과 → 물체 잡음 (접촉 위치에서 살짝 더 조인 뒤 유지)
// false: load 낮음 → 빈 손으로 닫힘 (미스) 또는 safety 개입으로 중단
bool gripperClose() {
  if (safetyPoll()) return false;

  // 최종 닫힘 위치까지 이동을 명령하되, 이동 중 load를 계속 확인해서
  // 물체가 잡히면 끝까지 밀어붙이지 않고 중간에 멈춘다.
  dxl.setGoalPosition(GRIPPER_ID, FINGER_CLOSE_RAW, UNIT_RAW);

  uint32_t start_ms = millis();
  uint8_t load_confirm_count = 0;

  while (millis() - start_ms < GRIPPER_CLOSE_TIMEOUT_MS) {
    if (safetyPoll()) return false;

    int32_t load = dxl.readControlTableItem(
      ControlTableItem::PRESENT_LOAD,
      GRIPPER_ID
    );

    int32_t abs_load = load < 0 ? -load : load;
    int32_t current_raw = dxl.getPresentPosition(GRIPPER_ID, UNIT_RAW);
    bool past_grace = (millis() - start_ms) >= GRIPPER_LOAD_GRACE_MS;

    // FINGER_CLOSE_RAW < FINGER_OPEN_RAW면 닫히는 방향은 raw 감소 방향이다.
    bool past_raw_gate = (FINGER_CLOSE_RAW < FINGER_OPEN_RAW)
      ? (current_raw <= GRIPPER_LOAD_CHECK_RAW)
      : (current_raw >= GRIPPER_LOAD_CHECK_RAW);

    // 순간적인 충격이나 노이즈를 피하기 위해 연속 기준을 둔다.
    // 닫기 시작 직후(grace 구간)나 아직 raw 게이트를 안 지난 구간은
    // 스티션/오탐으로 보고 load 판별 자체를 건너뛴다.
    if (past_grace && past_raw_gate && abs_load >= GRIP_LOAD_THRESHOLD) {
      load_confirm_count++;
    } else {
      load_confirm_count = 0;
    }

    if (load_confirm_count >= GRIPPER_LOAD_CONFIRM_COUNT) {
      int32_t squeeze_goal_raw;

      // FINGER_CLOSE_RAW < FINGER_OPEN_RAW면 닫힘 방향은 raw 감소 방향이다.
      if (FINGER_CLOSE_RAW < FINGER_OPEN_RAW) {
        squeeze_goal_raw = current_raw - GRIPPER_EXTRA_CLOSE_RAW;
        if (squeeze_goal_raw < FINGER_CLOSE_RAW) {
          squeeze_goal_raw = FINGER_CLOSE_RAW;
        }
      } else {
        squeeze_goal_raw = current_raw + GRIPPER_EXTRA_CLOSE_RAW;
        if (squeeze_goal_raw > FINGER_CLOSE_RAW) {
          squeeze_goal_raw = FINGER_CLOSE_RAW;
        }
      }

      dxl.setGoalPosition(GRIPPER_ID, squeeze_goal_raw, UNIT_RAW);
      if (!safeDelay(GRIPPER_EXTRA_CLOSE_MS)) return false;

      // 실제 도달 위치를 hold 목표로 다시 설정해서 최종 닫힘 위치까지 계속 밀지 않게 한다.
      int32_t hold_raw = dxl.getPresentPosition(GRIPPER_ID, UNIT_RAW);
      dxl.setGoalPosition(GRIPPER_ID, hold_raw, UNIT_RAW);

      Serial.print("[그리퍼] 잡음 — load: ");
      Serial.print(load);
      Serial.print(", contact raw: ");
      Serial.print(current_raw);
      Serial.print(", squeeze goal raw: ");
      Serial.print(squeeze_goal_raw);
      Serial.print(", hold raw: ");
      Serial.println(hold_raw);

      return true;
    }

    int32_t err = current_raw - FINGER_CLOSE_RAW;
    if (err < 0) err = -err;

    if (err <= GRIPPER_TARGET_TOLERANCE_RAW && millis() - start_ms > 200) {
      break;
    }

    if (!safeDelay(GRIPPER_LOAD_CHECK_INTERVAL_MS)) return false;
  }

  Serial.println("[그리퍼] 미스 — load 임계값 미달");
  return false;
}
