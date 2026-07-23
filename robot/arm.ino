/*
 * arm.ino — 팔(XL430×2, ID2 공유) + 바스켓 힌지(XL430×2, ID3 공유) 제어
 * ──────────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 구성 (전부 XL430, 12V):
 *   팔:    ID 2 — 물리 모터 2개가 같은 ID 공유, 한쪽은 Dynamixel Wizard에서
 *                 미리 DRIVE_MODE=Reverse로 구워놓음 (코드에서 재설정 안 함)
 *   바스켓: ID 3 — 위와 동일 구조
 *
 * dxl 인스턴스는 main.ino 에서 선언됨
 * safeSetGoalPosition은 safety.ino에서 제공 — overload 등을 감시하며 이동하고
 * 안전 문제가 생기면 false를 반환한다.
 */

extern Dynamixel2Arduino dxl;

bool safeSetGoalPosition(uint8_t id, int32_t goal_raw, uint32_t wait_ms);

// ── 모터 ID ──────────────────────────────────────────────
#define ARM_ID        2
#define CONT_ID       3

// ── 팔 위치 (raw, 0~4095, 실측) ───────────────────────────
#define ARM_DOWN_RAW   1580   // 집기 위치
#define ARM_UP_RAW     2850   // 투하 위치

// ── 바스켓 위치 (raw, 0~4095, 실측) ───────────────────────
#define CONT_CLOSED_RAW  2328
#define CONT_OPEN_RAW    1000

// ── 토크 제한 (%) ────────────────────────────────────────
#define ARM_TORQUE_PCT       80
#define CONTAINER_TORQUE_PCT 60

// ── 속도 (Profile Velocity, 낮을수록 느림) ───────────────
#define ARM_SPEED       150
#define CONTAINER_SPEED 40

// ── 모터 한 개 초기화 헬퍼 ───────────────────────────────
// DRIVE_MODE는 Wizard에서 이미 설정됨 — 여기서 건드리면 같은 ID를 공유하는
// 두 물리 모터 모두 같은 값으로 덮어써져 reverse 구분이 깨지므로 절대 쓰지 않는다.
static void _initOne(uint8_t id, int pwmLimit, int speed) {
  if (!dxl.ping(id)) {
    Serial.print("[팔] 초기화 실패 ID="); Serial.println(id);
    return;
  }
  dxl.torqueOff(id);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, pwmLimit);
  dxl.torqueOn(id);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, id, speed);
}

// ── 초기화 ───────────────────────────────────────────────
void armSetup() {
  int armLimit  = 885 * ARM_TORQUE_PCT  / 100;
  int contLimit = 885 * CONTAINER_TORQUE_PCT / 100;
  _initOne(ARM_ID,  armLimit,  ARM_SPEED);
  _initOne(CONT_ID, contLimit, CONTAINER_SPEED);
  containerClose();
  armUp();  // 시작 크기 규정 때문에 전원 켜질 때는 팔을 올린 상태로 대기 — "start" 명령 받으면 armDown()으로 내려감
  Serial.println("[팔] 초기화 완료 (팔 올림 + 바스켓 닫힘, start 명령 대기)");
}

// ── 팔 내리기 (집기 위치) ────────────────────────────────
// wait_ms 200 — 사이클 속도 우선으로 단축(2026-07-23, 실측 아닌 요청 반영).
// LIFTING 끝의 armDown() 완료 후 바로 "gripped"를 보내고 Jetson이 그 신호로
// UGV를 다시 움직이기 시작하므로, 이 대기가 그리퍼 암 내림↔UGV 재이동 사이
// 간격이다. 위치 폴링 도착 확인은 ARM_ID 공유 구조상 불가 — 팔이 다 안
// 내려간 채로 "gripped"가 가지 않는지 실물 확인 필수.
bool armDown() {
  return safeSetGoalPosition(ARM_ID, ARM_DOWN_RAW, 200);
}

// ── 팔 올리기 (바스켓 투하 위치) ────────────────────────
// wait_ms 1500 — 사이클 속도 우선으로 단축(2026-07-23, 실측 아닌 요청 반영).
// ARM_ID는 물리 모터 2개가 ID 공유라 위치 폴링으로 실도착 확인은 불가 —
// 패킷 충돌 위험. 실물에서 팔이 다 안 올라간 상태로 gripperOpen()이 불리지
// 않는지 반드시 확인 필요.
bool armUp() {
  return safeSetGoalPosition(ARM_ID, ARM_UP_RAW, 1500);
}

// ── 바스켓 닫기 ──────────────────────────────────────────
bool containerClose() {
  return safeSetGoalPosition(CONT_ID, CONT_CLOSED_RAW, 3000);
}

// ── 바스켓 열기 (합판 아래로 젖힘) ─────────────────────
bool containerOpen() {
  return safeSetGoalPosition(CONT_ID, CONT_OPEN_RAW, 3000);
}
