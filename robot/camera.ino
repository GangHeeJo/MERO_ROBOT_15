/*
 * camera.ino — 카메라 회전 서보 (XL430 × 1, ID 4)
 * ──────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 *
 * 전원 켤 때 카메라가 물리적으로 전방을 보고 있다고 가정하고, 그 위치를
 * 그대로 "정면" 기준으로 읽어서 저장한다 (라벨링 없이 그 자리 그대로 사용).
 * 보관함으로 이동할 때는 180도 돌려서 후방을 보게 하고, 복귀 시 다시 정면으로.
 */

extern Dynamixel2Arduino dxl;

bool safeSetGoalPosition(uint8_t id, int32_t goal_raw, uint32_t wait_ms);

#define CAM_ID 4
#define CAM_SPEED 50  // Profile Velocity, 낮을수록 느림 (180도 왕복 실측: 약 2.6초)

int32_t camForwardRaw = -1;  // 전원 켤 때 읽은 정면 위치 (raw)

void camSetup() {
  if (!dxl.ping(CAM_ID)) {
    Serial.println("[카메라모터] ❌ 초기화 실패 — ID/전원/배선 확인");
    return;
  }
  dxl.torqueOff(CAM_ID);
  dxl.setOperatingMode(CAM_ID, OP_POSITION);
  dxl.torqueOn(CAM_ID);
  dxl.writeControlTableItem(ControlTableItem::PROFILE_VELOCITY, CAM_ID, CAM_SPEED);

  camForwardRaw = dxl.getPresentPosition(CAM_ID, UNIT_RAW);
  Serial.print("[카메라모터] ✅ 초기화 완료 (정면 위치 raw=");
  Serial.print(camForwardRaw);
  Serial.println(")");
}

// ── 정면(전방) 보기 ──────────────────────────────────────
bool camForward() {
  if (camForwardRaw < 0) return false;
  bool ok = safeSetGoalPosition(CAM_ID, camForwardRaw, 3000);
  if (ok) Serial.println("[카메라모터] 정면 복귀");
  return ok;
}

// ── 후방(180도 회전) 보기 ────────────────────────────────
bool camBackward() {
  if (camForwardRaw < 0) return false;
  int32_t backward_raw = (camForwardRaw + 2048) % 4096;
  bool ok = safeSetGoalPosition(CAM_ID, backward_raw, 3000);
  if (ok) Serial.println("[카메라모터] 후방 회전");
  return ok;
}
