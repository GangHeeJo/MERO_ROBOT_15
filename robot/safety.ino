/*
 * safety.ino — Dynamixel overload/shutdown 자동 복구 루틴
 * ──────────────────────────────────────────────────────
 * 핵심 동작:
 *   1. Hardware Error Status(70)를 주기적으로 읽는다.
 *   2. Overload Error bit가 감지되면 dxl.reboot(id)를 보낸다.
 *   3. reboot 후 위치 제어 모드, PWM 제한, 현재 위치 hold 목표를 다시 설정한다.
 *   4. Torque On 후 현재 motion은 중단하고 IDLE로 복귀시킨다.
 *
 * 주의:
 *   - Overheating, Electrical Shock, Input Voltage, Encoder Error는 자동 재구동하지 않고 fatal fault로 둔다.
 *   - ARM_ID(2)/CONT_ID(3)는 각각 물리 모터 2개가 같은 ID를 공유한다.
 *     같은 ID로 응답이 겹치면 READ/PING 응답 충돌 때문에 Hardware Error Status 감지가
 *     불안정할 수 있고, reboot도 두 모터 모두에 걸린다 — 공유 ID 구조 자체의 제약이다.
 */

extern Dynamixel2Arduino dxl;

// -- Hardware Error Status bit ------------------------------
#define DXL_HW_ERR_INPUT_VOLTAGE    0x01
#define DXL_HW_ERR_OVERHEATING      0x04
#define DXL_HW_ERR_ENCODER          0x08
#define DXL_HW_ERR_ELECTRICAL_SHOCK 0x10
#define DXL_HW_ERR_OVERLOAD         0x20

#define DXL_HW_ERR_FATAL_MASK \
  (DXL_HW_ERR_INPUT_VOLTAGE | DXL_HW_ERR_OVERHEATING | DXL_HW_ERR_ENCODER | DXL_HW_ERR_ELECTRICAL_SHOCK)

// -- 복구 정책 ----------------------------------------------
#define DXL_SAFETY_CHECK_INTERVAL_MS 80
#define DXL_SAFETY_DELAY_SLICE_MS    20
#define DXL_REBOOT_TIMEOUT_MS        100
#define DXL_REBOOT_WAIT_MS           1200
#define DXL_RECOVERY_MAX_TRIES       2
#define DXL_RECOVERY_COOLDOWN_MS     1500

#define DXL_WATCHED_COUNT 3

const uint8_t DXL_WATCHED_IDS[DXL_WATCHED_COUNT] = {
  GRIPPER_ID,
  ARM_ID,
  CONT_ID,
};

uint8_t  safetyRecoverAttempts[DXL_WATCHED_COUNT] = {0, 0, 0};
uint32_t safetyLastRecoverMs[DXL_WATCHED_COUNT] = {0, 0, 0};
uint32_t safetyLastCheckMs = 0;

bool safetyRecovering = false;
bool safetyRecoveredFlag = false;
bool safetyFatalFault = false;
uint8_t safetyLastFaultId = 0;
uint8_t safetyLastHwError = 0;

int safetyIndexOf(uint8_t id) {
  for (int i = 0; i < DXL_WATCHED_COUNT; i++) {
    if (DXL_WATCHED_IDS[i] == id) return i;
  }
  return -1;
}

int safetyPwmLimitForId(uint8_t id) {
  if (id == GRIPPER_ID) return 885 * GRIPPER_TORQUE_LIMIT_PCT / 100;
  if (id == ARM_ID) return 885 * ARM_TORQUE_PCT / 100;
  if (id == CONT_ID) return 885 * CONTAINER_TORQUE_PCT / 100;
  return 885 * 50 / 100;
}

int32_t safetyFallbackRawForId(uint8_t id) {
  if (id == GRIPPER_ID) return FINGER_OPEN_RAW;
  if (id == ARM_ID) return ARM_DOWN_RAW;
  if (id == CONT_ID) return CONT_CLOSED_RAW;
  return 2048;
}

void safetyPrintHardwareError(uint8_t hw_error) {
  Serial.print("0x");
  if (hw_error < 16) Serial.print("0");
  Serial.print(hw_error, HEX);
  Serial.print(" [");

  bool first = true;
  if (hw_error & DXL_HW_ERR_INPUT_VOLTAGE) {
    Serial.print("InputVoltage"); first = false;
  }
  if (hw_error & DXL_HW_ERR_OVERHEATING) {
    if (!first) Serial.print(",");
    Serial.print("Overheating"); first = false;
  }
  if (hw_error & DXL_HW_ERR_ENCODER) {
    if (!first) Serial.print(",");
    Serial.print("Encoder"); first = false;
  }
  if (hw_error & DXL_HW_ERR_ELECTRICAL_SHOCK) {
    if (!first) Serial.print(",");
    Serial.print("ElectricalShock"); first = false;
  }
  if (hw_error & DXL_HW_ERR_OVERLOAD) {
    if (!first) Serial.print(",");
    Serial.print("Overload"); first = false;
  }
  if (first) Serial.print("None");

  Serial.print("]");
}

bool safetyConfigureMotorAfterReboot(uint8_t id) {
  bool ping_ok = dxl.ping(id);
  if (!ping_ok) {
    Serial.print("[안전] reboot 후 ping 실패 ID=");
    Serial.println(id);
    return false;
  }

  int32_t present_raw = dxl.getPresentPosition(id, UNIT_RAW);
  if (present_raw < 0 || present_raw > 4095) {
    present_raw = safetyFallbackRawForId(id);
  }

  dxl.torqueOff(id);
  delay(20);
  dxl.setOperatingMode(id, OP_POSITION);
  dxl.writeControlTableItem(ControlTableItem::PWM_LIMIT, id, safetyPwmLimitForId(id));

  // Torque On 직후 갑자기 다른 위치로 튀지 않도록 현재 위치를 목표 위치로 잡는다.
  dxl.setGoalPosition(id, present_raw, UNIT_RAW);
  dxl.torqueOn(id);
  delay(50);

  Serial.print("[안전] 재설정 완료 ID=");
  Serial.print(id);
  Serial.print(", hold raw=");
  Serial.println(present_raw);

  return true;
}

bool safetyRecoverMotorFromOverload(uint8_t id, uint8_t hw_error) {
  int idx = safetyIndexOf(id);
  if (idx < 0) return false;

  uint32_t now_ms = millis();

  if (safetyLastRecoverMs[idx] != 0 && now_ms - safetyLastRecoverMs[idx] < DXL_RECOVERY_COOLDOWN_MS) {
    safetyFatalFault = true;
    safetyLastFaultId = id;
    safetyLastHwError = hw_error;
    Serial.print("[안전] 짧은 시간 안에 overload 재발 - 자동 복구 중단 ID=");
    Serial.println(id);
    return true;
  }

  if (safetyRecoverAttempts[idx] >= DXL_RECOVERY_MAX_TRIES) {
    safetyFatalFault = true;
    safetyLastFaultId = id;
    safetyLastHwError = hw_error;
    Serial.print("[안전] overload 복구 횟수 초과 - 자동 복구 중단 ID=");
    Serial.println(id);
    return true;
  }

  safetyRecoverAttempts[idx]++;
  safetyLastRecoverMs[idx] = now_ms;
  safetyLastFaultId = id;
  safetyLastHwError = hw_error;
  safetyRecovering = true;

  Serial.print("[안전] Overload 감지 ID=");
  Serial.print(id);
  Serial.print(", hw_error=");
  safetyPrintHardwareError(hw_error);
  Serial.println(" -> reboot 시도");

  bool reboot_ok = dxl.reboot(id, DXL_REBOOT_TIMEOUT_MS);
  if (!reboot_ok) {
    // 같은 ID가 여러 개 있으면 Status Packet 충돌로 false가 나올 수 있다.
    // 그래도 reboot instruction 자체는 전송되었을 수 있으므로 잠시 기다린 뒤 ping으로 확인한다.
    Serial.print("[안전] reboot status 응답 없음 ID=");
    Serial.println(id);
  }

  delay(DXL_REBOOT_WAIT_MS);

  bool config_ok = safetyConfigureMotorAfterReboot(id);
  safetyRecovering = false;

  if (!config_ok) {
    safetyFatalFault = true;
    safetyLastFaultId = id;
    safetyLastHwError = hw_error;
    Serial.print("[안전] reboot 후 재설정 실패 ID=");
    Serial.println(id);
    return true;
  }

  safetyRecoveredFlag = true;

  Serial.print("[안전] 자동 복구 완료 ID=");
  Serial.println(id);

  return true;
}

bool safetyCheckMotor(uint8_t id) {
  uint8_t hw_error = (uint8_t)dxl.readControlTableItem(
    ControlTableItem::HARDWARE_ERROR_STATUS,
    id
  );

  if (hw_error == 0) {
    return false;
  }

  safetyLastFaultId = id;
  safetyLastHwError = hw_error;

  Serial.print("[안전] Hardware Error 감지 ID=");
  Serial.print(id);
  Serial.print(", hw_error=");
  safetyPrintHardwareError(hw_error);
  Serial.println();

  // 과열/전압/전기 충격/엔코더 오류는 기계나 전원 상태가 불안정한 것이므로 자동 재구동하지 않는다.
  if (hw_error & DXL_HW_ERR_FATAL_MASK) {
    safetyFatalFault = true;
    Serial.print("[안전] fatal fault - 자동 재구동 보류 ID=");
    Serial.println(id);
    return true;
  }

  if (hw_error & DXL_HW_ERR_OVERLOAD) {
    return safetyRecoverMotorFromOverload(id, hw_error);
  }

  return false;
}

bool safetyPoll() {
  if (safetyRecovering || safetyFatalFault) {
    return false;
  }

  uint32_t now_ms = millis();
  if (now_ms - safetyLastCheckMs < DXL_SAFETY_CHECK_INTERVAL_MS) {
    return false;
  }
  safetyLastCheckMs = now_ms;

  for (int i = 0; i < DXL_WATCHED_COUNT; i++) {
    if (safetyCheckMotor(DXL_WATCHED_IDS[i])) {
      return true;
    }
  }

  return false;
}

bool safeDelay(uint32_t wait_ms) {
  uint32_t start_ms = millis();
  while (millis() - start_ms < wait_ms) {
    if (safetyPoll()) return false;
    delay(DXL_SAFETY_DELAY_SLICE_MS);
  }
  return !safetyHasFatalFault();
}

bool safeSetGoalPosition(uint8_t id, int32_t goal_raw, uint32_t wait_ms) {
  if (safetyHasFatalFault()) return false;
  if (safetyPoll()) return false;

  dxl.setGoalPosition(id, goal_raw, UNIT_RAW);
  return safeDelay(wait_ms);
}

bool safetyHasRecoveredFlag() {
  return safetyRecoveredFlag;
}

bool safetyConsumeRecoveredFlag() {
  bool ret = safetyRecoveredFlag;
  safetyRecoveredFlag = false;
  return ret;
}

bool safetyHasFatalFault() {
  return safetyFatalFault;
}

uint8_t safetyGetLastFaultId() {
  return safetyLastFaultId;
}

uint8_t safetyGetLastHardwareError() {
  return safetyLastHwError;
}

void safetyResetFaultFlags() {
  safetyRecoveredFlag = false;
  safetyFatalFault = false;
  safetyRecovering = false;
  safetyLastFaultId = 0;
  safetyLastHwError = 0;

  for (int i = 0; i < DXL_WATCHED_COUNT; i++) {
    safetyRecoverAttempts[i] = 0;
    safetyLastRecoverMs[i] = 0;
  }

  Serial.println("[안전] fault flag reset");
}
