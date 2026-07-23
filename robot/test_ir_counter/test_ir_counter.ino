/*
 * test_ir_counter.ino — KY-032 적외선 장애물 센서로 물체 통과 개수 세기 (단독 테스트)
 * ──────────────────────────────────────────────────────────────
 * 실행 보드: OpenRB-150 (ROBOTIS)
 * 독립 스케치 — robot.ino(메인 펌웨어)와 별개 폴더, 이것만 단독 업로드됨.
 *
 * KY-032는 VCC/GND/OUT 3핀 구성. OUT은 모듈에 따라 물체가 없을 때 HIGH,
 * 감지되면 LOW로 떨어지는 active-low 방식이 흔하지만 반대인 모듈도 있음
 * — ⚠️ 아래 ACTIVE_LOW 값은 실물로 꼭 확인할 것(시리얼 모니터로 손 가까이
 * 대봤을 때 카운트가 실제로 올라가는지 보면 됨). 모듈에 달린 포텐셔미터로
 * 감지 거리도 조절 가능.
 *
 * 카운트 로직: "물체 없음 → 있음"으로 바뀌는 순간(leading edge)에만 카운트를
 * 올린다 — 물체가 센서 앞에 한동안 머물러도 한 번만 세지고, 일단 "없음"으로
 * 돌아와야 다음 물체를 셀 준비가 된다. DEBOUNCE_MS 동안은 상태 전환을 무시해서
 * 신호가 흔들려도(bounce) 중복 카운트가 안 되게 한다.
 */

#define IR_SENSOR_PIN 2     // ⚠️ 실제 연결한 디지털 핀 번호로 바꿀 것 (OpenRB-150 핀맵 확인)
#define DEBOUNCE_MS   50    // ⚠️ 실측 필요 — 너무 짧으면 흔들림에 중복 카운트, 너무 길면 빠르게 지나가는 물체를 놓침
#define ACTIVE_LOW    true  // ⚠️ 실물 확인 필요 — OUT이 물체 감지 시 LOW면 true, HIGH면 false로

bool     objectPresent = false;
uint32_t lastChangeMs  = 0;
uint32_t objectCount   = 0;

bool readSensorRaw() {
  int v = digitalRead(IR_SENSOR_PIN);
  return ACTIVE_LOW ? (v == LOW) : (v == HIGH);
}

void setup() {
  Serial.begin(115200);
  pinMode(IR_SENSOR_PIN, INPUT);
  Serial.println("[IR카운터] 준비 완료 — 물체를 센서 앞으로 지나가게 해보세요");
}

void loop() {
  bool raw = readSensorRaw();
  uint32_t now = millis();

  if (raw != objectPresent && (now - lastChangeMs) >= DEBOUNCE_MS) {
    objectPresent = raw;
    lastChangeMs  = now;

    if (objectPresent) {
      objectCount++;
      Serial.print("[IR카운터] 물체 감지 — 누적 개수: ");
      Serial.println(objectCount);
    } else {
      Serial.println("[IR카운터] 물체 통과 완료 (다음 물체 감지 대기)");
    }
  }
}
