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
 *
 * 물체 하나가 지나가는 도중에 "있음→없음→있음"이 여러 번 반복되는 경우(물체
 * 표면 모양/각도 때문에 빔이 순간적으로 다시 들어오는 등)를 대비해, 한 번
 * 카운트한 뒤 RECOUNT_COOLDOWN_MS 동안은 leading edge가 또 와도 새 물체로
 * 세지 않고 무시한다(같은 물체가 아직 지나가는 중이라고 판단).
 */

#define IR_SENSOR_PIN       2     // ⚠️ 실제 연결한 디지털 핀 번호로 바꿀 것 (OpenRB-150 핀맵 확인)
#define DEBOUNCE_MS         50    // ⚠️ 실측 필요 — 너무 짧으면 흔들림에 중복 카운트, 너무 길면 빠르게 지나가는 물체를 놓침
#define ACTIVE_LOW          true  // ⚠️ 실물 확인 필요 — OUT이 물체 감지 시 LOW면 true, HIGH면 false로
#define RECOUNT_COOLDOWN_MS 2000  // ⚠️ 실측 필요 — 물체 하나가 센서 앞을 완전히 지나가는 데 걸리는
                                  // 시간보다 조금 길게 잡을 것. 너무 길면 실제로 빠르게 연속 지나간
                                  // 다음 물체를 못 세고, 너무 짧으면 같은 물체를 중복으로 셀 수 있음

bool     objectPresent = false;
uint32_t lastChangeMs  = 0;
uint32_t objectCount   = 0;
bool     hasCounted    = false;  // 아직 한 번도 카운트 안 했으면 쿨다운 없이 바로 셈
uint32_t lastCountMs   = 0;

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
      if (!hasCounted || (now - lastCountMs) >= RECOUNT_COOLDOWN_MS) {
        objectCount++;
        lastCountMs = now;
        hasCounted  = true;
        Serial.print("[IR카운터] 물체 감지 — 누적 개수: ");
        Serial.println(objectCount);
      } else {
        Serial.println("[IR카운터] 재감지 무시 (쿨다운 중 — 같은 물체로 판단)");
      }
    } else {
      Serial.println("[IR카운터] 물체 통과 완료 (다음 물체 감지 대기)");
    }
  }
}
